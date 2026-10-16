#!/usr/bin/env python3
"""
液压系统延迟仿真节点

订阅外部控制器直接下发的指令话题，加入液压动态后转发给实际控制器：

  /wheel_velocity_controller/commands  ──→ [dead time + 一阶惯性] ──→ /wheel_velocity_controller_act/commands
  /hinge_position_controller/commands  ──→ [dead time + 左右阀开度动态 + 阀口非线性] ──→ /hinge_position_controller_act/commands

注：铰接通道输入是左右液压阀开度 [left_valve, right_valve]，范围 0~127。
    左转时右阀开度增大，右转时左阀开度增大。
    本节点将阀开度经过死区、动态建立和非线性映射后，输出实际铰接角速度。

这里只保留“空载 / 满载”两种手动工况，不做自动负载估计。
"""

import random
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

DT = 0.02  # 控制更新周期 (s)，50 Hz


class HydraulicController(Node):
    def __init__(self):
        super().__init__("hydraulic_controller")

        # ── 负载状态（False=空载，True=满载） ────────────────────────
        # 仿真里手动切换两种工况即可。
        self.is_loaded = False

        # ── 车辆物理参数 ──────────────────────────────────────────────
        self.max_gamma = 0.61  # 铰接角上限 (rad)

        # ── 纯滞后参数 (dead time, 单位 s) ───────────────────────────
        # 轮驱延迟暂时旁路，便于排查轮速异常是否来自仿真接触/约束。
        # 铰接继续保留较明显的液压延迟。
        # 按实车观测调整：从指令下发到 phi 基本到位约 400 ms。
        # 这里用“纯滞后 + 一阶建压”共同逼近该响应，而不是把纯死区直接设成 400 ms。
        self.T_dead_wheel_base = 0.0
        self.T_dead_hinge_base = 0.18
        self.T_dead_jitter = 0.01
        self.T_dead_wheel_reversal = 0.0

        # ── 一阶惯性参数 (τ, 单位 s) ────────────────────────────────
        # 轮驱延迟暂时旁路；铰接保留更明显的液压建压感。
        self.tau_wheel_min = 0.0  # 车轮空载
        self.tau_wheel_max = 0.0  # 车轮满载
        self.tau_hinge_open_min = 0.10  # 转向阀开度建立，空载
        self.tau_hinge_open_max = 0.25  # 转向阀开度建立，满载
        self.tau_hinge_close_min = 0.08  # 收阀略快于开阀
        self.tau_hinge_close_max = 0.18

        # ── 内部状态 ──────────────────────────────────────────────────
        # 死区队列：(execute_time_ns, data)
        self._wheel_queue: deque = deque()
        self._hinge_queue: deque = deque()

        # 经过死区后等待滤波的期望值
        self._wheel_des: list = [0.0, 0.0, 0.0, 0.0]
        self._hinge_des_valves: list = [0.0, 0.0]

        # 滤波后的输出（发给实际控制器）
        self._wheel_out: list = [0.0, 0.0, 0.0, 0.0]
        self._hinge_out: list = [0.0]

        # 左右阀命令：原始输入范围 [0, 127]；低于 50 为开度死区，不响应。
        # 阀状态范围 [0, 1]。
        self.valve_input_max = 127.0
        self.valve_input_deadband = 50.0
        self.valve_boost_step = 20.0
        self.valve_boost_min = 30.0
        self.valve_boost_target = 55.0
        self.left_valve_cmd = 0.0
        self.right_valve_cmd = 0.0
        self.left_valve_state = 0.0
        self.right_valve_state = 0.0
        self.left_low_input_boost = 0.0
        self.right_low_input_boost = 0.0

        # 阀口静态特性
        self.max_hinge_joint_vel = 0.1525  # rad/s，关节坐标最大角速度
        self.valve_exponent = 2.0  # 大开度更敏感

        # 当前铰接角（关节坐标）
        self.current_hinge_joint = 0.0

        # 轮驱换向检测
        self._prev_wheel_sign = 0

        # ── ROS 接口 ──────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            self._on_wheel_cmd,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/hinge_position_controller/commands",
            self._on_hinge_cmd,
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )
        self._wheel_pub = self.create_publisher(
            Float64MultiArray, "/wheel_velocity_controller_act/commands", 10
        )
        self._hinge_pub = self.create_publisher(
            Float64MultiArray, "/hinge_position_controller_act/commands", 10
        )
        self._hinge_valve_debug_pub = self.create_publisher(
            Float64MultiArray, "/hinge_valve_debug", 10
        )
        self._hinge_actual_valve_pub = self.create_publisher(
            Float64MultiArray, "/hinge_actual_valve_opening", 10
        )
        self._hinge_phi_dot_pub = self.create_publisher(
            Float64MultiArray, "/hinge_phi_dot", 10
        )

        self.create_timer(DT, self._control_loop)

        self.get_logger().info(
            f"液压延迟节点已启动 | "
            f"mode={'loaded' if self.is_loaded else 'empty'} | "
            f"dead time wheel {self.T_dead_wheel_base*1000:.0f}±{self.T_dead_jitter*1000:.0f} ms, "
            f"hinge {self.T_dead_hinge_base*1000:.0f}±{self.T_dead_jitter*1000:.0f} ms | "
            f"τ_wheel {self.tau_wheel_min*1000:.0f}~{self.tau_wheel_max*1000:.0f} ms | "
            f"τ_hinge_open {self.tau_hinge_open_min*1000:.0f}~{self.tau_hinge_open_max*1000:.0f} ms | "
            f"τ_hinge_close {self.tau_hinge_close_min*1000:.0f}~{self.tau_hinge_close_max*1000:.0f} ms | "
            f"valve deadband {self.valve_input_deadband:.0f}/{self.valve_input_max:.0f} | "
            f"directional boost {self.valve_boost_min:.0f}~{self.valve_input_deadband:.0f} "
            f"+{self.valve_boost_step:.0f}/cycle -> {self.valve_boost_target:.0f}"
        )

    # ── 指令回调：入队 ────────────────────────────────────────────

    def _sample_dead_time(self, base: float) -> float:
        return max(0.0, base + random.uniform(-self.T_dead_jitter, self.T_dead_jitter))

    @staticmethod
    def _sign_with_deadband(value: float, threshold: float = 0.1) -> int:
        if value > threshold:
            return 1
        if value < -threshold:
            return -1
        return 0

    def _on_wheel_cmd(self, msg: Float64MultiArray):
        avg = sum(msg.data) / len(msg.data) if msg.data else 0.0
        cur_sign = self._sign_with_deadband(avg)
        reversal = (
            cur_sign != 0
            and self._prev_wheel_sign != 0
            and cur_sign != self._prev_wheel_sign
        )
        if cur_sign != 0:
            self._prev_wheel_sign = cur_sign

        T = 0.0
        if reversal:
            T += self.T_dead_wheel_reversal
        execute_at = self.get_clock().now().nanoseconds + int(T * 1e9)
        self._wheel_queue.append((execute_at, list(msg.data)))

    def _on_hinge_cmd(self, msg: Float64MultiArray):
        T = self._sample_dead_time(self.T_dead_hinge_base)
        execute_at = self.get_clock().now().nanoseconds + int(T * 1e9)
        self._hinge_queue.append((execute_at, list(msg.data)))

    def _on_joint_states(self, msg: JointState):
        try:
            idx = msg.name.index("behindbase_to_frontbase")
        except ValueError:
            return
        if idx < len(msg.position):
            self.current_hinge_joint = float(msg.position[idx])

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))

    def _normalize_valve_cmd(self, raw_opening: float) -> float:
        raw_opening = self._clamp(raw_opening, 0.0, self.valve_input_max)
        # 开度死区：低于 50 不产生有效阀开度；达到 50 后保留原始开度比例。
        if raw_opening < self.valve_input_deadband:
            return 0.0

        x = raw_opening / self.valve_input_max
        return self._clamp(x, 0.0, 1.0)

    def _select_active_valve(self, left_raw: float, right_raw: float) -> str | None:
        if left_raw <= 0.0 and right_raw <= 0.0:
            return None
        if left_raw > 0.0 and right_raw <= 0.0:
            return "left"
        if right_raw > 0.0 and left_raw <= 0.0:
            return "right"
        if left_raw > right_raw:
            return "left"
        if right_raw > left_raw:
            return "right"
        return None

    def _apply_directional_boost(
        self, left_raw: float, right_raw: float
    ) -> tuple[float, float]:
        left_raw = self._clamp(left_raw, 0.0, self.valve_input_max)
        right_raw = self._clamp(right_raw, 0.0, self.valve_input_max)
        if self.valve_boost_target <= 0.0:
            self.left_low_input_boost = 0.0
            self.right_low_input_boost = 0.0
            return left_raw, right_raw

        active = self._select_active_valve(left_raw, right_raw)

        if active is None:
            self.left_low_input_boost = 0.0
            self.right_low_input_boost = 0.0
            return 0.0, 0.0

        if active == "left":
            self.right_low_input_boost = 0.0
            if left_raw <= self.valve_boost_min or left_raw >= self.valve_input_deadband:
                self.left_low_input_boost = 0.0
                return left_raw, 0.0
            self.left_low_input_boost = min(
                self.valve_boost_target,
                self.left_low_input_boost + self.valve_boost_step,
            )
            return self.left_low_input_boost, 0.0

        self.left_low_input_boost = 0.0
        if right_raw <= self.valve_boost_min or right_raw >= self.valve_input_deadband:
            self.right_low_input_boost = 0.0
            return 0.0, right_raw
        self.right_low_input_boost = min(
            self.valve_boost_target,
            self.right_low_input_boost + self.valve_boost_step,
        )
        return 0.0, self.right_low_input_boost

    def _update_valve_state(
        self, current: float, target: float, tau_open: float, tau_close: float
    ) -> float:
        tau = tau_open if target > current else tau_close
        alpha = 1.0 if tau <= 0.0 else DT / (tau + DT)
        updated = current + alpha * (target - current)
        return self._clamp(updated, 0.0, 1.0)

    def _valve_opening_to_flow(self, opening: float) -> float:
        opening = self._clamp(opening, 0.0, 1.0)
        return opening**self.valve_exponent

    def _apply_valve_state_deadband(self, valve_state: float) -> float:
        actual_opening = valve_state * self.valve_input_max
        if actual_opening < self.valve_input_deadband:
            return 0.0
        return valve_state

    def _apply_hinge_limits(self, joint_vel: float) -> float:
        margin = 0.01
        joint_pos = self.current_hinge_joint

        if joint_pos >= self.max_gamma - margin and joint_vel > 0.0:
            return 0.0
        if joint_pos <= -self.max_gamma + margin and joint_vel < 0.0:
            return 0.0
        return joint_vel

    # ── 主控制循环 ────────────────────────────────────────────────

    def _control_loop(self):
        now = self.get_clock().now().nanoseconds

        while self._wheel_queue and self._wheel_queue[0][0] <= now:
            _, data = self._wheel_queue.popleft()
            self._wheel_des = data

        while self._hinge_queue and self._hinge_queue[0][0] <= now:
            _, data = self._hinge_queue.popleft()
            left_raw = float(data[0]) if len(data) > 0 else 0.0
            right_raw = float(data[1]) if len(data) > 1 else 0.0
            self._hinge_des_valves = [left_raw, right_raw]

        tau_w = self.tau_wheel_max if self.is_loaded else self.tau_wheel_min
        tau_h_open = (
            self.tau_hinge_open_max if self.is_loaded else self.tau_hinge_open_min
        )
        tau_h_close = (
            self.tau_hinge_close_max if self.is_loaded else self.tau_hinge_close_min
        )

        alpha_w = 1.0 if tau_w <= 0.0 else DT / (tau_w + DT)

        self._wheel_out = [
            w + alpha_w * (d - w) for w, d in zip(self._wheel_out, self._wheel_des)
        ]

        left_boosted_raw, right_boosted_raw = self._apply_directional_boost(
            self._hinge_des_valves[0], self._hinge_des_valves[1]
        )

        self.left_valve_cmd = self._normalize_valve_cmd(left_boosted_raw)
        self.right_valve_cmd = self._normalize_valve_cmd(right_boosted_raw)
        self.left_valve_state = self._update_valve_state(
            self.left_valve_state, self.left_valve_cmd, tau_h_open, tau_h_close
        )
        self.right_valve_state = self._update_valve_state(
            self.right_valve_state, self.right_valve_cmd, tau_h_open, tau_h_close
        )

        left_effective_state = self._apply_valve_state_deadband(self.left_valve_state)
        right_effective_state = self._apply_valve_state_deadband(
            self.right_valve_state
        )

        left_flow = self._valve_opening_to_flow(left_effective_state)
        right_flow = self._valve_opening_to_flow(right_effective_state)

        # 左转时右阀开度大 -> 关节速度为负；右转时左阀开度大 -> 关节速度为正。
        hinge_joint_vel = (left_flow - right_flow) * self.max_hinge_joint_vel
        hinge_joint_vel = self._apply_hinge_limits(hinge_joint_vel)
        self._hinge_out = [hinge_joint_vel]

        wheel_msg = Float64MultiArray()
        wheel_msg.data = self._wheel_out
        self._wheel_pub.publish(wheel_msg)

        hinge_msg = Float64MultiArray()
        hinge_msg.data = self._hinge_out
        self._hinge_pub.publish(hinge_msg)

        phi_dot_msg = Float64MultiArray()
        phi_dot_msg.data = [hinge_joint_vel]
        self._hinge_phi_dot_pub.publish(phi_dot_msg)

        actual_valve_msg = Float64MultiArray()
        actual_valve_msg.data = [
            left_effective_state * self.valve_input_max,
            right_effective_state * self.valve_input_max,
        ]
        self._hinge_actual_valve_pub.publish(actual_valve_msg)

        hinge_debug_msg = Float64MultiArray()
        hinge_debug_msg.data = [
            self._hinge_des_valves[0],
            self._hinge_des_valves[1],
            left_boosted_raw,
            right_boosted_raw,
            self.left_valve_cmd,
            self.right_valve_cmd,
            self.left_valve_state,
            self.right_valve_state,
            left_effective_state,
            right_effective_state,
            left_flow,
            right_flow,
            hinge_joint_vel,
        ]
        self._hinge_valve_debug_pub.publish(hinge_debug_msg)


def main():
    node = None
    try:
        rclpy.init()
        node = HydraulicController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
