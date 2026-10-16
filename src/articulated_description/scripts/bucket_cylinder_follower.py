#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# arm_to_rocker fixed joint rpy Y component (matches xacro origin rpy="0 -0.7 0").
ROCKER_ANGLE = -0.7


def clamp(value: float, lower: float, upper: float):
    return max(lower, min(upper, value))


def rotate_y(x: float, z: float, angle: float):
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    return (
        x * cos_angle + z * sin_angle,
        -x * sin_angle + z * cos_angle,
    )


class BucketCylinderFollower(Node):
    def __init__(self):
        super().__init__("bucket_cylinder_follower")

        # Cylinder pivot in frontbase_link frame (front_link_to_cylinder joint origin).
        self.cyl_pivot_x = -0.642805
        self.cyl_pivot_z = 1.867242

        # bucket_arm_axis joint origin in frontbase_link frame.
        self.arm_axis_x = -0.64
        self.arm_axis_z = 1.87

        # Rocker joint origin in bucket_arm_Link frame
        # = arm_to_rocker xyz + rotate_y(rocker_joint xyz, ROCKER_ANGLE)
        self.rocker_in_arm_x = 0.95719 + 0.262138454  # 1.219328
        self.rocker_in_arm_z = -0.082758 + 0.220796174  # 0.138038

        # Rocker-to-bucket connector geometry.
        # rocker_to_bucket joint origin is expressed in rocker_link.
        self.rocker_to_bucket_x = 0.70625
        self.rocker_to_bucket_z = -0.15
        # Connector far-end pin hole in connector_to_bucket_link frame.
        # Far boss X range in mesh: [0.5529, 0.6809], midplane = 0.6169.
        self.connector_far_local_x = 0.93088
        self.connector_far_local_z = 0.0

        self.rocker_joint_mount_rpy = math.pi / 2.0
        # front_link_to_cylinder joint has rpy="0 0 0"
        self.front_to_cyl_mount_rpy = 0.0

        self.bucket_joint_x = 2.03
        self.bucket_joint_z = -0.767588
        self.bucket_mount_rpy = -0.0689393308210947
        # Connector pin hole in bucket_Link frame, relative to bucket_pitch_joint origin.
        self.bucket_target_local_x = -0.38234
        self.bucket_target_local_z = 0.13884

        self.upper_pin_local_x = -0.5
        self.upper_pin_local_z = 0.0
        self.upper_pin_radius = 0.5
        self.cylinder_fixed_length = 0.87799  # boss center from pivot at tube_pos=0

        self.rocker_joint_lower = -1.57
        self.rocker_joint_upper = 1.57
        self.rocker_to_bucket_lower = -2.35619
        self.rocker_to_bucket_upper = 1.0
        self.front_to_cyl_lower = -3.1416
        self.front_to_cyl_upper = 3.1416
        self.bucket_pitch_lower = -3.14
        self.bucket_pitch_upper = 0.0

        self.rocker_joint_angle = None
        self.rocker_to_bucket_angle = 0.0
        self.bucket_arm_angle = 0.0
        self.bucket_pitch_angle = 0.0
        self.tube_pos = None

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self.rocker_pub = self.create_publisher(
            Float64MultiArray,
            "/rocker_joint_position_controller_act/commands",
            10,
        )
        self.rocker_to_bucket_pub = self.create_publisher(
            Float64MultiArray,
            "/rocker_to_bucket_position_controller_act/commands",
            10,
        )
        self.front_to_cyl_pub = self.create_publisher(
            Float64MultiArray,
            "/front_link_to_cylinder_position_controller_act/commands",
            10,
        )
        self.bucket_pub = self.create_publisher(
            Float64MultiArray,
            "/bucket_pitch_position_controller_act/commands",
            10,
        )

        self.create_timer(0.01, self._publish_target)
        self.get_logger().info("bucket_cylinder_follower started")

    def _on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == "rocker_joint":
                self.rocker_joint_angle = pos
            elif name == "rocker_to_bucket":
                self.rocker_to_bucket_angle = pos
            elif name == "bucket_pitch_joint":
                self.bucket_pitch_angle = pos
            elif name == "tube_to_inner_cylinder":
                self.tube_pos = pos
            elif name == "bucket_arm_axis":
                self.bucket_arm_angle = pos

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _rocker_joint_in_frontbase(self):
        """Rocker joint center in frontbase_link frame.

        bucket_arm_axis has axis="0 -1 0", so joint angle q rotates the arm
        frame by R_Y(-q) relative to frontbase_link.
        """
        dx, dz = rotate_y(
            self.rocker_in_arm_x, self.rocker_in_arm_z, -self.bucket_arm_angle
        )
        return self.arm_axis_x + dx, self.arm_axis_z + dz

    # ── Circle-circle IK: cylinder pivot ∩ rocker_joint (frontbase_link frame) ──

    def _solve_cylinder_rocker_geometry(self, tube_pos: float):
        """
        Intersect two circles in frontbase_link frame:
          Circle A: center = cylinder pivot, radius = cylinder_fixed_length + tube_pos
          Circle B: center = rocker_joint,   radius = upper_pin_radius

        Returns (rocker_joint_angle, front_to_cyl_angle), or (None, None) when the
        cylinder is geometrically out of reach (circles don't intersect).
        """
        ax, az = self.cyl_pivot_x, self.cyl_pivot_z
        bx, bz = self._rocker_joint_in_frontbase()
        r1 = self.cylinder_fixed_length + tube_pos
        r2 = self.upper_pin_radius

        dbx = bx - ax
        dbz = bz - az
        d = math.sqrt(dbx * dbx + dbz * dbz)

        # Cylinder out of reach: circles don't intersect → no valid IK solution.
        if r1 + r2 < d or r1 < abs(d - r2) - 1e-9:
            return None, None

        r1 = clamp(r1, abs(d - r2), d + r2)

        a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
        h = math.sqrt(max(0.0, r1 * r1 - a * a))

        px = ax + a * dbx / d
        pz = az + a * dbz / d
        perp_x = -dbz / d
        perp_z = dbx / d

        candidates = [
            (px + h * perp_x, pz + h * perp_z),
            (px - h * perp_x, pz - h * perp_z),
        ]

        local_angle = math.atan2(-self.upper_pin_local_z, self.upper_pin_local_x)
        current_rj = (
            self.rocker_joint_angle if self.rocker_joint_angle is not None else 0.0
        )
        best_rj = best_arm = None
        best_dist = float("inf")

        for cx, cz in candidates:
            # Direction from rocker_joint to intersection point in frontbase_link.
            # Total angle = (-bucket_arm_angle) + ROCKER_ANGLE + mount_rpy + rj + local_angle
            # => rj = theta - local_angle - mount_rpy - ROCKER_ANGLE + bucket_arm_angle
            theta = math.atan2(-(cz - bz), cx - bx)
            rj_raw = (
                theta
                - local_angle
                - self.rocker_joint_mount_rpy
                - ROCKER_ANGLE
                + self.bucket_arm_angle
            )
            rj_raw = math.atan2(math.sin(rj_raw), math.cos(rj_raw))
            rj = clamp(rj_raw, self.rocker_joint_lower, self.rocker_joint_upper)

            out_of_range_penalty = max(0.0, abs(rj_raw - rj))
            dist = abs(rj_raw - current_rj) + 1e6 * out_of_range_penalty

            if dist < best_dist:
                best_dist = dist
                best_rj = rj
                dx_ac = cx - ax
                dz_ac = cz - az
                best_arm = clamp(
                    math.atan2(-dz_ac, dx_ac) - self.front_to_cyl_mount_rpy,
                    self.front_to_cyl_lower,
                    self.front_to_cyl_upper,
                )

        return best_rj, best_arm

    # ── Circle-circle IK: connector loop (bucket_arm_Link frame) ─────────────

    def _solve_bucket_connector_geometry(self, rocker_joint_angle: float):
        """
        Close the kinematic loop: connector far-end pin hole == bucket pin hole.
        Computed in bucket_arm_Link frame.
        Returns (rocker_to_bucket_angle, bucket_pitch_angle).
        """
        total_rocker_link_angle = (
            ROCKER_ANGLE + self.rocker_joint_mount_rpy + rocker_joint_angle
        )

        cj_x, cj_z = rotate_y(
            self.rocker_to_bucket_x, self.rocker_to_bucket_z, total_rocker_link_angle
        )
        ax = self.rocker_in_arm_x + cj_x
        az = self.rocker_in_arm_z + cj_z

        bx = self.bucket_joint_x
        bz = self.bucket_joint_z

        r1 = math.sqrt(self.connector_far_local_x**2 + self.connector_far_local_z**2)
        r2 = math.sqrt(self.bucket_target_local_x**2 + self.bucket_target_local_z**2)

        dbx = bx - ax
        dbz = bz - az
        d = math.sqrt(dbx * dbx + dbz * dbz)
        r1 = clamp(r1, abs(d - r2), d + r2)

        a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
        h = math.sqrt(max(0.0, r1 * r1 - a * a))

        px = ax + a * dbx / d
        pz = az + a * dbz / d
        perp_x = -dbz / d
        perp_z = dbx / d

        candidates = [
            (px + h * perp_x, pz + h * perp_z),
            (px - h * perp_x, pz - h * perp_z),
        ]

        connector_local_angle = math.atan2(
            -self.connector_far_local_z, self.connector_far_local_x
        )
        bucket_local_angle = math.atan2(
            -self.bucket_target_local_z, self.bucket_target_local_x
        )
        current_bp = self.bucket_pitch_angle
        current_rtb = self.rocker_to_bucket_angle
        best_bp = best_rtb = None
        best_dist = float("inf")

        for pin_x, pin_z in candidates:
            theta = math.atan2(-(pin_z - bz), pin_x - bx)
            bp_raw = self.bucket_mount_rpy + bucket_local_angle - theta
            bp_raw = math.atan2(math.sin(bp_raw), math.cos(bp_raw))
            bp = clamp(bp_raw, self.bucket_pitch_lower, self.bucket_pitch_upper)

            # If bucket_pitch is clamped by its physical limit, the ideal
            # circle-intersection pin is no longer where the bucket pin will be.
            # Aim the connector at the actual clamped bucket pin instead.
            actual_bucket_angle = self.bucket_mount_rpy + bucket_local_angle - bp
            actual_pin_x = bx + r2 * math.cos(actual_bucket_angle)
            actual_pin_z = bz - r2 * math.sin(actual_bucket_angle)

            world_angle = math.atan2(-(actual_pin_z - az), actual_pin_x - ax)
            rtb_raw = world_angle - connector_local_angle - total_rocker_link_angle
            rtb_raw = math.atan2(math.sin(rtb_raw), math.cos(rtb_raw))
            rtb = clamp(rtb_raw, self.rocker_to_bucket_lower, self.rocker_to_bucket_upper)

            actual_connector_angle = (
                total_rocker_link_angle + rtb + connector_local_angle
            )
            actual_connector_x = ax + r1 * math.cos(actual_connector_angle)
            actual_connector_z = az - r1 * math.sin(actual_connector_angle)
            closure_error = math.hypot(
                actual_connector_x - actual_pin_x,
                actual_connector_z - actual_pin_z,
            )

            out_of_range_penalty = max(0.0, abs(bp_raw - bp)) + max(
                0.0, abs(rtb_raw - rtb)
            )
            candidate_dist = (
                100.0 * closure_error
                + abs(bp - current_bp)
                + abs(rtb - current_rtb)
                + 1e6 * out_of_range_penalty
            )

            if candidate_dist < best_dist:
                best_dist = candidate_dist
                best_bp = bp
                best_rtb = rtb

        return best_rtb, best_bp

    # ── Main publish loop ─────────────────────────────────────────────────────

    @staticmethod
    def _at_limit(value: float, lower: float, upper: float) -> bool:
        """Return True if value is at or past a joint limit (skip publishing to avoid lock-up)."""
        return value <= lower or value >= upper

    def _publish_target(self):
        if self.tube_pos is None:
            return

        rocker_joint_target, front_to_cyl_target = self._solve_cylinder_rocker_geometry(
            self.tube_pos
        )

        # rocker_joint and front_to_cyl are computed as a coupled pair: publish
        # both or neither. If a value is clamped to a limit, still publish the
        # limit; otherwise the position controller keeps its previous command.
        if rocker_joint_target is not None:
            rocker_msg = Float64MultiArray()
            rocker_msg.data = [rocker_joint_target]
            self.rocker_pub.publish(rocker_msg)

            arm_msg = Float64MultiArray()
            arm_msg.data = [front_to_cyl_target]
            self.front_to_cyl_pub.publish(arm_msg)

        # rtb 和 bp 都从同一个 IK 交点计算（用实际 rocker_joint 传感器读数），
        # bp 直接瞄准 connector 应到达的位置，不受 rtb 控制器滞后影响。
        actual_rj = self.rocker_joint_angle if self.rocker_joint_angle is not None else 0.0
        rocker_to_bucket_target, bucket_pitch_target = self._solve_bucket_connector_geometry(actual_rj)

        rocker_to_bucket_msg = Float64MultiArray()
        rocker_to_bucket_msg.data = [rocker_to_bucket_target]
        self.rocker_to_bucket_pub.publish(rocker_to_bucket_msg)

        bucket_msg = Float64MultiArray()
        bucket_msg.data = [bucket_pitch_target]
        self.bucket_pub.publish(bucket_msg)


def main():
    node = None
    try:
        rclpy.init()
        node = BucketCylinderFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
