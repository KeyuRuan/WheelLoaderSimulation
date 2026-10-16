# Wheel Loader Simulation

A ROS 2 and Gazebo simulation of an articulated wheel loader. The workspace
contains the vehicle model, a warehouse environment, ROS 2 controllers,
hydraulic response emulation, an IMU bridge, and a simulated Velodyne lidar.

## Supported environment

This project is developed and tested with:

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- `colcon`, `rosdep`, and a C++17 compiler

Other ROS 2 or Gazebo releases are not currently tested. A GPU and working
OpenGL driver are recommended for the Gazebo GUI and GPU lidar.

## Repository layout

| Package | Purpose |
| --- | --- |
| `articulated_description` | Wheel-loader model, controllers, launch files, and helper nodes |
| `environment_description` | Warehouse world and local Gazebo models |
| `velodyne_description` | VLP-16 URDF/Xacro model and meshes |
| `velodyne_gazebo_plugins` | Gazebo Fortress lidar system plugin |

## Installation

Install ROS 2 Humble by following the official ROS 2 instructions, then install
the main runtime and build dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-desktop \
  ros-humble-ign-ros2-control \
  ros-humble-ros-gz \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

Initialize `rosdep` once per machine if it has not already been initialized:

```bash
sudo rosdep init
rosdep update
```

Clone and build the repository as a standalone ROS 2 workspace:

```bash
git clone https://github.com/KeyuRuan/WheelLoaderSimulation.git ~/wheel_loader_ws
cd ~/wheel_loader_ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Source the workspace in every new terminal before running its launch files:

```bash
source /opt/ros/humble/setup.bash
source ~/wheel_loader_ws/install/setup.bash
```

## Running the simulation

Launch the warehouse, wheel loader, controllers, IMU bridge, and lidar:

```bash
ros2 launch articulated_description sim.launch.py
```

Controller loading begins approximately six seconds after Gazebo starts.

To run the complete simulation without the Gazebo graphical interface:

```bash
ros2 launch articulated_description sim.launch.py headless:=true
```

To launch only the environment without spawning the vehicle or its controller
nodes:

```bash
ros2 launch articulated_description sim.launch.py spawn_vehicle:=false
```

To inspect the robot model in RViz without running Gazebo:

```bash
ros2 launch articulated_description display.launch.py
```

## Sending commands

The low-level command topics use `std_msgs/msg/Float64MultiArray`.

Drive all four wheels at 2 rad/s:

```bash
ros2 topic pub --once /wheel_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [2.0, 2.0, 2.0, 2.0]}"
```

Steer left by opening the right steering valve. Valve inputs are in the range
0 to 127; values below 50 are inside the configured deadband:

```bash
ros2 topic pub --once /hinge_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 80.0]}"
```

Raise the loader arm to 0.8 rad:

```bash
ros2 topic pub --once /bucket_arm_position_controller_act/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.8]}"
```

Extend the bucket hydraulic-cylinder joint to 0.20 m. The follower node
computes the linked bucket mechanism automatically:

```bash
ros2 topic pub --once /tube_position_controller_act/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.20]}"
```

The included helper script runs a complete raise/lower sequence:

```bash
./loader_motion_sequence.sh
```

Run `./loader_motion_sequence.sh --help` for its configurable target values.
Stop the vehicle by publishing zero wheel velocities and zero valve openings.

## Useful topics

| Topic | Type | Description |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Simulated joint state feedback |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU data with covariance metadata |
| `/wheel_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | Four wheel velocity commands |
| `/hinge_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Left/right steering valve commands |
| `/bucket_arm_position_controller_act/commands` | `std_msgs/msg/Float64MultiArray` | Loader arm position command |
| `/tube_position_controller_act/commands` | `std_msgs/msg/Float64MultiArray` | Bucket-cylinder extension command |

Use `ros2 topic list` after launch to inspect all available sensor, controller,
and debugging topics.

## Troubleshooting

If packages are not found, source both ROS 2 and the workspace setup files. If
controllers do not appear immediately, wait for the delayed launch sequence and
check their state with:

```bash
ros2 control list_controllers
```

If Gazebo cannot find a model or plugin, launch through `sim.launch.py`; it sets
`IGN_GAZEBO_RESOURCE_PATH` and `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` for this
workspace.

For a clean rebuild:

```bash
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## License

Original project code is released under the MIT License; see `LICENSE`.
Bundled third-party components retain their original licenses and attribution;
see `THIRD_PARTY_NOTICES.md` and the files under `LICENSES/`.
