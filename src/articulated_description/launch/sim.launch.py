import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import xacro


def generate_launch_description():
    spawn_vehicle = LaunchConfiguration("spawn_vehicle")
    headless = LaunchConfiguration("headless")
    pkg = get_package_share_directory("articulated_description")
    pkg_env = get_package_share_directory("environment_description")
    pkg_velodyne_plugins = get_package_prefix("velodyne_gazebo_plugins")
    pkg_ign_ros2_control = get_package_prefix("ign_ros2_control")
    urdf_file = os.path.join(pkg, "urdf", "articulated.urdf.xacro")
    dynamic_tf_urdf_file = os.path.join(pkg, "urdf", "articulated_dynamic_tf.urdf")

    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toxml()
    with open(dynamic_tf_urdf_file, "r", encoding="utf-8") as file:
        dynamic_tf_robot_desc = file.read()

    controllers_yaml = os.path.join(pkg, "config", "controllers.yaml")
    robot_desc = robot_desc.replace(
        "$(find articulated_description)/config/controllers.yaml", controllers_yaml
    )

    pkg_velodyne_desc = get_package_share_directory("velodyne_description")

    # Tell Gazebo where to find package meshes and local models. Each package is
    # installed under its own install/<package>/share directory.
    set_env = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=":".join([
            os.path.join(pkg, ".."),
            os.path.join(pkg_velodyne_desc, ".."),
            os.path.join(pkg_env, "models"),
        ]),
    )

    set_plugin_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=os.pathsep.join([
            os.path.join(pkg_velodyne_plugins, "lib"),
            os.path.join(pkg_ign_ros2_control, "lib"),
        ]),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        condition=IfCondition(spawn_vehicle),
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("/tf", "/sim_debug/full_robot/tf"),
            ("/tf_static", "/sim_debug/full_robot/tf_static"),
        ],
    )
    dynamic_tf_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="sim_dynamic_state_publisher",
        condition=IfCondition(spawn_vehicle),
        parameters=[
            {
                "robot_description": dynamic_tf_robot_desc,
                "use_sim_time": True,
            }
        ],
    )

    world_file = os.path.join(pkg_env, "worlds", "concrete_batching_plant.sdf")
    gz_sim_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
        )
    )

    ign_gazebo_gui = IncludeLaunchDescription(
        gz_sim_launch,
        condition=UnlessCondition(headless),
        launch_arguments={
            "gz_args": "-r "
            + world_file
            + " --gui-config "
            + os.path.join(pkg, "config", "gui.config")
        }.items(),
    )

    ign_gazebo_headless = IncludeLaunchDescription(
        gz_sim_launch,
        condition=IfCondition(headless),
        launch_arguments={
            "gz_args": "--headless-rendering -s -r " + world_file
        }.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        condition=IfCondition(spawn_vehicle),
        arguments=[
            "-string",
            robot_desc,
            "-name",
            "articulated_vehicle",
            "-x",
            "-28.7",
            "-y",
            "-15.7",
            "-z",
            "0.75",
            "-Y",
            "-1.5708",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(spawn_vehicle),
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        remappings=[
            ("/imu/data_raw", "/imu/data_raw_raw"),
        ],
        output="screen",
    )

    imu_covariance_relay = Node(
        package="articulated_description",
        executable="imu_covariance_relay",
        condition=IfCondition(spawn_vehicle),
        parameters=[
            {
                "orientation_variance": 1e-2,
                "angular_velocity_variance": 1e-4,
                "linear_acceleration_variance": 1e-2,
            }
        ],
        output="screen",
    )

    load_jsb = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    load_wheel = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "wheel_velocity_controller_act",
        ],
        output="screen",
    )
    load_hinge = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "hinge_position_controller_act",
        ],
        output="screen",
    )
    load_bucket_arm = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "bucket_arm_position_controller_act",
        ],
        output="screen",
    )
    load_rocker_joint = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "rocker_joint_position_controller_act",
        ],
        output="screen",
    )
    load_rocker_to_bucket = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "rocker_to_bucket_position_controller_act",
        ],
        output="screen",
    )
    load_front_to_cylinder = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "front_link_to_cylinder_position_controller_act",
        ],
        output="screen",
    )
    load_tube = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "tube_position_controller_act",
        ],
        output="screen",
    )
    load_bucket_pitch = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "bucket_pitch_position_controller_act",
        ],
        output="screen",
    )
    hydraulic_controller = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "/usr/bin/python3",
            os.path.join(
                get_package_share_directory("articulated_description"),
                "..", "..", "lib", "articulated_description", "hydraulic_controller.py",
            ),
        ],
        output="screen",
    )

    bucket_cylinder_follower = ExecuteProcess(
        condition=IfCondition(spawn_vehicle),
        cmd=[
            "/usr/bin/python3",
            os.path.join(
                get_package_share_directory("articulated_description"),
                "..", "..", "lib", "articulated_description", "bucket_cylinder_follower.py",
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_vehicle",
                default_value="true",
                description="Whether to spawn the wheel loader and start its ROS nodes/controllers.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run the Gazebo server without its graphical interface.",
            ),
            set_env,
            set_plugin_path,
            ign_gazebo_gui,
            ign_gazebo_headless,
            robot_state_publisher,
            dynamic_tf_publisher,
            TimerAction(period=2.0, actions=[spawn]),
            TimerAction(period=3.0, actions=[bridge]),
            TimerAction(period=3.5, actions=[imu_covariance_relay]),
            TimerAction(period=6.0, actions=[load_jsb, load_wheel, load_hinge, load_bucket_arm, load_rocker_joint, load_rocker_to_bucket, load_front_to_cylinder, load_tube, load_bucket_pitch]),
            TimerAction(period=8.0, actions=[hydraulic_controller]),
            TimerAction(period=6.1, actions=[bucket_cylinder_follower]),
        ]
    )
