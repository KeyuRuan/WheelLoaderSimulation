cmake_minimum_required(VERSION 3.8)
project(articulated_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_covariance_relay src/imu_covariance_relay.cpp)
ament_target_dependencies(imu_covariance_relay rclcpp sensor_msgs)

install(DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  imu_covariance_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/hydraulic_controller.py
  scripts/bucket_cylinder_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
