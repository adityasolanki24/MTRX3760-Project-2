cmake_minimum_required(VERSION 3.8)
project(robot_drive)

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# === Node 1: wall_follower ===
add_executable(wall_follower
  src/wall_follower.cpp
)
ament_target_dependencies(wall_follower
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# === Node 2: low_battery_homing ===
add_executable(low_battery_homing
  src/low_battery_homing.cpp
)
ament_target_dependencies(low_battery_homing
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# === Installation ===
install(TARGETS
  wall_follower
  low_battery_homing
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# === Export dependencies ===
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_package()
