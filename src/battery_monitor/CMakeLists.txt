cmake_minimum_required(VERSION 3.8)
project(battery_monitor)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- Include directories ---
include_directories(
  include
)

# --- Executable ---
add_executable(battery_monitor_node src/battery_monitor_node.cpp)
ament_target_dependencies(battery_monitor_node rclcpp std_msgs)

# --- Install targets ---
install(TARGETS
  battery_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install header files ---
install(DIRECTORY include/
  DESTINATION include/
)

# --- Install launch files (optional, for future) ---
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

