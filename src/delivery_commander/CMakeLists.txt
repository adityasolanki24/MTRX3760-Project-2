cmake_minimum_required(VERSION 3.8)
project(delivery_commander)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
)

# C++ executable - Delivery Commander
add_executable(delivery_commander src/delivery_commander.cpp)
ament_target_dependencies(delivery_commander
  rclcpp
  std_msgs
  geometry_msgs
)

# C++ executable - Delivery Navigator
add_executable(delivery_navigator src/delivery_navigator.cpp)
ament_target_dependencies(delivery_navigator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Install executables
install(TARGETS
  delivery_commander
  delivery_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()


