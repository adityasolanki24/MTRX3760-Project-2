cmake_minimum_required(VERSION 3.8)
project(wall_follower)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# C++ executable
add_executable(wall_follower_cpp src/wall_follower.cpp)
ament_target_dependencies(wall_follower_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Install executable
install(TARGETS
  wall_follower_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
