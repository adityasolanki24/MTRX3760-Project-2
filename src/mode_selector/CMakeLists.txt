cmake_minimum_required(VERSION 3.5)
project(mode_selector)

# Default to C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# Base robot class
add_library(base_robot
  src/base_robot.cpp
)
target_include_directories(base_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(base_robot
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# Inspection robot class
add_library(inspection_robot
  src/inspection_robot.cpp
)
target_include_directories(inspection_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(inspection_robot
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(inspection_robot base_robot)

# Delivery robot class
add_library(delivery_robot
  src/delivery_robot.cpp
)
target_include_directories(delivery_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(delivery_robot
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_action
  nav2_msgs
)
target_link_libraries(delivery_robot base_robot)

# Main executable - polymorphic mode selector
add_executable(robot_mode_selector
  src/robot_mode_selector.cpp
)
target_include_directories(robot_mode_selector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_mode_selector
  rclcpp
)
target_link_libraries(robot_mode_selector
  base_robot
  inspection_robot
  delivery_robot
)

# Install targets
install(TARGETS
  base_robot
  inspection_robot
  delivery_robot
  robot_mode_selector
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
