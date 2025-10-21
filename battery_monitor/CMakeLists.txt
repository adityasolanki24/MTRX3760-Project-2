cmake_minimum_required(VERSION 3.8)
project(battery_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(battery_lib
  src/battery.cpp
  src/logger.cpp
)
ament_target_dependencies(battery_lib rclcpp std_msgs nav_msgs)

add_executable(battery_monitor_node src/battery_monitor_node.cpp)
ament_target_dependencies(battery_monitor_node rclcpp std_msgs nav_msgs)
target_link_libraries(battery_monitor_node battery_lib)

install(TARGETS battery_lib battery_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
