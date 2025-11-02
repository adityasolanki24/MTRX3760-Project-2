cmake_minimum_required(VERSION 3.5)
project(mode_selector)

find_package(ament_cmake REQUIRED)

add_executable(robot_mode_selector src/robot_mode_selector.cpp)
set_target_properties(robot_mode_selector PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS robot_mode_selector
  DESTINATION lib/${PROJECT_NAME})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()


