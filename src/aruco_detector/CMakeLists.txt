# MTRX3760 2025 Project 2: Warehouse Robot DevKit 
# File: CMakeLists.txt
# Author(s): Raquel Kampel 
#
# Builds ArucoPoseNode and shared ArucoBase class for ROS 2 detection system.

cmake_minimum_required(VERSION 3.16)
project(aruco_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d highgui aruco)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(aruco_pose_node 
  src/aruco_pose_node.cpp
  src/aruco_base.cpp
)

ament_target_dependencies(aruco_pose_node
  rclcpp sensor_msgs geometry_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(aruco_pose_node ${OpenCV_LIBS})
target_compile_features(aruco_pose_node PRIVATE cxx_std_17)

install(TARGETS aruco_pose_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
