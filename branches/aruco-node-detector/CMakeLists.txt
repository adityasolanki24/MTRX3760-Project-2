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

add_executable(aruco_detector_node src/aruco_detector_node.cpp)

target_include_directories(aruco_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(aruco_detector_node
  rclcpp sensor_msgs geometry_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs visualization_msgs)

target_link_libraries(aruco_detector_node ${OpenCV_LIBS})
target_compile_features(aruco_detector_node PRIVATE cxx_std_17)

install(TARGETS aruco_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
