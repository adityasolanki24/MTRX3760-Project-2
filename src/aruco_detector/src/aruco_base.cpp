// MTRX3760 2025 Project 2: Warehouse Robot DevKit 
// File: aruco_base.cpp
// Author(s): Raquel Kampel 
//
// Implements the ArucoBase class, which encapsulates shared ROS2 and OpenCV 
// functionality for ArUco marker detection. This base class provides camera 
// calibration handling, parameter management, and dictionary setup for derived 
// nodes such as ArucoPoseNode and ArucoDetectorNode.

#include "aruco_detector/aruco_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/aruco.hpp>

ArucoBase::ArucoBase(const std::string &node_name)
    : rclcpp::Node(node_name), have_camera_(false) {
  params_ = cv::aruco::DetectorParameters::create();
}

ArucoBase::~ArucoBase() = default;

void ArucoBase::cameraInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
  if (have_camera_)
    return;

  K_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i)
    K_.at<double>(i / 3, i % 3) = msg->k[i];

  D_ = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
  for (size_t i = 0; i < msg->d.size(); ++i)
    D_.at<double>(static_cast<int>(i), 0) = msg->d[i];

  have_camera_ = true;
  RCLCPP_INFO(get_logger(), "Camera intrinsics received (K, D).");
}

cv::Ptr<cv::aruco::Dictionary> ArucoBase::getDictionaryByName(const std::string &name) {
  using namespace cv::aruco;
  static const std::unordered_map<std::string, PREDEFINED_DICTIONARY_NAME> lut = {
      {"DICT_4X4_50", DICT_4X4_50}, {"DICT_4X4_100", DICT_4X4_100},
      {"DICT_5X5_50", DICT_5X5_50}, {"DICT_5X5_100", DICT_5X5_100},
      {"DICT_6X6_50", DICT_6X6_50}, {"DICT_6X6_100", DICT_6X6_100},
      {"DICT_7X7_50", DICT_7X7_50}, {"DICT_7X7_100", DICT_7X7_100},
      {"DICT_ARUCO_ORIGINAL", DICT_ARUCO_ORIGINAL}};
  auto it = lut.find(name);
  auto dict_id = (it == lut.end()) ? DICT_5X5_50 : it->second;
  return cv::aruco::getPredefinedDictionary(dict_id);
}
