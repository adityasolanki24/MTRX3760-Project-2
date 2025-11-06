#ifndef ARUCO_DETECTOR_ARUCO_BASE_HPP
#define ARUCO_DETECTOR_ARUCO_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

#include <unordered_map>
#include <string>
#include <memory>

class ArucoBase : public rclcpp::Node {
public:
  explicit ArucoBase(const std::string &node_name)
      : Node(node_name), have_camera_(false) {
    params_ = cv::aruco::DetectorParameters::create();
  }

  virtual ~ArucoBase() = default;

protected:
  // --- Common Camera Info Callback ---
  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
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

  // --- Helper: Get dictionary by name ---
  static cv::Ptr<cv::aruco::Dictionary> getDictionaryByName(const std::string &name) {
    using namespace cv::aruco;
    static const std::unordered_map<std::string, PREDEFINED_DICTIONARY_NAME> lut = {
        {"DICT_4X4_50", DICT_4X4_50}, {"DICT_5X5_50", DICT_5X5_50},
        {"DICT_6X6_50", DICT_6X6_50}, {"DICT_7X7_50", DICT_7X7_50},
        {"DICT_ARUCO_ORIGINAL", DICT_ARUCO_ORIGINAL}};
    auto it = lut.find(name);
    return cv::aruco::getPredefinedDictionary(
        it == lut.end() ? DICT_5X5_50 : it->second);
  }

  // --- Pure virtual to enforce polymorphism ---
  virtual void processImage(const cv::Mat &frame,
                            const std_msgs::msg::Header &header) = 0;

  // --- Shared members for derived classes ---
  bool have_camera_;
  cv::Mat K_, D_;
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
};

#endif  // ARUCO_DETECTOR_ARUCO_BASE_HPP


