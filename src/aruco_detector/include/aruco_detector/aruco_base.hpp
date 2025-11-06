// MTRX3760 2025 Project 2: Warehouse Robot DevKit 
// File: aruco_base.hpp
// Author(s): Raquel Kampel 
//
// Base class providing shared ArUco detection and camera calibration functionality.
// Implements encapsulation of OpenCV and ROS2 internals, and defines a polymorphic
// interface for derived detector nodes.

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
  explicit ArucoBase(const std::string &node_name);
  virtual ~ArucoBase();

protected:
  // --- Shared Callbacks ---
  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  // --- Shared Helper ---
  static cv::Ptr<cv::aruco::Dictionary> getDictionaryByName(const std::string &name);

  // --- Pure virtual to enforce polymorphism ---
  virtual void processImage(const cv::Mat &frame,
                            const std_msgs::msg::Header &header) = 0;

  // --- Shared members ---
  bool have_camera_;
  cv::Mat K_, D_;
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
};

#endif  // ARUCO_DETECTOR_ARUCO_BASE_HPP
