#ifndef ARUCO_DETECTOR_ARUCO_POSE_NODE_HPP
#define ARUCO_DETECTOR_ARUCO_POSE_NODE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

// STL
#include <unordered_map>
#include <string>
#include <memory>

class ArucoPoseNode : public rclcpp::Node {
public:
  ArucoPoseNode();

private:
  // --- Callbacks ---
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  // --- Helpers ---
  static cv::Ptr<cv::aruco::Dictionary> getDictionaryByName(const std::string &name);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr getPosePublisherForId(int id);

  // --- Parameters / config ---
  bool have_camera_;
  double marker_length_;
  bool publish_tf_;
  bool debug_image_;
  std::string camera_frame_;
  std::string pose_topic_base_;

  // --- OpenCV / detection ---
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
  cv::Mat K_;  // 3x3 CV_64F camera matrix
  cv::Mat D_;  // Nx1 CV_64F distortion coefficients

  // --- ROS interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif  // ARUCO_DETECTOR_ARUCO_POSE_NODE_HPP
