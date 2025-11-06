#ifndef ARUCO_DETECTOR_ARUCO_POSE_NODE_HPP
#define ARUCO_DETECTOR_ARUCO_POSE_NODE_HPP

#include "aruco_detector/aruco_base.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ArucoPoseNode : public ArucoBase {
public:
  ArucoPoseNode();

private:
  // --- Callbacks ---
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void processImage(const cv::Mat &frame,
                    const std_msgs::msg::Header &header) override;

  // --- Helpers ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
  getPosePublisherForId(int id);

  // --- Parameters ---
  double marker_length_;
  bool publish_tf_;
  bool debug_image_;
  std::string camera_frame_;
  std::string pose_topic_base_;

  // --- ROS Interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif  // ARUCO_DETECTOR_ARUCO_POSE_NODE_HPP
