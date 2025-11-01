#ifndef ARUCO_DETECTOR_NODE_HPP_
#define ARUCO_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
// #include <opencv2/objdetect/aruco_detector.hpp>

#include <memory>
#include <map>
#include <string>
#include <vector>

namespace aruco_detector{

class ArucoDetectorNode : public rclcpp::Node{
public:
  ArucoDetectorNode();

private:
  // ---- Callbacks ----
  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  // ---- Helper ----
  cv::aruco::Dictionary getDictionary(const std::string &name);

  // ---- Parameters ----
  std::string dict_name_;
  std::string img_transport_;
  std::string frame_prefix_;
  std::string camera_frame_;
  double marker_length_{0.05};
  bool publish_tf_{true};
  bool show_rejected_{false};
  int  refine_corners_{1};
  bool publish_markers_{true};
  int  skip_n_frames_{0};

  // ---- State ----
  bool   have_cam_info_{false};
  size_t frame_count_{0};
  cv::Mat cam_matrix_;
  cv::Mat dist_coeffs_;

  // ---- ROS Interfaces ----
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  image_transport::Publisher debug_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // OpenCV ArUco (legacy)
  cv::aruco::DetectorParameters dp_;     // filled from ROS params
  cv::Ptr<cv::aruco::DetectorParameters> dp_ptr_; // Ptr passed to detectMarkers
  cv::Ptr<cv::aruco::Dictionary> dict_ptr_;
};

}  // namespace aruco_detector

#endif  // ARUCO_DETECTOR_NODE_HPP_


	

