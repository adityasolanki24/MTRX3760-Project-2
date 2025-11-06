// MTRX3760 2025 Project 2: Warehouse Robot DevKit 
// File: aruco_pose_node.cpp
// Author(s): Raquel Kampel 
//
// Implements the ArucoPoseNode class, which detects individual ArUco markers,
// estimates their 3D poses, and publishes each marker’s pose and transform.

#include "aruco_detector/aruco_pose_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ArucoPoseNode::ArucoPoseNode()
    : ArucoBase("aruco_pose_node"),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {

  // --- Parameters ---
  marker_length_ = this->declare_parameter<double>("marker_length", 0.05);
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
  publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
  debug_image_ = this->declare_parameter<bool>("debug_image", true);
  pose_topic_base_ = this->declare_parameter<std::string>("pose_topic_base", "aruco/pose");
  std::string dict_name = this->declare_parameter<std::string>("dictionary", "DICT_5X5_50");

  dict_ = getDictionaryByName(dict_name);

  // --- Subscriptions ---
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera/camera_info", 10,
      std::bind(&ArucoBase::cameraInfoCb, this, std::placeholders::_1));

  auto sensor_qos = rclcpp::SensorDataQoS();
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", sensor_qos,
      std::bind(&ArucoPoseNode::imageCb, this, std::placeholders::_1));

  // --- Publishers ---
  if (debug_image_)
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco/debug_image", 10);

  RCLCPP_INFO(get_logger(), "ArucoPoseNode ready. marker_length=%.3f, dict=%s",
              marker_length_, dict_name.c_str());
}

void ArucoPoseNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  if (!have_camera_) return;
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
    return;
  }
  processImage(cv_ptr->image, msg->header);
}

void ArucoPoseNode::processImage(const cv::Mat &frame,
                                 const std_msgs::msg::Header &header) {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  cv::aruco::detectMarkers(frame, dict_, corners, ids, params_, rejected);
  if (ids.empty()) return;

  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, K_, D_, rvecs, tvecs);

  cv::Mat dbg = frame.clone();
  for (size_t i = 0; i < ids.size(); ++i) {
    cv::aruco::drawDetectedMarkers(dbg, corners, ids);
    cv::drawFrameAxes(dbg, K_, D_, rvecs[i], tvecs[i], marker_length_ * 0.5);

    // --- Rotation and Pose ---
    cv::Mat R_cv;
    cv::Rodrigues(rvecs[i], R_cv);
    tf2::Matrix3x3 R_tf(R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                        R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                        R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2));
    tf2::Quaternion q;
    R_tf.getRotation(q);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.header.frame_id = camera_frame_;
    pose.pose.position.x = tvecs[i][0];
    pose.pose.position.y = tvecs[i][1];
    pose.pose.position.z = tvecs[i][2];
    pose.pose.orientation = tf2::toMsg(q);
    getPosePublisherForId(ids[i])->publish(pose);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped T;
      T.header = pose.header;
      T.child_frame_id = "aruco_" + std::to_string(ids[i]);
      T.transform.translation.x = pose.pose.position.x;
      T.transform.translation.y = pose.pose.position.y;
      T.transform.translation.z = pose.pose.position.z;
      T.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(T);
    }
  }

  if (debug_image_) {
    auto out = cv_bridge::CvImage(header, "bgr8", dbg).toImageMsg();
    debug_pub_->publish(*out);
  }
}

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
ArucoPoseNode::getPosePublisherForId(int id) {
  auto it = pose_pubs_.find(id);
  if (it != pose_pubs_.end()) return it->second;
  auto pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_topic_base_ + "/id_" + std::to_string(id), 10);
  pose_pubs_[id] = pub;
  return pub;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoPoseNode>());
  rclcpp::shutdown();
  return 0;
}
