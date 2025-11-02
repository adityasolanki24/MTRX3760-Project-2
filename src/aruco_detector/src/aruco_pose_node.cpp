/ MTRX3760 2025 Project 2: Warehouse Robot DevKit 
// File: aruco_pose_node.cpp 
// Author(s): Raquel Kampel
// 
// 3D drawing and position estimation of aruco node + file logging 


// src/aruco_pose_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp> 
#include <cv_bridge/cv_bridge.hpp>

#include <unordered_map>
#include <string>
#include <memory>

class ArucoPoseNode : public rclcpp::Node {
public:
  ArucoPoseNode() : Node("aruco_pose_node"),
                    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
    // Parameters
    marker_length_ = this->declare_parameter<double>("marker_length", 0.05); // meters
    camera_frame_  = this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    publish_tf_    = this->declare_parameter<bool>("publish_tf", true);
    debug_image_   = this->declare_parameter<bool>("debug_image", true);
    pose_topic_base_ = this->declare_parameter<std::string>("pose_topic_base", "aruco/pose");
    std::string dict_name = this->declare_parameter<std::string>("dictionary", "DICT_5X5_50");
 
    RCLCPP_INFO(get_logger(),
            "Param dictionary='%s' (declared default 5x5)",
            this->get_parameter("dictionary").as_string().c_str());
  
    dict_   = getDictionaryByName(dict_name);
    params_ = cv::aruco::DetectorParameters::create();

    // QoS: images + cam info are sensor streams
    auto sensor_qos = rclcpp::SensorDataQoS();

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera/camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (!have_camera_) {
          // Copy K and D
          K_ = cv::Mat(3, 3, CV_64F);
          for (int i = 0; i < 9; ++i) K_.at<double>(i/3, i%3) = msg->k[i];
          D_ = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
          for (size_t i = 0; i < msg->d.size(); ++i) D_.at<double>(static_cast<int>(i),0) = msg->d[i];
          have_camera_ = true;
          RCLCPP_INFO(get_logger(), "Camera intrinsics received (K,D).");
        }
      });

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", sensor_qos,
      std::bind(&ArucoPoseNode::imageCb, this, std::placeholders::_1));

    if (debug_image_) {
      debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco/debug_image", 10);
    }

    RCLCPP_INFO(get_logger(), "ArucoPoseNode up. marker_length=%.3f m, dict=%s",
                marker_length_, dict_name.c_str());
  }

private:
  // --- Helpers ---
  static cv::Ptr<cv::aruco::Dictionary> getDictionaryByName(const std::string &name) {
    using namespace cv::aruco;
    static const std::unordered_map<std::string, PREDEFINED_DICTIONARY_NAME> lut = {
      {"DICT_4X4_50", DICT_4X4_50}, {"DICT_4X4_100", DICT_4X4_100},
      {"DICT_4X4_250", DICT_4X4_250}, {"DICT_4X4_1000", DICT_4X4_1000},
      {"DICT_5X5_50", DICT_5X5_50}, {"DICT_5X5_100", DICT_5X5_100},
      {"DICT_5X5_250", DICT_5X5_250}, {"DICT_5X5_1000", DICT_5X5_1000},
      {"DICT_6X6_50", DICT_6X6_50}, {"DICT_6X6_100", DICT_6X6_100},
      {"DICT_6X6_250", DICT_6X6_250}, {"DICT_6X6_1000", DICT_6X6_1000},
      {"DICT_7X7_50", DICT_7X7_50}, {"DICT_7X7_100", DICT_7X7_100},
      {"DICT_7X7_250", DICT_7X7_250}, {"DICT_7X7_1000", DICT_7X7_1000},
      {"DICT_ARUCO_ORIGINAL", DICT_ARUCO_ORIGINAL},
      {"DICT_APRILTAG_16h5", DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25h9", DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36h10", DICT_APRILTAG_36h10},
      {"DICT_APRILTAG_36h11", DICT_APRILTAG_36h11},
    };
    auto it = lut.find(name);
    return cv::aruco::getPredefinedDictionary(it == lut.end() ? cv::aruco::DICT_5X5_50 : it->second);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr getPosePublisherForId(int id) {
    auto it = pose_pubs_.find(id);
    if (it != pose_pubs_.end()) return it->second;

    // Create a dedicated topic per marker id, e.g., aruco/pose/23
    //std::string topic = pose_topic_base_ + "/" + std::to_string(id);
    std::string topic = pose_topic_base_ + "/id_" + std::to_string(id); //aruco/pose/id_23

    auto pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    pose_pubs_.emplace(id, pub);
    RCLCPP_INFO(get_logger(), "Advertising pose topic: %s", topic.c_str());
    return pub;
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!have_camera_) return;

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }
    cv::Mat frame = cv_ptr->image;

    // Detect markers
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(frame, dict_, corners, ids, params_, rejected);
    if (ids.empty()) {
      if (debug_image_) {
        // Optionally publish the raw image still
        auto out = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        debug_img_pub_->publish(*out);
      }
      return;
    }

    // Pose for each marker
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, K_, D_, rvecs, tvecs);

    for (size_t i = 0; i < ids.size(); ++i) {
      // Rodrigues to rotation matrix
      cv::Mat R_cv;
      cv::Rodrigues(rvecs[i], R_cv);

      // Convert to tf2 quaternion
      tf2::Matrix3x3 R_tf(
        R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
        R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
        R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2));
      tf2::Quaternion q; R_tf.getRotation(q);

      // PoseStamped (in camera optical frame)
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = msg->header.stamp;
      pose_msg.header.frame_id = camera_frame_;
      pose_msg.pose.position.x = tvecs[i][0];
      pose_msg.pose.position.y = tvecs[i][1];
      pose_msg.pose.position.z = tvecs[i][2];
      pose_msg.pose.orientation = tf2::toMsg(q);

      // Publish on per-id topic
      getPosePublisherForId(ids[i])->publish(pose_msg);
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "ID %d  pos=[%.3f, %.3f, %.3f]  dist=%.3f m",
      ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2], cv::norm(tvecs[i]));


      // TF: camera_frame -> aruco_<id>
      if (publish_tf_) {
        geometry_msgs::msg::TransformStamped T;
        T.header = pose_msg.header;
        T.child_frame_id = "aruco_" + std::to_string(ids[i]);
        T.transform.translation.x = pose_msg.pose.position.x;
        T.transform.translation.y = pose_msg.pose.position.y;
        T.transform.translation.z = pose_msg.pose.position.z;
        T.transform.rotation = pose_msg.pose.orientation;
        tf_broadcaster_->sendTransform(T);
      }

      // Draw annotation for debugging
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
      // Most OpenCV builds have either drawAxis (aruco) or drawFrameAxes (calib3d)
      // Try aruco::drawAxis first:
      try {
        cv::drawFrameAxes(frame, K_, D_, rvecs[i], tvecs[i],
                  static_cast<float>(marker_length_ * 0.5f), 2);
      } catch (...) {
        // Fallback if aruco::drawAxis is unavailable in your build
        try {
          cv::drawFrameAxes(frame, K_, D_, rvecs[i], tvecs[i], marker_length_ * 0.5);
        } catch (...) {
          // ignore
        }
      }
    }

    if (debug_image_) {
      auto out = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
      debug_img_pub_->publish(*out);
    }
  }

  // --- Members ---
  bool have_camera_ = false;
  double marker_length_;
  bool publish_tf_;
  bool debug_image_;
  std::string camera_frame_;
  std::string pose_topic_base_;

  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
  cv::Mat K_, D_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoPoseNode>());
  rclcpp::shutdown();
  return 0;
}
