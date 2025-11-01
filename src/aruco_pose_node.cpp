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
  ArucoPoseNode() 
    : Node("aruco_pose_node"),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) 
  {
    // --- Parameters ---
    marker_length_ = this->declare_parameter<double>("marker_length", 0.05);
    camera_frame_  = this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    publish_tf_    = this->declare_parameter<bool>("publish_tf", true);
    debug_image_   = this->declare_parameter<bool>("debug_image", true);
    pose_topic_base_ = this->declare_parameter<std::string>("pose_topic_base", "aruco/pose");
    std::string dict_name = this->declare_parameter<std::string>("dictionary", "DICT_5X5_50");

    RCLCPP_INFO(get_logger(), "ArucoPoseNode initialized with marker_length=%.3f m, dictionary=%s",
                marker_length_, dict_name.c_str());

    dict_   = getDictionaryByName(dict_name);
    params_ = cv::aruco::DetectorParameters::create();

    // QoS for image data
    auto sensor_qos = rclcpp::SensorDataQoS();

    // --- Subscriptions ---
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (!have_camera_) {
          // Store intrinsics
          K_ = cv::Mat(3, 3, CV_64F);
          for (int i = 0; i < 9; ++i)
            K_.at<double>(i / 3, i % 3) = msg->k[i];

          D_ = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
          for (size_t i = 0; i < msg->d.size(); ++i)
            D_.at<double>(static_cast<int>(i), 0) = msg->d[i];

          have_camera_ = true;
          RCLCPP_INFO(get_logger(), "Camera calibration received (K,D). Frame: %s",
                      msg->header.frame_id.c_str());
        }
      });

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", sensor_qos,
      std::bind(&ArucoPoseNode::imageCb, this, std::placeholders::_1));

    // --- Publishers ---
    if (debug_image_)
      debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("aruco/debug_image", 10);

    RCLCPP_INFO(get_logger(), "ArucoPoseNode ready.");
  }

private:
  // --- Helpers ---
  static cv::Ptr<cv::aruco::Dictionary> getDictionaryByName(const std::string &name) {
    using namespace cv::aruco;
    static const std::unordered_map<std::string, PREDEFINED_DICTIONARY_NAME> lut = {
      {"DICT_4X4_50", DICT_4X4_50}, {"DICT_4X4_100", DICT_4X4_100},
      {"DICT_5X5_50", DICT_5X5_50}, {"DICT_5X5_100", DICT_5X5_100},
      {"DICT_6X6_250", DICT_6X6_250}, {"DICT_7X7_1000", DICT_7X7_1000},
      {"DICT_ARUCO_ORIGINAL", DICT_ARUCO_ORIGINAL},
      {"DICT_APRILTAG_36h11", DICT_APRILTAG_36h11}
    };
    auto it = lut.find(name);
    return cv::aruco::getPredefinedDictionary(it == lut.end() ? DICT_5X5_50 : it->second);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr getPosePublisherForId(int id) {
    auto it = pose_pubs_.find(id);
    if (it != pose_pubs_.end()) return it->second;
    std::string topic = pose_topic_base_ + "/id_" + std::to_string(id);
    auto pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    pose_pubs_.emplace(id, pub);
    RCLCPP_INFO(get_logger(), "Advertising pose topic: %s", topic.c_str());
    return pub;
  }

  // --- Image callback ---
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!have_camera_ || K_.empty() || D_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Waiting for valid camera calibration...");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    // Detect markers
    cv::aruco::detectMarkers(frame, dict_, corners, ids, params_, rejected);
    if (ids.empty()) {
      if (debug_image_) {
        auto out = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        debug_img_pub_->publish(*out);
      }
      return;
    }

    // Draw markers
    cv::aruco::drawDetectedMarkers(frame, corners, ids);

    // Estimate poses (uses solvePnP internally)
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, K_, D_, rvecs, tvecs);

    for (size_t i = 0; i < ids.size(); ++i) {
      cv::Mat R_cv;
      cv::Rodrigues(rvecs[i], R_cv);

      tf2::Matrix3x3 R_tf(
        R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
        R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
        R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2));
      tf2::Quaternion q; R_tf.getRotation(q);

      // Pose message
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = msg->header.stamp;
      pose_msg.header.frame_id = camera_frame_;
      pose_msg.pose.position.x = tvecs[i][0];
      pose_msg.pose.position.y = tvecs[i][1];
      pose_msg.pose.position.z = tvecs[i][2];
      pose_msg.pose.orientation = tf2::toMsg(q);

      // Publish pose
      getPosePublisherForId(ids[i])->publish(pose_msg);
      
      // ---- NEW SECTION: compute and print distance ----
      double distance = cv::norm(tvecs[i]);  // Euclidean distance in meters
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Marker ID %d | X: %.3f m | Y: %.3f m | Z: %.3f m | Distance: %.3f m",
      ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2], distance);

      // Publish TF
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

      // Draw coordinate axes
      try {
        cv::drawFrameAxes(frame, K_, D_, rvecs[i], tvecs[i], marker_length_ * 0.5f);
      } catch (...) {
        RCLCPP_DEBUG(get_logger(), "drawFrameAxes unavailable on this OpenCV build.");
      }
    }

    // Publish debug overlay
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

