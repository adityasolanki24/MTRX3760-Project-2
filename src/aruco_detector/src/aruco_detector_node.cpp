#include "aruco_detector/aruco_detector_node.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>


using std::placeholders::_1;

namespace aruco_detector{

ArucoDetectorNode::ArucoDetectorNode() : Node("aruco_detector") {
  // Parameters
  dict_name_      = declare_parameter<std::string>("dictionary", "DICT_5X5_50");
  marker_length_  = declare_parameter<double>("marker_length_m", 0.05);
  publish_tf_     = declare_parameter<bool>("publish_tf", true);
  frame_prefix_   = declare_parameter<std::string>("frame_prefix", "aruco_");
  camera_frame_   = declare_parameter<std::string>("camera_frame", "camera_rgb_optical_frame");
  show_rejected_  = declare_parameter<bool>("show_rejected", false);
  refine_corners_ = declare_parameter<int>("corner_refine_method", 1);
  img_transport_  = declare_parameter<std::string>("image_transport", "raw");
  skip_n_frames_  = declare_parameter<int>("skip_n_frames", 0);
  publish_markers_= declare_parameter<bool>("publish_marker_array", true);
  
  dp_.adaptiveThreshWinSizeMin = declare_parameter<int>("dp.adaptiveThreshWinSizeMin", 5);
  dp_.adaptiveThreshWinSizeMax = declare_parameter<int>("dp.adaptiveThreshWinSizeMax", 29);
  dp_.adaptiveThreshWinSizeStep= declare_parameter<int>("dp.adaptiveThreshWinSizeStep", 4);
  dp_.minMarkerPerimeterRate   = declare_parameter<double>("dp.minMarkerPerimeterRate", 0.02);
  dp_.maxMarkerPerimeterRate   = declare_parameter<double>("dp.maxMarkerPerimeterRate", 4.0);
  dp_.polygonalApproxAccuracyRate = declare_parameter<double>("dp.polygonalApproxAccuracyRate", 0.035);
  dp_.minCornerDistanceRate    = declare_parameter<double>("dp.minCornerDistanceRate", 0.05);
  dp_.minMarkerDistanceRate    = declare_parameter<double>("dp.minMarkerDistanceRate", 0.05);
  dp_.minDistanceToBorder      = declare_parameter<int>("dp.minDistanceToBorder", 3);
  dp_.markerBorderBits         = declare_parameter<int>("dp.markerBorderBits", 1);
  dp_.perspectiveRemoveIgnoredMarginPerCell = declare_parameter<double>("dp.perspectiveRemoveIgnoredMarginPerCell", 0.15);
  dp_.perspectiveRemovePixelPerCell = declare_parameter<int>("dp.perspectiveRemovePixelPerCell", 10);
  dp_.minOtsuStdDev            = declare_parameter<double>("dp.minOtsuStdDev", 5.0);
  dp_.errorCorrectionRate      = declare_parameter<double>("dp.errorCorrectionRate", 0.6);
  dp_.cornerRefinementMethod   = refine_corners_;


  // Legacy objects (OpenCV 4.6)
  dp_ptr_  = cv::aruco::DetectorParameters::create();
  *dp_ptr_ = dp_; // copy tuned values
  // Wrap dictionary as Ptr for legacy API
  dict_ptr_ = cv::makePtr<cv::aruco::Dictionary>( getDictionary(dict_name_) );

  //subscriptions
  image_sub_ = image_transport::create_subscription(
    this, "/camera/image_raw", std::bind(&ArucoDetectorNode::imageCb, this, _1), img_transport_);

  auto qos = rclcpp::SensorDataQoS();
  caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", qos, std::bind(&ArucoDetectorNode::cameraInfoCb, this, _1));
    
 //publisher 
  pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseArray>("/aruco/poses", 10);
  debug_pub_ = image_transport::create_publisher(this, "/aruco/debug_image");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(get_logger(), "Aruco detector ready. dict=%s, L=%.3f m, transport=%s",
              dict_name_.c_str(), marker_length_, img_transport_.c_str());
}

cv::aruco::Dictionary ArucoDetectorNode::getDictionary(const std::string &name) {
  // OpenCV 4.6 has no cv::aruco::PredefinedDictionaryType
  static const std::map<std::string, int> kMap = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
  };
  auto it = kMap.find(name);
  int dict_id = (it == kMap.end()) ? cv::aruco::DICT_5X5_50 : it->second;
  return cv::aruco::getPredefinedDictionary(dict_id);
}

void ArucoDetectorNode::cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  cam_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
  dist_coeffs_ = cv::Mat((int)msg->d.size(), 1, CV_64F);
  for (int i = 0; i < (int)msg->d.size(); ++i) dist_coeffs_.at<double>(i,0) = msg->d[i];
  if (!msg->header.frame_id.empty()) camera_frame_ = msg->header.frame_id;
  have_cam_info_ = true;
}

void ArucoDetectorNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  if (skip_n_frames_ > 0 && (++frame_count_ % (skip_n_frames_+1)) != 1) 
    return;

  cv_bridge::CvImageConstPtr cv_ptr;
  try { cv_ptr = cv_bridge::toCvShare(msg, "bgr8"); 
  }
  catch (const cv_bridge::Exception &e) { 
    RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what()); 
    return; 
}

  const cv::Mat &image = cv_ptr->image;

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  
  cv::aruco::detectMarkers(image, dict_ptr_, corners, ids, dp_ptr_, rejected);

  cv::Mat dbg = image.clone();

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = msg->header;
  if (!ids.empty()) {
    cv::aruco::drawDetectedMarkers(dbg, corners, ids);

    if (have_cam_info_ && marker_length_ > 0.0) {
      cv::Mat objPoints(4, 1, CV_32FC3);
      objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_length_/2.f,  marker_length_/2.f, 0);
      objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f( marker_length_/2.f,  marker_length_/2.f, 0);
      objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f( marker_length_/2.f, -marker_length_/2.f, 0);
      objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_length_/2.f, -marker_length_/2.f, 0);
      std::vector<cv::Vec3d> rvecs(ids.size()), tvecs(ids.size());
      for (size_t i = 0; i < ids.size(); ++i) {
        cv::solvePnP(objPoints, corners[i], cam_matrix_, dist_coeffs_, rvecs[i], tvecs[i]);
        cv::drawFrameAxes(dbg, cam_matrix_, dist_coeffs_, rvecs[i], tvecs[i], (float)(marker_length_ * 0.5f), 2);
        cv::Mat R; cv::Rodrigues(rvecs[i], R);
        tf2::Matrix3x3 tfR(
          R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
        tf2::Quaternion q; tfR.getRotation(q);
        geometry_msgs::msg::Pose p;
        p.position.x = tvecs[i][0];
        p.position.y = tvecs[i][1];
        p.position.z = tvecs[i][2];
        p.orientation = tf2::toMsg(q);
        pose_array.poses.push_back(p);
        if (publish_tf_) {
          geometry_msgs::msg::TransformStamped tfm;
          tfm.header = msg->header;
          tfm.header.frame_id = camera_frame_;
          tfm.child_frame_id = frame_prefix_ + std::to_string(ids[i]);
          tfm.transform.translation.x = p.position.x;
          tfm.transform.translation.y = p.position.y;
          tfm.transform.translation.z = p.position.z;
          tfm.transform.rotation = p.orientation;
          tf_broadcaster_->sendTransform(tfm);
        }
      }
    }
  }
  if (show_rejected_ && !rejected.empty()) {
    cv::aruco::drawDetectedMarkers(dbg, rejected, cv::noArray(), cv::Scalar(100,0,255));
  }
  pose_pub_->publish(pose_array);
  auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, dbg).toImageMsg();
  debug_pub_.publish(out_msg);
}

} //namespace


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aruco_detector::ArucoDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
