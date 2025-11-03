// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: delivery_robot.cpp
// Author(s): Aditya Solanki
//
// Implementation of DeliveryRobot class - Navigation2-based delivery robot.

#include "mode_selector/delivery_robot.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ctime>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

namespace mode_selector
{

DeliveryRobot::DeliveryRobot(bool mapping_mode)
    : BaseRobot("delivery_robot"),
      current_target_tag_(-1),
      current_status_("IDLE"),
      completed_deliveries_(0),
      current_route_index_(0),
      current_waypoint_index_(0),
      home_position_(0.0, 0.0, 0.0, "Home"),
      home_position_set_(false),
      delivery_in_progress_(false),
      delivery_log_file_("/tmp/delivery_robot_log.txt"),
      mapping_mode_enabled_(mapping_mode),
      map_saved_(false),
      saved_map_path_(""),
      nav2_ready_(false),
      mapping_start_time_(this->now())
{
    RCLCPP_INFO(this->get_logger(), "DeliveryRobot created");
    if (mapping_mode_enabled_) {
        RCLCPP_INFO(this->get_logger(), "Mapping mode enabled - will save map automatically");
        RCLCPP_INFO(this->get_logger(), "Navigation2 will be connected after map is saved");
    }
}

DeliveryRobot::~DeliveryRobot()
{
    if (delivery_log_stream_.is_open()) {
        delivery_log_stream_.close();
    }
    stopRobot();
    RCLCPP_INFO(this->get_logger(), "DeliveryRobot shutting down");
}

void DeliveryRobot::initialize()
{
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize Navigation2 action client
    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
    // Publishers
    target_tag_pub_ = this->create_publisher<std_msgs::msg::Int32>(
        "target_aruco_tag", 10);
    delivery_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "delivery_status", 10);
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10);
    
    // Subscribers
    nav_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "navigation_status", 10,
        std::bind(&DeliveryRobot::navigationStatusCallback, this, std::placeholders::_1));
    
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&DeliveryRobot::goalPoseCallback, this, std::placeholders::_1));
    
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", 10,
        std::bind(&DeliveryRobot::clickedPointCallback, this, std::placeholders::_1));
    
    // State management timer
    state_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&DeliveryRobot::stateManagementCallback, this));
    
    // Initialize delivery routes
    initializeRoutes();
    
    // Initialize delivery log (only if not in mapping mode)
    if (!mapping_mode_enabled_) {
        initializeDeliveryLog();
    }
    
    // Setup mapping completion check if in mapping mode
    if (mapping_mode_enabled_) {
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "MAPPING MODE: Enabled");
        RCLCPP_INFO(this->get_logger(), "  - Wall follower will explore the maze autonomously");
        RCLCPP_INFO(this->get_logger(), "  - SLAM is building the map");
        RCLCPP_INFO(this->get_logger(), "  - Map will be saved automatically after 3 minutes");
        RCLCPP_INFO(this->get_logger(), "  - Delivery functions disabled during mapping");
        RCLCPP_INFO(this->get_logger(), "  - Navigation2 will connect after map is saved");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        
        // Check mapping completion every 30 seconds
        mapping_completion_timer_ = this->create_wall_timer(
            30s,
            std::bind(&DeliveryRobot::checkMappingCompletion, this));
        
        // In mapping mode, DON'T wait for Navigation2 yet - wait until map is saved
    } else {
        // Normal delivery mode: initialize log and wait for Navigation2 action server immediately
        initializeDeliveryLog();
        RCLCPP_INFO(this->get_logger(), "Waiting for Navigation2 action server...");
        while (!nav_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Navigation2 action server...");
        }
        
        RCLCPP_INFO(this->get_logger(), "DeliveryRobot initialized");
        RCLCPP_INFO(this->get_logger(), "Ready to receive delivery requests");
    }
}

void DeliveryRobot::run()
{
    // Main loop runs via ROS 2 callbacks
    if (mapping_mode_enabled_) {
        // Mapping mode: wall follower is running separately
        // Robot explores automatically, map saves after 3 minutes
        RCLCPP_INFO(this->get_logger(), "Mapping mode active - wall follower is exploring maze");
        RCLCPP_INFO(this->get_logger(), "No delivery commands will be processed until map is saved and Nav2 is ready");
    } else {
        // Normal delivery mode: operates through:
        // - Navigation2 action client for route execution
        // - State management timer for processing delivery queue
        // - RViz2 goal callbacks for interactive navigation
    }
}

void DeliveryRobot::queueDeliveryRequest(int destination_id)
{
    // In mapping mode, don't queue deliveries
    if (mapping_mode_enabled_) {
        RCLCPP_WARN(this->get_logger(), 
            "Delivery request ignored - robot is in mapping mode. "
            "Wait for map to be saved before sending delivery requests.");
        return;
    }
    
    std::lock_guard<std::mutex> lock(delivery_mutex_);
    pending_deliveries_.push(destination_id);
    RCLCPP_INFO(this->get_logger(), "Queued delivery request for destination %d", destination_id);
    processNextDelivery();
}

void DeliveryRobot::navigateToPose(double x, double y, double yaw)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    
    sendNavigationGoal(pose);
}

// ========== Private Methods ==========

void DeliveryRobot::initializeRoutes()
{
    // Delivery routes can be configured here
    // For now, routes are handled via RViz2 goals or ArUco tag navigation
    RCLCPP_INFO(this->get_logger(), "Delivery routes initialized (use RViz2 for navigation)");
}

void DeliveryRobot::initializeDeliveryLog()
{
    delivery_log_stream_.open(delivery_log_file_, std::ios::out);
    if (delivery_log_stream_.is_open()) {
        delivery_log_stream_ << "==========================================\n";
        delivery_log_stream_ << "DELIVERY ROBOT - LOG\n";
        delivery_log_stream_ << "==========================================\n";
        delivery_log_stream_ << "System: Navigation2 + SLAM (No Camera)\n";
        delivery_log_stream_ << "Log Started: " << getCurrentTimestamp() << "\n";
        delivery_log_stream_ << "==========================================\n\n";
        delivery_log_stream_.flush();
        RCLCPP_INFO(this->get_logger(), "Delivery log initialized: %s", delivery_log_file_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create delivery log file!");
    }
}

void DeliveryRobot::logDelivery(int tag_id, const std::string& event, const std::string& details)
{
    if (!delivery_log_stream_.is_open()) return;
    
    delivery_log_stream_ << "Delivery Event:\n";
    delivery_log_stream_ << "  Destination: " << tag_id << "\n";
    delivery_log_stream_ << "  Event: " << event << "\n";
    delivery_log_stream_ << "  Details: " << details << "\n";
    delivery_log_stream_ << "  Timestamp: " << getCurrentTimestamp() << "\n";
    delivery_log_stream_ << "  Robot Position: (" << robot_x_ << ", " << robot_y_ << ")\n";
    delivery_log_stream_ << "  Completed Deliveries: " << completed_deliveries_ << "\n";
    delivery_log_stream_ << "------------------------------------------\n\n";
    delivery_log_stream_.flush();
    
    RCLCPP_INFO(this->get_logger(), "Delivery logged: Tag %d, Event: %s", tag_id, event.c_str());
}

std::string DeliveryRobot::getCurrentTimestamp() const
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

void DeliveryRobot::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!home_position_set_) {
        RCLCPP_WARN(this->get_logger(), "Home position not set. Please set initial pose in RViz2 first.");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received goal pose from RViz2: (%.2f, %.2f)",
                msg->pose.position.x, msg->pose.position.y);
    
    geometry_msgs::msg::PoseStamped pose = *msg;
    if (pose.header.frame_id != "map") {
        pose.header.frame_id = "map";
    }
    pose.header.stamp = this->now();
    
    sendNavigationGoal(pose);
}

void DeliveryRobot::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (!home_position_set_) {
        RCLCPP_WARN(this->get_logger(), "Home position not set. Please set initial pose in RViz2 first.");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received clicked point: (%.2f, %.2f, %.2f)",
                msg->point.x, msg->point.y, msg->point.z);
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    if (pose.header.frame_id != "map") {
        pose.header.frame_id = "map";
    }
    pose.header.stamp = this->now();
    pose.pose.position = msg->point;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(q);
    
    sendNavigationGoal(pose);
}

void DeliveryRobot::sendNavigationGoal(const geometry_msgs::msg::PoseStamped& pose)
{
    // In mapping mode, don't send goals until map is saved and Nav2 is ready
    if (mapping_mode_enabled_ && !nav2_ready_) {
        RCLCPP_WARN(this->get_logger(), 
            "Navigation2 not ready yet (mapping mode - waiting for map to be saved). "
            "Robot is currently exploring with wall follower.");
        return;
    }
    
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Navigation2 action server not available");
        return;
    }
    
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&DeliveryRobot::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&DeliveryRobot::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&DeliveryRobot::resultCallback, this, std::placeholders::_1);
    
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
    
    delivery_in_progress_ = true;
    current_status_ = "NAVIGATING";
    
    RCLCPP_INFO(this->get_logger(), "Sending navigation goal to (%.2f, %.2f)",
                pose.pose.position.x, pose.pose.position.y);
}

void DeliveryRobot::goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Navigation goal rejected");
        delivery_in_progress_ = false;
        current_status_ = "IDLE";
    } else {
        RCLCPP_INFO(this->get_logger(), "Navigation goal accepted");
    }
}

void DeliveryRobot::feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    if (feedback) {
        RCLCPP_DEBUG(this->get_logger(), "Navigation progress: %.2f m remaining",
                    feedback->distance_remaining);
    }
}

void DeliveryRobot::resultCallback(const GoalHandleNav::WrappedResult& result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded!");
            completed_deliveries_++;
            logDelivery(current_target_tag_, "COMPLETED", "Successfully reached destination");
            delivery_in_progress_ = false;
            current_status_ = "IDLE";
            current_target_tag_ = -1;
            processNextDelivery();
            break;
            
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Navigation goal aborted");
            logDelivery(current_target_tag_, "FAILED", "Navigation aborted");
            delivery_in_progress_ = false;
            current_status_ = "IDLE";
            processNextDelivery();
            break;
            
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation goal canceled");
            delivery_in_progress_ = false;
            current_status_ = "IDLE";
            break;
            
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown navigation result");
            delivery_in_progress_ = false;
            current_status_ = "IDLE";
            break;
    }
}

void DeliveryRobot::navigationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    current_status_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Navigation status: %s", current_status_.c_str());
}

void DeliveryRobot::stateManagementCallback()
{
    // Set home position once from odometry when localized
    if (!home_position_set_) {
        double map_x, map_y, map_yaw;
        if (getRobotPoseInMap(map_x, map_y, map_yaw)) {
            home_position_.x = map_x;
            home_position_.y = map_y;
            home_position_.yaw = map_yaw;
            home_position_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Home position set: (%.2f, %.2f, %.2f)",
                       map_x, map_y, map_yaw);
            logDelivery(0, "SYSTEM", "Home position initialized");
        }
    }
    
    // Process next delivery if idle
    if (!delivery_in_progress_) {
        processNextDelivery();
    }
}

void DeliveryRobot::processNextDelivery()
{
    std::lock_guard<std::mutex> lock(delivery_mutex_);
    
    if (pending_deliveries_.empty() || delivery_in_progress_) {
        return;
    }
    
    if (!home_position_set_) {
        RCLCPP_DEBUG(this->get_logger(), "Waiting for home position to be set...");
        return;
    }
    
    int next_id = pending_deliveries_.front();
    pending_deliveries_.pop();
    
    current_target_tag_ = next_id;
    delivery_in_progress_ = true;
    
    // Publish target tag
    std_msgs::msg::Int32 tag_msg;
    tag_msg.data = next_id;
    target_tag_pub_->publish(tag_msg);
    
    logDelivery(next_id, "STARTED", "Delivery request queued");
    
    RCLCPP_INFO(this->get_logger(), "Processing delivery to destination %d", next_id);
    RCLCPP_INFO(this->get_logger(), "Use RViz2 '2D Goal Pose' or 'Publish Point' to navigate");
}

bool DeliveryRobot::getRobotPoseInMap(double& x, double& y, double& yaw)
{
    try {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(
            "map", "base_link",
            rclcpp::Time(0),
            std::chrono::milliseconds(500));
        
        x = transform.transform.translation.x;
        y = transform.transform.translation.y;
        
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        return true;
    } catch (const tf2::TransformException& ex) {
        return false;
    }
}

geometry_msgs::msg::PoseStamped DeliveryRobot::waypointToPoseStamped(const Waypoint& waypoint)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, waypoint.yaw);
    pose.pose.orientation = tf2::toMsg(q);
    
    return pose;
}

// ========== Mapping Mode ==========

void DeliveryRobot::checkMappingCompletion()
{
    if (!mapping_mode_enabled_ || map_saved_) {
        return;
    }
    
    // Check elapsed time (map after 3 minutes of exploration)
    auto elapsed = this->now() - mapping_start_time_;
    auto elapsed_seconds = elapsed.seconds();
    
    // Save map after 3 minutes (180 seconds) of exploration
    // You can adjust this time or add more sophisticated completion detection
    if (elapsed_seconds >= 180.0) {
        RCLCPP_WARN(this->get_logger(), "============================================================");
        RCLCPP_WARN(this->get_logger(), "Mapping exploration time reached (%.0f seconds)", elapsed_seconds);
        RCLCPP_WARN(this->get_logger(), "Saving map automatically...");
        RCLCPP_WARN(this->get_logger(), "============================================================");
        
        // Give a brief moment for any last updates
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        saveMap();
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60,
            "Mapping in progress: %.0f seconds elapsed (will save after 180 seconds). "
            "Wall follower is exploring the maze...", elapsed_seconds);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10,
            "Make sure SLAM Toolbox is running and robot is moving to build the map");
    }
}

void DeliveryRobot::saveMap()
{
    if (map_saved_) {
        RCLCPP_WARN(this->get_logger(), "Map already saved, skipping...");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Executing map save command...");
    
    // Generate map filename with timestamp
    auto now = std::time(nullptr);
    char time_str[80];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&now));
    std::string map_file = std::string(getenv("HOME")) + "/my_map_" + time_str;
    
    // Save map using nav2_map_server
    // Subscribe to /map topic and save to file
    std::string save_cmd = "ros2 run nav2_map_server map_saver_cli -f " + map_file;
    
    RCLCPP_INFO(this->get_logger(), "Saving map with command: %s", save_cmd.c_str());
    
    // Execute map save (this will block, but it's okay in a timer)
    int result = system(save_cmd.c_str());
    
    if (result == 0) {
        map_saved_ = true;
        saved_map_path_ = map_file;  // Store the map path
        RCLCPP_WARN(this->get_logger(), "============================================================");
        RCLCPP_WARN(this->get_logger(), "MAP SAVED SUCCESSFULLY: %s", map_file.c_str());
        RCLCPP_WARN(this->get_logger(), "Map files: %s.pgm and %s.yaml", map_file.c_str(), map_file.c_str());
        RCLCPP_WARN(this->get_logger(), "============================================================");
        
        // Cancel the mapping timer since we're done
        if (mapping_completion_timer_) {
            mapping_completion_timer_->cancel();
            mapping_completion_timer_.reset();
        }
        
        // Stop wall follower - mapping is complete
        RCLCPP_INFO(this->get_logger(), "Stopping wall follower - mapping complete");
        system("pkill -f wall_follower");
        
        RCLCPP_WARN(this->get_logger(), "============================================================");
        RCLCPP_WARN(this->get_logger(), "MAPPING COMPLETE!");
        RCLCPP_WARN(this->get_logger(), "Map saved: %s", map_file.c_str());
        RCLCPP_WARN(this->get_logger(), "Wall follower stopped.");
        RCLCPP_WARN(this->get_logger(), "Please choose next action from the menu.");
        RCLCPP_WARN(this->get_logger(), "============================================================");
        
        // Do NOT auto-connect to Navigation2 - wait for user choice
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save map! Command: %s", save_cmd.c_str());
        RCLCPP_ERROR(this->get_logger(), "Return code: %d", result);
        RCLCPP_ERROR(this->get_logger(), "Make sure nav2_map_server is installed and SLAM is running");
        RCLCPP_ERROR(this->get_logger(), "Check if /map topic exists: ros2 topic list | grep map");
        RCLCPP_ERROR(this->get_logger(), "Check if /map has data: ros2 topic echo /map --once");
    }
}

void DeliveryRobot::enableDeliveryMode()
{
    if (!map_saved_) {
        RCLCPP_WARN(this->get_logger(), "Cannot enable delivery mode - map not saved yet");
        return;
    }
    
    // Initialize delivery log if not already done
    if (!delivery_log_stream_.is_open()) {
        initializeDeliveryLog();
    }
    
    // Set initial pose for AMCL using last known position
    // This is critical - AMCL needs an initial pose estimate to start localization
    setInitialPoseFromOdometry();
    
    // Start connecting to Navigation2
    connectToNavigation2();
}

void DeliveryRobot::setInitialPoseFromOdometry()
{
    // Try to get robot pose in map frame from TF first (if SLAM transform still exists)
    double map_x, map_y, map_yaw;
    bool got_map_pose = getRobotPoseInMap(map_x, map_y, map_yaw);
    
    // If TF transform doesn't exist yet, use odometry position as estimate
    // This is okay because the robot should be roughly where it was when mapping stopped
    if (!got_map_pose) {
        RCLCPP_WARN(this->get_logger(), 
            "Could not get robot pose in map frame from TF. Using odometry as initial estimate.");
        map_x = robot_x_;
        map_y = robot_y_;
        map_yaw = robot_yaw_;
    }
    
    // Create initial pose message for AMCL
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->now();
    
    // Set position
    initial_pose.pose.pose.position.x = map_x;
    initial_pose.pose.pose.position.y = map_y;
    initial_pose.pose.pose.position.z = 0.0;
    
    // Set orientation (quaternion from yaw)
    tf2::Quaternion q;
    q.setRPY(0, 0, map_yaw);
    initial_pose.pose.pose.orientation.x = q.x();
    initial_pose.pose.pose.orientation.y = q.y();
    initial_pose.pose.pose.orientation.z = q.z();
    initial_pose.pose.pose.orientation.w = q.w();
    
    // Set covariance (position uncertainty)
    // Diagonal covariance matrix - we're reasonably confident about position
    // x, y: 0.25 m^2 uncertainty (0.5m std dev)
    // z: small uncertainty
    // orientation: 0.0685 rad^2 (~15 degrees std dev)
    initial_pose.pose.covariance[0] = 0.25;   // x
    initial_pose.pose.covariance[7] = 0.25;   // y
    initial_pose.pose.covariance[35] = 0.0685; // yaw
    
    // Publish initial pose
    initial_pose_pub_->publish(initial_pose);
    
    RCLCPP_WARN(this->get_logger(), "============================================================");
    RCLCPP_WARN(this->get_logger(), "INITIAL POSE SET FOR AMCL");
    RCLCPP_WARN(this->get_logger(), "  Position: (%.2f, %.2f)", map_x, map_y);
    RCLCPP_WARN(this->get_logger(), "  Orientation: %.2f rad (%.1f deg)", map_yaw, map_yaw * 180.0 / M_PI);
    RCLCPP_WARN(this->get_logger(), "  AMCL should now localize the robot on the map");
    RCLCPP_WARN(this->get_logger(), "============================================================");
    
    // Wait a moment for AMCL to process the initial pose
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void DeliveryRobot::connectToNavigation2()
{
    if (nav2_ready_) {
        return; // Already connected
    }
    
    RCLCPP_INFO(this->get_logger(), "Attempting to connect to Navigation2 action server...");
    
    // Check if Navigation2 is available (non-blocking)
    if (nav_action_client_->wait_for_action_server(std::chrono::seconds(0))) {
        nav2_ready_ = true;
        RCLCPP_WARN(this->get_logger(), "============================================================");
        RCLCPP_WARN(this->get_logger(), "Navigation2 ACTION SERVER CONNECTED!");
        RCLCPP_WARN(this->get_logger(), "DeliveryRobot is now ready for navigation");
        RCLCPP_WARN(this->get_logger(), "You can now send navigation goals via RViz2 or commands");
        RCLCPP_WARN(this->get_logger(), "============================================================");
        
        // Cancel the connection timer
        if (nav2_connect_timer_) {
            nav2_connect_timer_->cancel();
            nav2_connect_timer_.reset();
        }
    } else {
        // Set up timer to keep checking for Navigation2
        if (!nav2_connect_timer_) {
            nav2_connect_timer_ = this->create_wall_timer(
                1s,
                std::bind(&DeliveryRobot::connectToNavigation2, this));
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Waiting for Navigation2 action server... (make sure Navigation2 is launched)");
    }
}

}  // namespace mode_selector



