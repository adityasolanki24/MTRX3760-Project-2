// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: delivery_navigator.cpp
// Author(s): Project Team
//
// Delivery navigation implementation using Navigation2 with SLAM.
// Navigates predefined routes on SLAM map using LiDAR + Odometry (no camera required).
// Listens for delivery requests and executes them sequentially.

#include "delivery_navigator/delivery_navigator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

namespace delivery_navigator
{

DeliveryNavigator::DeliveryNavigator()
    : Node("delivery_navigator"),
      current_route_index_(0),
      current_waypoint_index_(0),
      home_position_(0.0, 0.0, 0.0, "Home"),
      home_position_set_(false),
      state_(DeliveryState::IDLE),
      robot_x_(0.0),
      robot_y_(0.0),
      robot_yaw_(0.0),
      delivery_log_file_("/tmp/delivery_navigation_log.txt"),
      pending_delivery_request_(""),
      delivery_in_progress_(false)
{
    // Initialize TF buffer and listener for getting robot pose in map frame
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize Navigation2 action client
    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
    // Publishers
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "navigation_status", 10);
    
    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&DeliveryNavigator::odomCallback, this, std::placeholders::_1));
    
    // Subscribe to delivery requests from commander
    delivery_request_sub_ = this->create_subscription<std_msgs::msg::String>(
        "delivery_request", 10,
        std::bind(&DeliveryNavigator::deliveryRequestCallback, this, 
                 std::placeholders::_1));
    
    // Subscribe to goal poses from RViz2 (2D Goal Pose tool)
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&DeliveryNavigator::goalPoseCallback, this, 
                 std::placeholders::_1));
    
    // Subscribe to clicked points from RViz2 (Publish Point tool)
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", 10,
        std::bind(&DeliveryNavigator::clickedPointCallback, this, 
                 std::placeholders::_1));
    
    // Initialize delivery routes
    initializeRoutes();
    
    // Wait for action server
    while (!nav_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Navigation2 action server...");
    }
    
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "Delivery Navigator - Navigation2 + SLAM Based");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "System: SLAM Map + LiDAR Localization (NO CAMERA)");
    RCLCPP_INFO(this->get_logger(), "Odometry: Uses robot odometry for precise localization");
    RCLCPP_INFO(this->get_logger(), "Initialized %zu delivery routes:", delivery_routes_.size());
    
    for (size_t i = 0; i < delivery_routes_.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "  Route %zu: %s (%zu waypoints)",
                    i+1, 
                    delivery_routes_[i].route_name.c_str(),
                    delivery_routes_[i].waypoints.size());
    }
    
    RCLCPP_INFO(this->get_logger(), "Waiting for localization on SLAM map...");
    RCLCPP_INFO(this->get_logger(), "Ready to receive delivery requests on 'delivery_request' topic");
    RCLCPP_INFO(this->get_logger(), "Ready to receive goals from RViz2:");
    RCLCPP_INFO(this->get_logger(), "  - Use '2D Goal Pose' tool -> publishes to 'goal_pose'");
    RCLCPP_INFO(this->get_logger(), "  - Use 'Publish Point' tool -> publishes to 'clicked_point'");
    RCLCPP_INFO(this->get_logger(), "============================================================\n");
    
    // Initialize delivery log
    initializeDeliveryLog();
    
    // Timer for state management
    auto timer_callback = [this]() {
        this->stateManagementCallback();
    };
    state_timer_ = this->create_wall_timer(100ms, timer_callback);
}

DeliveryNavigator::~DeliveryNavigator()
{
    logDelivery("SYSTEM", "SHUTDOWN", "Navigation system shutdown");
    RCLCPP_INFO(this->get_logger(), "Delivery Navigator Shutdown");
}

void DeliveryNavigator::initializeDeliveryLog()
{
    std::ofstream log_file(delivery_log_file_, std::ios::out);
    if (log_file.is_open()) {
        log_file << "==========================================\n";
        log_file << "DELIVERY NAVIGATOR - LOG\n";
        log_file << "==========================================\n";
        log_file << "System: SLAM Map + LiDAR Navigation\n";
        log_file << "No Camera Required\n";
        log_file << "Log Started: " << getCurrentTimestamp() << "\n";
        log_file << "==========================================\n\n";
        log_file.close();
        RCLCPP_INFO(this->get_logger(), "Delivery log initialized: %s", 
                   delivery_log_file_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create delivery log file!");
    }
}

void DeliveryNavigator::initializeRoutes()
{
    // NOTE: Predefined routes are disabled by default
    // Use RViz2 "Publish Point" or "2D Goal Pose" to navigate instead
    // 
    // To enable predefined routes, uncomment and modify the coordinates below:
    // 1. Use RViz2 "Publish Point" to find coordinates on your map
    // 2. Replace the hardcoded coordinates below with your map coordinates
    
    // Route 1: Warehouse A delivery route (DISABLED - coordinates likely don't match your map)
    // DeliveryRoute route1("Warehouse_A");
    // route1.waypoints.push_back(Waypoint(1.0, 0.5, 0.0, "Shelf_A1"));
    // route1.waypoints.push_back(Waypoint(2.5, 0.5, 0.0, "Shelf_A2"));
    // route1.waypoints.push_back(Waypoint(4.0, 0.5, 0.0, "Shelf_A3"));
    // delivery_routes_.push_back(route1);
    
    // Route 2: Warehouse B delivery route (DISABLED)
    // DeliveryRoute route2("Warehouse_B");
    // route2.waypoints.push_back(Waypoint(1.0, 2.0, 0.0, "Shelf_B1"));
    // route2.waypoints.push_back(Waypoint(2.5, 2.0, 0.0, "Shelf_B2"));
    // route2.waypoints.push_back(Waypoint(4.0, 2.0, 0.0, "Shelf_B3"));
    // delivery_routes_.push_back(route2);
    
    // Route 3: Quick delivery route (DISABLED)
    // DeliveryRoute route3("Quick_Route");
    // route3.waypoints.push_back(Waypoint(0.5, 1.0, 0.0, "Quick_Pickup"));
    // route3.waypoints.push_back(Waypoint(0.5, 3.0, 0.0, "Quick_Delivery"));
    // delivery_routes_.push_back(route3);
    
    RCLCPP_INFO(this->get_logger(), "No predefined routes configured.");
    RCLCPP_INFO(this->get_logger(), "Use RViz2 'Publish Point' or '2D Goal Pose' to navigate to points.");
}

void DeliveryNavigator::deliveryRequestCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string request = msg->data;
    
    // Parse request format: "DELIVER:route_name:waypoint_id" or "DELIVER:route_name" or "HOME"
    RCLCPP_INFO(this->get_logger(), "Received delivery request: %s", request.c_str());
    
    if (request == "HOME" || request == "RETURN_HOME") {
        pending_delivery_request_ = "HOME";
        RCLCPP_INFO(this->get_logger(), "Queued return home command");
    } else if (request.find("DELIVER:") == 0) {
        // Format: DELIVER:route_name or DELIVER:route_name:waypoint_id
        pending_delivery_request_ = request;
        RCLCPP_INFO(this->get_logger(), "Queued delivery request: %s", request.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid request format: %s", request.c_str());
    }
}

void DeliveryNavigator::stateManagementCallback()
{
    // Only process new requests if not currently delivering and home is set
    if (!home_position_set_) {
        return;  // Still waiting for localization
    }
    
    // Process pending delivery request if system is idle
    if (!delivery_in_progress_ && !pending_delivery_request_.empty()) {
        std::string request = pending_delivery_request_;
        pending_delivery_request_ = "";
        
        if (request == "HOME" || request == "RETURN_HOME") {
            startReturnHome();
            delivery_in_progress_ = true;
        } else if (request.find("DELIVER:") == 0) {
            processDeliveryRequest(request);
            delivery_in_progress_ = true;
        }
        // Note: RViz2 goals are handled directly in callbacks, not through stateManagementCallback
    }
}

void DeliveryNavigator::processDeliveryRequest(const std::string& request)
{
    // Parse: DELIVER:route_name or DELIVER:route_name:waypoint_name
    std::vector<std::string> parts = splitString(request, ':');
    
    if (parts.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Invalid delivery request format");
        delivery_in_progress_ = false;
        return;
    }
    
    std::string route_name = parts[1];
    
    // Check if any routes are configured
    if (delivery_routes_.empty()) {
        RCLCPP_ERROR(this->get_logger(), 
                    "No delivery routes configured! Use RViz2 'Publish Point' to navigate instead.");
        RCLCPP_ERROR(this->get_logger(), 
                    "To configure routes, edit initializeRoutes() in delivery_navigator.cpp");
        delivery_in_progress_ = false;
        return;
    }
    
    // Find the route
    int route_idx = -1;
    for (size_t i = 0; i < delivery_routes_.size(); i++) {
        if (delivery_routes_[i].route_name == route_name) {
            route_idx = i;
            break;
        }
    }
    
    if (route_idx == -1) {
        RCLCPP_ERROR(this->get_logger(), "Route not found: %s", route_name.c_str());
        RCLCPP_ERROR(this->get_logger(), "Available routes: %zu", delivery_routes_.size());
        delivery_in_progress_ = false;
        return;
    }
    
    current_route_index_ = route_idx;
    current_waypoint_index_ = 0;
    state_ = DeliveryState::NAVIGATING;
    
    const auto& waypoint = delivery_routes_[route_idx].waypoints[0];
    
    RCLCPP_INFO(this->get_logger(), "Starting delivery route: %s", route_name.c_str());
    RCLCPP_INFO(this->get_logger(), "First waypoint: %s at (%.2f, %.2f)",
               waypoint.name.c_str(), waypoint.x, waypoint.y);
    
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "STARTED:" + route_name + ":" + waypoint.name;
    status_pub_->publish(status_msg);
    
    logDelivery(route_name, waypoint.name, "STARTED");
    
    sendNavigationGoal(waypoint);
}

void DeliveryNavigator::startReturnHome()
{
    state_ = DeliveryState::RETURNING_HOME;
    
    RCLCPP_INFO(this->get_logger(), "Returning to home position (%.2f, %.2f)",
               home_position_.x, home_position_.y);
    
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "RETURNING_HOME";
    status_pub_->publish(status_msg);
    
    logDelivery("SYSTEM", "HOME", "RETURNING");
    
    sendNavigationGoal(home_position_);
}

void DeliveryNavigator::sendNavigationGoal(const Waypoint& waypoint)
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypointToPoseStamped(waypoint);
    
    RCLCPP_INFO(this->get_logger(), "Sending Navigation2 goal: %s at (%.2f, %.2f)",
               waypoint.name.c_str(), waypoint.x, waypoint.y);
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&DeliveryNavigator::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&DeliveryNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&DeliveryNavigator::resultCallback, this, std::placeholders::_1);
    
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void DeliveryNavigator::sendNavigationGoal(const geometry_msgs::msg::PoseStamped& pose)
{
    auto goal_msg = NavigateToPose::Goal();
    
    // Create a fresh pose with correct frame and timestamp
    // IMPORTANT: Use this->now() not Time(0) - Navigation2 needs current timestamp
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = pose.pose.position.x;
    goal_msg.pose.pose.position.y = pose.pose.position.y;
    goal_msg.pose.pose.position.z = pose.pose.position.z;
    goal_msg.pose.pose.orientation = pose.pose.orientation;
    
    // Verify the position is valid (not NaN or infinite)
    if (!std::isfinite(goal_msg.pose.pose.position.x) || 
        !std::isfinite(goal_msg.pose.pose.position.y)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid goal position! (NaN or infinite)");
        return;
    }
    
    // Extract yaw from quaternion for logging
    tf2::Quaternion q(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Get robot position in map frame using TF (more accurate than odom)
    double map_x, map_y, map_yaw;
    bool got_pose = getRobotPoseInMap(map_x, map_y, map_yaw);
    
    if (!got_pose) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose in map frame, using odom position");
        map_x = robot_x_;
        map_y = robot_y_;
        map_yaw = robot_yaw_;
    }
    
    // Calculate distance from current robot position in map frame
    double dx = goal_msg.pose.pose.position.x - map_x;
    double dy = goal_msg.pose.pose.position.y - map_y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    RCLCPP_INFO(this->get_logger(), "Sending Navigation2 goal from RViz2:");
    RCLCPP_INFO(this->get_logger(), "  Goal position: (%.2f, %.2f, %.2f) in frame '%s'",
               goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, yaw,
               goal_msg.pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Robot position (map frame): (%.2f, %.2f, %.2f)",
               map_x, map_y, map_yaw);
    RCLCPP_INFO(this->get_logger(), "  Robot position (odom frame): (%.2f, %.2f, %.2f)",
               robot_x_, robot_y_, robot_yaw_);
    RCLCPP_INFO(this->get_logger(), "  Distance to goal: %.2f m", distance);
    
    if (distance < 0.1) {
        RCLCPP_WARN(this->get_logger(), "Goal is very close to current position (%.2f m). Navigation may succeed immediately.", distance);
    }
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&DeliveryNavigator::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&DeliveryNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&DeliveryNavigator::resultCallback, this, std::placeholders::_1);
    
    // Verify goal position one more time before sending
    RCLCPP_INFO(this->get_logger(), "Final goal before sending:");
    RCLCPP_INFO(this->get_logger(), "  Frame: '%s'", goal_msg.pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Timestamp: sec=%ld nanosec=%lu", 
               goal_msg.pose.header.stamp.sec, goal_msg.pose.header.stamp.nanosec);
    RCLCPP_INFO(this->get_logger(), "  Position: (%.3f, %.3f, %.3f)", 
               goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, goal_msg.pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  Orientation: (%.3f, %.3f, %.3f, %.3f)",
               goal_msg.pose.pose.orientation.x, goal_msg.pose.pose.orientation.y,
               goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w);
    
    // Check if goal is reasonable
    if (std::abs(goal_msg.pose.pose.position.x) > 100 || std::abs(goal_msg.pose.pose.position.y) > 100) {
        RCLCPP_ERROR(this->get_logger(), "WARNING: Goal position seems unreasonably large!");
    }
    
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
    
    // Update state
    state_ = DeliveryState::NAVIGATING;
    delivery_in_progress_ = true;
    
    // Log the goal
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "RViz2_GOAL:" + std::to_string(pose.pose.position.x) + 
                     ":" + std::to_string(pose.pose.position.y);
    status_pub_->publish(status_msg);
}

geometry_msgs::msg::PoseStamped DeliveryNavigator::waypointToPoseStamped(const Waypoint& waypoint)
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

void DeliveryNavigator::goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Navigation goal was rejected by server");
        RCLCPP_ERROR(this->get_logger(), "This usually means Navigation2 couldn't process the goal");
        RCLCPP_ERROR(this->get_logger(), "Check: 1) Navigation2 is running, 2) Map is loaded, 3) Robot is localized");
        state_ = DeliveryState::IDLE;
        delivery_in_progress_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Navigation goal accepted by server");
        
        // Log the goal that was accepted - get goal from the handle
        try {
            auto goal_id = goal_handle->get_goal_id();
            double map_x, map_y, map_yaw;
            if (getRobotPoseInMap(map_x, map_y, map_yaw)) {
                RCLCPP_INFO(this->get_logger(), 
                           "Goal accepted - Robot current position: (%.2f, %.2f)",
                           map_x, map_y);
            }
        } catch (...) {
            // Goal ID access failed, but goal was accepted so continue
        }
    }
}

void DeliveryNavigator::feedbackCallback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    // Log navigation feedback with debugging
    if (feedback) {
        // Get current robot position for debugging
        double map_x, map_y, map_yaw;
        bool got_pose = getRobotPoseInMap(map_x, map_y, map_yaw);
        
        RCLCPP_WARN(this->get_logger(), 
                   "Nav2 feedback: %.2f m remaining | Robot position (map): (%.2f, %.2f)",
                   feedback->distance_remaining, map_x, map_y);
        
        if (feedback->distance_remaining < 0.01 && got_pose) {
            RCLCPP_ERROR(this->get_logger(), 
                        "WARNING: Navigation2 says distance is 0.00 m but robot is at (%.2f, %.2f)!",
                        map_x, map_y);
            RCLCPP_ERROR(this->get_logger(), 
                        "This suggests Navigation2 goal checker tolerance is too high or configuration issue.");
        }
    }
}

void DeliveryNavigator::resultCallback(const GoalHandleNav::WrappedResult& result)
{
    // Get robot position when result is received
    double map_x, map_y, map_yaw;
    bool got_pose = getRobotPoseInMap(map_x, map_y, map_yaw);
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (got_pose) {
                RCLCPP_WARN(this->get_logger(), 
                           "Navigation goal succeeded! Robot final position: (%.2f, %.2f)",
                           map_x, map_y);
            } else {
                RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded!");
            }
            handleNavigationSuccess();
            break;
            
        case rclcpp_action::ResultCode::ABORTED:
            if (got_pose) {
                RCLCPP_ERROR(this->get_logger(), 
                           "Navigation goal was ABORTED! Robot position: (%.2f, %.2f)",
                           map_x, map_y);
                RCLCPP_ERROR(this->get_logger(), 
                           "Possible causes:");
                RCLCPP_ERROR(this->get_logger(), 
                           "  1. Navigation2 goal checker tolerance too high");
                RCLCPP_ERROR(this->get_logger(), 
                           "  2. Path planner failed immediately");
                RCLCPP_ERROR(this->get_logger(), 
                           "  3. Goal is outside map bounds");
                RCLCPP_ERROR(this->get_logger(), 
                           "  4. Navigation2 configuration issue");
            } else {
                RCLCPP_WARN(this->get_logger(), "Navigation goal was aborted");
            }
            handleNavigationFailure();
            break;
            
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation goal was canceled");
            handleNavigationFailure();
            break;
            
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown navigation result code");
            handleNavigationFailure();
            break;
    }
}

void DeliveryNavigator::handleNavigationSuccess()
{
    if (state_ == DeliveryState::RETURNING_HOME) {
        RCLCPP_INFO(this->get_logger(), "Returned home successfully!");
        
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "MISSION_COMPLETE";
        status_pub_->publish(status_msg);
        
        logDelivery("SYSTEM", "HOME", "MISSION_COMPLETE");
        
        state_ = DeliveryState::IDLE;
        delivery_in_progress_ = false;
        
    } else if (state_ == DeliveryState::NAVIGATING) {
        // Check if we're in a predefined route or just navigating to RViz2 goal
        if (current_route_index_ < delivery_routes_.size() &&
            current_waypoint_index_ < delivery_routes_[current_route_index_].waypoints.size()) {
            
            // In a predefined route - proceed to next waypoint
            const auto& waypoint = delivery_routes_[current_route_index_].waypoints[current_waypoint_index_];
            const auto& route = delivery_routes_[current_route_index_];
            
            RCLCPP_INFO(this->get_logger(), "Arrived at: %s", waypoint.name.c_str());
            
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "ARRIVED:" + route.route_name + ":" + waypoint.name;
            status_pub_->publish(status_msg);
            
            logDelivery(route.route_name, waypoint.name, "ARRIVED");
            
            // Simulate delivery/pickup duration (2 seconds)
            delivery_start_time_ = this->now();
            state_ = DeliveryState::DELIVERING;
        } else {
            // This was a direct RViz2 goal (not part of a route)
            RCLCPP_INFO(this->get_logger(), "Arrived at RViz2 goal! Ready for next goal.");
            
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "GOAL_REACHED";
            status_pub_->publish(status_msg);
            
            // Don't return home - just go to IDLE and wait for next goal
            state_ = DeliveryState::IDLE;
            delivery_in_progress_ = false;
        }
    }
}

void DeliveryNavigator::handleNavigationFailure()
{
    RCLCPP_WARN(this->get_logger(), "Navigation failed - returning to IDLE state");
    
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "NAVIGATION_FAILED";
    status_pub_->publish(status_msg);
    
    // Don't return home automatically - just go to IDLE and wait for next goal
    state_ = DeliveryState::IDLE;
    delivery_in_progress_ = false;
    
    // Only proceed to next waypoint if we're in a predefined route
    if (current_route_index_ < delivery_routes_.size() &&
        current_waypoint_index_ < delivery_routes_[current_route_index_].waypoints.size()) {
        proceedToNextWaypoint();
    } else {
        RCLCPP_INFO(this->get_logger(), "Ready for next navigation goal (use RViz2 to send new goal)");
    }
}

void DeliveryNavigator::proceedToNextWaypoint()
{
    current_waypoint_index_++;
    
    if (current_route_index_ < delivery_routes_.size() &&
        current_waypoint_index_ < delivery_routes_[current_route_index_].waypoints.size()) {
        
        // More waypoints in current route
        state_ = DeliveryState::NAVIGATING;
        const auto& waypoint = delivery_routes_[current_route_index_].waypoints[current_waypoint_index_];
        
        RCLCPP_INFO(this->get_logger(), "Proceeding to next waypoint: %s", waypoint.name.c_str());
        
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "NEXT_WAYPOINT:" + waypoint.name;
        status_pub_->publish(status_msg);
        
        sendNavigationGoal(waypoint);
    } else {
        // Route complete - DON'T automatically return home
        if (current_route_index_ < delivery_routes_.size()) {
            RCLCPP_INFO(this->get_logger(), "Route '%s' completed!",
                       delivery_routes_[current_route_index_].route_name.c_str());
            logDelivery(delivery_routes_[current_route_index_].route_name, "ROUTE", "COMPLETED");
        }
        
        // Just go to IDLE - wait for next goal from RViz2 or commander
        state_ = DeliveryState::IDLE;
        delivery_in_progress_ = false;
        RCLCPP_INFO(this->get_logger(), "Route complete. Ready for next navigation goal.");
    }
}

void DeliveryNavigator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
    
    // Set home position once after localization settles - MUST use map frame
    if (!home_position_set_) {
        static int odom_count = 0;
        odom_count++;
        
        // Try to get position in map frame (Navigation2 requires map frame)
        double map_x, map_y, map_yaw;
        if (getRobotPoseInMap(map_x, map_y, map_yaw)) {
            // Successfully got map frame position - use it as home
            home_position_.x = map_x;
            home_position_.y = map_y;
            home_position_.yaw = map_yaw;
            home_position_set_ = true;
            
            RCLCPP_INFO(this->get_logger(), 
                       "============================================================");
            RCLCPP_INFO(this->get_logger(), "HOME POSITION SET (map frame): (%.2f, %.2f, %.2f)",
                       home_position_.x, home_position_.y, home_position_.yaw);
            RCLCPP_INFO(this->get_logger(), "Robot is ready to accept navigation goals!");
            RCLCPP_INFO(this->get_logger(), "  - Use RViz2 'Publish Point' or '2D Goal Pose' to navigate");
            RCLCPP_INFO(this->get_logger(), 
                       "============================================================");
            
            logDelivery("SYSTEM", "HOME", "POSITION_SET");
        } else {
            // TF transform not available yet - keep trying
            if (odom_count == 20) {
                RCLCPP_WARN(this->get_logger(), 
                           "Waiting for robot localization in map frame...");
                RCLCPP_WARN(this->get_logger(), 
                           "Make sure Navigation2/AMCL is running and you've set initial pose in RViz2");
            }
        }
    }
    
    // Check delivery timing
    if (state_ == DeliveryState::DELIVERING) {
        auto elapsed = (this->now() - delivery_start_time_).seconds();
        if (elapsed >= 2.0) {  // 2 second delivery/pickup
            
            if (current_route_index_ < delivery_routes_.size() &&
                current_waypoint_index_ < delivery_routes_[current_route_index_].waypoints.size()) {
                
                const auto& waypoint = delivery_routes_[current_route_index_].waypoints[current_waypoint_index_];
                const auto& route = delivery_routes_[current_route_index_];
                
                RCLCPP_INFO(this->get_logger(), "Delivery complete at: %s", waypoint.name.c_str());
                
                auto status_msg = std_msgs::msg::String();
                status_msg.data = "DELIVERED:" + route.route_name + ":" + waypoint.name;
                status_pub_->publish(status_msg);
                
                logDelivery(route.route_name, waypoint.name, "DELIVERED");
            }
            
            // Proceed to next waypoint
            proceedToNextWaypoint();
        }
    }
}

void DeliveryNavigator::logDelivery(const std::string& route_name, 
                                    const std::string& waypoint_name,
                                    const std::string& status)
{
    std::ofstream log_file(delivery_log_file_, std::ios::app);
    if (log_file.is_open()) {
        log_file << "Route: " << route_name << "\n";
        log_file << "Waypoint: " << waypoint_name << "\n";
        log_file << "Status: " << status << "\n";
        log_file << "Timestamp: " << getCurrentTimestamp() << "\n";
        log_file << "------------------------------------------\n\n";
        log_file.close();
    }
}

std::string DeliveryNavigator::getCurrentTimestamp() const
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

std::vector<std::string> DeliveryNavigator::splitString(const std::string& str, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

bool DeliveryNavigator::getRobotPoseInMap(double& x, double& y, double& yaw)
{
    try {
        // Get transform from base_link to map using most recent transform
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(
            "map", "base_link", 
            rclcpp::Time(0),  // Use most recent available transform
            std::chrono::milliseconds(500));  // Wait up to 500ms
        
        x = transform.transform.translation.x;
        y = transform.transform.translation.y;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        return true;
    } catch (const tf2::TransformException& ex) {
        // Transform not available yet (localization might not be ready)
        RCLCPP_DEBUG(this->get_logger(), "TF transform not available: %s", ex.what());
        return false;
    }
}

void DeliveryNavigator::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!home_position_set_) {
        RCLCPP_WARN(this->get_logger(), "Received goal from RViz2 but home position not set yet. Ignoring.");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Received goal pose from RViz2:");
    RCLCPP_INFO(this->get_logger(), "  Position: (%.2f, %.2f) in frame '%s'",
               msg->pose.position.x, msg->pose.position.y, msg->header.frame_id.c_str());
    
    // Create a copy and ensure frame is "map"
    geometry_msgs::msg::PoseStamped pose = *msg;
    if (pose.header.frame_id != "map") {
        RCLCPP_WARN(this->get_logger(), "Goal pose frame is '%s', forcing to 'map' frame",
                   pose.header.frame_id.c_str());
        pose.header.frame_id = "map";
    }
    pose.header.stamp = this->now();
    
    // If currently on a mission, interrupt and go to new goal
    if (delivery_in_progress_ && state_ != DeliveryState::NAVIGATING) {
        RCLCPP_INFO(this->get_logger(), "Interrupting current mission to go to new goal");
    }
    
    // Reset route tracking since this is a direct RViz2 goal, not part of a route
    current_route_index_ = delivery_routes_.size();  // Invalid index = no route active
    current_waypoint_index_ = 0;
    
    sendNavigationGoal(pose);
}

void DeliveryNavigator::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "CLICKED POINT RECEIVED!");
    RCLCPP_INFO(this->get_logger(), "  Point: (%.2f, %.2f, %.2f)", 
               msg->point.x, msg->point.y, msg->point.z);
    RCLCPP_INFO(this->get_logger(), "  Frame: '%s'", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Home position set: %s", home_position_set_ ? "YES" : "NO");
    
    if (!home_position_set_) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Home position not set yet!");
        RCLCPP_ERROR(this->get_logger(), "Make sure:");
        RCLCPP_ERROR(this->get_logger(), "  1. Navigation2/AMCL is running");
        RCLCPP_ERROR(this->get_logger(), "  2. You've set initial pose in RViz2 using '2D Pose Estimate'");
        RCLCPP_ERROR(this->get_logger(), "  3. Wait for 'HOME POSITION SET' message in logs");
        return;
    }
    
    // Convert PointStamped to PoseStamped (orientation will face forward, yaw=0)
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position = msg->point;
    
    // IMPORTANT: Ensure frame_id is "map" - Navigation2 requires map frame
    // The clicked point from RViz2 should already be in map frame if you're using the map view
    if (pose.header.frame_id.empty() || pose.header.frame_id != "map") {
        RCLCPP_WARN(this->get_logger(), "Click point frame is '%s', forcing to 'map' frame",
                   pose.header.frame_id.c_str());
        pose.header.frame_id = "map";
    }
    
    // Set orientation to face forward (yaw = 0)
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(q);
    pose.header.stamp = this->now();
    
    // If currently on a mission, interrupt and go to new goal
    if (delivery_in_progress_ && state_ != DeliveryState::NAVIGATING) {
        RCLCPP_INFO(this->get_logger(), "Interrupting current mission to go to clicked point");
    }
    
    // Reset route tracking since this is a direct RViz2 goal, not part of a route
    current_route_index_ = delivery_routes_.size();  // Invalid index = no route active
    current_waypoint_index_ = 0;
    
    RCLCPP_INFO(this->get_logger(), "Sending navigation goal for clicked point...");
    sendNavigationGoal(pose);
    RCLCPP_INFO(this->get_logger(), "========================================");
}

}  // namespace delivery_navigator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::cout << "\n============================================================\n";
    std::cout << "Delivery Navigator - SLAM + LiDAR Based\n";
    std::cout << "============================================================\n";
    std::cout << "NO CAMERA REQUIRED\n";
    std::cout << "Uses: SLAM Map + LiDAR Localization + Navigation2\n";
    std::cout << "Listens for delivery requests on 'delivery_request' topic\n";
    std::cout << "============================================================\n\n";
    
    auto node = std::make_shared<delivery_navigator::DeliveryNavigator>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
}   

