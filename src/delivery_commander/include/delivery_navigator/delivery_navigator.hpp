// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: delivery_navigator.hpp
// Author(s): Project Team
//
// Delivery Navigator header file for SLAM-based navigation.

#ifndef DELIVERY_NAVIGATOR_HPP
#define DELIVERY_NAVIGATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>

namespace delivery_navigator
{

// Waypoint structure
struct Waypoint {
    double x, y, yaw;
    std::string name;
    
    Waypoint(double x = 0.0, double y = 0.0, double yaw = 0.0, 
             const std::string& name = "")
        : x(x), y(y), yaw(yaw), name(name) {}
};

// Delivery route structure
struct DeliveryRoute {
    std::string route_name;
    std::vector<Waypoint> waypoints;
    
    DeliveryRoute(const std::string& name = "")
        : route_name(name) {}
};

// Delivery state enum
enum class DeliveryState {
    INITIALIZING,
    IDLE,
    NAVIGATING,
    ARRIVED,
    DELIVERING,
    RETURNING_HOME,
    MISSION_COMPLETE
};

class DeliveryNavigator : public rclcpp::Node {
public:
    DeliveryNavigator();
    ~DeliveryNavigator();
    
private:
    // Action client for Navigation2
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr delivery_request_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr state_timer_;
    
    // TF for getting robot position in map frame
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Delivery routes
    std::vector<DeliveryRoute> delivery_routes_;
    size_t current_route_index_;
    size_t current_waypoint_index_;
    
    // Logging
    std::string delivery_log_file_;
    
    // Home position
    Waypoint home_position_;
    bool home_position_set_;
    
    // Robot state
    DeliveryState state_;
    double robot_x_, robot_y_, robot_yaw_;
    
    // Delivery tracking
    std::string pending_delivery_request_;
    bool delivery_in_progress_;
    rclcpp::Time delivery_start_time_;
    double delivery_duration_ = 2.0;  // 2 seconds per delivery
    
    // Private methods
    void initializeRoutes();
    void initializeDeliveryLog();
    void deliveryRequestCallback(const std_msgs::msg::String::SharedPtr msg);
    void stateManagementCallback();
    void processDeliveryRequest(const std::string& request);
    void startReturnHome();
    void sendNavigationGoal(const Waypoint& waypoint);
    void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& pose);
    geometry_msgs::msg::PoseStamped waypointToPoseStamped(const Waypoint& waypoint);
    
    // RViz2 goal callbacks
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    // Action callbacks
    void goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle);
    void feedbackCallback(GoalHandleNav::SharedPtr,
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNav::WrappedResult& result);
    
    // Navigation handling
    void handleNavigationSuccess();
    void handleNavigationFailure();
    void proceedToNextWaypoint();
    
    // Odometry callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Get robot pose in map frame using TF
    bool getRobotPoseInMap(double& x, double& y, double& yaw);
    
    // Logging
    void logDelivery(const std::string& route_name, 
                    const std::string& waypoint_name,
                    const std::string& status);
    
    // Utility methods
    std::string getCurrentTimestamp() const;
    std::vector<std::string> splitString(const std::string& str, char delimiter);
};

}  // namespace delivery_navigator

#endif  // DELIVERY_NAVIGATOR_HPP
