// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: base_robot.hpp
// Author(s): Project Team
//
// Abstract base class for warehouse robots. Provides shared functionality:
// - Basic motion control (differential drive)
// - Battery monitoring and tracking
// - Odometry subscription and pose tracking
// Derived classes implement specialized behaviors.

#ifndef MODE_SELECTOR__BASE_ROBOT_HPP_
#define MODE_SELECTOR__BASE_ROBOT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <memory>

namespace mode_selector
{

/**
 * @class BaseRobot
 * @brief Abstract base class for warehouse robot types
 * 
 * This class provides shared functionality required by all robot types:
 * - Basic motion commands (TwistStamped publisher)
 * - Battery level monitoring and tracking
 * - Odometry subscription for pose tracking
 * 
 * Derived classes must implement:
 * - initialize(): Set up robot-specific subscriptions and publishers
 * - run(): Main execution loop with robot-specific behavior
 */
class BaseRobot : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     * @param node_name Name for the ROS 2 node
     */
    explicit BaseRobot(const std::string& node_name);
    
    /**
     * @brief Virtual destructor
     */
    virtual ~BaseRobot() = default;
    
    // ========== Shared Functionality (Public Interface) ==========
    
    /**
     * @brief Publish velocity command to robot
     * @param linear Linear velocity (m/s)
     * @param angular Angular velocity (rad/s)
     */
    void publishVelocity(double linear, double angular);
    
    /**
     * @brief Stop the robot (set all velocities to zero)
     */
    void stopRobot();
    
    /**
     * @brief Get current battery level
     * @return Battery level as percentage (0-100)
     */
    float getBatteryLevel() const;
    
    /**
     * @brief Get current robot pose from odometry
     * @return Current pose (position and orientation)
     */
    geometry_msgs::msg::Pose getCurrentPose() const;
    
    // ========== Virtual Interface (Must be implemented by derived classes) ==========
    
    /**
     * @brief Initialize robot-specific functionality
     * 
     * Derived classes should:
     * - Set up specialized subscribers (ArUco, LiDAR, etc.)
     * - Initialize specialized publishers
     * - Configure robot-specific parameters
     * - Set up any timers or action clients
     */
    virtual void initialize() = 0;
    
    /**
     * @brief Main execution loop
     * 
     * Derived classes implement their specific behavior here:
     * - InspectionRobot: Wall following, damage detection
     * - DeliveryRobot: Route navigation, delivery management
     */
    virtual void run() = 0;

protected:
    // ========== Shared ROS 2 Objects ==========
    
    /// Publisher for velocity commands (TwistStamped for TurtleBot3)
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    
    /// Subscriber for battery level updates
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    
    /// Subscriber for odometry (robot pose and velocity)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ========== Shared State Variables ==========
    
    /// Current battery level (0-100 percentage)
    float battery_level_;
    
    /// Current robot pose from odometry
    geometry_msgs::msg::Pose current_pose_;
    
    /// Robot position (x, y) from odometry
    double robot_x_, robot_y_;
    
    /// Robot orientation (yaw) from odometry
    double robot_yaw_;

protected:
    // ========== Callback Functions ==========
    
    /**
     * @brief Callback for battery level updates
     * @param msg Battery level message (Float32)
     * Can be overridden by derived classes for specialized behavior
     */
    virtual void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
    
    /**
     * @brief Callback for odometry updates
     * @param msg Odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace mode_selector

#endif  // MODE_SELECTOR__BASE_ROBOT_HPP_



