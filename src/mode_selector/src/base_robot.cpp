// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: base_robot.cpp
// Author(s): Project Team
//
// Implementation of BaseRobot class - shared functionality for all robot types.

#include "mode_selector/base_robot.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mode_selector
{

BaseRobot::BaseRobot(const std::string& node_name)
    : Node(node_name),
      battery_level_(100.0f),
      robot_x_(0.0),
      robot_y_(0.0),
      robot_yaw_(0.0)
{
    // Initialize current pose
    current_pose_.position.x = 0.0;
    current_pose_.position.y = 0.0;
    current_pose_.position.z = 0.0;
    current_pose_.orientation.x = 0.0;
    current_pose_.orientation.y = 0.0;
    current_pose_.orientation.z = 0.0;
    current_pose_.orientation.w = 1.0;
    
    // Publisher for velocity commands (TwistStamped for TurtleBot3)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10);
    
    // Subscriber for battery level
    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/battery/level", 10,
        std::bind(&BaseRobot::batteryCallback, this, std::placeholders::_1));
    
    // Subscriber for odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&BaseRobot::odomCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "BaseRobot initialized: %s", node_name.c_str());
}

void BaseRobot::publishVelocity(double linear, double angular)
{
    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = linear;
    cmd.twist.angular.z = angular;
    
    cmd_vel_pub_->publish(cmd);
}

void BaseRobot::stopRobot()
{
    publishVelocity(0.0, 0.0);
}

float BaseRobot::getBatteryLevel() const
{
    return battery_level_;
}

geometry_msgs::msg::Pose BaseRobot::getCurrentPose() const
{
    return current_pose_;
}

void BaseRobot::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    battery_level_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Battery level: %.2f%%", battery_level_);
}

void BaseRobot::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Update position
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    
    // Update current pose
    current_pose_ = msg->pose.pose;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
}

}  // namespace mode_selector




