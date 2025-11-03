// MTRX3760 2025 Project 2: Warehouse Robot DevKit 
// File: low_battery_homing
// Author(s): Raquel Kampel
// 
// Header file to return turtlebot to id0 when low battery

#ifndef LOW_BATTERY_HOMING_HPP
#define LOW_BATTERY_HOMING_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <cmath>

class LowBatteryHoming : public rclcpp::Node
{
public:
  LowBatteryHoming();

private:
  void onBatteryLevel(const std_msgs::msg::Float32::SharedPtr msg);
  void onTagPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void startHoming();
  void performHomingStep();
  void stopRobot();

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_enable_pub_;   // NEW

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag0_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr homing_timer_;

  // Variables
  geometry_msgs::msg::PoseStamped last_tag_pose_;
  float battery_threshold_;
  bool homing_active_;
};

#endif

