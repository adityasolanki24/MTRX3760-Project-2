// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: battery_monitor.hpp
// Author: Raquel Kampel
//
// Description:
//   Header file for the BatteryMonitorNode class.
//   This node simulates or reads the robot’s battery percentage and
//   publishes it to the /battery/level topic as a std_msgs::msg::Float32.
//
//   By default, the node simulates a slow discharge (Option 2).
//   For real hardware integration, you can adapt it to read values
//   from a CSV file, ADC, or other sensor input.

#ifndef BATTERY_MONITOR_BATTERY_MONITOR_HPP_
#define BATTERY_MONITOR_BATTERY_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <memory>
#include <fstream>

class BatteryMonitorNode : public rclcpp::Node {
public:
  BatteryMonitorNode();

private:
  // === Publishers & Timers ===
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // === Battery state ===
  float battery_level_;     // Current battery level (0–100%)
  std::string battery_file_; // Optional path to CSV or log file

  // === Callbacks ===
  void updateBatteryLevel();  // Called each timer tick

  // === Helper for reading from file (optional future use) ===
  bool readBatteryFromFile(float &value);
};

#endif  // BATTERY_MONITOR_BATTERY_MONITOR_HPP_

