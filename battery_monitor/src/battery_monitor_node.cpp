// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: battery_monitor_node.cpp
// Author(s): Ayesha Musarrat
//

//   ROS2 node that simulates a virtual battery, publishes battery level,
//   logs events, and issues a dock command when the battery is low or a job
//   completes. 

//optional: velocity-based drain via /odom.
//
// Topics (publish):
//   /battery/level        : std_msgs/Float32   (battery %)
//   /battery/state        : std_msgs/String    ("normal"|"low"|"critical")
//   /battery/dock_command : std_msgs/Bool      (true => request docking)

// Topics (subscribe):
//   /odom                 : nav_msgs/Odometry  (optional, for speed-based drain)
//   /job/status           : std_msgs/String    (expects "complete" to trigger docking)
//
// Params:
//   log_path (string, default: ~/.ros/battery_log.txt)
//   start_percent, base_drain_per_sec, velocity_drain_gain, low_threshold, critical_threshold
//   use_odom (bool, default: false)
//   tick_ms (int, default: 200)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "battery_monitor/battery.hpp"
#include "battery_monitor/logger.hpp"

using std::placeholders::_1;

class BatteryMonitorNode : public rclcpp::Node {
public:
  BatteryMonitorNode() : Node("battery_monitor") {
    // Parameters
    std::string log_path = this->declare_parameter<std::string>("log_path", std::string(get_log_default()));
    bm::BatteryParams p;
    p.start_percent       = this->declare_parameter<float>("start_percent",        100.0f);
    p.base_drain_per_sec  = this->declare_parameter<float>("base_drain_per_sec",   0.05f);
    p.velocity_drain_gain = this->declare_parameter<float>("velocity_drain_gain",  0.02f);
    p.low_threshold       = this->declare_parameter<float>("low_threshold",        20.0f);
    p.critical_threshold  = this->declare_parameter<float>("critical_threshold",   10.0f);
    use_odom_             = this->declare_parameter<bool>("use_odom",              false);
    int tick_ms           = this->declare_parameter<int>("tick_ms",                200);

    battery_ = std::make_unique<bm::Battery>(p);
    logger_  = std::make_unique<bm::Logger>(log_path);

    pub_level_ = this->create_publisher<std_msgs::msg::Float32>("/battery/level", 10);
    pub_state_ = this->create_publisher<std_msgs::msg::String> ("/battery/state", 10);
    pub_dock_  = this->create_publisher<std_msgs::msg::Bool>   ("/battery/dock_command", 10);

    if (use_odom_) {
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&BatteryMonitorNode::onOdom, this, _1));
    }
    sub_job_ = this->create_subscription<std_msgs::msg::String>(
      "/job/status", 10, std::bind(&BatteryMonitorNode::onJobStatus, this, _1));

    last_speed_ = 0.0f;
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(tick_ms),
      std::bind(&BatteryMonitorNode::onTick, this));

    logger_->write("[Init] Battery monitor node started. Params: start=" + std::to_string(p.start_percent) +
                   "% low=" + std::to_string(p.low_threshold) +
                   "% crit=" + std::to_string(p.critical_threshold) + "%");
  }

private:
  static const char* get_log_default() {
    // Write into ~/.ros by default (exists when running any ROS node)
    return "/home/ubuntu/.ros/battery_log.txt"; // tweak username if needed
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Approximate linear speed magnitude from twist
    const auto& v = msg->twist.twist.linear;
    last_speed_ = static_cast<float>(std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z));
  }

  void onJobStatus(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "complete") {
      // Auto-dock on job complete (extension requirement)
      std_msgs::msg::Bool dock;
      dock.data = true;
      pub_dock_->publish(dock);
      logger_->write("[Dock] Job complete -> docking requested.");
      RCLCPP_WARN(this->get_logger(), "Job complete -> docking requested.");
    }
  }

  void onTick() {
    // Advance model
    const float dt = 0.2f; // seconds for 200ms tick
    const float speed = use_odom_ ? last_speed_ : 0.0f;
    battery_->step(dt, speed);

    // Publish level
    std_msgs::msg::Float32 lvl;
    lvl.data = battery_->percent();
    pub_level_->publish(lvl);

    // Publish state & dock decision
    std_msgs::msg::String state;
    if (battery_->isCritical()) {
      state.data = "critical";
      pub_state_->publish(state);
      maybeRequestDock("[Dock] Battery critical -> docking requested.");
    } else if (battery_->isLow()) {
      state.data = "low";
      pub_state_->publish(state);
      maybeRequestDock("[Dock] Battery low -> docking requested.");
    } else {
      state.data = "normal";
      pub_state_->publish(state);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Battery: %.2f%% (state=%s, speed=%.2f m/s)",
                         lvl.data, state.data.c_str(), speed);
  }

  void maybeRequestDock(const std::string& reason) {
    // Only publish once per condition trip to avoid spam
    if (!dock_requested_) {
      dock_requested_ = true;
      std_msgs::msg::Bool dock;
      dock.data = true;
      pub_dock_->publish(dock);
      logger_->write(reason);
      RCLCPP_WARN(this->get_logger(), "%s", reason.c_str());
    }
  }

  // Members
  std::unique_ptr<bm::Battery> battery_;
  std::unique_ptr<bm::Logger>  logger_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_level_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  pub_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_dock_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   sub_job_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool use_odom_{false};
  bool dock_requested_{false};
  float last_speed_{0.0f};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
