#include "robot_drive/low_battery_homing.hpp"

LowBatteryHoming::LowBatteryHoming()
: Node("low_battery_homing"), battery_threshold_(20.0f), homing_active_(false)
{
  RCLCPP_INFO(this->get_logger(), "LowBatteryHoming node started. Threshold = %.1f%%", battery_threshold_);

  // Publishers
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  wall_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/wall_follow_enable", 10);

  // Subscriptions
  battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/battery/level", 10,
    std::bind(&LowBatteryHoming::onBatteryLevel, this, std::placeholders::_1));

  tag0_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco/pose/id_0", 10,
    std::bind(&LowBatteryHoming::onTagPose, this, std::placeholders::_1));
}

void LowBatteryHoming::onBatteryLevel(const std_msgs::msg::Float32::SharedPtr msg)
{
  float level = msg->data;

  if (level < battery_threshold_ && !homing_active_) {
    RCLCPP_WARN(this->get_logger(), "Battery low (%.2f%%). Initiating homing sequence...", level);
    homing_active_ = true;

    // Disable wall following
    std_msgs::msg::Bool disable_msg;
    disable_msg.data = false;
    wall_enable_pub_->publish(disable_msg);

    startHoming();
  }
}

void LowBatteryHoming::onTagPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_tag_pose_ = *msg;
}

void LowBatteryHoming::startHoming()
{
  stopRobot();
  homing_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&LowBatteryHoming::performHomingStep, this));

  RCLCPP_INFO(this->get_logger(), "Homing started. Waiting for ArUco tag ID_0...");
}

void LowBatteryHoming::performHomingStep()
{
  if (!homing_active_)
    return;

  if (last_tag_pose_.header.frame_id.empty()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Battery low — searching for ArUco tag ID_0...");
    return;
  }

  double x = last_tag_pose_.pose.position.x;
  double z = last_tag_pose_.pose.position.z;
  double distance = std::sqrt(x * x + z * z);

  if (distance < 0.25) {
    stopRobot();
    homing_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Reached home tag (%.2f m). Stopping robot.", distance);

    // Re-enable wall following
    std_msgs::msg::Bool enable_msg;
    enable_msg.data = true;
    wall_enable_pub_->publish(enable_msg);

    return;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = 0.1;
  cmd.twist.angular.z = -0.6 * std::atan2(x, z);
  cmd_pub_->publish(cmd);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Homing to tag ID_0: dist=%.2f m, ang_z=%.2f", distance, cmd.twist.angular.z);
}

void LowBatteryHoming::stopRobot()
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = this->now();
  stop.header.frame_id = "base_link";
  stop.twist.linear.x = 0.0;
  stop.twist.angular.z = 0.0;

  cmd_pub_->publish(stop);
  RCLCPP_WARN(this->get_logger(), "Robot stopped.");
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowBatteryHoming>());
  rclcpp::shutdown();
  return 0;
}

