#include "robot_drive/wall_follower.hpp"
#include <algorithm>
#include <cmath>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

RightHandWallFollower::RightHandWallFollower()
: Node("right_hand_wall_follower"),
  linear_speed_(0.12),
  angular_speed_(0.5),
  corner_angular_speed_(0.8),
  desired_wall_distance_(0.35),
  front_threshold_(0.45),
  side_threshold_(0.4),
  min_obstacle_distance_(0.18),
  corner_clearance_(0.32),
  wall_found_(false),
  right_turn_counter_(0)
{
  RCLCPP_INFO(this->get_logger(), "Right-Hand Wall Follower node started.");

  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

  // Subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS(),
    std::bind(&RightHandWallFollower::scan_callback, this, std::placeholders::_1));

  wall_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/wall_follow_enable",
    10,
    std::bind(&RightHandWallFollower::onEnable, this, std::placeholders::_1));

  // PID Controller setup
  wall_pid_ = std::make_unique<PIDController>(2.5, 0.0, 0.1, 0.6);
}

RightHandWallFollower::~RightHandWallFollower()
{
  geometry_msgs::msg::TwistStamped stop;
  stop.header.stamp = this->now();
  stop.twist.linear.x = 0.0;
  stop.twist.angular.z = 0.0;
  cmd_vel_pub_->publish(stop);
}

void RightHandWallFollower::onEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  wall_follow_enabled_ = msg->data;
  if (!wall_follow_enabled_) {
    RCLCPP_WARN(this->get_logger(), "Wall following disabled (battery homing active)");
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = this->now();
    stop.twist.linear.x = 0.0;
    stop.twist.angular.z = 0.0;
    cmd_vel_pub_->publish(stop);
  } else {
    RCLCPP_INFO(this->get_logger(), "Wall following re-enabled");
  }
}

float RightHandWallFollower::find_min(const std::vector<float>& ranges, size_t start, size_t end)
{
  float min_val = 3.5;
  for (size_t i = start; i < end; ++i) {
    float r = ranges[i];
    if (std::isfinite(r) && r > 0.01 && r < min_val)
      min_val = r;
  }
  return min_val;
}

RightHandWallFollower::RangeData
RightHandWallFollower::get_scan_ranges(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const size_t size = msg->ranges.size();
  RangeData data;
  data.front = find_min(msg->ranges, size / 2 - 10, size / 2 + 10);
  data.right = find_min(msg->ranges, size * 3 / 4, size * 3 / 4 + 10);
  data.front_right = find_min(msg->ranges, size * 5 / 8, size * 5 / 8 + 10);
  data.left = find_min(msg->ranges, size / 8, size / 8 + 10);
  return data;
}

void RightHandWallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!wall_follow_enabled_) return;  // skip control if disabled

  RangeData ranges = get_scan_ranges(msg);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = this->now();

  // ===== 1. Emergency stop =====
  if (ranges.front < min_obstacle_distance_) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    RCLCPP_WARN(this->get_logger(), "Emergency stop: obstacle %.2f m ahead", ranges.front);
    return;
  }

  // ===== 2. Corner turn =====
  if (ranges.front < front_threshold_ && ranges.right < side_threshold_) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = corner_angular_speed_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Corner detected, turning left");
  }
  // ===== 3. Left turn =====
  else if (ranges.front < front_threshold_) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = angular_speed_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Front obstacle, turning left");
  }
  // ===== 4. Adjust left if front-right blocked =====
  else if (ranges.front_right < corner_clearance_) {
    cmd.twist.linear.x = 0.08;
    cmd.twist.angular.z = 0.3;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Front-right close, adjusting left");
  }
  // ===== 5. Follow right wall using PID =====
  else if (ranges.right < side_threshold_) {
    double error = desired_wall_distance_ - ranges.right;
    rclcpp::Time now = this->now();
    double dt = last_scan_time_.nanoseconds() > 0 ? (now - last_scan_time_).seconds() : 0.1;
    last_scan_time_ = now;

    double correction = wall_pid_->calculate(error, dt);

    cmd.twist.linear.x = linear_speed_;
    cmd.twist.angular.z = correction;
  }
  // ===== 6. No wall on right: turn right to find one =====
  else {
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = -angular_speed_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "No wall found, turning right");
  }

  cmd_vel_pub_->publish(cmd);
}

double RightHandWallFollower::PIDController::calculate(double error, double dt)
{
  if (first_call_) {
    prev_error_ = error;
    first_call_ = false;
  }

  integral_ += error * dt;
  double derivative = (error - prev_error_) / dt;
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  output = std::clamp(output, -max_output_, max_output_);
  prev_error_ = error;
  return output;
}

void RightHandWallFollower::PIDController::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
  first_call_ = true;
}

RightHandWallFollower::PIDController::PIDController(
  double kp, double ki, double kd, double max_output)
: kp_(kp), ki_(ki), kd_(kd),
  max_output_(max_output),
  integral_(0.0),
  prev_error_(0.0),
  first_call_(true)
{}

void RightHandWallFollower::performWallFollowingStep()
{
  RCLCPP_INFO(this->get_logger(), "Manual call: performing wall following step (if active)");
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightHandWallFollower>());
  rclcpp::shutdown();
  return 0;
}

