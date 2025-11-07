// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: wall_follower.hpp
// Author(s): Christopher Krsevan
//
// This node implements a right-hand wall following algorithm for a TurtleBot3 robot.
// It processes LiDAR data to populate a small set of directional ranges (front, right,
// front-right, left) and applies a PID-controlled state machine to stay ~15 cm from the
// right wall. The controller prioritises safety (emergency stop), sharp cornering, and a
// multi-phase right-turn sequence when an opening appears. Additional smoothing and rate
// limiting ensure the robot remains stable on real hardware.

#include "wall_follower/wall_follower.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using namespace std::chrono_literals;

// PID controller implementation

RightHandWallFollower::PIDController::PIDController(double kp, double ki, double kd, double max_output)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  max_output_(max_output),
  integral_(0.0),
  prev_error_(0.0),
  first_call_(true)
{
}

double RightHandWallFollower::PIDController::calculate(double error, double dt)
{
    // Handle the first call separately
    if (first_call_) {
        prev_error_ = error;
        first_call_ = false;
        return kp_ * error;  // Use proportional term only on the first call
    }

    // Proportional term
    double p_term = kp_ * error;

    // Integral term with anti-windup
    integral_ += error * dt;
    // Limit integral growth to prevent windup
    double max_integral = max_output_ / (ki_ + 0.001);  // Prevent division by zero
    integral_ = std::clamp(integral_, -max_integral, max_integral);
    double i_term = ki_ * integral_;

    // Derivative term
    double derivative = (error - prev_error_) / dt;
    double d_term = kd_ * derivative;

    // Calculate PID output
    double output = p_term + i_term + d_term;

    // Clamp output to the configured bounds
    output = std::clamp(output, -max_output_, max_output_);

    // Store error for next iteration
    prev_error_ = error;

    return output;
}

void RightHandWallFollower::PIDController::reset()
{
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_call_ = true;
}

// Node construction

RightHandWallFollower::RightHandWallFollower()
: Node("right_hand_wall_follower"),
  linear_speed_(0.15),
  angular_speed_(0.25),
  corner_angular_speed_(0.6),
  desired_wall_distance_(0.15),
  front_threshold_(0.35),
  side_threshold_(0.35),
  min_obstacle_distance_(0.25),
  corner_clearance_(0.35),
  state_("FINDING_WALL"),
  wall_found_(false),
  right_turn_counter_(0),
  last_scan_time_(this->now())
{
    // Initialize PID controller for wall following with damped gains
    wall_pid_ = std::make_unique<PIDController>(0.70, 0.05, 1.75, angular_speed_);

    // Configure LiDAR QoS to best match sensor behavior
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();

    // Create publishers and subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos, std::bind(&RightHandWallFollower::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "Right-Hand Wall Follower");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "Linear speed: %.2f m/s", linear_speed_);
    RCLCPP_INFO(this->get_logger(), "Angular speed: %.2f rad/s (corners: %.2f)", angular_speed_, corner_angular_speed_);
    RCLCPP_INFO(this->get_logger(), "Target wall distance: %.2f m (25cm)", desired_wall_distance_);
    RCLCPP_INFO(this->get_logger(), "PID Gains: Kp=0.7, Ki=0.05, Kd=1.75 (damped)");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "SAFETY: Press Ctrl+C to emergency stop!");
    RCLCPP_INFO(this->get_logger(), "============================================================");
}

// Node teardown

RightHandWallFollower::~RightHandWallFollower()
{
    // Publish a stop command before shutting down
    auto stop_cmd = geometry_msgs::msg::TwistStamped();
    stop_cmd.header.stamp = this->now();
    stop_cmd.header.frame_id = "base_link";
    cmd_vel_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "Robot stopped.");
}

// Helper methods

float RightHandWallFollower::find_min(const std::vector<float>& ranges, size_t start, size_t end)
{
    // Defensive scan helper: ignore invalid readings and return a conservative default (3.5 m)
    float min_val = std::numeric_limits<float>::max();
    for (size_t i = start; i < end && i < ranges.size(); i++) {
        float val = ranges[i];
        if (!std::isinf(val) && !std::isnan(val) && val > 0.0f) {
            min_val = std::min(min_val, val);
        }
    }
    return (min_val == std::numeric_limits<float>::max()) ? 3.5f : min_val;
}

RightHandWallFollower::RangeData RightHandWallFollower::get_scan_ranges(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RangeData ranges;

    // Front ranges use 0-15 and 345-360 degrees
    float front_min1 = find_min(msg->ranges, 0, 15);
    float front_min2 = find_min(msg->ranges, 345, 360);
    ranges.front = std::min(front_min1, front_min2);

    // Right ranges use 250-290 degrees
    ranges.right = find_min(msg->ranges, 250, 290);

    // Front-right ranges use 300-330 degrees
    ranges.front_right = find_min(msg->ranges, 300, 330);

    // Left ranges use 70-110 degrees
    ranges.left = find_min(msg->ranges, 70, 110);

    return ranges;
}

// Main callback with PID control

void RightHandWallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Extract processed range data
    auto distances = get_scan_ranges(msg);

    // Calculate time delta for PID computation
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_scan_time_).seconds();
    last_scan_time_ = current_time;

    // Prevent division by zero and handle the first call
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.03;
    }

    // Prepare velocity command message
    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.header.stamp = current_time;
    cmd.header.frame_id = "base_link";

    // Priority 1: emergency stop if too close to an obstacle
    if (distances.front < min_obstacle_distance_) {
        state_ = "EMERGENCY_STOP";
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = corner_angular_speed_;
        wall_pid_->reset();  // Reset PID when not following wall
        right_turn_counter_ = 0;  // Reset turn sequence counter
        RCLCPP_WARN(this->get_logger(), "EMERGENCY! Obstacle at %.2fm", distances.front);
    }
    // Priority 2: obstacle ahead and wall on the right; perform a sharp left turn
    else if (distances.front < front_threshold_ && distances.right < side_threshold_) {
        state_ = "CORNER_LEFT";
        cmd.twist.linear.x = 0.0;  // Stop completely
        cmd.twist.angular.z = corner_angular_speed_;
        wall_pid_->reset();  // Reset PID during corner turn
        right_turn_counter_ = 0;  // Reset turn sequence counter
        RCLCPP_INFO(this->get_logger(), "CORNER! Front: %.2fm, Right: %.2fm - SHARP LEFT",
                   distances.front, distances.right);
        wall_found_ = true;
    }
    // Priority 3: obstacle ahead; perform a sharp left turn
    else if (distances.front < front_threshold_) {
        state_ = "TURNING_LEFT";
        cmd.twist.linear.x = 0.0;  // Stop completely for sharp turn
        cmd.twist.angular.z = corner_angular_speed_;
        wall_pid_->reset();  // Reset PID during turn
        right_turn_counter_ = 0;  // Reset turn sequence counter
        RCLCPP_INFO(this->get_logger(), "Obstacle ahead (%.2fm) - SHARP LEFT", distances.front);
        wall_found_ = true;
    }
    // Priority 4: obstacle at the front-right corner; adjust to the left
    else if (distances.front_right < corner_clearance_) {
        state_ = "ADJUSTING_LEFT";
        cmd.twist.linear.x = linear_speed_ * 0.4;
        cmd.twist.angular.z = angular_speed_ * 0.5;
        wall_pid_->reset();  // Reset PID during adjustment
        right_turn_counter_ = 0;  // Reset turn sequence counter
        RCLCPP_INFO(this->get_logger(), "Front-right obstacle (%.2fm) - Adjust left", distances.front_right);
        wall_found_ = true;
    }
    // Priority 5: wall detected on the right; follow using PID control
    else if (distances.right < side_threshold_) {
        wall_found_ = true;
        right_turn_counter_ = 0;  // Reset counter when wall is detected
        state_ = "FOLLOWING_PID";

        // Calculate error (positive means too far, negative means too close)
        double error = distances.right - desired_wall_distance_;
        // Apply a deadband to reduce noise-driven corrections
        if (std::fabs(error) < 0.04) {
            error = 0.0;
        }

        // Use PID to calculate the angular velocity correction
        double angular_correction = -wall_pid_->calculate(error, dt);  // Negative to turn toward the wall

        // Apply exponential smoothing and rate limiting to avoid jitter
        static double prev_angular_cmd = 0.0;
        const double alpha = 0.4;  // Smoothing factor
        double smoothed = alpha * angular_correction + (1.0 - alpha) * prev_angular_cmd;
        const double max_delta = 0.15;  // Maximum change per cycle
        double delta = smoothed - prev_angular_cmd;
        if (delta > max_delta) smoothed = prev_angular_cmd + max_delta;
        if (delta < -max_delta) smoothed = prev_angular_cmd - max_delta;
        prev_angular_cmd = smoothed;

        // Reduce forward speed if large corrections are required
        double speed_factor = 1.0 - std::min(0.6, std::abs(angular_correction) / angular_speed_);
        speed_factor = std::clamp(speed_factor, 0.25, 1.0);
        cmd.twist.linear.x = linear_speed_ * speed_factor;
        cmd.twist.angular.z = prev_angular_cmd;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "PID Control: dist=%.2fm, error=%.3fm, angular=%.2f rad/s",
            distances.right, error, angular_correction);
    }
    // Priority 6: no wall detected on the right; execute turn-forward-turn sequence
    else {
        wall_pid_->reset();  // Reset PID when no wall is detected

        // If a wall was previously detected, execute the turn-forward-turn sequence
        if (wall_found_) {
            // Phase 1: turn right
            if (right_turn_counter_ < 8) {
                state_ = "TURNING_RIGHT_OPENING";
                cmd.twist.linear.x = 0.0;  // Stop while turning for a sharp turn
                cmd.twist.angular.z = -corner_angular_speed_;  // Full-speed right turn
                right_turn_counter_++;
                RCLCPP_INFO(this->get_logger(), "Opening! Phase 1: Turning right (count: %d)", right_turn_counter_);
            }
            // Phase 2: drive forward
            else if (right_turn_counter_ < 20) {
                state_ = "DRIVING_FORWARD_AFTER_TURN";
                cmd.twist.linear.x = linear_speed_;  // Full-speed forward
                cmd.twist.angular.z = 0.0;  // No turning, straight ahead
                right_turn_counter_++;
                RCLCPP_INFO(this->get_logger(), "Opening! Phase 2: Driving forward (count: %d)", right_turn_counter_);
            }
            // Phase 3: turn right again
            else if (right_turn_counter_ < 25) {
                state_ = "TURNING_RIGHT_SECOND";
                cmd.twist.linear.x = 0.0;  // Stop while turning
                cmd.twist.angular.z = -corner_angular_speed_ * 0.7;  // Medium right turn
                right_turn_counter_++;
                RCLCPP_INFO(this->get_logger(), "Opening! Phase 3: Turning right again (count: %d)", right_turn_counter_);
            }
            // Phase 4: drive forward until a wall is detected
            else {
                state_ = "SEEKING_WALL_FORWARD";
                cmd.twist.linear.x = linear_speed_;  // Full-speed forward
                cmd.twist.angular.z = 0.0;  // Straight
                right_turn_counter_++;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Opening! Phase 4: Seeking wall forward (count: %d)", right_turn_counter_);
            }
        }
        // If no wall has been seen yet, continue searching
        else {
            state_ = "FINDING_WALL";
            cmd.twist.linear.x = linear_speed_ * 0.7;
            cmd.twist.angular.z = -angular_speed_ * 0.5;
            right_turn_counter_ = 0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Searching for wall (right: %.2fm)", distances.right);
        }
    }

    // Publish velocity command
    cmd_vel_pub_->publish(cmd);
}

// Entry point

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::cout << "\n============================================================\n";
    std::cout << "Wall Follower Active\n";
    std::cout << "============================================================\n";
    std::cout << "SAFETY: Keep robot in clear area\n";
    std::cout << "EMERGENCY STOP: Press Ctrl+C\n";
    std::cout << "Wall following enabled\n";
    std::cout << "============================================================\n\n";

    auto node = std::make_shared<RightHandWallFollower>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    std::cout << "\n============================================================\n";
    std::cout << "Emergency Stop Activated!\n";
    std::cout << "============================================================\n\n";

    rclcpp::shutdown();
    return 0;
}
