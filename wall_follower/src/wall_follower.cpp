/*
 * Right-Hand Wall Following Algorithm for Real TurtleBot3
 * C++ implementation for ROS 2 with PID Control
 */

#include "wall_follower/wall_follower.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;

// ========== PID Controller Implementation ==========

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
    // Handle first call
    if (first_call_) {
        prev_error_ = error;
        first_call_ = false;
        return kp_ * error;  // Only proportional on first call
    }
    
    // Proportional term
    double p_term = kp_ * error;
    
    // Integral term (with anti-windup)
    integral_ += error * dt;
    // Limit integral to prevent windup
    double max_integral = max_output_ / (ki_ + 0.001);  // Avoid division by zero
    integral_ = std::clamp(integral_, -max_integral, max_integral);
    double i_term = ki_ * integral_;
    
    // Derivative term
    double derivative = (error - prev_error_) / dt;
    double d_term = kd_ * derivative;
    
    // Calculate total output
    double output = p_term + i_term + d_term;
    
    // Clamp output to maximum
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

// ========== Constructor ==========

RightHandWallFollower::RightHandWallFollower()
: Node("right_hand_wall_follower"),
  linear_speed_(0.1),
  angular_speed_(0.4),
  corner_angular_speed_(1.0),
  desired_wall_distance_(0.20),
  front_threshold_(0.25),
  side_threshold_(0.35),
  min_obstacle_distance_(0.25),
  corner_clearance_(0.25),
  state_("FINDING_WALL"),
  wall_found_(false),
  right_turn_counter_(0),
  last_scan_time_(this->now())
{
    // Initialize PID controller for wall following
    // Tuned gains for 20cm distance: Kp=2.0, Ki=0.05, Kd=1.0, max_output=angular_speed
    // Lower Kp = less aggressive, Higher Kd = more damping, Lower Ki = less windup
    wall_pid_ = std::make_unique<PIDController>(2.0, 0.05, 1.0, angular_speed_);
    
    // QoS Profile for LiDAR - MUST use BEST_EFFORT to match sensor
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    
    // Publishers and Subscribers - REAL ROBOT USES TWISTSTAMPED
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos, std::bind(&RightHandWallFollower::scan_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "Right-Hand Wall Follower - PID CONTROL VERSION");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "Linear speed: %.2f m/s", linear_speed_);
    RCLCPP_INFO(this->get_logger(), "Angular speed: %.2f rad/s (corners: %.2f)", angular_speed_, corner_angular_speed_);
    RCLCPP_INFO(this->get_logger(), "Target wall distance: %.2f m (20cm)", desired_wall_distance_);
    RCLCPP_INFO(this->get_logger(), "PID Gains: Kp=2.0, Ki=0.05, Kd=1.0 (SMOOTH TUNING)");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "SAFETY: Press Ctrl+C to emergency stop!");
    RCLCPP_INFO(this->get_logger(), "============================================================");
}

// ========== Destructor ==========

RightHandWallFollower::~RightHandWallFollower()
{
    // Send stop command before shutting down
    auto stop_cmd = geometry_msgs::msg::TwistStamped();
    stop_cmd.header.stamp = this->now();
    stop_cmd.header.frame_id = "base_link";
    cmd_vel_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "Robot stopped.");
}

// ========== Helper Methods ==========

float RightHandWallFollower::find_min(const std::vector<float>& ranges, size_t start, size_t end)
{
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
    
    // Front: 0-15 and 345-360 degrees
    float front_min1 = find_min(msg->ranges, 0, 15);
    float front_min2 = find_min(msg->ranges, 345, 360);
    ranges.front = std::min(front_min1, front_min2);
    
    // Right: 250-290 degrees
    ranges.right = find_min(msg->ranges, 250, 290);
    
    // Front-right: 300-330 degrees
    ranges.front_right = find_min(msg->ranges, 300, 330);
    
    // Left: 70-110 degrees
    ranges.left = find_min(msg->ranges, 70, 110);
    
    return ranges;
}

// ========== Main Callback with PID Control ==========

void RightHandWallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Get processed range data
    auto distances = get_scan_ranges(msg);
    
    // Calculate time delta for PID
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_scan_time_).seconds();
    last_scan_time_ = current_time;
    
    // Prevent division by zero and handle first call
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.03;  // Assume ~30Hz if invalid
    }
    
    // Create velocity command
    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.header.stamp = current_time;
    cmd.header.frame_id = "base_link";
    
    // PRIORITY 1: Emergency stop if too close to obstacle
    if (distances.front < min_obstacle_distance_) {
        state_ = "EMERGENCY_STOP";
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = corner_angular_speed_;
        wall_pid_->reset();  // Reset PID when not following wall
        right_turn_counter_ = 0;  // Reset turn sequence counter
        RCLCPP_WARN(this->get_logger(), "EMERGENCY! Obstacle at %.2fm", distances.front);
    }
    // PRIORITY 2: Obstacle ahead AND right wall - CORNER! Turn left SHARP
    else if (distances.front < front_threshold_ && distances.right < side_threshold_) {
        state_ = "CORNER_LEFT";
        cmd.twist.linear.x = 0.0;  // STOP completely - no forward motion for sharp turn
        cmd.twist.angular.z = corner_angular_speed_;
        wall_pid_->reset();  // Reset PID during corner turn
        right_turn_counter_ = 0;  // Reset turn sequence counter
        RCLCPP_INFO(this->get_logger(), "CORNER! Front: %.2fm, Right: %.2fm - SHARP LEFT", 
                   distances.front, distances.right);
        wall_found_ = true;
    }
    // PRIORITY 3: Obstacle ahead - Turn left SHARP
    else if (distances.front < front_threshold_) {
        state_ = "TURNING_LEFT";
        cmd.twist.linear.x = 0.0;  // STOP completely for sharp turn
        cmd.twist.angular.z = corner_angular_speed_;
        wall_pid_->reset();  // Reset PID during turn
        right_turn_counter_ = 0;  // Reset turn sequence counter
        RCLCPP_INFO(this->get_logger(), "Obstacle ahead (%.2fm) - SHARP LEFT", distances.front);
        wall_found_ = true;
    }
    // PRIORITY 4: Obstacle at front-right corner - Adjust left
    else if (distances.front_right < corner_clearance_) {
        state_ = "ADJUSTING_LEFT";
        cmd.twist.linear.x = linear_speed_ * 0.4;
        cmd.twist.angular.z = angular_speed_ * 0.5;
        wall_pid_->reset();  // Reset PID during adjustment
        right_turn_counter_ = 0;  // Reset turn sequence counter
        RCLCPP_INFO(this->get_logger(), "Front-right obstacle (%.2fm) - Adjust left", distances.front_right);
        wall_found_ = true;
    }
    // PRIORITY 5: Wall on right - Follow it using PID CONTROL
    else if (distances.right < side_threshold_) {
        wall_found_ = true;
        right_turn_counter_ = 0;  // Reset counter when wall is detected
        state_ = "FOLLOWING_PID";
        
        // Calculate error: positive = too far, negative = too close
        double error = distances.right - desired_wall_distance_;
        
        // Use PID to calculate angular velocity correction
        double angular_correction = -wall_pid_->calculate(error, dt);  // Negative to turn toward wall
        
        // Set forward speed (reduce if large correction needed)
        double speed_factor = 1.0 - std::min(0.4, std::abs(angular_correction) / angular_speed_);
        cmd.twist.linear.x = linear_speed_ * speed_factor;
        cmd.twist.angular.z = angular_correction;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "PID Control: dist=%.2fm, error=%.3fm, angular=%.2f rad/s", 
            distances.right, error, angular_correction);
    }
    // PRIORITY 6: No wall on right - Turn-Forward sequence
    else {
        wall_pid_->reset();  // Reset PID when no wall detected
        
        // If we've found a wall before, this is an opening - execute turn-forward sequence
        if (wall_found_) {
            // Phase 1: Turn right (first 8 cycles)
            if (right_turn_counter_ < 8) {
                state_ = "TURNING_RIGHT_OPENING";
                cmd.twist.linear.x = 0.0;  // STOP while turning for sharp turn
                cmd.twist.angular.z = -corner_angular_speed_;  // Full speed right turn
                right_turn_counter_++;
                RCLCPP_INFO(this->get_logger(), "Opening! Phase 1: Turning right (count: %d)", right_turn_counter_);
            }
            // Phase 2: Drive forward (cycles 8-20)
            else if (right_turn_counter_ < 20) {
                state_ = "DRIVING_FORWARD_AFTER_TURN";
                cmd.twist.linear.x = linear_speed_;  // Full speed forward
                cmd.twist.angular.z = 0.0;  // No turning, straight ahead
                right_turn_counter_++;
                RCLCPP_INFO(this->get_logger(), "Opening! Phase 2: Driving forward (count: %d)", right_turn_counter_);
            }
            // Phase 3: Turn right again (cycles 20-25)
            else if (right_turn_counter_ < 25) {
                state_ = "TURNING_RIGHT_SECOND";
                cmd.twist.linear.x = 0.0;  // Stop while turning
                cmd.twist.angular.z = -corner_angular_speed_ * 0.7;  // Medium turn
                right_turn_counter_++;
                RCLCPP_INFO(this->get_logger(), "Opening! Phase 3: Turning right again (count: %d)", right_turn_counter_);
            }
            // Phase 4: Drive forward until wall detected (cycles 25+)
            else {
                state_ = "SEEKING_WALL_FORWARD";
                cmd.twist.linear.x = linear_speed_;  // Full speed forward
                cmd.twist.angular.z = 0.0;  // Straight
                right_turn_counter_++;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Opening! Phase 4: Seeking wall forward (count: %d)", right_turn_counter_);
            }
        }
        // If we haven't found a wall yet, search for one
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

// ========== Main Function ==========

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::cout << "\n============================================================\n";
    std::cout << "Wall Follower Active\n";
    std::cout << "============================================================\n";
    std::cout << "SAFETY: Keep robot in clear area\n";
    std::cout << "EMERGENCY STOP: Press Ctrl+C\n";
    std::cout << "PID Controller: Smooth wall following enabled\n";
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
