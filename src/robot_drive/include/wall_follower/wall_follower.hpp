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

#ifndef ROBOT_DRIVE__WALL_FOLLOWER_HPP_
#define ROBOT_DRIVE__WALL_FOLLOWER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RightHandWallFollower : public rclcpp::Node
{
public:
    RightHandWallFollower();  // Constructor - initializes publishers, subscribers, and parameters
    ~RightHandWallFollower(); // Destructor - sends stop command before shutdown

private:
    // ========== Control Parameters ==========

    const double linear_speed_;
    const double angular_speed_;
    const double corner_angular_speed_;

    // ========== Distance Thresholds ==========

    const double desired_wall_distance_;
    const double front_threshold_;
    const double side_threshold_;
    const double min_obstacle_distance_;
    const double corner_clearance_;

    // ========== State Variables ==========

    std::string state_;
    bool wall_found_;
    int right_turn_counter_;

    // ========== ROS 2 Objects ==========

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // ========== Helper Structures ==========

    struct RangeData {
        float front;        // Minimum distance in front zone
        float right;        // Minimum distance in right zone
        float front_right;  // Minimum distance in front-right zone
        float left;         // Minimum distance in left zone
    };

    class PIDController {
    public:
        PIDController(double kp, double ki, double kd, double max_output);
        double calculate(double error, double dt);

        void reset();

    private:
        double kp_;              // Proportional gain
        double ki_;              // Integral gain
        double kd_;              // Derivative gain
        double max_output_;      // Maximum absolute output
        double integral_;        // Accumulated integral term
        double prev_error_;      // Previous error for derivative calculation
        bool first_call_;        // Flag for first calculate() call
    };

    // ========== PID Controller Instance ==========

    std::unique_ptr<PIDController> wall_pid_;
    rclcpp::Time last_scan_time_;

    // ========== Private Methods ==========

    float find_min(const std::vector<float>& ranges, size_t start, size_t end);
    RangeData get_scan_ranges(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif  // ROBOT_DRIVE__WALL_FOLLOWER_HPP_
