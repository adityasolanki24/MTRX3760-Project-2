// MTRX3760 2025 Project 2: Warehouse Robot DevKit 
// File: <filename> 
// Author(s): Aditya Solanki
// Homedetion Algorithm to return turtlebot to tag0 when job is complete


#ifndef HOME_DETECTION_HPP_
#define HOME_DETECTION_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/**
 * @class RightHandWallFollowerAruco
 * @brief Implements right-hand wall following algorithm using LiDAR data with ArUco marker detection
 * 
 * This node subscribes to laser scan data and ArUco pose messages, and publishes velocity commands
 * to follow the right wall at a target distance. Stops when ArUco marker ID 1 is detected.
 */
class RightHandWallFollowerAruco : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes publishers, subscribers, and parameters
     */
    RightHandWallFollowerAruco();

    /**
     * @brief Destructor - sends stop command before shutdown
     */
    ~RightHandWallFollowerAruco();

private:
    // ========== Control Parameters ==========
    
    /// Linear speed for forward motion (m/s)
    const double linear_speed_;
    
    /// Angular speed for normal turns (rad/s)
    const double angular_speed_;
    
    /// Angular speed for sharp corner turns (rad/s)
    const double corner_angular_speed_;
    
    // ========== Distance Thresholds ==========
    
    /// Target distance from right wall (m)
    const double desired_wall_distance_;
    
    /// Stop if front obstacle closer than this (m)
    const double front_threshold_;
    
    /// Detect wall presence threshold (m)
    const double side_threshold_;
    
    /// Emergency stop distance (m)
    const double min_obstacle_distance_;
    
    /// Front-right corner clearance (m)
    const double corner_clearance_;

    /// Stop when ArUco ID 1 is within this distance (m)
    double aruco_stop_distance_m_;
    
    // ========== State Variables ==========
    
    /// Current state of the robot
    std::string state_;
    
    /// Flag indicating if a wall has been found
    bool wall_found_;
    
    /// Counter for tracking right turn and forward motion sequence
    int right_turn_counter_;
    
    /// Flag indicating if ArUco marker ID 1 has been detected
    bool aruco_id_1_detected_;
    
    // ========== ROS 2 Objects ==========
    
    /// Publisher for velocity commands
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    
    /// Subscriber for laser scan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    /// Subscriber for ArUco marker ID 1 pose
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_id1_sub_;
    
    // ========== Helper Structures ==========
    
    /**
     * @struct RangeData
     * @brief Stores processed range data from different LiDAR zones
     */
    struct RangeData {
        float front;        ///< Minimum distance in front zone
        float right;        ///< Minimum distance in right zone
        float right_avg;    ///< Average distance in right zone (unused)
        float front_right;  ///< Minimum distance in front-right zone
        float left;         ///< Minimum distance in left zone
    };
    
    /**
     * @class PIDController
     * @brief Simple PID controller for smooth wall following
     * 
     * Calculates control output based on proportional, integral, and derivative terms:
     * output = Kp * error + Ki * integral + Kd * derivative
     */
    class PIDController {
    public:
        /**
         * @brief Constructor
         * @param kp Proportional gain
         * @param ki Integral gain
         * @param kd Derivative gain
         * @param max_output Maximum absolute output value
         */
        PIDController(double kp, double ki, double kd, double max_output);
        
        /**
         * @brief Calculate PID control output
         * @param error Current error (setpoint - measured_value)
         * @param dt Time delta since last update (seconds)
         * @return Control output (clamped to max_output)
         */
        double calculate(double error, double dt);
        
        /**
         * @brief Reset integral term (call when starting new control sequence)
         */
        void reset();
        
    private:
        double kp_;              ///< Proportional gain
        double ki_;              ///< Integral gain
        double kd_;              ///< Derivative gain
        double max_output_;      ///< Maximum absolute output
        double integral_;        ///< Accumulated integral term
        double prev_error_;      ///< Previous error for derivative calculation
        bool first_call_;        ///< Flag for first calculate() call
    };
    
    // ========== PID Controller Instance ==========
    
    /// PID controller for wall following
    std::unique_ptr<PIDController> wall_pid_;
    
    /// Last scan timestamp for PID dt calculation
    rclcpp::Time last_scan_time_;
    
    // ========== Private Methods ==========
    
    /**
     * @brief Find minimum valid distance in a range of LiDAR readings
     * @param ranges Vector of LiDAR range measurements
     * @param start Start index in the ranges vector
     * @param end End index in the ranges vector
     * @return Minimum valid distance, or 3.5m if no valid readings
     */
    float find_min(const std::vector<float>& ranges, size_t start, size_t end);
    
    /**
     * @brief Process laser scan and extract minimum distances for each zone
     * @param msg Shared pointer to LaserScan message
     * @return RangeData structure with processed distances
     */
    RangeData get_scan_ranges(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * @brief Callback function for laser scan messages
     * 
     * Implements the priority-based wall following algorithm with PID control:
     * 1. Stop if ArUco ID 1 detected
     * 2. Emergency stop if too close to front obstacle
     * 3. Corner turn if front and right walls detected
     * 4. Left turn if front wall detected
     * 5. Adjust left if front-right obstacle detected
     * 6. Follow right wall at target distance using PID control
     * 7. Turn right if no wall on right
     * 
     * PID controller smoothly adjusts angular velocity based on distance error.
     * 
     * @param msg Shared pointer to LaserScan message
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * @brief Callback function for ArUco marker ID 1 pose messages
     * 
     * Sets the aruco_id_1_detected_ flag when marker ID 1 is detected.
     * Once detected, the robot will stop wall following.
     * 
     * @param msg Shared pointer to PoseStamped message
     */
    void aruco_id1_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

#endif  // HOME_DETECTION_HPP_

