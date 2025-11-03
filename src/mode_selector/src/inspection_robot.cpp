// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: inspection_robot.cpp
// Author(s): Aditya Solanki
//
// Implementation of InspectionRobot class - camera-based damage detection robot.

#include "mode_selector/inspection_robot.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
#include <thread>
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

namespace mode_selector
{

// ========== PID Controller Implementation ==========

InspectionRobot::PIDController::PIDController(double kp, double ki, double kd, double max_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output),
      integral_(0.0), prev_error_(0.0), first_call_(true)
{
}

double InspectionRobot::PIDController::calculate(double error, double dt)
{
    if (first_call_) {
        prev_error_ = error;
        first_call_ = false;
        return kp_ * error;
    }
    
    double p_term = kp_ * error;
    
    integral_ += error * dt;
    double max_integral = max_output_ / (ki_ + 0.001);
    integral_ = std::clamp(integral_, -max_integral, max_integral);
    double i_term = ki_ * integral_;
    
    double derivative = (error - prev_error_) / dt;
    double d_term = kd_ * derivative;
    
    double output = p_term + i_term + d_term;
    output = std::clamp(output, -max_output_, max_output_);
    
    prev_error_ = error;
    return output;
}

void InspectionRobot::PIDController::reset()
{
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_call_ = true;
}

// ========== InspectionRobot Implementation ==========

InspectionRobot::InspectionRobot(const std::string& mode)
    : BaseRobot("inspection_robot"),
      operating_mode_(mode),
      low_battery_homing_enabled_(mode == "low_battery_homing"),
      damage_detection_enabled_(mode == "home_detection"),
      homing_active_(false),
      docked_(false),
      aruco_id1_detected_(false),  // Reused: true = home/damage detected, false = not detected
      linear_speed_(0.15),
      angular_speed_(0.25),
      corner_angular_speed_(0.6),
      desired_wall_distance_(0.15),
      front_threshold_(0.35),
      side_threshold_(0.35),
      min_obstacle_distance_(0.25),
      corner_clearance_(0.35),
      aruco_stop_distance_m_(0.30),
      wall_follow_state_("FINDING_WALL"),
      wall_found_(false),
      right_turn_counter_(0),
      last_scan_time_(this->now()),
      damage_log_file_("/tmp/inspection_damage_log.txt"),
      battery_threshold_(20.0f),
      paused_for_damage_(false)
{
    wall_pid_ = std::make_unique<PIDController>(0.70, 0.05, 1.75, angular_speed_);
    
    RCLCPP_INFO(this->get_logger(), "InspectionRobot created with mode: %s", mode.c_str());
}

InspectionRobot::~InspectionRobot()
{
    if (damage_log_stream_.is_open()) {
        damage_log_stream_.close();
    }
    stopRobot();
    RCLCPP_INFO(this->get_logger(), "InspectionRobot shutting down");
}

void InspectionRobot::initialize()
{
    // QoS for LiDAR (best effort)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    
    // Override battery subscriber to use our custom callback
    battery_sub_.reset(); // Remove base class subscription
    
    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos,
        std::bind(&InspectionRobot::scanCallback, this, std::placeholders::_1));
    
    // Create our own battery subscriber with custom callback
    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/battery/level", 10,
        std::bind(&InspectionRobot::onBatteryLevel, this, std::placeholders::_1));
    
    // Always subscribe to ArUco topics if damage detection or homing is enabled
    if (damage_detection_enabled_ || low_battery_homing_enabled_) {
        // Subscribe to ID 0 for special handling (home/charging station)
        aruco_id0_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco/pose/id_0", 10,
            std::bind(&InspectionRobot::arucoId0Callback, this, std::placeholders::_1));
        
        if (damage_detection_enabled_) {
            if (operating_mode_ == "home_detection") {
                RCLCPP_INFO(this->get_logger(), "Subscribed to /aruco/pose/id_0 for home detection (will stop at ID 0)");
                RCLCPP_INFO(this->get_logger(), "Initializing subscriptions for all ArUco marker IDs (0-50) for damage logging");
            } else {
                RCLCPP_INFO(this->get_logger(), "Subscribed to /aruco/pose/id_0 for home detection");
                RCLCPP_INFO(this->get_logger(), "Initializing subscriptions for all ArUco marker IDs (0-50) for damage detection");
            }
        }
        
        if (low_battery_homing_enabled_) {
            RCLCPP_INFO(this->get_logger(), "Subscribed to /aruco/pose/id_0 for charging station");
        }
        
        // Initialize subscriptions for common ArUco marker IDs (0-50)
        // This allows logging of ALL detected markers, not just ID 1
        initializeArucoSubscriptions();
    }
    
    // Publisher for wall follow enable
    wall_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/wall_follow_enable", 10);
    
    // Initialize damage log
    if (damage_detection_enabled_) {
        initializeDamageLog();
    }
    
    RCLCPP_INFO(this->get_logger(), "InspectionRobot initialized");
    RCLCPP_INFO(this->get_logger(), "  Mode: %s", operating_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Damage detection: %s", 
                damage_detection_enabled_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Low battery homing: %s", 
                low_battery_homing_enabled_ ? "enabled" : "disabled");
}

void InspectionRobot::run()
{
    // Main loop runs via ROS 2 callbacks
    // Battery monitoring and homing trigger is handled in batteryCallback override
    // The robot operates through scan_callback and ArUco callbacks
}

void InspectionRobot::enableLowBatteryHoming(bool enable)
{
    low_battery_homing_enabled_ = enable;
}

void InspectionRobot::enableDamageDetection(bool enable)
{
    damage_detection_enabled_ = enable;
}

// Override battery callback to trigger homing when battery < 20%
void InspectionRobot::onBatteryLevel(const std_msgs::msg::Float32::SharedPtr msg)
{
    // Update battery level (call base class implementation)
    battery_level_ = msg->data;
    
    // Check if low battery homing should be triggered
    if (low_battery_homing_enabled_ && !homing_active_ && !docked_ && 
        battery_level_ < battery_threshold_) {
        RCLCPP_WARN(this->get_logger(), 
            "Battery low (%.2f%% < %.2f%%). Initiating homing to charging station...", 
            battery_level_, battery_threshold_);
        startHoming();
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Battery level: %.2f%%", battery_level_);
}

// ========== Helper Methods ==========

float InspectionRobot::findMin(const std::vector<float>& ranges, size_t start, size_t end)
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

InspectionRobot::RangeData InspectionRobot::getScanRanges(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RangeData ranges;
    ranges.front = std::min(
        findMin(msg->ranges, 0, 15),
        findMin(msg->ranges, 345, 360));
    ranges.right = findMin(msg->ranges, 250, 290);
    ranges.front_right = findMin(msg->ranges, 300, 330);
    ranges.left = findMin(msg->ranges, 70, 110);
    return ranges;
}

// ========== ArUco Callbacks ==========

void InspectionRobot::arucoId0Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    last_tag0_pose_ = *msg;
    
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    double distance = std::sqrt(x * x + y * y + z * z);
    
    // Handle low battery homing mode - stop when within threshold (like home_detection)
    if (homing_active_) {
        // Log distance periodically for debugging
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Low battery homing: ArUco ID 0 distance=%.3f m, stop_threshold=%.2f m",
            distance, aruco_stop_distance_m_);
        
        // Stop when within threshold (same as home_detection behavior)
        if (distance <= aruco_stop_distance_m_) {
            // Set flags IMMEDIATELY to prevent any further movement
            homing_active_ = false;
            docked_ = true;
            
            // Cancel homing timer FIRST to stop navigation commands
            if (homing_timer_) {
                homing_timer_->cancel();
                homing_timer_.reset();
            }
            
            // Stop robot immediately - publish multiple times rapidly
            for (int i = 0; i < 20; i++) {
                stopRobot();
                rclcpp::sleep_for(std::chrono::milliseconds(5));
            }
            
            // Create a persistent timer to continuously publish stop commands
            if (!stop_timer_) {
                stop_timer_ = this->create_wall_timer(
                    100ms,  // 10 Hz - continuously stop
                    [this]() {
                        if (docked_) {
                            this->stopRobot();
                        } else {
                            // Cancel timer if not docked anymore
                            if (stop_timer_) {
                                stop_timer_->cancel();
                                stop_timer_.reset();
                            }
                        }
                    });
            }
            
            RCLCPP_WARN(this->get_logger(), "============================================================");
            RCLCPP_WARN(this->get_logger(), "CHARGING STATION REACHED - ArUco Marker ID 0");
            RCLCPP_WARN(this->get_logger(), "Distance: %.3f m (threshold: %.2f m)", distance, aruco_stop_distance_m_);
            RCLCPP_WARN(this->get_logger(), "Position: [%.3f, %.3f, %.3f]", x, y, z);
            RCLCPP_WARN(this->get_logger(), "Robot docked and stopped - all movement halted");
            RCLCPP_WARN(this->get_logger(), "============================================================");
        }
        return;
    }
    
    // Handle home detection mode (stop at ArUco ID 0, but don't log as damage)
    if (damage_detection_enabled_ && !aruco_id1_detected_) {
        // Log distance periodically for debugging
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "ArUco ID 0 (home) detected: distance=%.3f m, stop_threshold=%.2f m",
            distance, aruco_stop_distance_m_);
        
        // Stop when within threshold (but don't log as damage)
        if (distance <= aruco_stop_distance_m_) {
            aruco_id1_detected_ = true; // Reuse this flag to indicate "home detected"
            
            RCLCPP_WARN(this->get_logger(), "============================================================");
            RCLCPP_WARN(this->get_logger(), "HOME DETECTED - ArUco Marker ID 0");
            RCLCPP_WARN(this->get_logger(), "Distance: %.3f m (threshold: %.2f m)", distance, aruco_stop_distance_m_);
            RCLCPP_WARN(this->get_logger(), "Position: [%.3f, %.3f, %.3f]", x, y, z);
            RCLCPP_WARN(this->get_logger(), "STOPPING ROBOT AT HOME");
            RCLCPP_WARN(this->get_logger(), "============================================================");
            
            stopRobot();
            // NOTE: Do NOT log this as damage - it's home detection, not damage
        }
    }
}

void InspectionRobot::arucoId1Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Forward to generic callback (for backward compatibility)
    arucoGenericCallback(1, msg);
}

// ========== Wall Following ==========

void InspectionRobot::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // PRIORITY 0: If docked (or homing completed), stop completely
    if (docked_ || (homing_active_ && docked_)) {
        wall_follow_state_ = "STOPPED_DOCKED";
        // Continuously publish stop command to ensure robot stops
        stopRobot();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "STOPPED: Docked at charging station (ArUco ID 0). Robot halted.");
        return;
    }
    
    // PRIORITY 0.3: If paused for damage detection, stay stopped
    if (paused_for_damage_) {
        wall_follow_state_ = "PAUSED_FOR_DAMAGE";
        stopRobot();
        return;
    }
    
    // PRIORITY 0.5: If homing active and NOT docked, navigate to ArUco ID 0
    if (homing_active_ && !docked_) {
        if (!last_tag0_pose_.header.frame_id.empty()) {
            // We have ArUco ID 0 pose - navigation is handled in performHomingStep() timer
            // Don't publish commands here to avoid conflicts - let timer handle it
            // Just check if we should continue or stop
            double x = last_tag0_pose_.pose.position.x;
            double y = last_tag0_pose_.pose.position.y;
            double z = last_tag0_pose_.pose.position.z;
            double distance = std::sqrt(x * x + y * y + z * z);
            
            if (distance <= aruco_stop_distance_m_) {
                // Close enough - stop immediately
                docked_ = true;
                stopRobot();
                return;
            }
            
            // Navigation handled by timer, but publish stop to avoid wall following commands
            // The timer will handle navigation
            return;
        } else {
            // No ArUco ID 0 yet - continue wall following until detected
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Homing active: Searching for charging station (ArUco ID 0)...");
        }
    }
    
    // PRIORITY 1: Stop if home detected (for home_detection mode) or damage detected (for wall_following mode)
    if (damage_detection_enabled_ && aruco_id1_detected_) {
        // In home_detection mode: only stop for ID 0 (home), not ID 1 (damage)
        if (operating_mode_ == "home_detection") {
            // Only stop if it's ID 0 (home), not ID 1 (damage)
            // ID 1 is logged but doesn't stop the robot in home_detection mode
            if (!last_tag0_pose_.header.frame_id.empty() && 
                std::sqrt(last_tag0_pose_.pose.position.x * last_tag0_pose_.pose.position.x +
                         last_tag0_pose_.pose.position.y * last_tag0_pose_.pose.position.y +
                         last_tag0_pose_.pose.position.z * last_tag0_pose_.pose.position.z) <= aruco_stop_distance_m_) {
                wall_follow_state_ = "STOPPED_ARUCO_DETECTED";
                stopRobot();
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "STOPPED: Home detected (ArUco ID 0). Wall following disabled. Robot halted.");
                return;
            }
            // If it's ID 1, don't stop (already logged in callback)
            return;
        } else {
            // Wall following mode: stop for both ID 0 and ID 1
            wall_follow_state_ = "STOPPED_ARUCO_DETECTED";
            stopRobot();
            // Check which one triggered
            if (!last_tag0_pose_.header.frame_id.empty() && 
                std::sqrt(last_tag0_pose_.pose.position.x * last_tag0_pose_.pose.position.x +
                         last_tag0_pose_.pose.position.y * last_tag0_pose_.pose.position.y +
                         last_tag0_pose_.pose.position.z * last_tag0_pose_.pose.position.z) <= aruco_stop_distance_m_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "STOPPED: Home detected (ArUco ID 0). Wall following disabled. Robot halted.");
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "STOPPED: ArUco marker ID 1 detected. Wall following disabled. Robot halted.");
            }
            return;
        }
    }
    
    // Check if we're receiving ArUco messages (for debugging)
    if (damage_detection_enabled_ && last_tag0_pose_.header.frame_id.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Waiting for ArUco ID 0 messages on /aruco/pose/id_0 (make sure aruco_pose_node is running)");
    }
    
    auto distances = getScanRanges(msg);
    
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_scan_time_).seconds();
    last_scan_time_ = current_time;
    
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.03;
    }
    
    // Priority-based wall following (simplified version)
    if (distances.front < min_obstacle_distance_) {
        wall_follow_state_ = "EMERGENCY_STOP";
        publishVelocity(0.0, corner_angular_speed_);
        wall_pid_->reset();
    }
    else if (distances.front < front_threshold_ && distances.right < side_threshold_) {
        wall_follow_state_ = "CORNER_LEFT";
        publishVelocity(0.0, corner_angular_speed_);
        wall_pid_->reset();
        wall_found_ = true;
    }
    else if (distances.front < front_threshold_) {
        wall_follow_state_ = "TURNING_LEFT";
        publishVelocity(0.0, corner_angular_speed_);
        wall_pid_->reset();
        wall_found_ = true;
    }
    else if (distances.right < side_threshold_) {
        wall_found_ = true;
        wall_follow_state_ = "FOLLOWING_PID";
        
        double error = distances.right - desired_wall_distance_;
        if (std::fabs(error) < 0.04) error = 0.0;
        
        double angular_correction = -wall_pid_->calculate(error, dt);
        double speed_factor = 1.0 - std::min(0.6, std::abs(angular_correction) / angular_speed_);
        speed_factor = std::clamp(speed_factor, 0.25, 1.0);
        
        publishVelocity(linear_speed_ * speed_factor, angular_correction);
    }
    else {
        wall_pid_->reset();
        if (wall_found_) {
            // Opening detected - turn right sequence
            if (right_turn_counter_ < 8) {
                wall_follow_state_ = "TURNING_RIGHT";
                publishVelocity(0.0, -corner_angular_speed_);
                right_turn_counter_++;
            }
            else if (right_turn_counter_ < 20) {
                wall_follow_state_ = "DRIVING_FORWARD";
                publishVelocity(linear_speed_, 0.0);
                right_turn_counter_++;
            }
            else if (right_turn_counter_ < 25) {
                wall_follow_state_ = "TURNING_RIGHT_SECOND";
                publishVelocity(0.0, -corner_angular_speed_ * 0.7);
                right_turn_counter_++;
            }
            else {
                wall_follow_state_ = "SEEKING_WALL";
                publishVelocity(linear_speed_, 0.0);
                right_turn_counter_++;
            }
        }
        else {
            wall_follow_state_ = "FINDING_WALL";
            publishVelocity(linear_speed_ * 0.7, -angular_speed_ * 0.5);
            right_turn_counter_ = 0;
        }
    }
}

// ========== Low Battery Homing ==========

void InspectionRobot::startHoming()
{
    homing_active_ = true;
    docked_ = false;  // Reset docked flag when starting new homing
    
    // Stop wall follower node if running
    system("pkill -f right_hand_wall_follower");
    
    // Disable wall following
    std_msgs::msg::Bool disable_msg;
    disable_msg.data = false;
    wall_enable_pub_->publish(disable_msg);
    
    // Stop robot immediately
    stopRobot();
    
    // Publish stop command multiple times to ensure it takes effect
    for (int i = 0; i < 5; i++) {
        stopRobot();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Start homing timer
    homing_timer_ = this->create_wall_timer(
        200ms,
        std::bind(&InspectionRobot::performHomingStep, this));
    
    RCLCPP_INFO(this->get_logger(), "Homing started. Searching for ArUco ID 0...");
}

void InspectionRobot::performHomingStep()
{
    // PRIORITY: If docked, stop and cancel timer
    if (docked_) {
        stopRobot();
        if (homing_timer_) {
            homing_timer_->cancel();
            homing_timer_.reset();
        }
        return;
    }
    
    if (!homing_active_) {
        // Cancel timer if homing is not active
        if (homing_timer_) {
            homing_timer_->cancel();
        }
        return;
    }
    
    if (last_tag0_pose_.header.frame_id.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "Searching for charging station (ArUco ID 0)...");
        return;
    }
    
    double x = last_tag0_pose_.pose.position.x;
    double y = last_tag0_pose_.pose.position.y;
    double z = last_tag0_pose_.pose.position.z;
    double distance = std::sqrt(x * x + y * y + z * z);
    
    // Stop when within threshold (same as home_detection)
    // Note: This check is also done in arucoId0Callback, but keep it here as backup
    if (distance <= aruco_stop_distance_m_) {
        homing_active_ = false;
        docked_ = true;
        
        // Stop immediately
        for (int i = 0; i < 10; i++) {
            stopRobot();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Cancel timer
        if (homing_timer_) {
            homing_timer_->cancel();
            homing_timer_.reset();
        }
        
        RCLCPP_WARN(this->get_logger(), "Reached charging station at distance %.2f m", distance);
        return;
    }
    
    // Only navigate if NOT docked
    if (docked_) {
        stopRobot();
        return;
    }
    
    // Navigate towards ArUco ID 0
    // Use similar control as original, but check all dimensions
    double linear_vel = 0.1;
    double angular_vel = -0.6 * std::atan2(x, z);
    
    // Only publish velocity if we're still homing and not docked
    if (homing_active_ && !docked_) {
        publishVelocity(linear_vel, angular_vel);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Homing to charging station: distance=%.3f m, ang_z=%.2f", distance, angular_vel);
    } else {
        stopRobot();
    }
}

// ========== Damage Logging ==========

void InspectionRobot::initializeDamageLog()
{
    damage_log_stream_.open(damage_log_file_, std::ios::out);
    if (damage_log_stream_.is_open()) {
        damage_log_stream_ << "==========================================\n";
        damage_log_stream_ << "INSPECTION ROBOT - DAMAGE LOG\n";
        damage_log_stream_ << "==========================================\n";
        damage_log_stream_ << "System: Camera-based ArUco Detection\n";
        damage_log_stream_ << "Log Started: " << getCurrentTimestamp() << "\n";
        damage_log_stream_ << "==========================================\n\n";
        damage_log_stream_.flush();
        RCLCPP_INFO(this->get_logger(), "Damage log initialized: %s", damage_log_file_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create damage log file!");
    }
}

void InspectionRobot::logDamage(int marker_id, double x, double y, double z)
{
    if (!damage_log_stream_.is_open()) return;
    
    double distance = std::sqrt(x*x + y*y + z*z);
    
    // Format: "Crack [id] detected at coordinates"
    damage_log_stream_ << "Crack [" << marker_id << "] detected at coordinates (" 
                       << std::fixed << std::setprecision(3)
                       << x << ", " << y << ", " << z << ")\n";
    damage_log_stream_ << "  Distance: " << std::fixed << std::setprecision(3) << distance << " m\n";
    damage_log_stream_ << "  Timestamp: " << getCurrentTimestamp() << "\n";
    damage_log_stream_ << "  Robot Position: (" << std::fixed << std::setprecision(3)
                       << robot_x_ << ", " << robot_y_ << ")\n";
    damage_log_stream_ << "------------------------------------------\n\n";
    damage_log_stream_.flush();
    
    RCLCPP_INFO(this->get_logger(), "Crack [%d] logged at coordinates (%.3f, %.3f, %.3f)", 
                marker_id, x, y, z);
}

std::string InspectionRobot::getCurrentTimestamp() const
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

// ========== Dynamic ArUco Subscription ==========

void InspectionRobot::initializeArucoSubscriptions()
{
    // Subscribe to common ArUco marker IDs (1-50)
    // Note: ID 0 is already subscribed separately for special handling
    for (int id = 1; id <= 50; id++) {
        subscribeToArucoId(id);
    }
    RCLCPP_INFO(this->get_logger(), "Initialized subscriptions for ArUco marker IDs 1-50");
}

void InspectionRobot::subscribeToArucoId(int id)
{
    // Skip ID 0 (handled separately)
    if (id == 0) return;
    
    // Don't subscribe twice
    if (aruco_subs_.find(id) != aruco_subs_.end()) {
        return;
    }
    
    std::string topic = "/aruco/pose/id_" + std::to_string(id);
    
    // Create subscription with lambda that captures the marker ID
    auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 10,
        [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->arucoGenericCallback(id, msg);
        });
    
    aruco_subs_[id] = sub;
    RCLCPP_DEBUG(this->get_logger(), "Subscribed to %s for damage detection", topic.c_str());
}

void InspectionRobot::arucoGenericCallback(int marker_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Store last pose
    last_tag_poses_[marker_id] = *msg;
    
    if (!damage_detection_enabled_) {
        RCLCPP_DEBUG(this->get_logger(), "ArUco ID %d received but damage detection disabled", marker_id);
        return;
    }
    
    // Skip ID 0 (handled by arucoId0Callback)
    if (marker_id == 0) {
        return;
    }
    
    // Skip if we're already stopped for this marker (avoid multiple stops)
    if (stopping_for_ids_.find(marker_id) != stopping_for_ids_.end()) {
        return;
    }
    
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    double distance = std::sqrt(x * x + y * y + z * z);
    
    // Check if marker is within detection threshold
    if (distance <= aruco_stop_distance_m_) {
        // Avoid duplicate logging (check if already detected)
        bool should_stop_and_log = false;
        
        if (detected_aruco_ids_.find(marker_id) == detected_aruco_ids_.end()) {
            // First time detecting this marker
            should_stop_and_log = true;
            detected_aruco_ids_.insert(marker_id);
        } else {
            // Check if marker moved significantly (re-detect)
            auto it = last_tag_poses_.find(marker_id);
            if (it != last_tag_poses_.end()) {
                double old_x = it->second.pose.position.x;
                double old_y = it->second.pose.position.y;
                double old_z = it->second.pose.position.z;
                double old_distance = std::sqrt(old_x * old_x + old_y * old_y + old_z * old_z);
                if (std::abs(distance - old_distance) > 0.1) {
                    should_stop_and_log = true;
                }
            }
        }
        
        if (should_stop_and_log) {
            // Mark that we're stopping for this ID
            stopping_for_ids_.insert(marker_id);
            
            // Stop robot immediately
            paused_for_damage_ = true;
            stopRobot();
            
            RCLCPP_WARN(this->get_logger(), "============================================================");
            RCLCPP_WARN(this->get_logger(), "Crack [%d] detected at coordinates (%.3f, %.3f, %.3f)", 
                       marker_id, x, y, z);
            RCLCPP_WARN(this->get_logger(), "Distance: %.3f m (threshold: %.2f m)", distance, aruco_stop_distance_m_);
            RCLCPP_WARN(this->get_logger(), "STOPPING FOR 5 SECONDS TO LOG DAMAGE SITE");
            RCLCPP_WARN(this->get_logger(), "============================================================");
            
            // Log damage site
            logDamage(marker_id, x, y, z);
            
            // For ID 1, also set the backward compatibility flag
            if (marker_id == 1) {
                aruco_id1_detected_ = true;
            }
            
            // Create timer to resume after 5 seconds
            resume_movement_timer_ = this->create_wall_timer(
                5000ms,  // 5 seconds
                [this, marker_id]() {
                    // Remove from stopping set
                    stopping_for_ids_.erase(marker_id);
                    
                    // Resume movement
                    paused_for_damage_ = false;
                    
                    RCLCPP_INFO(this->get_logger(), "Resuming wall following after damage detection pause (ArUco ID %d)", marker_id);
                    
                    // Cancel timer (one-shot)
                    if (resume_movement_timer_) {
                        resume_movement_timer_->cancel();
                        resume_movement_timer_.reset();
                    }
                });
        }
    }
}

void InspectionRobot::resumeAfterDamagePause()
{
    paused_for_damage_ = false;
    RCLCPP_INFO(this->get_logger(), "Resuming movement after damage detection pause");
}

}  // namespace mode_selector

