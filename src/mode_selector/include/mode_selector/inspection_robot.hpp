// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: inspection_robot.hpp
// Author(s): Project Team
//
// Inspection robot implementation. Inherits from BaseRobot and adds:
// - Camera-based ArUco marker detection for damage sites
// - Wall following algorithm for exploration
// - Low battery homing to charging station (ArUco ID 0)
// - Damage site logging to disk

#ifndef MODE_SELECTOR__INSPECTION_ROBOT_HPP_
#define MODE_SELECTOR__INSPECTION_ROBOT_HPP_

#include "mode_selector/base_robot.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

namespace mode_selector
{

/**
 * @class InspectionRobot
 * @brief Inspection robot with camera, LiDAR, and odometry
 * 
 * Specialized functionality:
 * - Wall following for exploration
 * - ArUco marker detection (ID 1 = damage sites, ID 0 = charging station)
 * - Damage site logging
 * - Low battery homing
 */
class InspectionRobot : public BaseRobot
{
public:
    /**
     * @brief Constructor
     * @param mode Additional mode (e.g., "low_battery_homing", "home_detection", or "wall_following")
     */
    explicit InspectionRobot(const std::string& mode = "wall_following");
    
    /**
     * @brief Destructor - saves damage log
     */
    ~InspectionRobot();
    
    // ========== Virtual Interface Implementation ==========
    
    void initialize() override;
    void run() override;
    
    // ========== Specialized Methods ==========
    
    /**
     * @brief Enable/disable low battery homing mode
     * @param enable True to enable low battery homing
     */
    void enableLowBatteryHoming(bool enable);
    
    /**
     * @brief Enable/disable damage detection mode
     * @param enable True to enable damage detection (ArUco ID 1)
     */
    void enableDamageDetection(bool enable);
    
    // Override battery callback to trigger homing when battery is low
    void onBatteryLevel(const std_msgs::msg::Float32::SharedPtr msg);

private:
    // ========== Operating Mode ==========
    
    std::string operating_mode_;  // "wall_following", "home_detection", "low_battery_homing"
    bool low_battery_homing_enabled_;
    bool damage_detection_enabled_;
    bool homing_active_;
    bool docked_;
    
    // ========== ArUco Detection ==========
    
    /// Subscriber for ArUco marker ID 0 (charging station/home)
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_id0_sub_;
    
    /// Map of subscribers for all ArUco marker IDs (dynamic subscription)
    std::unordered_map<int, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> aruco_subs_;
    
    /// Last detected pose of ArUco ID 0
    geometry_msgs::msg::PoseStamped last_tag0_pose_;
    
    /// Map of last detected poses for all ArUco marker IDs
    std::unordered_map<int, geometry_msgs::msg::PoseStamped> last_tag_poses_;
    
    /// Set of detected ArUco IDs (to avoid logging duplicates immediately)
    std::unordered_set<int> detected_aruco_ids_;
    
    /// Set of ArUco IDs that are currently causing the robot to stop (to avoid multiple stops)
    std::unordered_set<int> stopping_for_ids_;
    
    /// Timer to resume movement after stopping for damage detection
    rclcpp::TimerBase::SharedPtr resume_movement_timer_;
    
    /// Flag indicating ArUco ID 1 has been detected (for backward compatibility)
    bool aruco_id1_detected_;
    
    /// Flag to pause wall following when damage is detected
    bool paused_for_damage_;
    
    /// Resume movement after damage detection pause
    void resumeAfterDamagePause();
    
    /// Generic callback for any ArUco marker ID
    void arucoGenericCallback(int marker_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /// Create subscription for a specific ArUco marker ID
    void subscribeToArucoId(int id);
    
    /// Initialize subscriptions for common ArUco marker IDs (0-20)
    void initializeArucoSubscriptions();
    
    // ========== Wall Following ==========
    
    /// Subscriber for LiDAR scan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    /// Publisher for wall follow enable/disable
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_enable_pub_;
    
    // Wall following parameters
    double linear_speed_;
    double angular_speed_;
    double corner_angular_speed_;
    double desired_wall_distance_;
    double front_threshold_;
    double side_threshold_;
    double min_obstacle_distance_;
    double corner_clearance_;
    double aruco_stop_distance_m_;
    
    // Wall following state
    std::string wall_follow_state_;
    bool wall_found_;
    int right_turn_counter_;
    rclcpp::Time last_scan_time_;
    
    // ========== PID Controller ==========
    
    class PIDController {
    public:
        PIDController(double kp, double ki, double kd, double max_output);
        double calculate(double error, double dt);
        void reset();
    private:
        double kp_, ki_, kd_, max_output_;
        double integral_;
        double prev_error_;
        bool first_call_;
    };
    
    std::unique_ptr<PIDController> wall_pid_;
    
    // ========== Damage Logging ==========
    
    std::string damage_log_file_;
    std::ofstream damage_log_stream_;
    void initializeDamageLog();
    void logDamage(int marker_id, double x, double y, double z);
    std::string getCurrentTimestamp() const;
    
    // ========== Range Data Structure ==========
    
    struct RangeData {
        float front;
        float right;
        float front_right;
        float left;
    };
    
    // ========== Private Methods ==========
    
    // ArUco callbacks
    void arucoId0Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void arucoId1Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    // Wall following
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    RangeData getScanRanges(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    float findMin(const std::vector<float>& ranges, size_t start, size_t end);
    
    // Low battery homing
    void performHomingStep();
    void startHoming();
    rclcpp::TimerBase::SharedPtr homing_timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;  // Timer to continuously stop when docked
    
    // Battery threshold
    float battery_threshold_;
};

}  // namespace mode_selector

#endif  // MODE_SELECTOR__INSPECTION_ROBOT_HPP_



