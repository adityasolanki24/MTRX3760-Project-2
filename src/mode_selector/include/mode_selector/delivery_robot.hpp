// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: delivery_robot.hpp
// Author(s): Project Team
//
// Delivery robot implementation. Inherits from BaseRobot and adds:
// - Navigation2 integration for SLAM-based navigation
// - Delivery route management
// - Multiple delivery request queue
// - Delivery logging to disk

#ifndef MODE_SELECTOR__DELIVERY_ROBOT_HPP_
#define MODE_SELECTOR__DELIVERY_ROBOT_HPP_

#include "mode_selector/base_robot.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <thread>
#include <mutex>

namespace mode_selector
{

// Forward declarations for Waypoint and DeliveryRoute
struct Waypoint {
    double x, y, yaw;
    std::string name;
    
    Waypoint(double x = 0.0, double y = 0.0, double yaw = 0.0, const std::string& name = "")
        : x(x), y(y), yaw(yaw), name(name) {}
};

struct DeliveryRoute {
    std::string route_name;
    std::vector<Waypoint> waypoints;
    
    DeliveryRoute(const std::string& name = "") : route_name(name) {}
};

/**
 * @class DeliveryRobot
 * @brief Delivery robot with LiDAR and odometry (no camera)
 * 
 * Specialized functionality:
 * - Navigation2-based route execution
 * - Multiple delivery request management
 * - Delivery logging
 */
class DeliveryRobot : public BaseRobot
{
public:
    /**
     * @brief Constructor
     * @param mapping_mode If true, enables mapping mode with SLAM
     */
    explicit DeliveryRobot(bool mapping_mode = false);
    
    /**
     * @brief Destructor - saves delivery log
     */
    ~DeliveryRobot();
    
    // ========== Virtual Interface Implementation ==========
    
    void initialize() override;
    void run() override;
    
    // ========== Specialized Methods ==========
    
    /**
     * @brief Queue a delivery request
     * @param destination_id ArUco tag ID or waypoint identifier
     */
    void queueDeliveryRequest(int destination_id);
    
    /**
     * @brief Navigate to a specific pose using Navigation2
     * @param x X coordinate in map frame
     * @param y Y coordinate in map frame
     * @param yaw Orientation (radians)
     */
    void navigateToPose(double x, double y, double yaw);
    
    /**
     * @brief Check if mapping mode is complete
     * @return true if mapping mode was enabled and map has been saved
     */
    bool isMappingComplete() const { return mapping_mode_enabled_ && map_saved_; }
    
    /**
     * @brief Enable delivery mode after mapping is complete
     * Initializes delivery log and starts connecting to Navigation2
     */
    void enableDeliveryMode();
    
    /**
     * @brief Get the path to the saved map file
     * @return Path to map file (without extension) or empty string if not saved
     */
    std::string getSavedMapPath() const { return saved_map_path_; }

private:
    // ========== Navigation2 Integration ==========
    
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    
    // TF for robot pose in map frame
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // ========== Delivery Management ==========
    
    /// Publisher for target ArUco tag ID
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_tag_pub_;
    
    /// Publisher for delivery status
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr delivery_status_pub_;
    
    /// Publisher for initial pose (for AMCL localization)
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    
    /// Subscriber for navigation status
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
    
    /// Subscriber for RViz2 goal poses
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    
    /// Subscriber for RViz2 clicked points
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    
    // Delivery state
    std::queue<int> pending_deliveries_;
    std::mutex delivery_mutex_;
    int current_target_tag_;
    std::string current_status_;
    int completed_deliveries_;
    
    // Delivery routes
    std::vector<DeliveryRoute> delivery_routes_;
    size_t current_route_index_;
    size_t current_waypoint_index_;
    
    // Home position
    Waypoint home_position_;
    bool home_position_set_;
    
    // State management
    rclcpp::TimerBase::SharedPtr state_timer_;
    bool delivery_in_progress_;
    
    // ========== Logging ==========
    
    std::string delivery_log_file_;
    std::ofstream delivery_log_stream_;
    void initializeDeliveryLog();
    void logDelivery(int tag_id, const std::string& event, const std::string& details);
    std::string getCurrentTimestamp() const;
    
    // ========== Mapping Mode ==========
    
    /// Flag indicating mapping mode is enabled
    bool mapping_mode_enabled_;
    
    /// Mapping start time
    rclcpp::Time mapping_start_time_;
    
    /// Timer to check mapping completion
    rclcpp::TimerBase::SharedPtr mapping_completion_timer_;
    
    /// Timer to connect to Navigation2 after map is saved
    rclcpp::TimerBase::SharedPtr nav2_connect_timer_;
    
    /// Flag indicating map has been saved
    bool map_saved_;
    
    /// Path to the saved map file (without extension)
    std::string saved_map_path_;
    
    /// Flag indicating Navigation2 is ready
    bool nav2_ready_;
    
    /// Check if mapping is complete and save map
    void checkMappingCompletion();
    
    /// Save the current map
    void saveMap();
    
    /// Connect to Navigation2 after map is saved
    void connectToNavigation2();
    
    /// Set initial pose for AMCL using last known position from odometry
    void setInitialPoseFromOdometry();
    
    // ========== Private Methods ==========
    
    // Navigation callbacks
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    // Action callbacks
    void goalResponseCallback(const GoalHandleNav::SharedPtr& goal_handle);
    void feedbackCallback(GoalHandleNav::SharedPtr,
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNav::WrappedResult& result);
    
    // Delivery management
    void navigationStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void stateManagementCallback();
    void processNextDelivery();
    void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& pose);
    
    // Utility
    bool getRobotPoseInMap(double& x, double& y, double& yaw);
    void initializeRoutes();
    geometry_msgs::msg::PoseStamped waypointToPoseStamped(const Waypoint& waypoint);
};

}  // namespace mode_selector

#endif  // MODE_SELECTOR__DELIVERY_ROBOT_HPP_

