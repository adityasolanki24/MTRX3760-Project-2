// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: maze_explorer.hpp
// Author(s): Aditya Solanki
//
// Tremaux Algorithm-based maze exploration with topological mapping.
// Systematically explores maze, detects intersections, and builds a map
// while avoiding loops using path marking strategy.

#ifndef MAZE_EXPLORER_HPP_
#define MAZE_EXPLORER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/**
 * @class MazeExplorer
 * @brief Implements Tremaux algorithm for systematic maze exploration
 * 
 * This node:
 * - Detects intersections using LiDAR analysis
 * - Maintains topological map of intersections and paths
 * - Marks paths to avoid loops (Tremaux algorithm)
 * - Saves map to disk in human-readable format
 */
class MazeExplorer : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes publishers, subscribers, and parameters
     */
    MazeExplorer();

    /**
     * @brief Destructor - saves map and stops robot
     */
    ~MazeExplorer();

private:
    // ========== Exploration States ==========
    enum class ExplorationState {
        FINDING_WALL,        // Initial state: search for first wall
        FOLLOWING_WALL,      // Following wall toward intersection
        AT_INTERSECTION,     // Detected intersection, deciding next action
        TURNING,             // Executing turn
        BACKTRACKING,        // Returning along marked path
        EXPLORATION_COMPLETE // Finished exploring
    };

    // ========== Path Direction Constants ==========
    enum class Direction {
        FRONT = 0,
        LEFT = 1,
        RIGHT = 2,
        BACK = 3,
        NONE = -1
    };

    // ========== Intersection Node Structure ==========
    struct IntersectionNode {
        int id;
        double x, y;                    // Position in odom frame
        double yaw;                     // Orientation when entered
        std::vector<Direction> paths;   // Available directions
        std::unordered_map<Direction, int> path_marks; // Marks per direction (Tremaux)
        std::unordered_map<Direction, int> connections; // Connected node IDs
        bool visited;
        int visit_count;
        
        IntersectionNode() : id(-1), x(0), y(0), yaw(0), visited(false), visit_count(0) {}
    };

    // ========== Path Segment Structure ==========
    struct PathSegment {
        int from_node;
        int to_node;
        Direction direction;
        std::vector<std::pair<double, double>> trajectory; // Path waypoints
    };

    // ========== Control Parameters ==========
    const double linear_speed_;
    const double angular_speed_;
    const double turn_duration_;
    
    // ========== Distance Thresholds ==========
    const double front_threshold_;      // Minimum distance for front path
    const double side_threshold_;       // Minimum distance for side paths
    const double wall_follow_distance_; // Distance to maintain from wall
    const double intersection_distance_; // Distance threshold for intersection detection
    const double min_safe_distance_;    // Emergency stop distance (critical obstacle)
    
    // ========== State Variables ==========
    ExplorationState state_;
    Direction current_heading_;
    int current_node_id_;
    int next_node_id_;
    bool exploration_complete_;
    
    // ========== Mapping Data ==========
    std::unordered_map<int, IntersectionNode> map_nodes_;
    std::vector<PathSegment> map_paths_;
    std::vector<std::pair<double, double>> current_path_trajectory_;
    
    // ========== Intersection Detection ==========
    struct AvailablePaths {
        bool front;
        bool left;
        bool right;
        bool back;
        double front_dist;
        double left_dist;
        double right_dist;
        double front_left_dist;   // Front-left corner distance
        double front_right_dist;  // Front-right corner distance
        
        // Default constructor with safe values
        AvailablePaths() 
            : front(false), left(false), right(false), back(false),
              front_dist(3.5), left_dist(3.5), right_dist(3.5),
              front_left_dist(3.5), front_right_dist(3.5) {}
    };
    
    AvailablePaths last_detected_paths_;
    bool intersection_detected_;
    
    // ========== Odometry Tracking ==========
    double robot_x_, robot_y_, robot_yaw_;
    double last_intersection_x_, last_intersection_y_;
    double distance_since_intersection_;
    
    // ========== Turn Control ==========
    Direction pending_turn_;
    rclcpp::Time turn_start_time_;
    bool turning_;
    double turn_angle_target_;
    double turn_angle_current_;
    
    // ========== ROS 2 Objects ==========
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // ========== Scan Data Flag ==========
    bool scan_received_;  // Flag to track if we've received scan data
    
    // ========== File Output ==========
    std::string map_file_path_;
    
    // ========== Helper Methods ==========
    
    /**
     * @brief Process laser scan to detect available paths
     */
    AvailablePaths detectAvailablePaths(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    /**
     * @brief Check if robot is at an intersection
     */
    bool isAtIntersection(const AvailablePaths& paths);
    
    /**
     * @brief Find minimum valid distance in LiDAR ranges
     */
    float findMinDistance(const std::vector<float>& ranges, 
                         double angle_start, double angle_end,
                         double angle_min, double angle_increment);
    
    /**
     * @brief Convert yaw angle to Direction enum
     */
    Direction yawToDirection(double yaw);
    
    /**
     * @brief Get relative direction based on current heading
     */
    Direction getRelativeDirection(Direction absolute_dir);
    
    /**
     * @brief Get opposite direction
     */
    Direction getOppositeDirection(Direction dir);
    
    /**
     * @brief Choose next path at intersection using Tremaux algorithm
     */
    Direction chooseNextPath(IntersectionNode& node);
    
    /**
     * @brief Mark a path at an intersection
     */
    void markPath(IntersectionNode& node, Direction dir);
    
    /**
     * @brief Create or update intersection node
     */
    int createIntersectionNode(double x, double y, double yaw, const AvailablePaths& paths);
    
    /**
     * @brief Follow wall toward next intersection
     */
    void followWall(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    /**
     * @brief Execute turn to specified direction
     */
    void executeTurn(Direction target_direction);
    
    /**
     * @brief Normalize angle to [-π, π]
     */
    double normalizeAngle(double angle);
    
    /**
     * @brief Calculate distance between two points
     */
    double calculateDistance(double x1, double y1, double x2, double y2);
    
    /**
     * @brief Save map to disk
     */
    void saveMapToFile();
    
    // ========== Callbacks ==========
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlTimerCallback();
};

#endif  // MAZE_EXPLORER_HPP_

