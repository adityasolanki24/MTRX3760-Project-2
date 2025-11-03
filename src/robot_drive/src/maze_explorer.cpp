// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: maze_explorer.cpp
// Author(s): Project Team
//
// Tremaux Algorithm-based maze exploration implementation.
// Systematically explores and maps maze using intersection detection
// and path marking to avoid loops.

#include "maze_explorer/maze_explorer.hpp"
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

// ========== Constructor ==========

MazeExplorer::MazeExplorer()
    : Node("maze_explorer"),
      linear_speed_(0.15),
      angular_speed_(0.6),
      turn_duration_(2.5),  // seconds for 90-degree turn
      front_threshold_(0.5),
      side_threshold_(0.4),
      wall_follow_distance_(0.25),
      intersection_distance_(0.35),
      min_safe_distance_(0.25),  // Emergency stop distance
      state_(ExplorationState::FINDING_WALL),
      current_heading_(Direction::FRONT),
      current_node_id_(-1),
      next_node_id_(0),
      exploration_complete_(false),
      intersection_detected_(false),
      robot_x_(0.0),
      robot_y_(0.0),
      robot_yaw_(0.0),
      last_intersection_x_(0.0),
      last_intersection_y_(0.0),
      distance_since_intersection_(0.0),
      pending_turn_(Direction::NONE),
      turning_(false),
      turn_angle_target_(0.0),
      turn_angle_current_(0.0),
      scan_received_(false),
      map_file_path_("/tmp/maze_exploration_map.txt")
{
    // QoS Profile for LiDAR
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    
    // Publishers and Subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos, std::bind(&MazeExplorer::scanCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&MazeExplorer::odomCallback, this, std::placeholders::_1));
    
    // Control timer - runs at 20Hz
    control_timer_ = this->create_wall_timer(
        50ms, std::bind(&MazeExplorer::controlTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "Maze Explorer - Tremaux Algorithm");
    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "Linear speed: %.2f m/s", linear_speed_);
    RCLCPP_INFO(this->get_logger(), "Angular speed: %.2f rad/s", angular_speed_);
    RCLCPP_INFO(this->get_logger(), "Map output: %s", map_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "============================================================");
}

// ========== Destructor ==========

MazeExplorer::~MazeExplorer()
{
    // Save map before shutdown
    saveMapToFile();
    
    // Stop robot
    auto stop_cmd = geometry_msgs::msg::TwistStamped();
    stop_cmd.header.stamp = this->now();
    stop_cmd.header.frame_id = "base_link";
    cmd_vel_pub_->publish(stop_cmd);
    
    RCLCPP_INFO(this->get_logger(), "Map saved to %s", map_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Exploration complete. Robot stopped.");
}

// ========== Helper Methods ==========

float MazeExplorer::findMinDistance(const std::vector<float>& ranges,
                                    double angle_start, double angle_end,
                                    double angle_min, double angle_increment)
{
    float min_val = std::numeric_limits<float>::max();
    
    // Convert angles to indices
    int start_idx = static_cast<int>((angle_start - angle_min) / angle_increment);
    int end_idx = static_cast<int>((angle_end - angle_min) / angle_increment);
    
    // Handle wrap-around for negative angles
    if (start_idx < 0) start_idx += ranges.size();
    if (end_idx < 0) end_idx += ranges.size();
    
    for (int i = start_idx; i <= end_idx && i < static_cast<int>(ranges.size()); i++) {
        int idx = (i < 0) ? i + ranges.size() : i;
        if (idx >= 0 && idx < static_cast<int>(ranges.size())) {
            float val = ranges[idx];
            if (!std::isinf(val) && !std::isnan(val) && val > 0.1f && val < 3.5f) {
                min_val = std::min(min_val, val);
            }
        }
    }
    
    return (min_val == std::numeric_limits<float>::max()) ? 3.5f : min_val;
}

MazeExplorer::AvailablePaths MazeExplorer::detectAvailablePaths(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    AvailablePaths paths;
    
    if (!scan || scan->ranges.empty()) {
        return paths;
    }
    
    double angle_min = scan->angle_min;
    double angle_increment = scan->angle_increment;
    
    // Front: -30° to +30° (wider detection for better obstacle avoidance)
    double front_start = -0.524;  // -30 degrees
    double front_end = 0.524;     // +30 degrees
    paths.front_dist = findMinDistance(scan->ranges, front_start, front_end, 
                                       angle_min, angle_increment);
    paths.front = (paths.front_dist > front_threshold_);
    
    // Front-left: 30° to 60° (corner detection)
    double front_left_start = 0.524;  // 30 degrees
    double front_left_end = 1.047;    // 60 degrees
    paths.front_left_dist = findMinDistance(scan->ranges, front_left_start, front_left_end,
                                            angle_min, angle_increment);
    
    // Front-right: -60° to -30° (corner detection)
    double front_right_start = -1.047;  // -60 degrees
    double front_right_end = -0.524;     // -30 degrees
    paths.front_right_dist = findMinDistance(scan->ranges, front_right_start, front_right_end,
                                              angle_min, angle_increment);
    
    // Left: 60° to 120° (90° = left)
    double left_start = 1.047;   // 60 degrees
    double left_end = 2.094;      // 120 degrees
    paths.left_dist = findMinDistance(scan->ranges, left_start, left_end,
                                      angle_min, angle_increment);
    paths.left = (paths.left_dist > side_threshold_);
    
    // Right: -120° to -60° (-90° = right)
    double right_start = -2.094;  // -120 degrees
    double right_end = -1.047;    // -60 degrees
    paths.right_dist = findMinDistance(scan->ranges, right_start, right_end,
                                        angle_min, angle_increment);
    paths.right = (paths.right_dist > side_threshold_);
    
    // Back: 150° to 210° and -210° to -150° (180° = back)
    double back1 = findMinDistance(scan->ranges, 2.618, 3.665, angle_min, angle_increment);
    double back2 = findMinDistance(scan->ranges, -3.665, -2.618, angle_min, angle_increment);
    paths.back = (std::min(back1, back2) > side_threshold_);
    
    return paths;
}

bool MazeExplorer::isAtIntersection(const AvailablePaths& paths)
{
    // Count available paths
    int path_count = 0;
    if (paths.front) path_count++;
    if (paths.left) path_count++;
    if (paths.right) path_count++;
    
    // Intersection if 2 or more paths available (T-junction or crossroad)
    return path_count >= 2;
}

MazeExplorer::Direction MazeExplorer::yawToDirection(double yaw)
{
    yaw = normalizeAngle(yaw);
    
    if (yaw >= -0.785 && yaw < 0.785) return Direction::FRONT;
    if (yaw >= 0.785 && yaw < 2.356) return Direction::LEFT;
    if (yaw >= -2.356 && yaw < -0.785) return Direction::RIGHT;
    return Direction::BACK;
}

MazeExplorer::Direction MazeExplorer::getRelativeDirection(Direction absolute_dir)
{
    // Convert absolute direction based on current heading
    // Simplified: assume we track heading and adjust accordingly
    return absolute_dir;
}

MazeExplorer::Direction MazeExplorer::getOppositeDirection(Direction dir)
{
    switch (dir) {
        case Direction::FRONT: return Direction::BACK;
        case Direction::BACK: return Direction::FRONT;
        case Direction::LEFT: return Direction::RIGHT;
        case Direction::RIGHT: return Direction::LEFT;
        default: return Direction::NONE;
    }
}

double MazeExplorer::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double MazeExplorer::calculateDistance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

MazeExplorer::Direction MazeExplorer::chooseNextPath(IntersectionNode& node)
{
    // Tremaux Algorithm:
    // 1. Choose unmarked path if available
    // 2. If all paths marked once, choose path marked once (backtrack)
    // 3. Avoid paths marked twice
    
    Direction best_dir = Direction::NONE;
    int best_mark_count = 999;
    
    for (Direction dir : node.paths) {
        if (dir == getOppositeDirection(current_heading_)) {
            continue;  // Don't go back the way we came (unless forced)
        }
        
        int mark_count = node.path_marks[dir];
        
        // Prefer unmarked paths
        if (mark_count == 0 && best_mark_count > 0) {
            best_dir = dir;
            best_mark_count = 0;
        }
        // Accept paths marked once if no unmarked available
        else if (mark_count == 1 && best_mark_count > 1) {
            best_dir = dir;
            best_mark_count = 1;
        }
        // Avoid paths marked twice
        else if (mark_count >= 2) {
            continue;
        }
    }
    
    // If no good path found, try going back
    if (best_dir == Direction::NONE && current_node_id_ >= 0) {
        Direction back_dir = getOppositeDirection(current_heading_);
        if (std::find(node.paths.begin(), node.paths.end(), back_dir) != node.paths.end()) {
            best_dir = back_dir;
            RCLCPP_WARN(this->get_logger(), "Forced to backtrack at node %d", node.id);
        }
    }
    
    return best_dir;
}

void MazeExplorer::markPath(IntersectionNode& node, Direction dir)
{
    node.path_marks[dir]++;
    RCLCPP_DEBUG(this->get_logger(), "Marked path %d at node %d (mark count: %d)",
                 static_cast<int>(dir), node.id, node.path_marks[dir]);
}

int MazeExplorer::createIntersectionNode(double x, double y, double yaw, 
                                         const AvailablePaths& paths)
{
    // Check if we're close to an existing node
    for (auto& pair : map_nodes_) {
        IntersectionNode& existing = pair.second;
        double dist = calculateDistance(x, y, existing.x, existing.y);
        if (dist < 0.5) {  // Within 50cm, consider it the same node
            // Update paths if new ones detected
            if (paths.front && std::find(existing.paths.begin(), existing.paths.end(), 
                                         Direction::FRONT) == existing.paths.end()) {
                existing.paths.push_back(Direction::FRONT);
            }
            if (paths.left && std::find(existing.paths.begin(), existing.paths.end(), 
                                        Direction::LEFT) == existing.paths.end()) {
                existing.paths.push_back(Direction::LEFT);
            }
            if (paths.right && std::find(existing.paths.begin(), existing.paths.end(), 
                                         Direction::RIGHT) == existing.paths.end()) {
                existing.paths.push_back(Direction::RIGHT);
            }
            existing.visit_count++;
            return existing.id;
        }
    }
    
    // Create new node
    IntersectionNode node;
    node.id = next_node_id_++;
    node.x = x;
    node.y = y;
    node.yaw = yaw;
    node.visited = false;
    node.visit_count = 1;
    
    if (paths.front) node.paths.push_back(Direction::FRONT);
    if (paths.left) node.paths.push_back(Direction::LEFT);
    if (paths.right) node.paths.push_back(Direction::RIGHT);
    if (paths.back) node.paths.push_back(Direction::BACK);
    
    map_nodes_[node.id] = node;
    
    RCLCPP_INFO(this->get_logger(), "Created intersection node %d at (%.2f, %.2f) with %zu paths",
                node.id, x, y, node.paths.size());
    
    return node.id;
}

void MazeExplorer::followWall(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    
    // Use provided scan or last detected paths
    AvailablePaths paths;
    if (scan) {
        paths = detectAvailablePaths(scan);
        last_detected_paths_ = paths;  // Update stored paths
    } else {
        paths = last_detected_paths_;  // Use stored paths
    }
    
    // Extract ranges for wall following
    double right_dist = paths.right_dist;
    double front_dist = paths.front_dist;
    double front_left_dist = paths.front_left_dist;
    double front_right_dist = paths.front_right_dist;
    
    // PRIORITY 1: EMERGENCY STOP - Obstacle too close (front or front-right corner)
    if (front_dist < min_safe_distance_ || front_right_dist < min_safe_distance_) {
        // CRITICAL: Stop immediately and turn left sharply
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = angular_speed_ * 1.2;  // Sharp left turn
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP! Front: %.2fm, Front-Right: %.2fm", 
                   front_dist, front_right_dist);
        cmd_vel_pub_->publish(cmd);
        return;
    }
    
    // PRIORITY 2: Wall very close ahead - slow down and turn
    if (front_dist < 0.35) {
        // Very close wall - slow down linear speed, increase angular speed slightly
        cmd.twist.linear.x = linear_speed_ * 0.3;  // Slow down
        cmd.twist.angular.z = angular_speed_ * 0.8;  // Increase angular slightly
        RCLCPP_INFO(this->get_logger(), "Close wall ahead (%.2fm) - slowing and turning", front_dist);
    }
    // PRIORITY 3: Front-right corner obstacle - slow down and turn
    else if (front_right_dist < 0.35) {
        // Front-right corner too close - slow down, increase angular slightly
        cmd.twist.linear.x = linear_speed_ * 0.4;  // Slow down
        cmd.twist.angular.z = angular_speed_ * 0.7;  // Increase angular slightly
        RCLCPP_INFO(this->get_logger(), "Front-right corner (%.2fm) - slowing and turning", front_right_dist);
    }
    // PRIORITY 4: Wall ahead but safe distance - slow down and turn
    else if (front_dist < 0.5) {
        // Obstacle ahead but safe distance - slow down, increase angular slightly
        cmd.twist.linear.x = linear_speed_ * 0.5;  // Slow down
        cmd.twist.angular.z = angular_speed_ * 0.6;  // Increase angular slightly
        RCLCPP_INFO(this->get_logger(), "Wall ahead (%.2fm) - slowing and turning", front_dist);
    }
    // PRIORITY 5: Too close to right wall - adjust left
    else if (right_dist < wall_follow_distance_ - 0.05) {
        // Too close to right wall - turn left
        cmd.twist.linear.x = linear_speed_ * 0.8;
        cmd.twist.angular.z = angular_speed_ * 0.5;
    }
    // PRIORITY 6: Too far from right wall - adjust right
    else if (right_dist > wall_follow_distance_ + 0.10) {
        // Too far from right wall - turn right gently
        cmd.twist.linear.x = linear_speed_ * 0.8;
        cmd.twist.angular.z = -angular_speed_ * 0.3;
    }
    // PRIORITY 7: Good distance - move forward
    else {
        // Good distance - move forward
        cmd.twist.linear.x = linear_speed_;
        cmd.twist.angular.z = 0.0;
    }
    
    cmd_vel_pub_->publish(cmd);
}

void MazeExplorer::executeTurn(Direction target_direction)
{
    if (!turning_) {
        turning_ = true;
        turn_start_time_ = this->now();
        
        // Calculate target angle based on direction
        double current_angle = robot_yaw_;
        double target_angle = current_angle;
        
        switch (target_direction) {
            case Direction::LEFT:
                target_angle = current_angle + M_PI / 2.0;
                break;
            case Direction::RIGHT:
                target_angle = current_angle - M_PI / 2.0;
                break;
            case Direction::BACK:
                target_angle = current_angle + M_PI;
                break;
            default:
                target_angle = current_angle;
        }
        
        turn_angle_target_ = normalizeAngle(target_angle);
        turn_angle_current_ = current_angle;
    }
    
    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    
    double angle_diff = normalizeAngle(turn_angle_target_ - robot_yaw_);
    
    if (std::abs(angle_diff) > 0.15) {  // 0.15 rad ≈ 8.5 degrees
        // Still turning
        cmd.twist.linear.x = 0.0;
        
        if (angle_diff > 0) {
            cmd.twist.angular.z = angular_speed_;
        } else {
            cmd.twist.angular.z = -angular_speed_;
        }
        
        cmd_vel_pub_->publish(cmd);
    } else {
        // Turn complete
        turning_ = false;
        current_heading_ = target_direction;
        pending_turn_ = Direction::NONE;
        state_ = ExplorationState::FOLLOWING_WALL;
        
        RCLCPP_INFO(this->get_logger(), "Turn complete. New heading: %d", 
                    static_cast<int>(target_direction));
    }
}

void MazeExplorer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Store latest scan data - processing happens in control timer
    if (msg && !msg->ranges.empty()) {
        last_detected_paths_ = detectAvailablePaths(msg);
        intersection_detected_ = isAtIntersection(last_detected_paths_);
        scan_received_ = true;  // Mark that we have valid scan data
    }
}

void MazeExplorer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
    
    // Update distance since last intersection
    if (current_node_id_ >= 0) {
        distance_since_intersection_ = calculateDistance(
            robot_x_, robot_y_, last_intersection_x_, last_intersection_y_);
    }
}

void MazeExplorer::controlTimerCallback()
{
    if (exploration_complete_) {
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd_vel_pub_->publish(cmd);
        return;
    }
    
    // GLOBAL SAFETY CHECK: Always check for obstacles first (only if we have scan data)
    if (scan_received_ && 
        ((last_detected_paths_.front_dist < min_safe_distance_ && last_detected_paths_.front_dist > 0.01) ||
         (last_detected_paths_.front_right_dist < min_safe_distance_ && last_detected_paths_.front_right_dist > 0.01))) {
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = angular_speed_ * 1.2;  // Emergency sharp turn left
        cmd_vel_pub_->publish(cmd);
        RCLCPP_WARN(this->get_logger(), "SAFETY STOP! Front: %.2fm, Front-Right: %.2fm", 
                   last_detected_paths_.front_dist, last_detected_paths_.front_right_dist);
        return;
    }
    
    // If no scan data yet, wait
    if (!scan_received_) {
        auto cmd = geometry_msgs::msg::TwistStamped();
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
        RCLCPP_DEBUG(this->get_logger(), "Waiting for LiDAR scan data...");
        return;
    }
    
    auto cmd = geometry_msgs::msg::TwistStamped();
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    
    switch (state_) {
        case ExplorationState::FINDING_WALL:
            // Search for first wall
            if (last_detected_paths_.front_dist < 1.5 && last_detected_paths_.front_dist > min_safe_distance_) {
                state_ = ExplorationState::FOLLOWING_WALL;
                RCLCPP_INFO(this->get_logger(), "Wall found. Starting exploration.");
            } else {
                // Safe search - spin slowly, check front distance
                if (last_detected_paths_.front_dist > min_safe_distance_ + 0.2) {
                    cmd.twist.linear.x = linear_speed_ * 0.4;
                    cmd.twist.angular.z = -angular_speed_ * 0.3;
                } else {
                    // Too close, just turn
                    cmd.twist.linear.x = 0.0;
                    cmd.twist.angular.z = -angular_speed_ * 0.3;
                }
            }
            cmd_vel_pub_->publish(cmd);
            break;
            
        case ExplorationState::FOLLOWING_WALL:
            if (intersection_detected_ && distance_since_intersection_ > 0.3) {
                // At intersection
                state_ = ExplorationState::AT_INTERSECTION;
                last_intersection_x_ = robot_x_;
                last_intersection_y_ = robot_y_;
                distance_since_intersection_ = 0.0;
                
                // Create/update intersection node
                int node_id = createIntersectionNode(robot_x_, robot_y_, robot_yaw_, 
                                                     last_detected_paths_);
                
                if (node_id != current_node_id_) {
                    // New or different intersection
                    current_node_id_ = node_id;
                    
                    // Choose next path using Tremaux algorithm
                    IntersectionNode& node = map_nodes_[node_id];
                    Direction next_dir = chooseNextPath(node);
                    
                    if (next_dir != Direction::NONE) {
                        markPath(node, next_dir);
                        pending_turn_ = next_dir;
                        state_ = ExplorationState::TURNING;
                        RCLCPP_INFO(this->get_logger(), 
                                   "At intersection %d. Choosing path: %d",
                                   node_id, static_cast<int>(next_dir));
                    } else {
                        // No valid path - exploration might be complete
                        RCLCPP_WARN(this->get_logger(), 
                                   "No valid path at node %d. Checking completion...", node_id);
                        exploration_complete_ = true;
                        state_ = ExplorationState::EXPLORATION_COMPLETE;
                    }
                } else {
                    // Revisited intersection - choose different path
                    IntersectionNode& node = map_nodes_[node_id];
                    Direction next_dir = chooseNextPath(node);
                    if (next_dir != Direction::NONE) {
                        markPath(node, next_dir);
                        pending_turn_ = next_dir;
                        state_ = ExplorationState::TURNING;
                    } else {
                        exploration_complete_ = true;
                        state_ = ExplorationState::EXPLORATION_COMPLETE;
                    }
                }
            } else {
                // Continue following wall - use stored path data
                followWall(nullptr);
            }
            break;
            
        case ExplorationState::AT_INTERSECTION:
            // Brief pause at intersection
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            break;
            
        case ExplorationState::TURNING:
            if (pending_turn_ != Direction::NONE) {
                executeTurn(pending_turn_);
            } else {
                state_ = ExplorationState::FOLLOWING_WALL;
            }
            break;
            
        case ExplorationState::BACKTRACKING:
            // Similar to following, but marked path
            followWall(nullptr);
            break;
            
        case ExplorationState::EXPLORATION_COMPLETE:
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            saveMapToFile();
            RCLCPP_INFO(this->get_logger(), "Exploration complete! Map saved.");
            exploration_complete_ = true;
            break;
    }
}

void MazeExplorer::saveMapToFile()
{
    std::ofstream map_file(map_file_path_);
    
    if (!map_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open map file for writing!");
        return;
    }
    
    // Get current timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    
    map_file << "==========================================\n";
    map_file << "MAZE EXPLORATION MAP\n";
    map_file << "Algorithm: Tremaux (Path Marking)\n";
    map_file << "Generated: " << ss.str() << "\n";
    map_file << "==========================================\n\n";
    
    map_file << "INTERSECTIONS (" << map_nodes_.size() << " total):\n";
    map_file << "------------------------------------------\n";
    
    for (const auto& pair : map_nodes_) {
        const IntersectionNode& node = pair.second;
        map_file << "Node " << node.id << ":\n";
        map_file << "  Position: (" << std::fixed << std::setprecision(2) 
                 << node.x << ", " << node.y << ")\n";
        map_file << "  Orientation: " << std::setprecision(3) << node.yaw << " rad\n";
        map_file << "  Visits: " << node.visit_count << "\n";
        map_file << "  Available Paths: ";
        
        for (Direction dir : node.paths) {
            std::string dir_str;
            switch (dir) {
                case Direction::FRONT: dir_str = "FRONT"; break;
                case Direction::LEFT: dir_str = "LEFT"; break;
                case Direction::RIGHT: dir_str = "RIGHT"; break;
                case Direction::BACK: dir_str = "BACK"; break;
                default: dir_str = "UNKNOWN"; break;
            }
            map_file << dir_str << " ";
        }
        map_file << "\n  Path Marks: ";
        for (const auto& mark_pair : node.path_marks) {
            std::string dir_str;
            switch (mark_pair.first) {
                case Direction::FRONT: dir_str = "FRONT"; break;
                case Direction::LEFT: dir_str = "LEFT"; break;
                case Direction::RIGHT: dir_str = "RIGHT"; break;
                case Direction::BACK: dir_str = "BACK"; break;
                default: dir_str = "UNKNOWN"; break;
            }
            map_file << dir_str << ":" << mark_pair.second << " ";
        }
        map_file << "\n\n";
    }
    
    map_file << "EXPLORATION SUMMARY:\n";
    map_file << "------------------------------------------\n";
    map_file << "Total Intersections: " << map_nodes_.size() << "\n";
    map_file << "Exploration Status: " 
             << (exploration_complete_ ? "COMPLETE" : "IN PROGRESS") << "\n";
    map_file << "==========================================\n";
    
    map_file.close();
    RCLCPP_INFO(this->get_logger(), "Map saved to %s", map_file_path_.c_str());
}

// ========== Main Function ==========

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::cout << "\n============================================================\n";
    std::cout << "Maze Explorer - Tremaux Algorithm\n";
    std::cout << "============================================================\n";
    std::cout << "Systematic maze exploration with topological mapping\n";
    std::cout << "SAFETY: Keep robot in clear area\n";
    std::cout << "EMERGENCY STOP: Press Ctrl+C\n";
    std::cout << "============================================================\n\n";
    
    auto node = std::make_shared<MazeExplorer>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    
    std::cout << "\n============================================================\n";
    std::cout << "Exploration Complete!\n";
    std::cout << "============================================================\n\n";
    
    rclcpp::shutdown();
    return 0;
}

