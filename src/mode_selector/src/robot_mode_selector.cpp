// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_mode_selector.cpp
// Author(s): Raquel Kampel
//
// Unified entry point for warehouse robot system.
// Demonstrates polymorphism: creates appropriate robot type based on user selection.
// All robot types inherit from BaseRobot and implement virtual run() method.

#include "mode_selector/base_robot.hpp"
#include "mode_selector/inspection_robot.hpp"
#include "mode_selector/delivery_robot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <iostream>
#include <vector>
#include <signal.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <algorithm>
#include <dirent.h>

#ifdef _WIN32
    #define clear_screen() system("cls")
#else
    #define clear_screen() system("clear")
#endif

// =============================
// Process Management
// =============================

std::vector<pid_t> child_processes;

void cleanup_child_processes() {
    for (pid_t pid : child_processes) {
        if (pid > 0) {
            kill(pid, SIGTERM);
            waitpid(pid, nullptr, 0);
        }
    }
    child_processes.clear();
}

void signal_handler(int sig) {
    cleanup_child_processes();
    exit(0);
}

pid_t launch_node(const std::string& command) {
    pid_t pid = fork();
    if (pid == 0) {
        // Child process - redirect output to avoid cluttering terminal
        int fd = open("/dev/null", O_WRONLY);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            close(fd);
        }
        execl("/bin/sh", "sh", "-c", command.c_str(), (char*)nullptr);
        exit(1);
    } else if (pid > 0) {
        // Parent process
        child_processes.push_back(pid);
        printf("[Launched] %s (PID: %d)\n", command.c_str(), pid);
        return pid;
    } else {
        fprintf(stderr, "Failed to fork process for: %s\n", command.c_str());
        return -1;
    }
}

// =============================
// Helper Functions
// =============================

/**
 * @brief Find the most recent map file in the home directory
 * @return Path to map file (without extension) or empty string if not found
 */
std::string findLastSavedMap() {
    const char* home = getenv("HOME");
    if (!home) {
        return "";
    }
    
    std::string home_dir = std::string(home);
    std::string map_prefix = "my_map_";
    
    // Find all map files (yaml files starting with my_map_)
    std::vector<std::pair<std::string, std::time_t>> map_files;
    
    DIR* dir = opendir(home_dir.c_str());
    if (!dir) {
        return "";
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename.find(map_prefix) == 0 && filename.find(".yaml") != std::string::npos) {
            std::string full_path = home_dir + "/" + filename;
            // Get file modification time
            struct stat file_stat;
            if (stat(full_path.c_str(), &file_stat) == 0) {
                std::string map_path = home_dir + "/" + filename.substr(0, filename.find(".yaml"));
                map_files.push_back({map_path, file_stat.st_mtime});
            }
        }
    }
    closedir(dir);
    
    if (map_files.empty()) {
        return "";
    }
    
    // Sort by modification time (most recent first)
    std::sort(map_files.begin(), map_files.end(), 
              [](const auto& a, const auto& b) { return a.second > b.second; });
    
    return map_files[0].first;  // Return most recent
}

// =============================
// Selection Functions
// =============================

int select_robot_type() {
    int choice = 0;
    printf("\n=== Warehouse Robot DevKit - Polymorphic Design ===\n\n");
    printf("Select Robot Type:\n");
    printf("1. Inspection Robot (Camera + LiDAR + Odometry)\n");
    printf("2. Delivery Robot (LiDAR + Odometry, no camera)\n");

    while (choice != 1 && choice != 2) {
        printf("\nEnter choice (1 or 2): ");
        scanf("%d", &choice);
        if (choice != 1 && choice != 2)
            printf("Invalid input, please enter 1 or 2.\n");
    }
    return choice;
}

int select_mode() {
    int choice = 0;
    printf("\nSelect Additional Mode:\n");
    printf("1. Low Battery Homing (Extension)\n");
    printf("2. Home Detection (Inspection only - ArUco ID detection)\n");
    printf("3. Wall Following (Basic exploration)\n");
    printf("4. Mapping Mode (Auto SLAM + map saving)\n");
    printf("5. None (Basic operation)\n");

    while (choice < 1 || choice > 5) {
        printf("\nEnter choice (1-5): ");
        scanf("%d", &choice);
        if (choice < 1 || choice > 5)
            printf("Invalid input, please enter 1, 2, 3, 4, or 5.\n");
    }
    return choice;
}

// =============================
// Main Entry Point
// =============================

int main(int argc, char** argv) {
    clear_screen();
    
    printf("============================================================\n");
    printf("MTRX3760 Project 2: Warehouse Robot DevKit\n");
    printf("Polymorphic Robot System - Common Platform\n");
    printf("============================================================\n\n");
    
    // User selection
    int robot_choice = select_robot_type();
    int mode_choice = select_mode();
    
    // Summary
    printf("\n============================================================\n");
    printf("Selection Summary:\n");
    printf("  Robot Type: %s\n", robot_choice == 1 ? "Inspection" : "Delivery");
    
    const char* mode_str = "None";
    if (mode_choice == 1) mode_str = "Low Battery Homing";
    else if (mode_choice == 2) mode_str = "Home Detection";
    else if (mode_choice == 3) mode_str = "Wall Following";
    else if (mode_choice == 4) mode_str = "Mapping Mode";
    printf("  Mode: %s\n", mode_str);
    printf("============================================================\n");
    printf("\nPress ENTER to start (or Ctrl+C to quit)...");
    getchar(); getchar();
    
    // Set up signal handlers for cleanup
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    atexit(cleanup_child_processes);
    
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // =============================
    // Launch Required External Nodes
    // =============================
    
    bool needs_aruco = false;
    bool needs_battery = false;
    bool needs_slam = false;
    
    if (robot_choice == 1) { // Inspection Robot
        if (mode_choice == 1 || mode_choice == 2) { // Low Battery Homing or Home Detection
            needs_aruco = true;
            if (mode_choice == 1) {
                needs_battery = true;
            }
        }
        if (mode_choice == 4) { // Mapping Mode
            needs_slam = true;
            printf("\n============================================================\n");
            printf("MAPPING MODE: Will launch SLAM automatically\n");
            printf("Map will be saved automatically after exploration\n");
            printf("============================================================\n");
        }
    } else if (robot_choice == 2) { // Delivery Robot
        if (mode_choice == 4) { // Mapping Mode
            needs_slam = true;
            printf("\n============================================================\n");
            printf("MAPPING MODE: Will launch SLAM automatically\n");
            printf("Map will be saved automatically after exploration\n");
            printf("============================================================\n");
        }
    }
    
    // Launch SLAM for mapping mode
    if (needs_slam) {
        printf("\nLaunching SLAM Toolbox for mapping...\n");
        std::string params_file = std::string(getenv("HOME")) + "/turtlebot3_ws/src/mode_selector/config/mapper_params_online_async.yaml";
        std::string slam_cmd = "ros2 launch slam_toolbox online_async_launch.py "
                               "use_sim_time:=false "
                               "params_file:=" + params_file;
        launch_node(slam_cmd);
        sleep(5); // Give SLAM time to initialize
        printf("SLAM launched with parameters from: %s\n", params_file.c_str());
        
        // Launch RViz2 for mapping visualization with pre-configured displays
        printf("\nLaunching RViz2 for mapping visualization...\n");
        // Get the path to the RViz2 config file
        std::string rviz_config = std::string(getenv("HOME")) + "/turtlebot3_ws/src/mode_selector/config/mapping.rviz";
        std::string rviz_cmd = "ros2 run rviz2 rviz2 -d " + rviz_config;
        launch_node(rviz_cmd);
        sleep(2); // Give RViz2 time to start
        printf("RViz2 launched with pre-configured displays:\n");
        printf("  - Map (topic: /map)\n");
        printf("  - LaserScan (topic: /scan, QoS: Best Effort)\n");
        printf("  - TF (coordinate frames)\n");
        
        // Launch wall follower for autonomous exploration
        printf("\nLaunching wall follower for autonomous maze exploration...\n");
        std::string wall_follower_cmd = "ros2 run robot_drive wall_follower";
        launch_node(wall_follower_cmd);
        sleep(2); // Give wall follower time to start
        printf("Wall follower launched - robot will explore automatically.\n");
        printf("Map will be saved automatically after 3 minutes of exploration.\n");
    }
    
    // Launch ArUco pose node if needed
    if (needs_aruco) {
        printf("\nLaunching ArUco pose node for marker detection...\n");
        std::string aruco_cmd = "ros2 run aruco_detector aruco_pose_node --ros-args "
                                "-p marker_length:=0.05 "
                                "-p camera_frame:=camera_optical_frame "
                                "-p publish_tf:=true "
                                "-p camera_info_topic:=/camera/camera_info "
                                "-p image_topic:=/camera/image_raw";
        launch_node(aruco_cmd);
        sleep(2); // Give node time to start
    }
    
    // Launch battery monitor if needed
    if (needs_battery) {
        printf("Launching battery monitor node...\n");
        std::string battery_cmd = "ros2 run battery_monitor battery_monitor_node";
        launch_node(battery_cmd);
        sleep(1); // Give node time to start
    }
    
    // =============================
    // Polymorphic Instantiation
    // =============================
    
    // Base class shared pointer - demonstrates polymorphism
    std::shared_ptr<mode_selector::BaseRobot> robot;
    
    if (robot_choice == 1) {
        // Inspection Robot
        std::string mode = "wall_following";
        if (mode_choice == 1) mode = "low_battery_homing";
        else if (mode_choice == 2) mode = "home_detection";
        else if (mode_choice == 3) mode = "wall_following";
        else if (mode_choice == 4) mode = "wall_following"; // Inspection robot just does wall following
        
        robot = std::make_shared<mode_selector::InspectionRobot>(mode);
        
        printf("\n[POLYMORPHISM] Created InspectionRobot (derived from BaseRobot)\n");
        printf("  - Inherits: Basic motion, battery monitoring, odometry\n");
        printf("  - Adds: Camera/ArUco detection, wall following, damage logging\n");
        
    } else {
        // Delivery Robot
        bool mapping_mode = (mode_choice == 4);
        robot = std::make_shared<mode_selector::DeliveryRobot>(mapping_mode);
        
        printf("\n[POLYMORPHISM] Created DeliveryRobot (derived from BaseRobot)\n");
        printf("  - Inherits: Basic motion, battery monitoring, odometry\n");
        printf("  - Adds: Navigation2, route management, delivery logging\n");
        if (mapping_mode) {
            printf("  - Mapping mode: enabled (SLAM + auto map saving)\n");
        }
    }
    
    printf("\n============================================================\n");
    printf("Starting Robot System...\n");
    printf("============================================================\n\n");
    
    // =============================
    // Unified Interface - Polymorphism in Action
    // =============================
    
    // Initialize robot-specific functionality
    robot->initialize();
    
    printf("\nRobot initialized. Starting execution loop...\n");
    printf("Press Ctrl+C to stop.\n\n");
    
    // Virtual function call - different behavior based on robot type
    // InspectionRobot::run() - wall following, damage detection
    // DeliveryRobot::run() - route navigation, delivery management
    robot->run();
    
    // Check if we're in mapping mode - if so, spin until mapping completes
    bool is_mapping_mode = (robot_choice == 2 && mode_choice == 4);
    
    if (is_mapping_mode) {
        // For mapping mode, spin until mapping is complete
        printf("\nMapping mode active - waiting for map to be saved...\n");
        printf("Robot is exploring with wall follower. Map will save after 3 minutes.\n\n");
        
        auto delivery_robot = std::dynamic_pointer_cast<mode_selector::DeliveryRobot>(robot);
        if (delivery_robot) {
            // Spin until mapping is complete
            while (rclcpp::ok() && !delivery_robot->isMappingComplete()) {
                rclcpp::spin_some(robot);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // Mapping is complete - show prompt
            printf("\n============================================================\n");
            printf("MAPPING COMPLETE!\n");
            printf("============================================================\n\n");
            printf("Map has been saved successfully.\n");
            printf("What would you like to do next?\n\n");
            printf("1. Start Delivery Mode (requires Navigation2 to be launched)\n");
            printf("2. Exit to Main Menu\n");
            printf("3. Exit Program\n");
            
            int next_choice = 0;
            while (next_choice < 1 || next_choice > 3) {
                printf("\nEnter choice (1-3): ");
                if (scanf("%d", &next_choice) != 1) {
                    // Clear invalid input
                    int c;
                    while ((c = getchar()) != '\n' && c != EOF);
                    next_choice = 0;
                }
                if (next_choice < 1 || next_choice > 3) {
                    printf("Invalid input, please enter 1, 2, or 3.\n");
                }
            }
            // Clear newline after scanf
            int c;
            while ((c = getchar()) != '\n' && c != EOF);
            
            if (next_choice == 1) {
                // Start delivery mode - launch Navigation2 and connect
                printf("\nLaunching Navigation2 for delivery mode...\n");
                
                // Get the map file path
                std::string map_file = delivery_robot->getSavedMapPath();
                
                // If no saved map path (shouldn't happen), find the last one
                if (map_file.empty()) {
                    printf("Finding last saved map...\n");
                    map_file = findLastSavedMap();
                }
                
                std::string nav2_cmd;
                if (!map_file.empty()) {
                    printf("Using map file: %s\n", map_file.c_str());
                    // Navigation2 launch accepts map parameter
                    nav2_cmd = "ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=" + map_file;
                } else {
                    printf("WARNING: No map file found! Navigation2 will use default map.\n");
                    printf("You may need to set initial pose manually in RViz2.\n");
                    nav2_cmd = "ros2 launch turtlebot3_navigation2 navigation2.launch.py";
                }
                
                launch_node(nav2_cmd);
                sleep(5); // Give Navigation2 time to start
                printf("Navigation2 launched. Connecting to action server...\n");
                
                // Enable delivery mode - will initialize log and start connecting to Navigation2
                delivery_robot->enableDeliveryMode();
                
                // Connect to Navigation2 - this will be done via the timer callback
                // Create a thread to monitor connection
                printf("Waiting for Navigation2 action server to be ready...\n");
                printf("This may take a few moments...\n\n");
                
                // Give some time and check connection status
                int max_wait = 30; // 30 seconds max wait
                bool connected = false;
                for (int i = 0; i < max_wait && !connected; i++) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    rclcpp::spin_some(robot);
                    // Check connection status - we'll rely on logs for now
                    printf(".");
                    fflush(stdout);
                }
                printf("\n");
                
                printf("\nDelivery mode ready!\n");
                printf("You can now send navigation goals via RViz2 or queue delivery requests.\n\n");
                
                // Continue spinning for delivery operations
                rclcpp::spin(robot);
            } else if (next_choice == 2) {
                // Exit to main menu - will be handled by cleanup
                printf("\nExiting to main menu...\n");
                robot->stopRobot();
                rclcpp::shutdown();
                cleanup_child_processes();
                return 0; // Exit cleanly
            } else {
                // Exit program
                printf("\nExiting program...\n");
                robot->stopRobot();
                rclcpp::shutdown();
                cleanup_child_processes();
                return 0;
            }
        } else {
            // Fallback - shouldn't happen, but spin normally
            rclcpp::spin(robot);
        }
    } else {
        // Normal mode - spin normally
        rclcpp::spin(robot);
    }
    
    // Cleanup
    printf("\n============================================================\n");
    printf("Shutting down robot system...\n");
    printf("============================================================\n");
    
    robot->stopRobot();
    rclcpp::shutdown();
    
    // Cleanup child processes
    cleanup_child_processes();
    
    printf("\n============================================================\n");
    printf("Robot System Shutdown Complete\n");
    printf("============================================================\n\n");
    
    return 0;
}
