// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_mode_selector.cpp
// Author(s): Racquel Kampel
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
        std::string slam_cmd = "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false";
        launch_node(slam_cmd);
        sleep(5); // Give SLAM time to initialize
        printf("SLAM launched.\n");
        
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
        printf("Map will be saved automatically after 5 minutes of exploration.\n");
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
    
    // ROS 2 spin - handles callbacks
    // Note: run() is virtual and may start its own behavior,
    // but we still need to spin for callbacks
    rclcpp::spin(robot);
    
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
