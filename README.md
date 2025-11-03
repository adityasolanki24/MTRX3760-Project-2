# MTRX3760 Project 2: Warehouse Robot DevKit

## Overview

This project implements a **polymorphic warehouse robotics system** that emulates two different robot types on a single TurtleBot3 platform:

1. **Inspection Robot**: Uses camera, LiDAR, and odometry to detect and record damage sites (cracks) in the warehouse
2. **Delivery Robot**: Uses LiDAR and odometry only to deliver packages between locations using Navigation2

The system demonstrates a common development platform approach with shared functionality and specialized behaviors for different robot types. Both robot types are accessed through a unified **Mode Selector** interface that demonstrates polymorphism in action.

---

## System Architecture

### Core Components

#### 1. **Mode Selector** (`robot_mode_selector`) - Main Entry Point
- **Location**: `src/mode_selector/src/robot_mode_selector.cpp`
- **Function**: Unified interface for selecting and running different robot modes
- **Features**:
  - Interactive menu system for robot and mode selection
  - Automatic launch of required nodes (SLAM, RViz, wall follower, etc.)
  - Demonstrates polymorphism by using base class pointers
  - Manages process lifecycle (launches and cleans up child processes)

#### 2. **Base Robot** (`BaseRobot`) - Abstract Base Class
- **Location**: `src/mode_selector/include/mode_selector/base_robot.hpp`
- **Function**: Provides shared functionality for all robot types
- **Shared Features**:
  - Basic motion control (`publishVelocity`, `stopRobot`)
  - Battery level monitoring and tracking
  - Odometry subscription and pose tracking
- **Virtual Interface**: `initialize()` and `run()` must be implemented by derived classes

#### 3. **Inspection Robot** (`InspectionRobot`) - Derived Class
- **Location**: `src/mode_selector/src/inspection_robot.cpp`
- **Function**: Specialized robot for warehouse inspection tasks
- **Features**:
  - Camera-based ArUco marker detection for damage sites
  - Right-hand wall following for exploration
  - Damage site logging to disk
  - Low battery homing to charging station (ArUco ID 0)
  - PID-controlled wall following

#### 4. **Delivery Robot** (`DeliveryRobot`) - Derived Class
- **Location**: `src/mode_selector/src/delivery_robot.cpp`
- **Function**: Specialized robot for package delivery tasks
- **Features**:
  - Navigation2 integration for path planning
  - Mapping mode with automatic SLAM and map saving (3 minutes)
  - Automatic initial pose setting from last known position
  - Delivery request queue management
  - RViz2 goal subscription (2D Goal Pose and Publish Point)
  - Delivery logging to disk

#### 5. **ArUco Detector Node** (`aruco_pose_node`)
- **Location**: `src/aruco_detector/src/aruco_pose_node.cpp`
- **Function**: Detects ArUco markers using the camera and publishes pose information
- **Topics Published**: 
  - `/aruco/pose/id_0` (charging station/home)
  - `/aruco/pose/id_1` (inspection target)
  - `/aruco/pose/id_N` (for other marker IDs)
- **Output**: `geometry_msgs/PoseStamped` messages with marker position and distance from camera

#### 6. **Battery Monitor Node** (`battery_monitor_node`)
- **Location**: `src/battery_monitor/src/battery_monitor_node.cpp`
- **Function**: Simulates and monitors virtual battery level
- **Topics Published**: `/battery/level` (`std_msgs/Float32`)
- **Topics Subscribed**: Monitors battery consumption based on robot activity

#### 7. **Wall Follower** (`wall_follower`)
- **Location**: `src/robot_drive/src/wall_follower.cpp`
- **Function**: Pure right-hand wall following algorithm for autonomous exploration
- **Use Case**: Mapping and SLAM - allows robot to roam around the maze to build maps
- **Output**: Velocity commands to `/cmd_vel` (`geometry_msgs/TwistStamped`)

#### 8. **Home Detection** (`home_detection`)
- **Location**: `src/robot_drive/src/home_detection.cpp`
- **Function**: Right-hand wall following with ArUco ID 1 detection for inspection mode
- **Behavior**: 
  - Follows right wall using PID control
  - Subscribes to `/aruco/pose/id_1` for damage site detection
  - Stops when ArUco marker ID 1 is detected within 30cm distance
  - Records damage locations for inspection reporting
- **Parameters**: `aruco_stop_distance` (default: 0.30 meters)

#### 9. **Low Battery Homing** (`low_battery_homing`)
- **Location**: `src/robot_drive/src/low_battery_homing.cpp`
- **Function**: Monitors battery level and navigates to charging station (ArUco ID 0) when battery < 20%
- **Behavior**:
  - Subscribes to `/battery/level` from battery monitor
  - Subscribes to `/aruco/pose/id_0` for charging station location
  - Disables wall following when battery is low
  - Uses right-hand wall following to navigate to ArUco ID 0
  - Re-enables wall following after docking
- **Threshold**: 20% battery level

---

## Project Structure

```
turtlebot3_ws/
├── src/
│   ├── aruco_detector/          # ArUco marker detection
│   │   ├── src/
│   │   │   ├── aruco_detector_node.cpp
│   │   │   └── aruco_pose_node.cpp
│   │   └── launch/
│   ├── battery_monitor/         # Battery simulation and monitoring
│   │   ├── src/
│   │   │   ├── battery_monitor_node.cpp
│   │   │   ├── battery.cpp
│   │   │   └── logger.cpp
│   │   └── launch/
│   ├── mode_selector/           # Unified robot management (POLYMORPHIC)
│   │   ├── src/
│   │   │   ├── robot_mode_selector.cpp  # Main entry point
│   │   │   ├── base_robot.cpp           # Abstract base class
│   │   │   ├── inspection_robot.cpp     # Derived class
│   │   │   └── delivery_robot.cpp       # Derived class
│   │   ├── include/mode_selector/
│   │   │   ├── base_robot.hpp
│   │   │   ├── inspection_robot.hpp
│   │   │   └── delivery_robot.hpp
│   │   └── config/
│   │       ├── mapping.rviz            # RViz config for mapping
│   │       └── mapper_params_online_async.yaml  # SLAM configuration
│   ├── robot_drive/             # Wall following algorithms
│   │   ├── src/
│   │   │   ├── wall_follower.cpp        # Pure wall following (mapping)
│   │   │   ├── home_detection.cpp       # Inspection mode (ArUco detection)
│   │   │   ├── low_battery_homing.cpp   # Battery-aware navigation
│   │   │   └── maze_explorer.cpp        # Tremaux algorithm (unused)
│   │   └── launch/
│   │       └── low_battery_homing.launch.py
│   └── turtlebot3_navigation2/   # Nav2 configuration for delivery mode
│       ├── launch/navigation2.launch.py
│       └── param/
└── README.md (this file)
```

---

## Prerequisites

### Hardware
- TurtleBot3 (Burger, Waffle, or Waffle Pi)
- ROS 2 (tested on Humble/Hawksbill)

### Software Dependencies
```bash
# Core ROS 2 packages
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-cartographer ros-$ROS_DISTRO-cartographer-ros

# Additional dependencies
sudo apt install libopencv-dev
sudo apt install ros-$ROS_DISTRO-turtlebot3*
```

---

## Building the Workspace

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

**Note**: Use `--symlink-install` to allow Python launch files to work without rebuilding.

---

## Running the Robot

### Quick Start - Unified Interface

The easiest way to use the system is through the **Mode Selector** interface:

```bash
# Build and source the workspace
cd ~/turtlebot3_ws
colcon build
source install/setup.bash

# Run the mode selector
ros2 run mode_selector robot_mode_selector
```

This will present an interactive menu where you can:
1. Choose robot type (Inspection or Delivery)
2. Choose additional mode (Low Battery Homing, Home Detection, Wall Following, Mapping Mode, or None)
3. The system automatically launches required nodes (SLAM, RViz, wall follower, ArUco detector, etc.)

### Setup (Required for all modes)

1. **Set TurtleBot3 model**:
   ```bash
   export TURTLEBOT3_MODEL=burger  # or waffle, waffle_pi
   ```

2. **Set ROS domain** (if using multiple robots):
   ```bash
   export ROS_DOMAIN_ID=6
   ```

3. **Source the workspace**:
   ```bash
   source ~/turtlebot3_ws/install/setup.bash
   ```

---

## Mode 1: Inspection Mode

**Purpose**: Detect and record damage sites (cracks) marked with ArUco markers throughout the warehouse.

### Overview
- Robot uses **right-hand wall following** to explore the entire space
- Camera detects ArUco markers (each ID represents a damage site)
- When ArUco markers are detected within stop distance, robot stops and records the location
- Battery monitoring ensures robot can return to charging station when needed
- Damage records are saved to disk in human-readable format

### Running Inspection Mode

#### Recommended: Using Mode Selector (Unified Interface)
```bash
# Terminal 1: Robot hardware/simulation
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Mode selector
ros2 run mode_selector robot_mode_selector
```

Then select:
1. **Robot Type**: `1` (Inspection Robot)
2. **Mode**: 
   - `1` = Low Battery Homing (wall following + automatic homing when battery < 20%)
   - `2` = Home Detection (wall following + ArUco damage detection)
   - `3` = Wall Following (pure exploration)
   - `4` = Mapping Mode (SLAM + wall following for map building)

The mode selector will automatically launch:
- ArUco detector (if needed)
- Battery monitor (if needed)
- Wall follower
- RViz2 (for mapping mode)
- SLAM Toolbox (for mapping mode)

#### Alternative: Manual Launch
```bash
# Terminal 1: Robot hardware/simulation
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: ArUco detection
ros2 launch aruco_detector aruco_detector.launch.py

# Terminal 3: Inspection mode (home detection)
ros2 run robot_drive home_detection

# Terminal 4 (Optional): Battery monitoring
ros2 launch battery_monitor battery_monitor.launch.py
```

### Inspection Mode Parameters

When running `home_detection`, you can adjust the stop distance:
```bash
ros2 run robot_drive home_detection --ros-args -p aruco_stop_distance:=0.25
```

Default: 0.30 meters (30cm)

### Inspection Workflow

1. **Mapping Phase** (Optional):
   - Run `wall_follower` or Cartographer SLAM to map the warehouse
   - Save the map for later use

2. **Inspection Phase**:
   - Launch inspection system
   - Robot explores warehouse using right-hand wall following
   - When ArUco marker (damage site) is detected:
     - Robot stops at 30cm distance
     - Records marker ID, position, timestamp
     - Saves to damage log file

3. **Battery Management**:
   - Battery monitor tracks virtual battery level
   - When battery < 20%, `low_battery_homing` activates
   - Robot navigates to ArUco ID 0 (charging station)
   - After charging, continues inspection

---

## Mode 2: Delivery Mode

**Purpose**: Deliver packages between predefined locations using Navigation2.

### Overview
- Robot uses **Navigation2** for path planning and obstacle avoidance
- No camera required (LiDAR and odometry only)
- **Mapping Mode**: Automatically builds map using SLAM and saves after 3 minutes
- **Delivery Mode**: Navigates to destinations using saved maps
- Tracks multiple delivery requests
- Saves delivery records to disk
- **Automatic Initial Pose**: Uses last known position from mapping to initialize localization

### Running Delivery Mode

#### Step 1: Launch Robot
```bash
# Terminal 1: Robot hardware/simulation
ros2 launch turtlebot3_bringup robot.launch.py
```

#### Step 2: Launch Mode Selector
```bash
# Terminal 2: Mode selector
ros2 run mode_selector robot_mode_selector
```

Select:
1. **Robot Type**: `2` (Delivery Robot)
2. **Mode**: `4` (Mapping Mode) - OR `5` (None) for delivery without mapping

#### Mapping Mode Workflow

If you selected Mapping Mode:
1. **Mapping Phase**:
   - SLAM Toolbox launches automatically
   - Wall follower launches and explores autonomously
   - RViz2 opens with map visualization
   - Robot explores for **3 minutes**
   - Map is automatically saved to `~/my_map_YYYYMMDD_HHMMSS`
   - Wall follower stops automatically

2. **After Mapping Completes**:
   - Menu appears with options:
     - `1` = Start Delivery Mode (launches Navigation2 with saved map)
     - `2` = Exit to Main Menu
     - `3` = Exit Program

3. **Delivery Phase** (if you chose option 1):
   - Navigation2 launches with the saved map
   - Initial pose is automatically set from last known position
   - Robot is ready for navigation
   - Use RViz2 tools to send goals:
     - **"2D Goal Pose"** tool → sends navigation goal
     - **"Publish Point"** tool → sends point to navigate to

#### Direct Delivery Mode (Using Existing Map)

If you selected "None" mode and have an existing map:

1. **Launch Navigation2**:
   ```bash
   # Terminal 2: Navigation2 with your map
   ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/path/to/your/map
   ```

2. **Launch Delivery Robot**:
   ```bash
   # Terminal 3: Delivery robot
   ros2 run mode_selector robot_mode_selector
   # Select: Delivery Robot, None mode
   ```

3. **Set Initial Pose**: Use "2D Pose Estimate" in RViz2

4. **Send Navigation Goals**: Use RViz2 "2D Goal Pose" or "Publish Point"

### Delivery Mode Features

- **Mapping Mode**: 
  - Automatic SLAM initialization
  - Autonomous exploration with wall follower
  - Automatic map saving after 3 minutes
  - Seamless transition to delivery mode
  
- **Navigation Features**:
  - Navigation2 integration for path planning
  - Automatic initial pose setting (from last mapping position)
  - RViz2 goal subscription (2D Goal Pose and Publish Point)
  - Dynamic obstacle avoidance using LiDAR
  
- **Delivery Management**:
  - Delivery request queue
  - Delivery logging to `/tmp/delivery_robot_log.txt`
  - Tracks completed deliveries
  
- **Route Management**: Waypoint-based navigation support (configurable)

---

## Mode Comparison

| Feature | Inspection Mode | Delivery Mode |
|---------|----------------|---------------|
| **Sensors Used** | Camera + LiDAR + Odometry | LiDAR + Odometry |
| **Navigation Method** | Right-hand wall following | Navigation2 (Nav2) |
| **Purpose** | Detect damage sites (ArUco markers) | Deliver packages to locations |
| **Mapping** | Optional (via Mapping Mode) | Built-in Mapping Mode (3 min auto-explore) |
| **Camera** | ✅ Required (ArUco detection) | ❌ Not used |
| **Battery Management** | ✅ Automatic homing (if enabled) | ❌ Not implemented |
| **Initial Pose** | Manual or automatic (via mode) | Automatic (from last mapping position) |
| **Output** | Damage log file (`/tmp/inspection_log.txt`) | Delivery log file (`/tmp/delivery_robot_log.txt`) |
| **Entry Point** | Mode Selector or manual launch | Mode Selector (recommended) |

---

## Key Topics

### Published Topics
- `/cmd_vel` (`geometry_msgs/TwistStamped`) - Robot velocity commands
- `/battery/level` (`std_msgs/Float32`) - Battery percentage
- `/aruco/pose/id_N` (`geometry_msgs/PoseStamped`) - ArUco marker poses
- `/wall_follow_enable` (`std_msgs/Bool`) - Enable/disable wall following

### Subscribed Topics
- `/scan` (`sensor_msgs/LaserScan`) - LiDAR data
- `/camera/image_raw` (`sensor_msgs/Image`) - Camera feed
- `/camera/camera_info` (`sensor_msgs/CameraInfo`) - Camera calibration

---

## Troubleshooting

### Common Issues

1. **"No executable found" error**:
   ```bash
   # Rebuild and source workspace
   cd ~/turtlebot3_ws
   colcon build
   source install/setup.bash
   ```

2. **ArUco markers not detected**:
   - Check camera is connected and publishing: `ros2 topic echo /camera/image_raw`
   - Verify ArUco detector is running: `ros2 node list | grep aruco`
   - Check marker size matches parameter in `aruco_detector` package

3. **Navigation2 not working**:
   - Ensure map is loaded correctly
   - Set initial pose in RViz2
   - Check that LiDAR is publishing: `ros2 topic echo /scan`

4. **Battery level not updating**:
   - Verify battery monitor node is running: `ros2 node list | grep battery`
   - Check topic: `ros2 topic echo /battery/level`

---

## Development Notes

### Polymorphic Design

The system demonstrates **object-oriented polymorphism** through:

#### Class Hierarchy
```
BaseRobot (Abstract Base Class)
    ├── InspectionRobot (Derived Class)
    └── DeliveryRobot (Derived Class)
```

#### Key Polymorphic Features:

1. **Virtual Interface**:
   - `BaseRobot` defines pure virtual functions: `initialize()` and `run()`
   - Each derived class implements these differently
   - Same function call (`robot->initialize()`) executes different code based on object type

2. **Shared Base Functionality**:
   - Motion control (`publishVelocity`, `stopRobot`)
   - Battery monitoring (`getBatteryLevel`)
   - Odometry tracking
   - All robots use the same base implementations

3. **Specialized Behaviors**:
   - **InspectionRobot**: ArUco detection, wall following, damage logging
   - **DeliveryRobot**: Navigation2 integration, mapping mode, delivery queue

4. **Runtime Polymorphism**:
   ```cpp
   std::shared_ptr<BaseRobot> robot;
   
   // User selects robot type at runtime
   if (choice == 1) {
       robot = std::make_shared<InspectionRobot>(mode);
   } else {
       robot = std::make_shared<DeliveryRobot>(mapping_mode);
   }
   
   // Same interface, different behavior
   robot->initialize();  // Calls InspectionRobot::initialize() OR DeliveryRobot::initialize()
   robot->run();          // Calls InspectionRobot::run() OR DeliveryRobot::run()
   ```

5. **Benefits**:
   - Single code path handles multiple robot types
   - Easy to extend with new robot types
   - Clean separation of shared vs. specialized functionality
   - Runtime type selection based on user input

### Code Organization

- All custom C++ code is in `src/` directory
- Each package is self-contained with its own CMakeLists.txt and package.xml
- Launch files organize nodes for different operational modes

---

## Authors

Aditya Solanki
## License

[Your License - MIT/Apache/etc.]

---

## Project Requirements Summary

This project fulfills the MTRX3760 Project 2 requirements:

✅ **Common Platform**: Single TurtleBot3 platform  
✅ **Polymorphic Design**: Shared and specialized robot behaviors  
✅ **Inspection Robot**: Camera-based damage detection with ArUco markers  
✅ **Delivery Robot**: Nav2-based package delivery  
✅ **Shared Functionality**: Battery monitoring, basic motion control  
✅ **Extensions**: 
- Battery-aware homing (automatic docking)
- Map building using SLAM
- Damage site logging

---

## Quick Reference

### Unified Interface (Recommended)
```bash
# Terminal 1: Robot hardware/simulation
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Mode selector (interactive menu)
ros2 run mode_selector robot_mode_selector
```

### Inspection Mode
```bash
# Via mode selector (select Inspection Robot + desired mode)
ros2 run mode_selector robot_mode_selector

# Or manually:
ros2 launch robot_drive low_battery_homing.launch.py
```

### Delivery Mode - Mapping
```bash
# Via mode selector (select Delivery Robot + Mapping Mode)
ros2 run mode_selector robot_mode_selector

# Mapping completes in 3 minutes, then choose "Start Delivery Mode"
```

### Delivery Mode - Direct Navigation
```bash
# Terminal 1: Robot
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Navigation2 with map
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/path/to/map

# Terminal 3: Delivery robot
ros2 run mode_selector robot_mode_selector
# Select: Delivery Robot, None mode
```

### Individual Nodes
```bash
# Wall following for mapping
ros2 run robot_drive wall_follower

# Inspection with ArUco detection
ros2 run robot_drive home_detection

# Battery monitoring
ros2 run battery_monitor battery_monitor

# ArUco detection
ros2 run aruco_detector aruco_pose_node

# Low battery homing
ros2 run robot_drive low_battery_homing
```

---

## References

- [ROS 2 Navigation2 Documentation](https://navigation.ros.org/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [ArUco Marker Detection](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
