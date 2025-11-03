# MTRX3760 Project 2: Warehouse Robot DevKit

## Overview

This project implements a **polymorphic warehouse robotics system** that emulates two different robot types on a single TurtleBot3 platform:

1. **Inspection Robot**: Uses camera, LiDAR, and odometry to detect and record damage sites (cracks) in the warehouse
2. **Delivery Robot**: Uses LiDAR and odometry only to deliver packages between locations using Navigation2

The system demonstrates a common development platform approach with shared functionality and specialized behaviors for different robot types.

---

## System Architecture

### Core Components

#### 1. **ArUco Detector Node** (`aruco_pose_node`)
- **Location**: `src/aruco_detector/src/aruco_pose_node.cpp`
- **Function**: Detects ArUco markers using the camera and publishes pose information
- **Topics Published**: 
  - `/aruco/pose/id_0` (charging station/home)
  - `/aruco/pose/id_1` (inspection target)
  - `/aruco/pose/id_N` (for other marker IDs)
- **Output**: `geometry_msgs/PoseStamped` messages with marker position and distance from camera

#### 2. **Battery Monitor Node** (`battery_monitor_node`)
- **Location**: `src/battery_monitor/src/battery_monitor_node.cpp`
- **Function**: Simulates and monitors virtual battery level
- **Topics Published**: `/battery/level` (`std_msgs/Float32`)
- **Topics Subscribed**: Monitors battery consumption based on robot activity

#### 3. **Wall Follower** (`wall_follower`)
- **Location**: `src/robot_drive/src/wall_follower.cpp`
- **Function**: Pure right-hand wall following algorithm for autonomous exploration
- **Use Case**: Mapping and SLAM - allows robot to roam around the maze to build maps
- **Output**: Velocity commands to `/cmd_vel` (`geometry_msgs/TwistStamped`)

#### 4. **Home Detection** (`home_detection`)
- **Location**: `src/robot_drive/src/home_detection.cpp`
- **Function**: Right-hand wall following with ArUco ID 1 detection for inspection mode
- **Behavior**: 
  - Follows right wall using PID control
  - Subscribes to `/aruco/pose/id_1` for damage site detection
  - Stops when ArUco marker ID 1 is detected within 30cm distance
  - Records damage locations for inspection reporting
- **Parameters**: `aruco_stop_distance` (default: 0.30 meters)

#### 5. **Low Battery Homing** (`low_battery_homing`)
- **Location**: `src/robot_drive/src/low_battery_homing.cpp`
- **Function**: Monitors battery level and navigates to charging station (ArUco ID 0) when battery < 20%
- **Behavior**:
  - Subscribes to `/battery/level` from battery monitor
  - Subscribes to `/aruco/pose/id_0` for charging station location
  - Disables wall following when battery is low
  - Uses right-hand wall following to navigate to ArUco ID 0
  - Re-enables wall following after docking
- **Threshold**: 20% battery level

#### 6. **Delivery Commander** (For Delivery Mode)
- **Location**: `src/mode_selector/src/delivery_robot.cpp`
- **Function**: Manages delivery requests and waypoints using Navigation2
- **Behavior**: Coordinates delivery routes, tracks multiple delivery requests, saves delivery logs

---

## Project Structure

```
turtlebot3_ws/
├── src/
│   ├── aruco_detector/          # ArUco marker detection
│   ├── battery_monitor/         # Battery simulation and monitoring
│   ├── mode_selector/           # Delivery and inspection robot management
│   ├── robot_drive/             # Wall following algorithms
│   │   ├── src/
│   │   │   ├── wall_follower.cpp        # Pure wall following (mapping)
│   │   │   ├── home_detection.cpp       # Inspection mode (ArUco detection)
│   │   │   └── low_battery_homing.cpp   # Battery-aware navigation
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
- When ArUco ID 1 is detected within 30cm, robot stops and records the location
- Battery monitoring ensures robot can return to charging station when needed
- Damage records are saved to disk in human-readable format

### Running Inspection Mode

#### Option A: Complete Inspection System (with battery monitoring)
```bash
ros2 launch robot_drive low_battery_homing.launch.py
```

This launches:
- `battery_monitor` - Virtual battery monitoring
- `aruco_pose_node` - ArUco marker detection
- `wall_follower` - Base wall following (can be replaced with `home_detection`)
- `low_battery_homing` - Battery-aware navigation

#### Option B: Inspection with Damage Detection
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

#### Option C: Mapping for Inspection
```bash
# For building maps before inspection
ros2 run robot_drive wall_follower

# Or with SLAM:
ros2 launch turtlebot3_cartographer cartographer.launch.py
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
- Follows predefined delivery routes
- Tracks multiple delivery requests
- Saves delivery records to disk

### Prerequisites for Delivery Mode

1. **Map Required**: You need a pre-built map of the warehouse
   ```bash
   # Build map first using SLAM
   ros2 launch turtlebot3_cartographer cartographer.launch.py
   # Or use existing map in turtlebot3_navigation2/map/
   ```

2. **Mode Selector**: Launches delivery robot with mapping and navigation capabilities

### Running Delivery Mode

#### Step 1: Launch Robot
```bash
# Terminal 1: Robot hardware
ros2 launch turtlebot3_bringup robot.launch.py
```

#### Step 2: Launch Navigation2
```bash
# Terminal 2: Navigation2 stack
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

#### Step 3: Set Initial Pose (in RViz2)
- Open RViz2 (launched automatically with Nav2)
- Use "2D Pose Estimate" tool to set robot's starting position on the map

#### Step 4: Send Navigation Goals
```bash
# Using RViz2
# - Use "2D Goal Pose" tool in RViz2 to set destination
# - Use "Publish Point" tool to navigate to a point
# 
# The delivery robot automatically subscribes to these goals and navigates to them
```

### Delivery Mode Features

- **Path Planning**: Nav2 computes optimal paths using the map
- **Obstacle Avoidance**: Dynamic obstacle detection using LiDAR
- **Route Management**: Can follow multiple waypoints in sequence
- **Delivery Logging**: Records each delivery with timestamp and location

---

## Mode Comparison

| Feature | Inspection Mode | Delivery Mode |
|---------|----------------|---------------|
| **Sensors Used** | Camera + LiDAR + Odometry | LiDAR + Odometry |
| **Navigation Method** | Right-hand wall following | Navigation2 (Nav2) |
| **Purpose** | Detect damage sites (ArUco markers) | Deliver packages to locations |
| **Mapping** | Required for localization | Required for path planning |
| **Camera** | ✅ Required (ArUco detection) | ❌ Not used |
| **Battery Management** | ✅ Automatic homing | ❌ Not implemented |
| **Output** | Damage log file | Delivery log file |

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

The system demonstrates polymorphism through:
- **Shared base functionality**: Wall following, battery management, basic motion
- **Specialized behaviors**: 
  - Inspection: ArUco detection and damage logging
  - Delivery: Nav2 path planning and route management

### Code Organization

- All custom C++ code is in `src/` directory
- Each package is self-contained with its own CMakeLists.txt and package.xml
- Launch files organize nodes for different operational modes

---

## Authors

Aditya Soalnki
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

### Inspection Mode (One-Line Launch)
```bash
ros2 launch robot_drive low_battery_homing.launch.py
```

### Delivery Mode (One-Line Launch)
```bash
# Terminal 1: Robot
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Navigation2
ros2 launch turtlebot3_navigation2 navigation2.launch.py
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
