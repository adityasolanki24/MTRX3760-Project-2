# Battery Monitor Integration Guide

## 1. Overview

The `battery_monitor` package is a ROS 2 C++ node that tracks and simulates the robot’s battery status. It publishes the current battery percentage, detects low or critical levels, and logs messages accordingly. The node can also publish a `/dock_request` message when the battery falls below a set threshold, so other modules (like navigation or motion control) can handle automatic docking.


---

## 2. Integration and Setup Instructions

From our shared workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-github-username>/battery_monitor.git
cd ~/ros2_ws
colcon build --merge-install
source install/setup.bash
ros2 pkg list | grep battery_monitor

## 3. Launch Integration
To include this node in the system, add the following snippet to the project’s main launch file (for example, warehouse_robot.launch.py):



