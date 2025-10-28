# Battery Monitor Integration Guide

## 1. Overview

The `battery_monitor` package is a ROS 2 C++ node that tracks and simulates the robot’s battery status.  
It publishes the current battery percentage, detects low or critical levels, and logs messages accordingly.  
The node can also publish a `/dock_request` message when the battery falls below a set threshold, so other modules (like navigation or motion control) can handle automatic docking.

This package is implemented in C++ and can be reused for both the “delivery” and “inspection” robot types in the Warehouse Robot project.

---

## 2. Integration and Setup Instructions

From your shared workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-github-username>/battery_monitor.git
cd ~/ros2_ws
colcon build --merge-install
source install/setup.bash
ros2 pkg list | grep battery_monitor
You should see battery_monitor listed among the installed packages.
This confirms that the node has built successfully and can be launched from the workspace.

3. Launch Integration
To include this node in the system, add the following snippet to the project’s main launch file (for example, warehouse_robot.launch.py):

python
Copy code
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('battery_monitor'),
                'launch',
                'battery_monitor.launch.py'
            )
        )
    )

    return LaunchDescription([
        battery_launch,
        # Add other nodes here (navigation, task manager, etc.)
    ])
This ensures that the battery_monitor node launches automatically with all other system components.

4. Interaction with Other Nodes
Node	Role	Communication
/battery_monitor	Tracks battery percentage and state	Publishes /battery/state, logs to console
/motion_controller	Controls robot motion	May subscribe to /dock_request or /battery/state
/navigation_manager	Handles docking	Responds to /dock_request == true by moving to a charger pose
/task_manager	Oversees jobs	Can read /battery_monitor/low_threshold to decide when to pause or stop jobs

The battery_monitor runs independently and communicates through ROS 2 topics and parameters.
Other nodes can use this data without requiring any code changes in their logic.

5. Docking Integration
To enable automatic docking when the battery is low, teammates can add this code to their motion or docking node:

cpp
Copy code
auto dock_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/dock_request", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_WARN(this->get_logger(), "Dock request received. Navigating to charger...");
            navigateToDockPose();  // Replace with the team’s docking function
        }
    });
This ensures the docking node responds when the battery monitor issues a request.

6. Adding the /dock_request Publisher (Battery Monitor Side)
To prepare your own package for integration with docking behaviour, make these edits:

In include/battery_monitor/battery.hpp:
cpp
Copy code
#include "std_msgs/msg/bool.hpp"
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dock_pub_;
In src/battery.cpp (inside the constructor):
cpp
Copy code
dock_pub_ = this->create_publisher<std_msgs::msg::Bool>("/dock_request", 10);
Inside the periodic update or check function:
cpp
Copy code
if (battery_level_ <= low_threshold_) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    dock_pub_->publish(msg);
    RCLCPP_WARN(this->get_logger(), "Low battery detected. Dock request issued.");
}
This logic publishes a Boolean message whenever the battery level drops below the low-battery threshold.
Other nodes can subscribe to /dock_request to handle docking automatically.

7. Default Parameters and Launch Overrides
Parameter	Default	Description
start_percent	100.0	Starting battery level
base_drain_per_sec	0.5	Drain rate per second
velocity_drain_gain	0.1	Drain multiplier based on velocity
low_threshold	20.0	Battery percentage that triggers a low-battery alert
critical_threshold	10.0	Critical alert threshold
tick_ms	1000	Update interval in milliseconds

To override any parameters at runtime:

bash
Copy code
ros2 launch battery_monitor battery_monitor.launch.py low_threshold:=25.0 base_drain_per_sec:=0.8
