# Warehouse Robot Dashboard

A simple web-based UI for displaying ROS2 topics and robot status for MTRX3760 Project 2.

## Features

- 📊 Real-time robot status display (Inspection/Delivery mode)
- 🔋 Battery level monitoring
- 📷 Camera feed display
- 📝 Damage and delivery record tracking
- 📝 System logs in terminal-like interface
- 🎮 Control panel for mode switching

## Quick Start

### Option 1: Standalone (Demo Mode)

Just open the HTML file in your browser for a demo:

```bash
# Open in your default browser
xdg-open robot_dashboard.html

# Or use any browser
firefox robot_dashboard.html
```

This runs in simulation mode with demo data.

### Option 2: Connect to ROS2 (Real Data)

1. **Start the ROS2 bridge:**
   ```bash
   cd /home/aditya-solanki/turtlebot3_ws
   export ROS_DOMAIN_ID=6  # or your domain
   source install/setup.bash
   python3 robot_dashboard_bridge.py
   ```

2. **Open the dashboard:**
   Open your browser and go to: http://localhost:8080

3. **View the dashboard:**
   The dashboard will show:
   - Real robot mode and battery status
   - Camera feed (if /camera/image_raw is publishing)
   - Damage/delivery records
   - System logs from ROS2

## Integration with Your Robot Code

### Publishing Robot State

In your C++ robot nodes, publish mode changes:

```cpp
#include <std_msgs/msg/string.hpp>

// Publisher
auto mode_pub = create_publisher<std_msgs::msg::String>("/robot/mode", 10);

// When switching mode
std_msgs::msg::String mode_msg;
mode_msg.data = "INSPECTION";  // or "DELIVERY", "IDLE"
mode_pub->publish(mode_msg);
```

### Publishing Battery Level

```cpp
#include <sensor_msgs/msg/battery_state.hpp>

// Publisher
auto battery_pub = create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery_state", 10);

// Update battery
sensor_msgs::msg::BatteryState battery_msg;
battery_msg.percentage = 75.0;  // 0-100
battery_pub->publish(battery_msg);
```

### Camera Feed

Your dashboard will automatically display `/camera/image_raw` when available.

## API Endpoints

The bridge provides these endpoints:

- `GET /api/state` - Get current robot state (JSON)
- `GET /api/camera` - Get camera frame (JPEG)
- `POST /api/mode` - Set robot mode

## Customization

### Modify Dashboard Appearance

Edit `robot_dashboard.html` and customize:

- Colors: Change CSS variables in `<style>` section
- Layout: Modify grid layout in `.container`
- Panels: Add/remove panels as needed

### Add Custom Topics

Edit `robot_dashboard_bridge.py`:

1. Subscribe to new topics in `__init__`
2. Add new variables in `self.__init__`
3. Update `get_state()` to return new data
4. Update the HTML to display the new data

## Troubleshooting

### Can't see camera feed?
- Check that `/camera/image_raw` topic is being published:
  ```bash
  ros2 topic hz /camera/image_raw
  ```

### Dashboard shows "Disconnected"?
- Make sure the bridge script is running
- Check ROS_DOMAIN_ID matches
- Verify ports 8080 is not in use

### No battery data?
- Publish to `/battery_state` topic:
  ```bash
  ros2 topic pub /battery_state sensor_msgs/msg/BatteryState "{}" --once
  ```

## For Presentation

During your demo, this dashboard provides:

1. **Visual Status**: Shows robot mode and battery at a glance
2. **Camera Feed**: Displays what robot sees
3. **Activity Log**: Shows robot actions in real-time
4. **Records**: Displays damage findings and deliveries

This makes it easy for audience to follow what the robot is doing!

## Files

- `robot_dashboard.html` - Web dashboard UI
- `robot_dashboard_bridge.py` - ROS2-to-web bridge
- `DASHBOARD_README.md` - This file

