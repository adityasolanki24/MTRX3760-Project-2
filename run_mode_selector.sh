#!/bin/bash
# Script to run mode_selector with proper workspace sourcing

cd /home/aditya-solanki/turtlebot3_ws
source install/setup.bash
ros2 run mode_selector robot_mode_selector



