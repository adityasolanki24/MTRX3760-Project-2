# MTRX3760 2025 Project 2: Warehouse Robot DevKit
# File: battery_monitor.launch.py
# Author(s): Ayesha Musarrat
#
# Launches the battery monitor with sensible defaults. You can include this
# from your project's master launch file.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='battery_monitor',
            executable='battery_monitor_node',
            name='battery_monitor',
            output='screen',
            parameters=[{
                'log_path': '/home/ubuntu/.ros/battery_log.txt',  # change to your VM user if needed
                'start_percent': 100.0,
                'base_drain_per_sec': 0.05,
                'velocity_drain_gain': 0.02,
                'low_threshold': 20.0,
                'critical_threshold': 10.0,
                'use_odom': False,   # set True on robot if you want speed-based drain
                'tick_ms': 200
            }]
        )
    ])
