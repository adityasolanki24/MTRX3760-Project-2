# MTRX3760 2025 Project 2: Warehouse Robot DevKit
# File: battery_monitor.launch.py
# Author(s): Ayesha Musarrat
#
# Launches the battery monitor with sensible defaults. You can include this
# from your project's master launch file.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare command-line arguments 
    #for customisable drain and threshold values at launcg
    base_drain_arg = DeclareLaunchArgument(
        "base_drain_per_sec", default_value="1.0",
        description="Base percentage drain per second")
    low_threshold_arg = DeclareLaunchArgument(
        "low_threshold", default_value="20.0",
        description="Battery percentage threshold for low warning")

    return LaunchDescription([
        base_drain_arg,
        low_threshold_arg,

        Node(
            package="battery_monitor",
            executable="battery_monitor_node",
            name="battery_monitor",
            parameters=[{
                "base_drain_per_sec": LaunchConfiguration("base_drain_per_sec"),
                "low_threshold": LaunchConfiguration("low_threshold"),
            }],
            output="screen"
        )
    ])
