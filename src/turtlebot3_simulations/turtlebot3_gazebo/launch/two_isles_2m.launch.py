#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Start Gazebo with the custom 2m square world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'two_isles_2m.world')]
        }.items(),
    )

    # Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py'),
        ),
    )

    # Spawn TurtleBot3 near bottom center inside the square
    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'),
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '-0.7',
            'z_pose': '0.01',
        }.items(),
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_turtlebot3,
    ])



