from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='battery_monitor',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen'
        ),
        Node(
            package='aruco_detector',
            executable='aruco_pose_node',
            name='aruco_pose_node',
            output='screen'
        ),
        Node(
            package='robot_drive',
            executable='wall_follower',
            name='wall_follower',
            output='screen'
        ),
        Node(
            package='robot_drive',
            executable='low_battery_homing',
            name='low_battery_homing',
            output='screen'
        ),
    ])

