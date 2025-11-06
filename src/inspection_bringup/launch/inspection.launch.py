# MTRX3760 2025 Project 2: Warehouse Robot DevKit
# File: aruco_pose_node.launch.py
# Author(s): Raquel Kampel
#
# Launch file for the ArucoPoseNode and WallFollowerNode.
# The ArucoPoseNode performs ArUco marker pose estimation using
# camera image and camera info topics, while the WallFollowerNode
# handles wall-following navigation.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- Aruco Pose Node ---
    aruco_pose_node = Node(
        package='aruco_detector',
        executable='aruco_pose_node',
        name='aruco_pose_node',
        output='screen',
        parameters=[
            {'marker_length': 0.05},
            {'camera_frame': 'camera_optical_frame'},
            {'publish_tf': True},
            {'debug_image': True},
            {'pose_topic_base': '/aruco/pose'},
            {'dictionary': 'DICT_5X5_50'},
            {'camera_info_topic': '/camera/camera_info'},
            {'image_topic': '/camera/image_raw'}
        ]
    )

    # --- Wall Follower Node ---
    wall_follower_node = Node(
        package='robot_drive',
        executable='wall_follower',
        name='wall_follower',
        output='screen',
        parameters=[
            {'forward_speed': 0.15},
            {'turn_speed': 0.1},
            {'desired_distance_from_wall': 0.3}
        ]
    )

    # --- Launch both nodes ---
    return LaunchDescription([
        aruco_pose_node,
        wall_follower_node
    ])
