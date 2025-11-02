from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    aruco_node = Node(
        package='aruco_detector',
        executable='aruco_pose_node',
        name='aruco_pose_node',
        output='screen',
        parameters=[
            {'marker_length': 0.05},
            {'camera_frame': 'camera_optical_frame'},
            {'publish_tf': True},
            {'camera_info_topic': '/camera/camera_info'},
            {'image_topic': '/camera/image_raw'},
            {'log_file': '/home/raquel/turtlebot3_ws/crack_log.txt'}
        ]
    )

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

    return LaunchDescription([aruco_node, wall_follower_node])
