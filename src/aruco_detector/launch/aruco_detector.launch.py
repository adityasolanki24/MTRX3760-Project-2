from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('aruco_detector')
    params_file = os.path.join(pkg_share, 'params', 'aruco_params.yaml')
    
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    dictionary = LaunchConfiguration('dictionary')
    marker_length_m = LaunchConfiguration('marker_length_m')
    image_transport = LaunchConfiguration('image_transport')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('dictionary', default_value='DICT_6X6_250'),
        DeclareLaunchArgument('marker_length_m', default_value='0.05'),
        DeclareLaunchArgument('image_transport', default_value='raw'),  # or 'compressed'

        Node(
            package='aruco_detector',
            executable='aruco_detector_node',
            name='aruco_detector',
            output='screen',
            parameters=[
                params_file,
                {
                    'dictionary': dictionary,
                    'marker_length_m': marker_length_m,
                    'image_transport': image_transport,
                }
            ],
            remappings=[
                ('/camera/image_raw', image_topic),
                ('/camera/camera_info', camera_info_topic),
            ]
        )
    ])
