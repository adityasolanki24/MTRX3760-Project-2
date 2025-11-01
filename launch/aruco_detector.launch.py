from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
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
                'params/aruco_params.yaml',
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
