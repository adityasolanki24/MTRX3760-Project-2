from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    marker_length = LaunchConfiguration('marker_length', default='0.05')
    camera_frame = LaunchConfiguration('camera_frame', default='camera_rgb_optical_frame')
    dictionary = LaunchConfiguration('dictionary', default='DICT_5X5_50')
    publish_tf = LaunchConfiguration('publish_tf', default='true')
    image_topic = LaunchConfiguration('image_topic', default='/camera/image_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/camera/camera_info')

    return LaunchDescription([
        DeclareLaunchArgument('marker_length', default_value='0.05',
                              description='Marker side length in meters'),
        DeclareLaunchArgument('camera_frame', default_value='camera_rgb_optical_frame',
                              description='Camera optical frame name'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50',
                              description='ArUco marker dictionary'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Whether to publish TF transforms'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw',
                              description='Camera image topic'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info',
                              description='Camera info topic'),

        Node(
            package='aruco_detector',
            executable='aruco_pose_node',
            name='aruco_pose_node',
            output='screen',
            parameters=[{
                'marker_length': marker_length,
                'camera_frame': camera_frame,
                'dictionary': dictionary,
                'publish_tf': publish_tf,
            }],
            remappings=[
                ('/camera/image_raw', image_topic),
                ('/camera/camera_info', camera_info_topic),
            ]
        )
    ])

