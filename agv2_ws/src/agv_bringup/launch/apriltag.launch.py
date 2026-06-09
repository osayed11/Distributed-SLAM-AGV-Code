"""ROS2 apriltag launch (port of agv_bringup/launch/apriltag.launch).

The ROS2 apriltag_ros node subscribes to image_rect and camera_info.
We remap to the RealSense colour topics (same as ROS1 which also used
the raw colour image for detection).
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    settings_yaml = PathJoinSubstitution([
        FindPackageShare('agv_bringup'), 'config', 'settings.yaml'])
    tags_yaml = PathJoinSubstitution([
        FindPackageShare('agv_bringup'), 'config', 'tags.yaml'])

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_ros',
        remappings=[
            ('/image_rect',   '/camera/color/image_raw'),
            ('/camera_info',  '/camera/color/camera_info'),
        ],
        parameters=[
            settings_yaml,
            tags_yaml,
        ],
        output='screen',
    )

    return LaunchDescription([
        apriltag_node,
    ])
