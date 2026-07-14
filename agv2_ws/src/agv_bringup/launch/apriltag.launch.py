"""ROS2 apriltag launch for the AGV dataset stack.

The ROS2 apriltag_ros node subscribes to image_rect and camera_info.
We remap to the RealSense colour topics used by the D455 bringup.
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
