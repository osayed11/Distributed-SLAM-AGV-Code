"""ROS2 bringup launch for myAGV."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the AGV base')

    base_ros2_yaml = PathJoinSubstitution([
        FindPackageShare('agv_bringup'), 'config', 'base_ros2.yaml'])

    odometry_node = Node(
        package='myagv_odometry',
        executable='myagv_odometry_node',
        name='myagv_odometry_node',
        parameters=[
            base_ros2_yaml,
            {'serial_port': LaunchConfiguration('serial_port')},
        ],
        output='screen',
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '--x', '-0.132025', '--y', '0.000153', '--z', '0.187925',
            '--roll', '1.570796', '--pitch', '-0.007906', '--yaw', '-1.570796',
            '--frame-id', 'base_footprint', '--child-frame-id', 'camera_link',
        ],
    )

    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_baselink',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
        ],
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '3.14159', '--pitch', '3.14159', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'imu_link',
        ],
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.10',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'laser_frame',
        ],
    )

    ydlidar_params = PathJoinSubstitution([
        FindPackageShare('agv_bringup'), 'config', 'ydlidar_x2_agv.yaml'])

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[ydlidar_params],
        output='screen',
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'align_depth.enable':               'true',
            'pointcloud.enable':                'false',
            'enable_sync':                      'false',
            'rgb_camera.color_profile':         '640x480x15',
            'depth_module.depth_profile':       '640x480x15',
            'depth_module.infra_profile':       '640x480x15',
            'enable_accel':                     'false',
            'enable_gyro':                      'false',
            'enable_infra1':                    'false',
            'enable_infra2':                    'false',
            'rgb_camera.enable_auto_exposure':  'true',
            'depth_module.enable_auto_exposure':'true',
            'initial_reset':                    'false',
        }.items(),
    )

    return LaunchDescription([
        serial_port_arg,
        odometry_node,
        static_tf_camera,
        static_tf_base,
        static_tf_imu,
        static_tf_laser,
        ydlidar_node,
        realsense_launch,
    ])
