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

    # -----------------------------------------------------------------------
    # myagv_odometry node — uses ROS2-format params file
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Static TF publishers
    # ROS1 positional order was: x y z yaw pitch roll parent child
    # ROS2 uses named flags: --roll --pitch --yaw
    # -----------------------------------------------------------------------
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '--x',         '-0.132025',
            '--y',          '0.000153',
            '--z',          '0.187925',
            '--roll',       '1.570796',
            '--pitch',     '-0.007906',
            '--yaw',       '-1.570796',
            '--frame-id',   'base_footprint',
            '--child-frame-id', 'camera_link',
        ],
    )

    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_baselink',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '3.14159', '--pitch', '3.14159', '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'imu_link',
        ],
    )

    # -----------------------------------------------------------------------
    # YDLidar X2 — use AGV-specific params file (correct port: /dev/ttyAMA0)
    # -----------------------------------------------------------------------
    ydlidar_params = PathJoinSubstitution([
        FindPackageShare('agv_bringup'), 'config', 'ydlidar_x2_agv.yaml'])

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py',
            ])
        ]),
        launch_arguments={'params_file': ydlidar_params}.items(),
    )

    # -----------------------------------------------------------------------
    # RealSense D455 — ROS2 realsense2_camera parameter names
    # Profile format: "WIDTHxHEIGHTxFPS"
    # Camera IMU disabled (enable_accel/gyro = false)
    # -----------------------------------------------------------------------
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'align_depth.enable':               'true',
            'pointcloud.enable':                'false',
            'enable_sync':                      'true',
            'rgb_camera.color_profile':         '640x480x15',
            'depth_module.depth_profile':       '640x480x15',
            'enable_accel':                     'false',
            'enable_gyro':                      'false',
            'rgb_camera.enable_auto_exposure':  'true',
            'depth_module.enable_auto_exposure':'true',
        }.items(),
    )

    return LaunchDescription([
        serial_port_arg,
        odometry_node,
        static_tf_camera,
        static_tf_base,
        static_tf_imu,
        ydlidar_launch,
        realsense_launch,
    ])
