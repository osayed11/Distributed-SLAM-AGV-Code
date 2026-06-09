"""ROS2 bringup launch for myAGV (port of agv_bringup/launch/bringup.launch).

Static TF values are taken directly from the ROS1 bringup.launch.
ROS1 static_transform_publisher positional argument order was:
  x y z yaw pitch roll frame_id child_frame_id

ROS2 static_transform_publisher uses named flags:
  --x --y --z --roll --pitch --yaw --frame-id --child-frame-id
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -----------------------------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------------------------
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the AGV base')

    enable_realsense_sync_arg = DeclareLaunchArgument(
        'enable_realsense_sync',
        default_value='true',
        description='Enable RealSense hardware sync')

    color_width_arg = DeclareLaunchArgument(
        'color_width', default_value='640')
    color_height_arg = DeclareLaunchArgument(
        'color_height', default_value='480')
    color_fps_arg = DeclareLaunchArgument(
        'color_fps', default_value='15')
    depth_width_arg = DeclareLaunchArgument(
        'depth_width', default_value='640')
    depth_height_arg = DeclareLaunchArgument(
        'depth_height', default_value='480')
    depth_fps_arg = DeclareLaunchArgument(
        'depth_fps', default_value='15')

    # -----------------------------------------------------------------------
    # myagv_odometry node
    # -----------------------------------------------------------------------
    base_yaml = PathJoinSubstitution([
        FindPackageShare('agv_bringup'), 'config', 'base.yaml'])

    odometry_node = Node(
        package='myagv_odometry',
        executable='myagv_odometry_node',
        name='myagv_odometry_node',
        parameters=[
            base_yaml,
            {'serial_port': LaunchConfiguration('serial_port')},
        ],
        output='screen',
    )

    # -----------------------------------------------------------------------
    # Static TF: base_footprint -> camera_link
    # ROS1 args (yaw pitch roll): -1.570796 -0.007906 1.570796
    # ROS2 flags --roll --pitch --yaw
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

    # -----------------------------------------------------------------------
    # Static TF: base_footprint -> base_link (all zeros)
    # -----------------------------------------------------------------------
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_baselink',
        arguments=[
            '--x',         '0',
            '--y',          '0',
            '--z',          '0',
            '--roll',       '0',
            '--pitch',      '0',
            '--yaw',        '0',
            '--frame-id',   'base_footprint',
            '--child-frame-id', 'base_link',
        ],
    )

    # -----------------------------------------------------------------------
    # Static TF: base_footprint -> imu_link
    # ROS1 args (yaw pitch roll): 0 3.14159 3.14159
    # -----------------------------------------------------------------------
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '--x',         '0',
            '--y',          '0',
            '--z',          '0',
            '--roll',       '3.14159',
            '--pitch',      '3.14159',
            '--yaw',        '0',
            '--frame-id',   'base_footprint',
            '--child-frame-id', 'imu_link',
        ],
    )

    # -----------------------------------------------------------------------
    # YDLidar ROS2 driver
    # -----------------------------------------------------------------------
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py',
            ])
        ]),
    )

    # -----------------------------------------------------------------------
    # RealSense camera (ROS2 realsense2_camera)
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
            'align_depth.enable':    'true',
            'pointcloud.enable':     'false',
            'enable_sync':           LaunchConfiguration('enable_realsense_sync'),
            'rgb_camera.color_profile': [
                LaunchConfiguration('color_width'), 'x',
                LaunchConfiguration('color_height'), 'x',
                LaunchConfiguration('color_fps'),
            ],
            'depth_module.depth_profile': [
                LaunchConfiguration('depth_width'), 'x',
                LaunchConfiguration('depth_height'), 'x',
                LaunchConfiguration('depth_fps'),
            ],
            'enable_accel':          'false',
            'enable_gyro':           'false',
            'rgb_camera.enable_auto_exposure':        'true',
            'rgb_camera.enable_auto_white_balance':   'true',
            'depth_module.enable_auto_exposure':      'true',
        }.items(),
    )

    return LaunchDescription([
        serial_port_arg,
        enable_realsense_sync_arg,
        color_width_arg,
        color_height_arg,
        color_fps_arg,
        depth_width_arg,
        depth_height_arg,
        depth_fps_arg,
        odometry_node,
        static_tf_camera,
        static_tf_base,
        static_tf_imu,
        ydlidar_launch,
        realsense_launch,
    ])
