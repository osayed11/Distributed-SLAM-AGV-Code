"""ROS2 bringup launch for myAGV."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'agv_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the AGV base')

    color_profile_arg = DeclareLaunchArgument(
        'agv_color_profile',
        default_value='640x480x15',
        description='RealSense color stream profile (WIDTHxHEIGHTxFPS)')

    depth_profile_arg = DeclareLaunchArgument(
        'agv_depth_profile',
        default_value='640x480x15',
        description='RealSense depth stream profile (WIDTHxHEIGHTxFPS)')

    enable_sync_arg = DeclareLaunchArgument(
        'enable_sync',
        default_value='false',
        description='Enable RealSense hardware frame sync')

    imu_source_arg = DeclareLaunchArgument(
        'agv_imu_source',
        default_value='wrapper',
        description="'wrapper': D455 gyro/accel published by realsense2_camera (hard-stalls within minutes on every "
                    "fleet camera, measured 2 Nov 2026); 'standalone': wrapper runs video only and rs_imu_publisher "
                    "streams the motion module through librealsense directly (same topics, frames and extrinsics). 'none': wrapper runs video only and nothing publishes IMU (for external tools such as rs-data-collect).",
    )

    initial_reset_arg = DeclareLaunchArgument(
        'initial_reset',
        default_value='true',
        description='Reset the RealSense device during node startup')

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'agv_cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic that commands the AGV base')

    base_ros2_yaml = PathJoinSubstitution([
        FindPackageShare('agv_bringup'), 'config', 'base_ros2.yaml'])

    odometry_node = Node(
        package='myagv_odometry',
        executable='myagv_odometry_node',
        name='myagv_odometry_node',
        parameters=[
            base_ros2_yaml,
            {'serial_port': LaunchConfiguration('agv_serial_port')},
        ],
        remappings=[
            ('cmd_vel', LaunchConfiguration('agv_cmd_vel_topic')),
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
            'camera_namespace':                 '',
            'camera_name':                      'camera',
            'align_depth.enable':               'false',
            'pointcloud.enable':                'false',
            'enable_sync':                      LaunchConfiguration('enable_sync'),
            'rgb_camera.color_profile':         LaunchConfiguration('agv_color_profile'),
            'depth_module.depth_profile':       LaunchConfiguration('agv_depth_profile'),
            'depth_module.infra_profile':       LaunchConfiguration('agv_depth_profile'),
            'enable_accel':                     PythonExpression(["'true' if '", LaunchConfiguration('agv_imu_source'), "' == 'wrapper' else 'false'"]),
            'enable_gyro':                      PythonExpression(["'true' if '", LaunchConfiguration('agv_imu_source'), "' == 'wrapper' else 'false'"]),
            'unite_imu_method':                 '0',
            # With the wrapper running video-only next to the standalone IMU publisher its librealsense "global time"
            # estimate for colour/depth drifts by seconds (agv110, 3 Nov 2026: -1.0 s -> -2.8 s within 30 s); with the
            # option off the wrapper maps the device clock with a constant base and colour/depth/gyro agree within 8 ms.
            # rs_launch.py only forwards its own argument list, so these go in via its config_file hook.
            'config_file':                      PythonExpression(["'", PathJoinSubstitution([FindPackageShare('agv_bringup'), 'config', 'realsense_standalone_imu.yaml']), "' if '", LaunchConfiguration('agv_imu_source'), "' != 'wrapper' else \"''\""]),
            'enable_infra1':                    'false',
            'enable_infra2':                    'false',
            'rgb_camera.enable_auto_exposure':  'true',
            'depth_module.enable_auto_exposure':'true',
            'initial_reset':                    LaunchConfiguration('initial_reset'),
        }.items(),
    )

    # Standalone IMU path (docs/IMU_STANDALONE_FLEET_ROLLOUT.md): a capture process with no ROS in it owns the motion
    # module and forwards samples over UDP to a thin publisher node. Started 25 s after the wrapper: when both opened
    # the camera at the same instant the wrapper crashed (agv114, 2 Nov 2026).
    rs_imu_publisher = TimerAction(
        period=25.0,
        actions=[
            # capture process: librealsense only, no ROS (the collector path) -> UDP 127.0.0.1:27500
            ExecuteProcess(cmd=[PathJoinSubstitution([FindPackageShare('agv_bringup'), '..', '..', 'lib', 'agv_bringup', 'rs_imu_capture_supervised.sh']),
                                '--gyro-fps', '200', '--accel-fps', '100', '--port', '27500'], output='screen', name='rs_imu_capture'),
            # the supervisor restarts the capture only on exit code 3 (IMU silence): a fresh process recovers the camera
            # thin ROS publisher: UDP -> /camera/gyro/sample, /camera/accel/sample, static TF, extrinsics
            Node(package='agv_bringup', executable='rs_imu_udp_publisher_node', name='rs_imu_publisher', output='screen',
                 parameters=[{'port': 27500}]),
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('agv_imu_source'), "' == 'standalone'"])),
    )

    return LaunchDescription([
        imu_source_arg,
        serial_port_arg,
        color_profile_arg,
        depth_profile_arg,
        enable_sync_arg,
        initial_reset_arg,
        cmd_vel_topic_arg,
        odometry_node,
        static_tf_camera,
        static_tf_base,
        static_tf_imu,
        static_tf_laser,
        ydlidar_node,
        realsense_launch,
        rs_imu_publisher,
    ])
