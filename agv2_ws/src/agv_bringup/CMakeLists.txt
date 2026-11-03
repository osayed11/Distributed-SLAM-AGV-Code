cmake_minimum_required(VERSION 3.8)
project(agv_bringup)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2 REQUIRED)

# Standalone D455 IMU path (see docs/IMU_STANDALONE_FLEET_ROLLOUT.md):
#   rs_imu_capture            librealsense only, no ROS; motion module -> UDP 127.0.0.1:27500
#   rs_imu_capture_supervised restarts the capture only when it exits for IMU silence (fresh process = recovery)
#   rs_imu_udp_publisher_node UDP -> /camera/gyro/sample, /camera/accel/sample, static TF, extrinsics
add_executable(rs_imu_capture src/rs_imu_capture.cpp)
target_compile_features(rs_imu_capture PUBLIC cxx_std_17)
target_link_libraries(rs_imu_capture ${realsense2_LIBRARY} pthread)
target_include_directories(rs_imu_capture PRIVATE ${realsense2_INCLUDE_DIR})

add_executable(rs_imu_udp_publisher_node src/rs_imu_udp_publisher_node.cpp)
target_compile_features(rs_imu_udp_publisher_node PUBLIC cxx_std_17)
ament_target_dependencies(rs_imu_udp_publisher_node rclcpp sensor_msgs geometry_msgs tf2_ros realsense2_camera_msgs)

install(DIRECTORY launch config calibration
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/rs_imu_capture_supervised.sh DESTINATION lib/${PROJECT_NAME})
install(TARGETS rs_imu_capture rs_imu_udp_publisher_node DESTINATION lib/${PROJECT_NAME})

ament_package()
