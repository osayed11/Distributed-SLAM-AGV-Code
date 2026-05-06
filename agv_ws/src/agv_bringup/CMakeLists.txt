cmake_minimum_required(VERSION 3.0.2)
project(agv_bringup)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rosbag
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_msgs
)

catkin_package()

install(DIRECTORY
  calibration
  config
  launch
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
