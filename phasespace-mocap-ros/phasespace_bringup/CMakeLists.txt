cmake_minimum_required(VERSION 3.0.2)
project(phasespace_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  phasespace_msgs
  roscpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS phasespace_msgs roscpp std_msgs geometry_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

# linking directories
link_directories(${catkin_LIBRARY_DIRS} bin)

# adding owlsock library
add_library(owlsock SHARED IMPORTED)
set_target_properties(owlsock PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/bin/libowlsock.so)

add_executable(${PROJECT_NAME}_node src/phasespace_bringup_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  owlsock
  ${catkin_LIBRARIES}
)

add_executable(phasespace_tf_visualization src/phasespace_tf_visualization.cpp)
target_link_libraries(phasespace_tf_visualization ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############

