#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <phasespace_msgs/Rigids.h>
#include <phasespace_msgs/Cameras.h>

// Rigid topic callback function, will be called when there is new messages on the topic.
// It also publishes tf of rigid bodies
void poseRigidCallback(const phasespace_msgs::Rigids &msg) {
	// Transform broadcaster
	static tf2_ros::TransformBroadcaster br;
	
	for(size_t i = 0; i < msg.rigids.size(); i++) {
		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = ros::Time::now();
	  	transformStamped.header.frame_id = "phasespace_base";
	  	transformStamped.child_frame_id = "robot_" + std::to_string(msg.rigids[i].id);
	  	
	  	transformStamped.transform.translation.x = msg.rigids[i].x;
	  	transformStamped.transform.translation.y = msg.rigids[i].y;
	  	transformStamped.transform.translation.z = msg.rigids[i].z;

	  	transformStamped.transform.rotation.x = msg.rigids[i].qx;
	  	transformStamped.transform.rotation.y = msg.rigids[i].qy;
	  	transformStamped.transform.rotation.z = msg.rigids[i].qz;
	  	transformStamped.transform.rotation.w = msg.rigids[i].qw;
	 
	 		// Publishing the tf
	  	br.sendTransform(transformStamped);
  	}
}

// Camera topic callback function, will be called when there is new messages on the topic.
// It also publishes static tf of cameras
void staticCameraTf(const phasespace_msgs::Cameras &msg) {
	// Static Transform broadcaster
	static tf2_ros::StaticTransformBroadcaster static_br;
	
	for(size_t i = 0; i < msg.cameras.size(); i++) {
		geometry_msgs::TransformStamped static_transformStamped;

		static_transformStamped.header.stamp = ros::Time::now();
	  	static_transformStamped.header.frame_id = "phasespace_base";
	  	static_transformStamped.child_frame_id = "camera_" + std::to_string(msg.cameras[i].id);

	  	static_transformStamped.transform.translation.x = msg.cameras[i].x;
	  	static_transformStamped.transform.translation.y = msg.cameras[i].y;
	  	static_transformStamped.transform.translation.z = msg.cameras[i].z;

	  	static_transformStamped.transform.rotation.x = msg.cameras[i].qx;
	  	static_transformStamped.transform.rotation.y = msg.cameras[i].qy;
	  	static_transformStamped.transform.rotation.z = msg.cameras[i].qz;
	  	static_transformStamped.transform.rotation.w = msg.cameras[i].qw;
	 
	 		// Publishing the static tf
	  	static_br.sendTransform(static_transformStamped);
	}
}

int main(int argc, char** argv) {
	// Initializing the phasespace tf visualization node
  ros::init(argc, argv, "phasespace_tf_visualization");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("TF Publishing started.");
  
  // Rigid and Camera Sunscriber handler
  ros::Subscriber rigidSub = nh.subscribe("/phasespace/rigids", 1000, poseRigidCallback);
  ros::Subscriber cameraSub = nh.subscribe("/phasespace/cameras", 1000, staticCameraTf);
  
  ros::spin();  
  return 0;
}
