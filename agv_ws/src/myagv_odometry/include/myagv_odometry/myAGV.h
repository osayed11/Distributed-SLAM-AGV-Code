#ifndef MYAGV_H
#define MYAGV_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <string>


class MyAGV
{
public:
	MyAGV();
	~MyAGV();

	bool init();
	bool execute(double linearX, double linearY, double angularZ);

private:
	bool readSpeed();
	void writeSpeed(double movex, double movey, double rot);
	void publishImuSensor();

	ros::Time currentTime, lastTime;

	double x;
	double y;
	double theta;

	double vx;
	double vy;
	double vtheta;

	double ax;
	double ay;
	double az;

	double wx;
	double wy;
	double wz;

	ros::NodeHandle n;
	ros::NodeHandle private_n;
	ros::Publisher pub;
	ros::Publisher pub_imu;
	tf::TransformBroadcaster odomBroadcaster;

	int baud_rate;
	double linear_scale;
	double lateral_scale;
	double angular_scale;
	bool publish_imu;
	std::string imu_frame_id;
	bool debug_output;
};


#endif // !MYAGV_H
