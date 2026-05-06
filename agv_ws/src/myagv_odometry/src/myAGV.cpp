#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "myagv_odometry/myAGV.h"

//const unsigned char ender[2] = { 0x0d, 0x0a };
const unsigned char header[2] = { 0xfe, 0xfe };
const double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
const double STANDARD_GRAVITY = 9.80665;
//const int SPEED_INFO = 0xa55a;
//const int GET_SPEED = 0xaaaa;
//const double ROBOT_RADIUS = 105.00;
//const double ROBOT_LENGTH = 210.50;

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev);

boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };
boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };

void send()
{
    ;
}

void receive()
{
    ;
}

MyAGV::MyAGV() : private_n("~")
{
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    vx = 0.0;
    vy = 0.0;
    vtheta = 0.0;

    baud_rate = 115200;
    linear_scale = 1.0;
    lateral_scale = 1.0;
    angular_scale = 1.0;
    publish_imu = true;
    imu_frame_id = "imu_link";
    debug_output = false;
}

MyAGV::~MyAGV()
{
    ;
}

bool MyAGV::init()
{
    std::string serial_port;
    private_n.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
    private_n.param<int>("baud", baud_rate, 115200);
    private_n.param<double>("linear_scale", linear_scale, 1.0);
    private_n.param<double>("lateral_scale", lateral_scale, linear_scale);
    private_n.param<double>("angular_scale", angular_scale, 1.0);
    private_n.param<bool>("publish_imu", publish_imu, true);
    private_n.param<std::string>("imu_frame_id", imu_frame_id, "imu_link");
    private_n.param<bool>("debug_output", debug_output, false);

    if (baud_rate <= 0) {
        ROS_WARN("Invalid baud=%d; falling back to 115200", baud_rate);
        baud_rate = 115200;
    }
    if (linear_scale == 0.0) {
        ROS_WARN("Invalid linear_scale=0.0; falling back to 1.0");
        linear_scale = 1.0;
    }
    if (lateral_scale == 0.0) {
        ROS_WARN("Invalid lateral_scale=0.0; falling back to 1.0");
        lateral_scale = 1.0;
    }
    if (angular_scale == 0.0) {
        ROS_WARN("Invalid angular_scale=0.0; falling back to 1.0");
        angular_scale = 1.0;
    }

    sp.open(serial_port);
    ROS_INFO("Opened serial port: %s", serial_port.c_str());
    ROS_INFO("myAGV odom config: baud=%d linear_scale=%.6f lateral_scale=%.6f angular_scale=%.6f debug_output=%s",
             baud_rate, linear_scale, lateral_scale, angular_scale,
             debug_output ? "true" : "false");

    sp.set_option(boost::asio::serial_port::baud_rate(baud_rate));
    sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));

    ros::Time::init();
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();

    pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    if (publish_imu) {
        pub_imu = n.advertise<sensor_msgs::Imu>("imu", 50);
        ROS_INFO("Publishing base IMU on /imu with frame_id=%s", imu_frame_id.c_str());
    }

    return true;
}

bool MyAGV::readSpeed()
{
    int i, length = 0;
    unsigned char checkSum;
    unsigned char buf_header[1] = {0};
    unsigned char buf[16] = {0};

    size_t ret;
    boost::system::error_code er2;
    bool header_found = false;
    while (!header_found) {
        ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (ret != 1) {
            continue;
        }
        if (buf_header[0] != header[0]) {
            continue;
        }
        bool header_2_found = false;
        while (!header_2_found) {
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
            if (ret != 1) {
                continue;
            }
            if (buf_header[0] != header[0]) {
                continue;
            }
            header_2_found = true;
        }
        header_found = true;
    }

    // if (!(buf_header[0] == header[0] && buf_header[1] == header[1]))  {
    //     // not a header
    //     return false;
    // }

    ret = boost::asio::read(sp, boost::asio::buffer(buf), boost::asio::transfer_at_least(16), er2); // ready break
    if (ret != 16) {
        ROS_ERROR("Read error");
        return false;
    }
    // for (int i = 0; i < ret; ++i) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)(buf[i]) << " ";
    // }
    // std::cout << std::endl;


    // if (ret < 18) {
    //     //ROS_ERROR("Read less error");
    //     return false;
    // }
    // bool header_ok = false;
    // int header_idx = 0;
    // for (int i = 0; i < (ret-17); ++i) {
    //     if (buf[i] == header[0] && buf[i+1] == header[1])  {
    //         header_ok = true;
    //         header_idx = i;
    //         break;
    //     }
    // }
    // if (!header_ok) {
    //     //ROS_ERROR("Cannot find header");
    //     return false;
    // }


    //ROS_INFO("RED BYTES: %ul", ret);
	// if (er2 == boost::asio::error::eof){ 
	// 	// ROS_ERROR("asio error 1");
	// }


    // int index = 0;
    // for (index = 0; index < 40 - 17; ++index)
    // {
    //     if(buf[index] == header[0] && buf[index] == header[1])
    //         break;
    // }

    // if (index == 40 - 18)
    // {
    //     ROS_ERROR("Received message header error!");
    //     //return false;
    // }

    int index = 0;
    //index += 2;
    int check = 0;
    for (int i = 0; i < 15; ++i)
        check += buf[index + i];
    if (check % 256 != buf[index + 15])
	{
		ROS_ERROR("error 3!");	
    	return false;
	}

    vx = (static_cast<double>(buf[index]) - 128.0) * 0.01 * linear_scale;
    vy = (static_cast<double>(buf[index + 1]) - 128.0) * 0.01 * lateral_scale;
    vtheta = (static_cast<double>(buf[index + 2]) - 128.0) * 0.01 * angular_scale;

    ax = ((buf[index + 3] + buf[index + 4] * 256 ) - 10000) * 0.001 * STANDARD_GRAVITY;
    ay = ((buf[index + 5] + buf[index + 6] * 256 ) - 10000) * 0.001 * STANDARD_GRAVITY;
    az = ((buf[index + 7] + buf[index + 8] * 256 ) - 10000) * 0.001 * STANDARD_GRAVITY;

    wx = ((buf[index + 9] + buf[index + 10] * 256 ) - 10000) * 0.1 * DEG_TO_RAD;
    wy = ((buf[index + 11] + buf[index + 12] * 256 ) - 10000) * 0.1 * DEG_TO_RAD;
    wz = ((buf[index + 13] + buf[index + 14] * 256 ) - 10000) * 0.1 * DEG_TO_RAD;

    currentTime = ros::Time::now();

    double dt = (currentTime - lastTime).toSec();
    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
    double delta_th = vtheta * dt;

    x += delta_x;
    y += delta_y;
    theta += delta_th;
    lastTime = currentTime;

    // std::cout << "Received message is: " << dt << "|" << vx << "," << vy << "," << vtheta << "|"
    //                                       << ax << "," << ay << "," << az << "|"
    //                                     << wx << "," << wy << "," << wz << std::endl;
    // std::cout << "current pos is: " << x << "," << y << "," << theta << std::endl;

    return true;
}

void MyAGV::writeSpeed(double movex, double movey, double rot)
{
    if (movex > 1.0) movex = 1.0;
    if (movex < -1.0) movex = -1.0;
    if (movey > 1.0) movey = 1.0;
    if (movey < -1.0) movey = -1.0;
    if (rot > 1.0) rot = 1.0;
    if (rot < -1.0) rot = -1.0;

    //char x_send = static_cast<char>(movex * 100) + 128;
    //char y_send = static_cast<char>(movey * 100) + 128;
    //char rot_send = static_cast<char>(rot * 100) + 128;
   //char check = x_send + y_send + rot_send;
    unsigned char x_send = static_cast<signed char>(movex * 100) + 128;
    unsigned char y_send = static_cast<signed char>(movey * 100) + 128;
    unsigned char rot_send = static_cast<signed char>(rot * 100) + 128;
    unsigned char check = x_send + y_send + rot_send;

    char buf[8] = { 0 };
    buf[0] = header[0];
    buf[1] = header[1];
    buf[2] = x_send;
    buf[3] = y_send;
    buf[4] = rot_send;
    buf[5] = check;
    
    if (debug_output) {
        std::cout << "writeSpeed: " << movex << std::endl;
    }

    boost::asio::write(sp, boost::asio::buffer(buf));
}

void MyAGV::publishImuSensor()
{
    if (!publish_imu) {
        return;
    }

    sensor_msgs::Imu msg;
    msg.header.stamp = currentTime;
    msg.header.frame_id = imu_frame_id;

    // The base MCU packet contains accel and gyro only. Mark orientation as
    // unavailable so downstream filters do not treat the identity quaternion as
    // a measured attitude.
    msg.orientation.w = 1.0;
    msg.orientation_covariance[0] = -1.0;

    msg.angular_velocity.x = wx;
    msg.angular_velocity.y = wy;
    msg.angular_velocity.z = wz;

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

    pub_imu.publish(msg);
}

bool MyAGV::execute(double linearX, double linearY, double angularZ)
{
    if (debug_output) {
        std::cout << "execute: " << linearX << std::endl;
    }
    writeSpeed(linearX, linearY, angularZ);

    if (!readSpeed()) {
        return false;
    }

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(theta); // THETA
    odom_trans.transform.translation.x = x; //X
    odom_trans.transform.translation.y = y; //Y

    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odomBroadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry msgl;
    msgl.header.stamp = currentTime;
    msgl.header.frame_id = "odom";

    msgl.pose.pose.position.x = x;
    msgl.pose.pose.position.y = y;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_footprint";
    msgl.twist.twist.linear.x = vx;
    msgl.twist.twist.linear.y = vy;
    msgl.twist.twist.angular.z = vtheta;
    msgl.twist.covariance = odom_twist_covariance;

    pub.publish(msgl);
    publishImuSensor();
    return true;
}
