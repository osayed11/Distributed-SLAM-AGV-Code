#include "myagv_odometry/myAGV.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// -------------------------------------------------------------------------
// Constructor: declare ROS2 parameters with defaults
// -------------------------------------------------------------------------
MyAGV::MyAGV()
: Node("myagv_odometry_node"),
  sp_(io_)
{
  this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  this->declare_parameter<int>("baud", 115200);
  this->declare_parameter<double>("linear_scale", 1.0);
  this->declare_parameter<double>("lateral_scale", -1.0);   // sentinel: copy linear_scale
  this->declare_parameter<double>("angular_scale", 1.0);
  this->declare_parameter<bool>("publish_imu", true);
  this->declare_parameter<std::string>("imu_frame_id", "imu_link");
  this->declare_parameter<bool>("debug_output", false);
}

// -------------------------------------------------------------------------
// init(): read parameters, open serial port, create pub/sub/TF
// -------------------------------------------------------------------------
bool MyAGV::init()
{
  std::string serial_port = this->get_parameter("serial_port").as_string();
  baud_rate_              = this->get_parameter("baud").as_int();
  linear_scale_           = this->get_parameter("linear_scale").as_double();
  double lat_param        = this->get_parameter("lateral_scale").as_double();
  lateral_scale_          = (lat_param < 0.0) ? linear_scale_ : lat_param;
  angular_scale_          = this->get_parameter("angular_scale").as_double();
  publish_imu_            = this->get_parameter("publish_imu").as_bool();
  imu_frame_id_           = this->get_parameter("imu_frame_id").as_string();
  debug_output_           = this->get_parameter("debug_output").as_bool();

  // Open serial port via Boost.Asio
  try {
    sp_.open(serial_port);
    sp_.set_option(boost::asio::serial_port_base::baud_rate(
        static_cast<unsigned int>(baud_rate_)));
    sp_.set_option(boost::asio::serial_port_base::character_size(8));
    sp_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    sp_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    sp_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to open serial port %s: %s",
                 serial_port.c_str(), e.what());
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
              "Serial port %s opened at %d baud.",
              serial_port.c_str(), baud_rate_);

  // Publishers
  pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  if (publish_imu_) {
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 50);
  }

  // Subscriber
  sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&MyAGV::cmdCallback, this, std::placeholders::_1));

  // TF broadcaster
  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Initialise timestamps
  currentTime = this->now();
  lastTime    = this->now();

  RCLCPP_INFO(this->get_logger(),
              "myAGV init: linear_scale=%.3f lateral_scale=%.3f angular_scale=%.3f publish_imu=%s",
              linear_scale_, lateral_scale_, angular_scale_,
              publish_imu_ ? "true" : "false");
  return true;
}

// -------------------------------------------------------------------------
// cmd_vel callback: store latest velocity commands
// -------------------------------------------------------------------------
void MyAGV::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linearX  = msg->linear.x;
  linearY  = msg->linear.y;
  angularZ = msg->angular.z;
}

// -------------------------------------------------------------------------
// writeSpeed(): encode and send a 6-byte velocity frame
// -------------------------------------------------------------------------
void MyAGV::writeSpeed(double movex, double movey, double rot)
{
  // Clamp each component to [-1, 1]
  auto clamp1 = [](double v) -> double {
    return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v);
  };
  movex = clamp1(movex);
  movey = clamp1(movey);
  rot   = clamp1(rot);

  unsigned char x_send   = static_cast<unsigned char>(static_cast<int>(movex * 100) + 128);
  unsigned char y_send   = static_cast<unsigned char>(static_cast<int>(movey * 100) + 128);
  unsigned char rot_send = static_cast<unsigned char>(static_cast<int>(rot   * 100) + 128);
  unsigned char checksum = static_cast<unsigned char>(
      (static_cast<unsigned int>(x_send) +
       static_cast<unsigned int>(y_send) +
       static_cast<unsigned int>(rot_send)) % 256);

  unsigned char buf[6] = {HEADER, HEADER, x_send, y_send, rot_send, checksum};

  try {
    boost::asio::write(sp_, boost::asio::buffer(buf, 6));
  } catch (const boost::system::system_error & e) {
    RCLCPP_WARN(this->get_logger(), "writeSpeed error: %s", e.what());
  }
}

// -------------------------------------------------------------------------
// readSpeed(): read one 16-byte packet, decode odometry + IMU
// Returns true on a valid packet.
// -------------------------------------------------------------------------
bool MyAGV::readSpeed()
{
  // --- Find frame header (two consecutive 0xfe bytes) ---
  unsigned char byte = 0;
  unsigned char prev = 0;
  int attempts = 0;
  const int MAX_SCAN = 256;

  while (attempts < MAX_SCAN) {
    try {
      boost::asio::read(sp_, boost::asio::buffer(&byte, 1));
    } catch (const boost::system::system_error & e) {
      RCLCPP_WARN(this->get_logger(), "readSpeed header scan error: %s", e.what());
      return false;
    }
    if (prev == HEADER && byte == HEADER) {
      break;
    }
    prev = byte;
    ++attempts;
  }

  if (attempts >= MAX_SCAN) {
    RCLCPP_WARN(this->get_logger(), "readSpeed: could not find frame header");
    return false;
  }

  // --- Read 16 payload bytes (buf[0..15]) ---
  unsigned char buf[16];
  try {
    boost::asio::read(sp_, boost::asio::buffer(buf, 16));
  } catch (const boost::system::system_error & e) {
    RCLCPP_WARN(this->get_logger(), "readSpeed payload read error: %s", e.what());
    return false;
  }

  // --- Validate checksum: sum of buf[0..14] mod 256 == buf[15] ---
  unsigned int sum = 0;
  for (int i = 0; i < 15; ++i) {
    sum += static_cast<unsigned int>(buf[i]);
  }
  if ((sum % 256) != static_cast<unsigned int>(buf[15])) {
    RCLCPP_WARN(this->get_logger(), "readSpeed: checksum mismatch");
    return false;
  }

  // --- Decode velocities ---
  vx     = (static_cast<double>(buf[0]) - 128.0) * 0.01 * linear_scale_;
  vy     = (static_cast<double>(buf[1]) - 128.0) * 0.01 * lateral_scale_;
  vtheta = (static_cast<double>(buf[2]) - 128.0) * 0.01 * angular_scale_;

  // --- Decode IMU accelerometer (m/s^2) ---
  ax = (static_cast<double>(buf[3]) + static_cast<double>(buf[4]) * 256.0 - 10000.0)
       * 0.001 * STANDARD_GRAVITY;
  ay = (static_cast<double>(buf[5]) + static_cast<double>(buf[6]) * 256.0 - 10000.0)
       * 0.001 * STANDARD_GRAVITY;
  az = (static_cast<double>(buf[7]) + static_cast<double>(buf[8]) * 256.0 - 10000.0)
       * 0.001 * STANDARD_GRAVITY;

  // --- Decode IMU gyroscope (rad/s) ---
  wx = (static_cast<double>(buf[9])  + static_cast<double>(buf[10]) * 256.0 - 10000.0)
       * 0.1 * DEG_TO_RAD;
  wy = (static_cast<double>(buf[11]) + static_cast<double>(buf[12]) * 256.0 - 10000.0)
       * 0.1 * DEG_TO_RAD;
  wz = (static_cast<double>(buf[13]) + static_cast<double>(buf[14]) * 256.0 - 10000.0)
       * 0.1 * DEG_TO_RAD;

  if (debug_output_) {
    RCLCPP_INFO(this->get_logger(),
                "raw: vx=%.4f vy=%.4f vth=%.4f ax=%.4f ay=%.4f az=%.4f "
                "wx=%.4f wy=%.4f wz=%.4f",
                vx, vy, vtheta, ax, ay, az, wx, wy, wz);
  }

  return true;
}

// -------------------------------------------------------------------------
// publishImuSensor(): fill and publish sensor_msgs/Imu
// -------------------------------------------------------------------------
void MyAGV::publishImuSensor()
{
  if (!publish_imu_) {
    return;
  }

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp    = currentTime;
  imu_msg.header.frame_id = imu_frame_id_;

  // Orientation unavailable — signal with covariance[0] = -1
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = -1.0;

  imu_msg.angular_velocity.x = wx;
  imu_msg.angular_velocity.y = wy;
  imu_msg.angular_velocity.z = wz;

  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;

  pub_imu_->publish(imu_msg);
}

// -------------------------------------------------------------------------
// execute(): one control cycle — write speed, read odometry, publish
// Called from the main loop at 20 Hz.
// -------------------------------------------------------------------------
void MyAGV::execute()
{
  // Send velocity command
  writeSpeed(0.08 * linearX, 0.08 * linearY, 0.6 * angularZ);

  // Read sensor packet
  if (!readSpeed()) {
    return;
  }

  currentTime = this->now();

  // Compute dt
  double dt = (currentTime - lastTime).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    lastTime = currentTime;
    return;
  }

  // Integrate pose  (preserving the 2.0 * vtheta factor from original)
  double delta_x  = (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
  double delta_y  = (vx * std::sin(theta) + vy * std::cos(theta)) * dt;
  double delta_th = 2.0 * vtheta * dt;

  x     += delta_x;
  y     += delta_y;
  theta += delta_th;

  lastTime = currentTime;

  // Build quaternion from yaw
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(q);

  // ---- Publish odom → base_footprint TF ----
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp    = currentTime;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id  = "base_footprint";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation      = quat_msg;

  odom_broadcaster_->sendTransform(odom_trans);

  // ---- Publish nav_msgs/Odometry ----
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp    = currentTime;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id  = "base_footprint";

  odom_msg.pose.pose.position.x  = x;
  odom_msg.pose.pose.position.y  = y;
  odom_msg.pose.pose.position.z  = 0.0;
  odom_msg.pose.pose.orientation = quat_msg;

  for (std::size_t i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i]  = odom_pose_cov_[i];
    odom_msg.twist.covariance[i] = odom_twist_cov_[i];
  }

  odom_msg.twist.twist.linear.x  = vx;
  odom_msg.twist.twist.linear.y  = vy;
  odom_msg.twist.twist.angular.z = vtheta;

  pub_->publish(odom_msg);

  // ---- Publish IMU ----
  publishImuSensor();
}
