#ifndef MYAGV_HPP
#define MYAGV_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/asio.hpp>
#include <array>
#include <string>
#include <memory>

class MyAGV : public rclcpp::Node
{
public:
  MyAGV();
  ~MyAGV() = default;
  bool init();
  void execute();

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  bool readSpeed();
  void writeSpeed(double movex, double movey, double rot);
  void publishImuSensor();

  rclcpp::Time currentTime, lastTime;

  double x{0.0}, y{0.0}, theta{0.0};
  double vx{0.0}, vy{0.0}, vtheta{0.0};
  double ax{0.0}, ay{0.0}, az{0.0};
  double wx{0.0}, wy{0.0}, wz{0.0};

  double linearX{0.0}, linearY{0.0}, angularZ{0.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  int baud_rate_{115200};
  double linear_scale_{1.0}, lateral_scale_{1.0}, angular_scale_{1.0};
  bool publish_imu_{true};
  std::string imu_frame_id_{"imu_link"};
  bool debug_output_{false};

  boost::asio::io_context io_;
  boost::asio::serial_port sp_;

  static const unsigned char HEADER = 0xfe;
  static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
  static constexpr double STANDARD_GRAVITY = 9.80665;

  std::array<double, 36> odom_pose_cov_{
    1e-3,0,0,0,0,0,
    0,1e-3,0,0,0,0,
    0,0,1e6,0,0,0,
    0,0,0,1e6,0,0,
    0,0,0,0,1e6,0,
    0,0,0,0,0,1e-3
  };
  std::array<double, 36> odom_twist_cov_{
    1e-3,0,0,0,0,0,
    0,1e-3,0,0,0,0,
    0,0,1e6,0,0,0,
    0,0,0,1e6,0,0,
    0,0,0,0,1e6,0,
    0,0,0,0,0,1e-3
  };
};

#endif  // MYAGV_HPP
