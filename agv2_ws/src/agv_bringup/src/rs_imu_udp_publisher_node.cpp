// rs_imu_udp_publisher_node: receives rs_imu_capture's UDP samples and publishes /camera/gyro/sample and
// /camera/accel/sample (sensor-data QoS), static TF camera_link->camera_{gyro,accel}_frame->..._optical_frame and
// latched extrinsics, exactly like the wrapper. Stamps: device clock + running-minimum (host_t - device_t) offset.
// No librealsense in this process: DDS activity here cannot disturb the capture process holding the camera.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <realsense2_camera_msgs/msg/extrinsics.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <cmath>
#include <cstring>
#include <thread>

#pragma pack(push, 1)
struct Sample { uint8_t type; uint64_t device_us; double host_t; float x, y, z; };
struct Meta { uint8_t type; uint8_t which; float rot[9]; float trans[3]; };
#pragma pack(pop)

class UdpImu : public rclcpp::Node {
public:
  UdpImu() : Node("rs_imu_publisher") {
    port_ = declare_parameter<int>("port", 27500);
    pub_g_ = create_publisher<sensor_msgs::msg::Imu>("/camera/gyro/sample", rclcpp::SensorDataQoS());
    pub_a_ = create_publisher<sensor_msgs::msg::Imu>("/camera/accel/sample", rclcpp::SensorDataQoS());
    rclcpp::QoS latched(1); latched.transient_local();
    ext_g_ = create_publisher<realsense2_camera_msgs::msg::Extrinsics>("/camera/extrinsics/depth_to_gyro", latched);
    ext_a_ = create_publisher<realsense2_camera_msgs::msg::Extrinsics>("/camera/extrinsics/depth_to_accel", latched);
    static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(port_); addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    int one = 1; setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (bind(sock_, (sockaddr *)&addr, sizeof(addr)) != 0) throw std::runtime_error("bind failed");
    timeval tv{1, 0}; setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    rx_ = std::thread([this] { rx_loop(); });
    report_timer_ = create_wall_timer(std::chrono::seconds(10), [this] { report(); });
    RCLCPP_INFO(get_logger(), "listening on udp 127.0.0.1:%d for rs_imu_capture", port_);
  }
  ~UdpImu() override { stop_ = true; if (rx_.joinable()) rx_.join(); close(sock_); }
private:
  void rx_loop() {
    char buf[128];
    while (!stop_ && rclcpp::ok()) {
      ssize_t n = recv(sock_, buf, sizeof(buf), 0);
      if (n == (ssize_t)sizeof(Sample)) { Sample s; memcpy(&s, buf, sizeof(s)); on_sample(s); }
      else if (n == (ssize_t)sizeof(Meta)) { Meta m; memcpy(&m, buf, sizeof(m)); on_meta(m); }
    }
  }
  void on_sample(const Sample &s) {
    const double ts = s.device_us / 1e6, cand = s.host_t - ts;
    if (!have_off_) { off_ = cand; have_off_ = true; } else off_ = std::min(cand, off_ + 2e-4 * std::max(0.0, s.host_t - last_host_));
    last_host_ = s.host_t; const double stamp = ts + off_;
    sensor_msgs::msg::Imu msg; msg.header.stamp.sec = (int32_t)stamp; msg.header.stamp.nanosec = (uint32_t)((stamp - std::floor(stamp)) * 1e9);
    msg.orientation_covariance[0] = -1.0;
    if (s.type == 0) { msg.header.frame_id = "camera_gyro_optical_frame"; msg.angular_velocity.x = s.x; msg.angular_velocity.y = s.y; msg.angular_velocity.z = s.z; pub_g_->publish(msg); ++n_g_; }
    else { msg.header.frame_id = "camera_accel_optical_frame"; msg.linear_acceleration.x = s.x; msg.linear_acceleration.y = s.y; msg.linear_acceleration.z = s.z; pub_a_->publish(msg); ++n_a_; }
  }
  static void quat_from_rot(const float *m, double q[4]) {
    double r[9] = {m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]}; const double t = r[0] + r[4] + r[8];
    if (t > 0) { double S = std::sqrt(t + 1) * 2; q[0] = (r[7] - r[5]) / S; q[1] = (r[2] - r[6]) / S; q[2] = (r[3] - r[1]) / S; q[3] = 0.25 * S; }
    else if (r[0] > r[4] && r[0] > r[8]) { double S = std::sqrt(1 + r[0] - r[4] - r[8]) * 2; q[0] = 0.25 * S; q[1] = (r[1] + r[3]) / S; q[2] = (r[2] + r[6]) / S; q[3] = (r[7] - r[5]) / S; }
    else if (r[4] > r[8]) { double S = std::sqrt(1 + r[4] - r[0] - r[8]) * 2; q[0] = (r[1] + r[3]) / S; q[1] = 0.25 * S; q[2] = (r[5] + r[7]) / S; q[3] = (r[2] - r[6]) / S; }
    else { double S = std::sqrt(1 + r[8] - r[0] - r[4]) * 2; q[0] = (r[2] + r[6]) / S; q[1] = (r[5] + r[7]) / S; q[2] = 0.25 * S; q[3] = (r[3] - r[1]) / S; }
  }
  static void qmul(const double a[4], const double b[4], double o[4]) {
    o[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1]; o[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    o[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3]; o[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
  }
  void on_meta(const Meta &m) {
    if (meta_done_[m.which]) return; meta_done_[m.which] = true;
    const std::string name = m.which == 0 ? "gyro" : "accel"; const auto stamp = now();
    geometry_msgs::msg::TransformStamped t1; t1.header.stamp = stamp; t1.header.frame_id = "camera_link"; t1.child_frame_id = "camera_" + name + "_frame";
    t1.transform.translation.x = m.trans[2]; t1.transform.translation.y = -m.trans[0]; t1.transform.translation.z = -m.trans[1];
    double q[4], tmp[4], qr[4]; quat_from_rot(m.rot, q); const double qo[4] = {-0.5, 0.5, -0.5, 0.5}, qoi[4] = {0.5, -0.5, 0.5, 0.5}; qmul(qo, q, tmp); qmul(tmp, qoi, qr);
    t1.transform.rotation.x = qr[0]; t1.transform.rotation.y = qr[1]; t1.transform.rotation.z = qr[2]; t1.transform.rotation.w = qr[3];
    geometry_msgs::msg::TransformStamped t2; t2.header.stamp = stamp; t2.header.frame_id = t1.child_frame_id; t2.child_frame_id = "camera_" + name + "_optical_frame";
    t2.transform.rotation.x = -0.5; t2.transform.rotation.y = 0.5; t2.transform.rotation.z = -0.5; t2.transform.rotation.w = 0.5;
    tfs_.push_back(t1); tfs_.push_back(t2); static_br_->sendTransform(tfs_);
    realsense2_camera_msgs::msg::Extrinsics e; for (int i = 0; i < 9; ++i) e.rotation[i] = m.rot[i]; for (int i = 0; i < 3; ++i) e.translation[i] = m.trans[i];
    (m.which == 0 ? ext_g_ : ext_a_)->publish(e);
    RCLCPP_INFO(get_logger(), "static TF + extrinsics published for %s", name.c_str());
  }
  void report() { RCLCPP_INFO(get_logger(), "last 10 s: gyro %.1f Hz accel %.1f Hz off_est=%.4f", n_g_.exchange(0) / 10.0, n_a_.exchange(0) / 10.0, off_); }
  int port_, sock_; std::thread rx_; std::atomic<bool> stop_{false};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_g_, pub_a_;
  rclcpp::Publisher<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr ext_g_, ext_a_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_; std::vector<geometry_msgs::msg::TransformStamped> tfs_; bool meta_done_[2] = {false, false};
  rclcpp::TimerBase::SharedPtr report_timer_;
  double off_ = 0.0, last_host_ = 0.0; bool have_off_ = false; std::atomic<long> n_g_{0}, n_a_{0};
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<UdpImu>()); rclcpp::shutdown(); return 0; }
