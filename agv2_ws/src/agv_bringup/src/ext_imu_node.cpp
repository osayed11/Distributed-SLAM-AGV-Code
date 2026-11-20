// ext_imu_node: external IMU (ISM330DHCX on the myAGV's software SPI bus, kernel driver st_lsm6dsx) -> /imu.
// Reads the gyro and accel IIO buffers directly (interrupt-driven FIFO, kernel per-sample timestamps on
// CLOCK_REALTIME) and publishes one sensor_msgs/Imu per gyro sample, paired with the nearest accel sample.
// No librealsense and no UDP hop: unlike the D455 path there is nothing here that DDS could disturb.
// Devices are found by IIO name (the iio:deviceN index changes between boots).
// Every 10 s it logs rates, the pairing gap, and the stamp latency (host time at read minus sample stamp),
// which is the check that the kernel stamps sit on the host clock.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
std::string rd(const std::string &p) { std::ifstream f(p); std::string s; std::getline(f, s); return s; }
bool wr(const std::string &p, const std::string &v) { std::ofstream f(p); if (!f) return false; f << v; f.close(); return !f.fail(); }

struct Chan { int offset = 0, bytes = 0; bool is_signed = true; int shift = 0, bits = 0; };

// Parse an IIO scan_elements type such as "le:s16/16>>0" or "le:s64/64>>0".
bool parse_type(const std::string &t, Chan &c) {
  char sign = 's'; int bits = 0, storage = 0, shift = 0;
  if (sscanf(t.c_str(), "le:%c%d/%d>>%d", &sign, &bits, &storage, &shift) != 4) return false;
  c.is_signed = sign == 's'; c.bits = bits; c.bytes = storage / 8; c.shift = shift; return c.bytes > 0;
}
int64_t get(const uint8_t *p, const Chan &c) {
  uint64_t v = 0; memcpy(&v, p + c.offset, c.bytes); v >>= c.shift;
  if (c.bits < 64) { const uint64_t m = (1ULL << c.bits) - 1; v &= m; if (c.is_signed && (v >> (c.bits - 1))) v |= ~m; }
  return (int64_t)v;
}

struct Dev {
  std::string name, prefix, sys, node;  // prefix: "anglvel" | "accel"
  double scale = 1.0; int fd = -1; Chan ax[3], ts; int rec = 0;
};

bool find_dev(Dev &d) {
  DIR *dir = opendir("/sys/bus/iio/devices"); if (!dir) return false; bool ok = false;
  while (dirent *e = readdir(dir)) {
    std::string n = e->d_name; if (n.rfind("iio:device", 0) != 0) continue;
    if (rd("/sys/bus/iio/devices/" + n + "/name") == d.name) { d.sys = "/sys/bus/iio/devices/" + n; d.node = "/dev/" + n; ok = true; break; }
  }
  closedir(dir); return ok;
}

// Enable x/y/z + timestamp, set rate (Hz for st_lsm6dsx), work out the record layout, start the buffer.
std::string configure(Dev &d, double rate_hz) {
  wr(d.sys + "/buffer/enable", "0");
  wr(d.sys + "/current_timestamp_clock", "realtime");
  const char *axes[3] = {"x", "y", "z"};
  for (auto a : axes) if (!wr(d.sys + "/scan_elements/in_" + d.prefix + "_" + a + "_en", "1")) return "cannot enable " + d.prefix + "_" + a;
  if (!wr(d.sys + "/scan_elements/in_timestamp_en", "1")) return "cannot enable timestamp";
  char hz[32]; snprintf(hz, sizeof(hz), "%.3f", rate_hz);
  if (!wr(d.sys + "/sampling_frequency", hz)) return std::string("cannot set sampling_frequency ") + hz;
  // Layout: channels in scan_index order, each aligned to its own storage size; record padded to the largest.
  struct E { int idx; Chan *c; std::string t; };
  std::vector<E> es;
  for (int i = 0; i < 3; ++i) es.push_back({std::stoi(rd(d.sys + "/scan_elements/in_" + d.prefix + "_" + axes[i] + "_index")), &d.ax[i], rd(d.sys + "/scan_elements/in_" + d.prefix + "_" + axes[i] + "_type")});
  es.push_back({std::stoi(rd(d.sys + "/scan_elements/in_timestamp_index")), &d.ts, rd(d.sys + "/scan_elements/in_timestamp_type")});
  std::sort(es.begin(), es.end(), [](const E &a, const E &b) { return a.idx < b.idx; });
  int off = 0, maxb = 1;
  for (auto &e : es) {
    if (!parse_type(e.t, *e.c)) return "bad scan type " + e.t;
    off = (off + e.c->bytes - 1) / e.c->bytes * e.c->bytes; e.c->offset = off; off += e.c->bytes; maxb = std::max(maxb, e.c->bytes);
  }
  d.rec = (off + maxb - 1) / maxb * maxb;
  d.scale = std::atof(rd(d.sys + "/in_" + d.prefix + "_scale").c_str());
  wr(d.sys + "/buffer/length", "512");
  if (!wr(d.sys + "/buffer/enable", "1")) return "cannot enable buffer";
  d.fd = open(d.node.c_str(), O_RDONLY | O_NONBLOCK);
  return d.fd >= 0 ? "" : "cannot open " + d.node;
}
void release(Dev &d) { if (d.fd >= 0) { close(d.fd); d.fd = -1; } if (!d.sys.empty()) wr(d.sys + "/buffer/enable", "0"); }

struct S { int64_t t; double v[3]; };
}  // namespace

class ExtImu : public rclcpp::Node {
public:
  ExtImu() : Node("ext_imu") {
    g_.name = declare_parameter<std::string>("gyro_name", "ism330dhcx_gyro"); g_.prefix = "anglvel";
    a_.name = declare_parameter<std::string>("accel_name", "ism330dhcx_accel"); a_.prefix = "accel";
    rate_ = declare_parameter<double>("rate_hz", 208.0);
    frame_ = declare_parameter<std::string>("frame_id", "imu_link");
    silence_ = declare_parameter<double>("silence_restart_seconds", 1.0);
    gyro_var_ = declare_parameter<double>("gyro_variance", 0.0);   // 0 = unknown until Allan variance is measured
    accel_var_ = declare_parameter<double>("accel_variance", 0.0);
    pub_ = create_publisher<sensor_msgs::msg::Imu>(declare_parameter<std::string>("topic", "/imu"), rclcpp::SensorDataQoS());
    start();
    rx_ = std::thread([this] { loop(); });
    timer_ = create_wall_timer(std::chrono::seconds(10), [this] { report(); });
  }
  ~ExtImu() override { stop_ = true; if (rx_.joinable()) rx_.join(); release(g_); release(a_); }

private:
  void start() {
    for (int attempt = 1; attempt <= 30 && rclcpp::ok(); ++attempt) {
      release(g_); release(a_);
      std::string err = !find_dev(g_) ? "IIO device " + g_.name + " not found" : !find_dev(a_) ? "IIO device " + a_.name + " not found" : "";
      if (err.empty()) err = configure(g_, rate_);
      if (err.empty()) err = configure(a_, rate_);
      if (err.empty()) {
        RCLCPP_INFO(get_logger(), "streaming %s (%s) + %s (%s) at %.1f Hz -> %s [%s]; record %d/%d B", g_.name.c_str(), g_.node.c_str(),
                    a_.name.c_str(), a_.node.c_str(), rate_, pub_->get_topic_name(), frame_.c_str(), g_.rec, a_.rec);
        return;
      }
      RCLCPP_WARN(get_logger(), "IMU setup failed (attempt %d): %s; retrying in 2 s", attempt, err.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    throw std::runtime_error("external IMU not available");
  }

  void drain(Dev &d, std::deque<S> &q, int64_t &last_t) {
    uint8_t buf[8192];
    for (;;) {
      ssize_t n = read(d.fd, buf, sizeof(buf) - sizeof(buf) % d.rec);
      if (n <= 0) return;
      for (ssize_t off = 0; off + d.rec <= n; off += d.rec) {
        S s; s.t = get(buf + off, d.ts);
        for (int i = 0; i < 3; ++i) s.v[i] = get(buf + off, d.ax[i]) * d.scale;
        if (last_t && s.t <= last_t) { ++nonmono_; continue; }
        last_t = s.t; q.push_back(s);
      }
    }
  }

  void loop() {
    const int64_t half = (int64_t)(0.5e9 / rate_);
    auto last_rx = std::chrono::steady_clock::now();
    while (!stop_ && rclcpp::ok()) {
      pollfd pf[2] = {{g_.fd, POLLIN, 0}, {a_.fd, POLLIN, 0}};
      int r = poll(pf, 2, 100);
      if (r > 0) {
        drain(a_, aq_, last_a_); drain(g_, gq_, last_g_);
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (!gq_.empty()) {
          last_rx = std::chrono::steady_clock::now();
          clock_update(gq_.back().t, now_ns);
          std::lock_guard<std::mutex> lk(mu_); lat_.push_back((now_ns - map(gq_.back().t)) / 1e6);
        }
        // Pair each gyro sample with the accel sample nearest in time (same ODR, same chip FIFO). If the accel
        // stream has not caught up yet, wait up to 20 ms for it, then publish the gyro sample unpaired.
        while (!gq_.empty()) {
          const S &g = gq_.front();
          while (aq_.size() >= 2 && std::llabs(aq_[1].t - g.t) <= std::llabs(aq_[0].t - g.t)) aq_.pop_front();
          const bool accel_behind = aq_.empty() || aq_.back().t < g.t - half;
          if (accel_behind && now_ns - g.t < 20000000) break;
          const S *a = (!aq_.empty() && std::llabs(aq_.front().t - g.t) <= 2 * half) ? &aq_.front() : nullptr;
          publish(g, a, a ? std::llabs(a->t - g.t) : 0);
          gq_.pop_front();
        }
        while (aq_.size() > 64) aq_.pop_front();
      }
      if (std::chrono::steady_clock::now() - last_rx > std::chrono::duration<double>(silence_)) {
        RCLCPP_WARN(get_logger(), "no IMU samples for %.1f s: restarting the IIO buffers", silence_);
        ++restarts_; gq_.clear(); aq_.clear(); last_g_ = last_a_ = 0;
        try { start(); } catch (const std::exception &e) { RCLCPP_ERROR(get_logger(), "%s", e.what()); }
        last_rx = std::chrono::steady_clock::now();
      }
    }
  }

  // Clock mapping. The driver derives sample stamps from the chip's own timestamp counter at its nominal rate; on
  // agv111 (20 Nov 2026) that counter ran ~0.22 % slow against the host clock (stamps fell behind by ~2.2 ms per
  // second). Keep the chip stamps for their even spacing, but map them onto the host clock:
  //   stamp = t + skew * (t - t0) + off
  // skew: least-squares slope of the per-2 s minimum of (host_read - t) over the last 60 s (lower envelope, so
  // FIFO/read delay does not bias it); off: running minimum of the residual, allowed to rise 100 ppm (leak).
  int64_t map(int64_t t) const { return t + (int64_t)(skew_ * (double)(t - t0_)) + off_ns_; }
  void clock_update(int64_t t, int64_t host_ns) {
    if (!tbase_) { tbase_ = t; t0_ = t; }
    const double cand = (double)(host_ns - t);
    if (!win_n_ || cand < win_min_) { win_min_ = cand; win_t_ = t; }
    ++win_n_;
    if (host_ns - win_start_ > 2000000000LL) {
      if (win_start_) { env_.push_back({(double)(win_t_ - tbase_), win_min_}); while (env_.size() > 30) env_.pop_front(); }
      win_start_ = host_ns; win_n_ = 0;
      if (env_.size() >= 5) {
        double mx = 0, my = 0; for (auto &p : env_) { mx += p.first; my += p.second; } mx /= env_.size(); my /= env_.size();
        double sxx = 0, sxy = 0; for (auto &p : env_) { sxx += (p.first - mx) * (p.first - mx); sxy += (p.first - mx) * (p.second - my); }
        if (sxx > 0) {
          // Re-anchor at t so the mapping stays continuous when the skew estimate changes.
          off_ns_ += (int64_t)(skew_ * (double)(t - t0_)); t0_ = t; skew_ = sxy / sxx;
        }
      }
    }
    const int64_t resid = host_ns - (t + (int64_t)(skew_ * (double)(t - t0_)));
    const double leak = env_.size() >= 5 ? 1e-4 : 5e-3;  // until the skew is known, let the offset follow fast
    if (!have_off_) { off_ns_ = resid; have_off_ = true; }
    else off_ns_ = std::min<int64_t>(resid, off_ns_ + (int64_t)(leak * (double)std::max<int64_t>(0, host_ns - last_host_ns_)));
    last_host_ns_ = host_ns;
  }

  void publish(const S &g, const S *a, int64_t gap_ns) {
    sensor_msgs::msg::Imu m;
    m.header.frame_id = frame_;
    int64_t st = map(g.t);
    if (last_st_ && st <= last_st_) { st = last_st_ + 1000; ++clamped_; }  // never publish a stamp that goes backwards
    last_st_ = st;
    m.header.stamp.sec = (int32_t)(st / 1000000000LL); m.header.stamp.nanosec = (uint32_t)(st % 1000000000LL);
    m.orientation_covariance[0] = -1.0;  // no orientation estimate
    m.angular_velocity.x = g.v[0]; m.angular_velocity.y = g.v[1]; m.angular_velocity.z = g.v[2];
    for (int i = 0; i < 3; ++i) { m.angular_velocity_covariance[4 * i] = gyro_var_; m.linear_acceleration_covariance[4 * i] = accel_var_; }
    if (a) { m.linear_acceleration.x = a->v[0]; m.linear_acceleration.y = a->v[1]; m.linear_acceleration.z = a->v[2]; }
    else { ++unpaired_; m.linear_acceleration_covariance[0] = -1.0; }
    if (gap_ns > max_gap_ns_) max_gap_ns_ = gap_ns;
    if (last_pub_t_ && g.t - last_pub_t_ > max_step_ns_) max_step_ns_ = g.t - last_pub_t_;
    last_pub_t_ = g.t;
    pub_->publish(m); ++n_;
  }

  void report() {
    std::vector<double> l; { std::lock_guard<std::mutex> lk(mu_); l.swap(lat_); }
    std::sort(l.begin(), l.end());
    auto pct = [&](double p) { return l.empty() ? 0.0 : l[(size_t)(p * (l.size() - 1))]; };
    RCLCPP_INFO(get_logger(), "last 10 s: %.1f Hz published; max stamp step %.2f ms; gyro-accel pairing gap max %.2f ms; "
                "corrected stamp latency (host now - stamp at read) min %.2f / median %.2f / max %.2f ms; chip clock skew %+.0f ppm; "
                "unpaired %ld; non-monotonic %ld; clamped %ld; restarts %ld",
                n_.exchange(0) / 10.0, max_step_ns_.exchange(0) / 1e6, max_gap_ns_.exchange(0) / 1e6, pct(0), pct(0.5), pct(1.0),
                skew_.load() * 1e6, unpaired_.exchange(0), nonmono_.exchange(0), clamped_.exchange(0), restarts_.load());
  }

  Dev g_, a_;
  double rate_, silence_, gyro_var_, accel_var_; std::string frame_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::thread rx_; std::atomic<bool> stop_{false}; rclcpp::TimerBase::SharedPtr timer_;
  std::deque<S> gq_, aq_; int64_t last_g_ = 0, last_a_ = 0, last_pub_t_ = 0;
  std::atomic<int64_t> max_gap_ns_{0}, max_step_ns_{0};
  std::mutex mu_; std::vector<double> lat_;
  // clock mapping state (rx thread only, except skew_ which report() reads)
  int64_t tbase_ = 0, t0_ = 0, off_ns_ = 0, last_host_ns_ = 0, win_start_ = 0, win_t_ = 0, last_st_ = 0; bool have_off_ = false;
  std::atomic<long> clamped_{0};
  double win_min_ = 0; long win_n_ = 0; std::deque<std::pair<double, double>> env_; std::atomic<double> skew_{0.0};
  std::atomic<long> n_{0}, unpaired_{0}, nonmono_{0}, restarts_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtImu>());
  rclcpp::shutdown();
  return 0;
}
