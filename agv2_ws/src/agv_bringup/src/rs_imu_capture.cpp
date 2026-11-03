// rs_imu_capture: D455 motion-module capture with NO ROS in the process (same librealsense path as rs-data-collect:
// rs2::pipeline + frame callback). Each sample is forwarded as one small UDP datagram to 127.0.0.1:<port>, where
// rs_imu_udp_publisher_node turns it into sensor_msgs/Imu. Rationale (agv110, 3 Nov 2026): every in-process ROS
// publisher stalled the camera's IMU at DDS subscriber-join events; the collector, which has no DDS, never did.
// Watchdog: after `silence` seconds without frames the pipeline is stopped and restarted (≥30 s apart).
#include <librealsense2/rs.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#pragma pack(push, 1)
struct Sample { uint8_t type; uint64_t device_us; double host_t; float x, y, z; };          // type 0 gyro, 1 accel
struct Meta { uint8_t type; uint8_t which; float rot[9]; float trans[3]; };                   // type 2, which 0 gyro / 1 accel
#pragma pack(pop)

static std::atomic<bool> g_stop{false}; static std::atomic<bool> g_term{false};
static void on_sig(int sig) { g_stop = true; if (sig == SIGTERM) g_term = true; }
static double now_s() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }

int main(int argc, char **argv) {
  int port = 27500, gyro_fps = 200, accel_fps = 100; double silence = 5.0; std::string serial;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) port = atoi(argv[++i]);
    else if (a == "--gyro-fps" && i + 1 < argc) gyro_fps = atoi(argv[++i]);
    else if (a == "--accel-fps" && i + 1 < argc) accel_fps = atoi(argv[++i]);
    else if (a == "--silence-restart-seconds" && i + 1 < argc) silence = atof(argv[++i]);
    else if (a == "--serial" && i + 1 < argc) serial = argv[++i];
  }
  signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in dst{}; dst.sin_family = AF_INET; dst.sin_port = htons(port); dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  std::atomic<long> n_g{0}, n_a{0}; std::atomic<double> last_frame{0.0};
  auto cb = [&](const rs2::frame &f) {
    auto mf = f.as<rs2::motion_frame>(); if (!mf) return;
    Sample s{}; s.type = mf.get_profile().stream_type() == RS2_STREAM_GYRO ? 0 : 1;
    s.device_us = mf.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP)
                    ? (uint64_t)mf.get_frame_metadata(RS2_FRAME_METADATA_FRAME_TIMESTAMP) : (uint64_t)(mf.get_timestamp() * 1000.0);
    s.host_t = now_s(); auto d = mf.get_motion_data(); s.x = d.x; s.y = d.y; s.z = d.z;
    sendto(sock, &s, sizeof(s), 0, (sockaddr *)&dst, sizeof(dst));
    last_frame.store(s.host_t); (s.type == 0 ? n_g : n_a)++;
  };
  rs2::pipeline pipe; rs2::config cfg;
  if (!serial.empty()) cfg.enable_device(serial);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, gyro_fps);
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, accel_fps);
  auto start = [&](bool first) {
    rs2::pipeline_profile prof;
    for (int attempt = 0; attempt < 30; ++attempt) {
      try { prof = pipe.start(cfg, cb); break; }
      catch (const rs2::error &e) { if (attempt == 29) throw; fprintf(stderr, "start failed (%s); retrying\n", e.what()); std::this_thread::sleep_for(std::chrono::seconds(2)); }
    }
    auto dev = prof.get_device();
    if (first) {
      printf("device %s fw %s gyro %d Hz accel %d Hz -> udp 127.0.0.1:%d\n", dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION), gyro_fps, accel_fps, port);
      rs2::stream_profile depth;
      for (auto &s : dev.query_sensors()) for (auto &p : s.get_stream_profiles()) if (p.stream_type() == RS2_STREAM_DEPTH && !depth) depth = p;
      for (auto &p : prof.get_streams()) {
        Meta m{}; m.type = 2; m.which = p.stream_type() == RS2_STREAM_GYRO ? 0 : 1;
        rs2_extrinsics ex{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}}; if (depth) ex = depth.get_extrinsics_to(p);
        memcpy(m.rot, ex.rotation, sizeof(m.rot)); memcpy(m.trans, ex.translation, sizeof(m.trans));
        for (int k = 0; k < 3; ++k) { sendto(sock, &m, sizeof(m), 0, (sockaddr *)&dst, sizeof(dst)); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }   // repeated: UDP, publisher may start late
      }
      fflush(stdout);
    }
    last_frame.store(0.0);
  };
  start(true);
  int restarts = 0; double last_restart = 0.0, last_report = now_s(), started = now_s();
  while (!g_stop) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    const double now = now_s();
    if (now - last_report >= 10.0) { printf("last 10 s: gyro %.1f Hz accel %.1f Hz restarts=%d\n", n_g.exchange(0) / 10.0, n_a.exchange(0) / 10.0, restarts); fflush(stdout); last_report = now; }
    const double lf = last_frame.load();
    if (lf > 0.0 && now - lf >= silence && now - last_restart >= 30.0) {
      ++restarts; last_restart = now;
      // An in-process pipe.stop()/start() returned without frames (agv110, 3 Nov 19:03) and once blocked in D state;
      // a brand-new process always recovered. So: exit and let the launch respawn us (fresh process, fresh context).
      printf("no motion frames for %.1f s: exiting for respawn (fresh process recovers the camera; in-process restart does not)\n", now - lf); fflush(stdout);
      try { pipe.stop(); } catch (...) {}
      close(sock); _exit(3);
    }
    (void)started;
  }
  try { pipe.stop(); } catch (...) {}   // respawn-on-stall is handled by rs_imu_capture_supervised.sh, not by launch
  printf("rs_imu_capture: stopped cleanly\n"); fflush(stdout); close(sock); return 0;
}
