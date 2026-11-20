// rs_imu_iio_capture: D455 gyro/accel capture straight from the kernel's IIO devices (gyro_3d, accel_3d), with no
// librealsense streaming at all. librealsense is used once at start-up, without opening any stream, to read the
// depth->gyro/accel extrinsics; it is then released. Output is identical to rs_imu_capture: one UDP datagram per sample
// to 127.0.0.1:<port> for rs_imu_udp_publisher_node, plus extrinsics meta datagrams.
// Why: every librealsense process that streams anything runs a clock-sync keeper that sends the camera a firmware
// command every ~1.3 s (uvc-sensor.cpp / hid-sensor.cpp enable_time_diff_keeper), and the camera's control channel is
// ~96 % busy on our robots. This capture removes its share of that traffic and librealsense's HID code path entirely.
// Watchdog: no sample for `silence` seconds -> disable buffers and exit(3); rs_imu_capture_supervised.sh restarts it.
#include <librealsense2/rs.hpp>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <dirent.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>

#pragma pack(push, 1)
struct Sample { uint8_t type; uint64_t device_us; double host_t; float x, y, z; };          // type 0 gyro, 1 accel
struct Meta { uint8_t type; uint8_t which; float rot[9]; float trans[3]; };                   // type 2, which 0 gyro / 1 accel
#pragma pack(pop)

static std::atomic<bool> g_stop{false};
static void on_sig(int) { g_stop = true; }
static double now_s() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }

static bool wr(const std::string &path, const std::string &val) {
  std::ofstream f(path);
  if (!f) return false;
  f << val;
  f.close();
  return !f.fail();
}
static std::string rd(const std::string &path) {
  std::ifstream f(path);
  std::string s;
  std::getline(f, s);
  return s;
}

struct IioDev {
  std::string sys, dev, prefix;   // e.g. /sys/bus/iio/devices/iio:device0, /dev/iio:device0, "anglvel"
  double scale = 1.0;
  int fd = -1;
  uint8_t type = 0;
};

static bool find_dev(const std::string &name, IioDev &out) {
  DIR *d = opendir("/sys/bus/iio/devices");
  if (!d) return false;
  bool ok = false;
  while (dirent *e = readdir(d)) {
    std::string n = e->d_name;
    if (n.rfind("iio:device", 0) != 0) continue;
    std::string sys = "/sys/bus/iio/devices/" + n;
    if (rd(sys + "/name") == name) { out.sys = sys; out.dev = "/dev/" + n; ok = true; break; }
  }
  closedir(d);
  return ok;
}

static bool configure(IioDev &d, int fps) {
  wr(d.sys + "/buffer/enable", "0");
  for (const char *ax : {"x", "y", "z"})
    if (!wr(d.sys + "/scan_elements/in_" + d.prefix + "_" + ax + "_en", "1")) return false;
  if (!wr(d.sys + "/scan_elements/in_timestamp_en", "1")) return false;
  // The D455 firmware reads this HID field as an output-data-rate code, not Hz: librealsense (ds-motion-common.h,
  // Linux table) writes 1 = 63/100 Hz, 2 = 200 Hz, 3 = 250 Hz, 4 = 400 Hz. Writing "200" asks for the maximum.
  int code = fps >= 400 ? 4 : fps >= 250 ? 3 : fps >= 200 ? 2 : 1;
  wr(d.sys + "/in_" + d.prefix + "_sampling_frequency", std::to_string(code));
  wr(d.sys + "/buffer/length", "128");
  d.scale = atof(rd(d.sys + "/in_" + d.prefix + "_scale").c_str());
  if (!wr(d.sys + "/buffer/enable", "1")) return false;
  d.fd = open(d.dev.c_str(), O_RDONLY | O_NONBLOCK);
  return d.fd >= 0;
}

static void release(IioDev &d) {
  if (d.fd >= 0) { close(d.fd); d.fd = -1; }
  wr(d.sys + "/buffer/enable", "0");
}

int main(int argc, char **argv) {
  int port = 27500, gyro_fps = 200, accel_fps = 100; double silence = 5.0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) port = atoi(argv[++i]);
    else if (a == "--gyro-fps" && i + 1 < argc) gyro_fps = atoi(argv[++i]);
    else if (a == "--accel-fps" && i + 1 < argc) accel_fps = atoi(argv[++i]);
    else if (a == "--silence-restart-seconds" && i + 1 < argc) silence = atof(argv[++i]);
  }
  signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in dst{}; dst.sin_family = AF_INET; dst.sin_port = htons(port); dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  // One-shot extrinsics through librealsense, no streaming (so no clock keeper, no HID capture thread).
  Meta metas[2]{};
  bool have_meta = false;
  try {
    rs2::context ctx;
    auto devs = ctx.query_devices();
    if (devs.size() > 0) {
      auto dev = devs[0];
      rs2::stream_profile depth, gyro, accel;
      for (auto &s : dev.query_sensors())
        for (auto &p : s.get_stream_profiles()) {
          if (p.stream_type() == RS2_STREAM_DEPTH && !depth) depth = p;
          if (p.stream_type() == RS2_STREAM_GYRO && !gyro) gyro = p;
          if (p.stream_type() == RS2_STREAM_ACCEL && !accel) accel = p;
        }
      if (depth && gyro && accel) {
        for (int w = 0; w < 2; ++w) {
          auto ex = depth.get_extrinsics_to(w == 0 ? gyro : accel);
          metas[w].type = 2; metas[w].which = (uint8_t)w;
          memcpy(metas[w].rot, ex.rotation, sizeof(metas[w].rot)); memcpy(metas[w].trans, ex.translation, sizeof(metas[w].trans));
        }
        have_meta = true;
      }
      printf("device %s fw %s (extrinsics read, librealsense released)\n", dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));
    }
  } catch (const std::exception &e) { fprintf(stderr, "extrinsics: %s (continuing without)\n", e.what()); }

  IioDev g, a;
  g.prefix = "anglvel"; g.type = 0; a.prefix = "accel"; a.type = 1;
  if (!find_dev("gyro_3d", g) || !find_dev("accel_3d", a)) { fprintf(stderr, "IIO gyro_3d/accel_3d not found\n"); return 2; }
  bool ok = false;
  for (int attempt = 0; attempt < 30 && !g_stop; ++attempt) {
    if (configure(g, gyro_fps) && configure(a, accel_fps)) { ok = true; break; }
    release(g); release(a);
    fprintf(stderr, "IIO configure failed (attempt %d); retrying\n", attempt + 1);
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  if (!ok) return 2;
  printf("iio capture: %s gyro %d Hz scale %.9g, %s accel %d Hz scale %.9g -> udp 127.0.0.1:%d\n",
         g.dev.c_str(), gyro_fps, g.scale, a.dev.c_str(), accel_fps, a.scale, port);
  fflush(stdout);
  if (have_meta)
    for (int k = 0; k < 3; ++k) {
      for (auto &m : metas) sendto(sock, &m, sizeof(m), 0, (sockaddr *)&dst, sizeof(dst));
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

  // record: x,y,z s32 at 0,4,8; timestamp s64 at 16 (aligned) -> 24 bytes
  const size_t REC = 24;
  uint8_t buf[REC * 128];
  long n_g = 0, n_a = 0;
  double last_frame = now_s(), last_report = now_s();
  int rc = 0;
  pollfd pf[2] = {{g.fd, POLLIN, 0}, {a.fd, POLLIN, 0}};
  IioDev *devs[2] = {&g, &a};
  while (!g_stop) {
    int r = poll(pf, 2, 250);
    double now = now_s();
    if (r > 0)
      for (int k = 0; k < 2; ++k) {
        if (!(pf[k].revents & POLLIN)) continue;
        ssize_t got = read(pf[k].fd, buf, sizeof(buf));
        if (got <= 0) continue;
        for (size_t off = 0; off + REC <= (size_t)got; off += REC) {
          int32_t x, y, z; int64_t ts;
          memcpy(&x, buf + off, 4); memcpy(&y, buf + off + 4, 4); memcpy(&z, buf + off + 8, 4); memcpy(&ts, buf + off + 16, 8);
          Sample s{}; s.type = devs[k]->type; s.device_us = (uint64_t)(ts / 1000); s.host_t = now;
          s.x = (float)(x * devs[k]->scale); s.y = (float)(y * devs[k]->scale); s.z = (float)(z * devs[k]->scale);
          sendto(sock, &s, sizeof(s), 0, (sockaddr *)&dst, sizeof(dst));
          (k == 0 ? n_g : n_a)++;
        }
        last_frame = now;
      }
    if (now - last_report >= 10.0) {
      printf("last 10 s: gyro %.1f Hz accel %.1f Hz restarts=0\n", n_g / (now - last_report), n_a / (now - last_report));
      fflush(stdout); n_g = n_a = 0; last_report = now;
    }
    if (now - last_frame >= silence) {
      printf("no motion frames for %.1f s: exiting for respawn (iio capture)\n", now - last_frame); fflush(stdout);
      rc = 3; break;
    }
  }
  release(g); release(a);
  if (!rc) printf("rs_imu_iio_capture: stopped cleanly\n");
  fflush(stdout); close(sock);
  return rc;
}
