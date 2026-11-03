# D455 IMU: standalone capture path — fleet rollout guide

Written 2026-11-03 for whoever rolls the IMU fix to the rest of the fleet. Everything
here was measured on agv110 (Pi 4 2 GB, Ubuntu 22.04, ROS 2 Humble, realsense2_camera
4.58.4, D455 fw 5.17.0.10) during 2–3 Nov; the evidence log is
`docs/IMU_INVESTIGATION_HANDOFF_20261028.md`. Nothing below is from a datasheet.

## 1. What is wrong and what fixes it

| path that owns the camera's IMU | result under a full recording |
|---|---|
| realsense2_camera wrapper (IMU enabled) | hard stall within minutes, every camera, every run; unrecoverable by the wrapper's own re-arm |
| Python publisher, librealsense sensor API in a ROS process | 2 clean / 1 unrecoverable stall in 3 runs |
| C++ node, librealsense pipeline + ROS publish in one process | stalls at DDS subscriber joins (gate probes, recorder start) |
| Intel `rs-data-collect` (no ROS in the process) | 15/15 eight-minute cycles clean idle; 600 s clean under the full pipeline load |
| **capture process with no ROS + separate UDP publisher (this guide)** | 5 consecutive clean moving sessions after a camera reset; when the camera does stall, a fresh process recovers it and the run continues |

Two facts drive the design:

1. The stall is device-side (the kernel stops receiving HID reports) and it is triggered
   in processes that both hold the camera and take part in DDS. Keep DDS out of the
   process that holds the camera.
2. An in-process restart (`pipe.stop()/start()`) does not recover a stalled camera; it
   returned no frames and once blocked in uninterruptible sleep. A **new process** always
   did. Recovery therefore means "exit and respawn".

Separately: the camera degrades with the number of streaming sessions since power-up.
Sessions 1–3 after a boot were clean every time; sessions 4–7 produced stalls and
micro-gaps regardless of priority or anything else we changed. Until that is explained,
**reset or power-cycle the camera before the first session of a dataset run** and do not
chain many sessions on one boot.

## 2. The three processes

Launch with `agv_imu_source:=standalone` and the camera is served by:

**realsense2_camera, video only.** `enable_gyro`/`enable_accel` false. Two extra
parameters go in through rs_launch's `config_file` hook (rs_launch silently drops launch
arguments it does not know): `rgb_camera.global_time_enabled: false` and
`depth_module.global_time_enabled: false`, in
`agv2_ws/src/agv_bringup/config/realsense_standalone_imu.yaml` (a **flat** dict, no
`/**`/`ros__parameters` wrapper — rs_launch passes the dict straight to the node). With
global time left on, the wrapper's image stamps drifted 1.4 s over a 10-minute bag while
another process held the motion module; with it off, colour, depth and IMU stamps agree
within 8 ms.

**`rs_imu_capture`** (`agv2_ws/src/agv_bringup/src/rs_imu_capture.cpp`, links
librealsense only). Opens the motion module with `rs2::pipeline` + frame callback,
exactly the collector's call pattern, gyro 200 Hz / accel 100 Hz. Each sample is one UDP
datagram to 127.0.0.1:27500: stream type, device hardware timestamp (µs, from frame
metadata), host arrival time, x y z. At start it sends the depth→gyro and depth→accel
extrinsics three times (UDP; the publisher may be late). Prints a rate line every 10 s.
If no frame arrives for 5 s it exits with code 3.

**`rs_imu_capture_supervised.sh`** (`agv2_ws/src/agv_bringup/scripts/`) runs the capture
in a loop and restarts it only on exit code 3, after 2 s. Any other exit (SIGINT from
the launch, an error) ends the loop, so shutdown is immediate. Do not use the launch
system's `respawn=True` for this: it respawned the capture during teardown and made
every session stop take 70–80 s.

**`rs_imu_udp_publisher_node`** (`agv2_ws/src/agv_bringup/src/rs_imu_udp_publisher_node.cpp`,
rclcpp only). Listens on the UDP port and publishes `/camera/gyro/sample` and
`/camera/accel/sample` (sensor-data QoS, frame ids `camera_gyro_optical_frame` /
`camera_accel_optical_frame`, `orientation_covariance[0] = -1`), the static TF chain
`camera_link → camera_{gyro,accel}_frame → camera_{gyro,accel}_optical_frame` with the
wrapper's optical-to-body convention, and the latched
`/camera/extrinsics/depth_to_{gyro,accel}` topics. Same names and QoS as the wrapper, so
the recorder, the gates and the validator need no change.

Both are started by a `TimerAction` 25 s after the wrapper. Starting them together made
the wrapper crash on three cameras.

### Timestamps

The publisher stamps from the device clock. It keeps a running minimum of
(host arrival − device time), allowed to creep up at 200 ppm, and adds it to the device
time. HID bursts and host stalls only make arrival later, so they never move a stamp.
Measured at a subscriber: 1–2 ms behind arrival, stable over a run. Do not use
librealsense global time in the standalone process (0.5 s ahead of wall clock and
drifting ~4000 ppm) and do not stamp with arrival time (1–2 s HID bursts).

## 3. Session integration

- `scripts/logging/start_session.sh`: `IMU_SOURCE=standalone` (default `wrapper`),
  passed to the launch as `agv_imu_source`.
- `scripts/scenarios/run_s1_mocap_pilot_robot.sh`: `S1_IMU_SOURCE=standalone`, plus
  `S1_TOPIC_WAIT_SEC=90` (the publisher starts 25 s after the wrapper; the default 25 s
  readiness window times out on it) and `S1_RUNTIME_IMU_GUARD=false
  S1_RUNTIME_IMU_REARM=false` (the guard re-enables the **wrapper's** IMU, which would put
  two processes on the motion module).
- `scripts/scenarios/fleet_session.py` / `configs/s1_fleet.example.json`: the
  `runtime_imu_rearm` key must be false for standalone robots.
- The capture process lives at nice 0. **Do not renice it**: at −15 it was no better and
  in two runs worse. The thing that does need priority is the Zenoh GT client (section 6).

## 4. Rollout steps per robot

The code is on `origin/ros2-migration` (pushed 3 Nov, commits 4c3fe53..06dbd17). Robot
110/111/112 checkouts carry Riyaadh's uncommitted RealSense-key fix in three files — never
`git pull`/`reset` over them; fetch and copy the files in, or commit his fix first.

1. Stationary robot, nothing running. Back up the files you replace.
2. Copy in: `src/rs_imu_capture.cpp`, `src/rs_imu_udp_publisher_node.cpp`,
   `scripts/rs_imu_capture_supervised.sh` (+x), `config/realsense_standalone_imu.yaml`,
   `launch/bringup.launch.py`, `CMakeLists.txt`, `package.xml`, and the three scripts
   above.
3. Build, sequential (a parallel colcon build hangs a 2 GB Pi):
   `cd ~/slam_project/agv2_ws && colcon build --symlink-install --executor sequential --packages-select agv_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release`
   (~40 s first time). Check `install/agv_bringup/lib/agv_bringup/` has `rs_imu_capture`,
   `rs_imu_capture_supervised.sh`, `rs_imu_udp_publisher_node`, and
   `install/agv_bringup/share/agv_bringup/config/realsense_standalone_imu.yaml` exists
   (symlink install only links files that existed at build time; rebuild after adding).
4. Reset the camera (section 7) if it has streamed since the last power-up.
5. Sanity: `ROS_LOCALHOST_ONLY=1 ros2 launch agv_bringup bringup.launch.py agv_imu_source:=standalone ...`
   for 60 s, then `ros2 topic hz /camera/gyro/sample` ≈ 200.5, `/camera/accel/sample`
   ≈ 100.3, `ros2 topic info /camera/gyro/sample` shows **one** publisher, and
   `ros2 param get /camera enable_gyro` is False. Stop with SIGINT and confirm
   "Close Sensor - Done" twice and all processes "finished cleanly".
6. One stationary 10-minute `start_session.sh` recording with `IMU_SOURCE=standalone
   REQUIRE_IMU=true ROS_LOCALHOST_ONLY=1`. Expect: validator IMU coverage over the whole
   bag, max IMU gap ≤ 0.25 s (a recovered stall shows as one ~15–20 s hole), colour
   ≈ 13–14 Hz, teardown ≈ 10–15 s.
7. Then the moving run through the S1 pilot with the flags in section 3.

## 5. Verification numbers (so you know what "working" looks like)

Clean session (agv110, 3 Nov, five in a row after a reset): capture log lines
`last 10 s: gyro 200.4–200.6 Hz accel 100.3–100.4 Hz restarts=0`, no
`exiting for respawn`, validator `camera_gyro_sample_gaps` max 0.1–0.15 s, bag IMU
rate 200.5 / 100.3 Hz, gyro/accel/odom header stamps within 1 ms of recorder receive
time, colour/depth within 50 ms. A recovered stall: one `exiting for respawn`, one
`restarting in 2 s`, a 15–20 s IMU hole, then full rate; the session continues.

## 6. Things found on the way that the fleet needs anyway

- **ROS_LOCALHOST_ONLY**: the stock `ros_environment` hook exports `0` when the variable
  is unset, so any wrapper that sources `setup.bash` before `start_session.sh` defeats
  start_session's auto rule. Export `ROS_LOCALHOST_ONLY=1` explicitly and check
  `ros_localhost_only` in the manifest. Robots on the shared lab domain otherwise record
  each other's IMU and stream colour to each other's recorders.
- **Zenoh GT client priority**: during bringup (DDS churn plus the gate's `ros2 topic hz`
  probes) the robot's Zenoh client is starved of CPU and the MoCap pose feed drops out
  (50 of 187 s bad, worst pose age 0.80 s). Reniced to −15: 0 of 220 s bad. Wi-Fi was
  clean both times. Add `Nice=-15` to `configs/systemd/orkar-zenoh-gt.service`. Also
  set `reliable_routes_blocking: false` in `configs/zenoh/robot_gt.json5` — with it true
  the pilot's live-topic checker stalled the pose feed for every other subscriber.
- **MoCap straight driver** (`drive_mocap_straight_ros2.py`): new `--enable-lateral`
  switches on the existing strafe correction (it could not be enabled before), and the
  strafe sign now follows `--reverse` (it was positive feedback in reverse). agv110
  recipe: `--line-yaw-offset-deg 175 --enable-lateral --lateral-kp 1.5 --max-lateral
  0.08 --heading-kp 1.0 --pose-timeout 0.6` → 2.7 cm max lateral over 1.5 m. The yaw
  offset is per robot: measure it from a recorded straight (travel direction minus body
  yaw); a short nudge gave a value 21° off. The default `--pose-timeout 0.25` trips on
  normal feed jitter.

## 7. Camera recovery, in order

A streaming node that is SIGKILLed, or a capture process stuck in D state, leaves the
camera's control path dead (`set_xu … Connection timed out`, `VIDIOC_S_FMT` I/O error).

1. Make sure every process that held the camera is gone (wrapper, capture, `topic hz`).
2. `hardware_reset()` via pyrealsense2. Works only if the control path still answers.
3. If not: unbind/bind the VL805 (`echo 0000:01:00.0 > /sys/bus/pci/drivers/xhci_hcd/unbind`,
   5 s, `… /bind`). It usually needs **two** cycles: the first restores the SuperSpeed
   side (camera), the second the USB2 hub and the base's serial link. Check
   `/dev/ttyACM0` is back.
4. If streams still fail: physical replug for 10 s, then `hardware_reset()`.
5. After any reset, depth takes >6 s to start; test with a depth-only pipeline for 12 s.

Never SIGKILL the wrapper or the capture; stop with SIGINT and wait for
"Close Sensor - Done".

## 8. Measurement rules that bit us

- `pgrep -f`/`pkill -f` match your own shell's command text; use `[x]` bracket patterns
  and never put a node's literal name in a command that also runs a guard.
- A `cat` on `/sys/kernel/debug/hid/*/events` perturbs the HID path, and `stdbuf -oL cat`
  execs into plain `cat`, so a kill pattern on `stdbuf` leaks one reader per run. An
  afternoon of results had to be withdrawn because of ten stacked readers. Count probe
  processes before every run.
- rs_launch `config_file` wants a flat dict; the `/**: ros__parameters:` form is accepted
  silently and ignored.

## 9. Open

- Why the camera degrades with session count since power-up (power-cycle before dataset
  runs is the procedure, not the explanation).
- Fleet unit for the Zenoh client with `Nice=-15`; agv110 currently runs a
  session-scoped copy from `~/zenoh`.
- Roll to 111/114 (they still carry the 2 Nov Python draft) and the rest.
