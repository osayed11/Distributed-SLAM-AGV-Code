# Distributed SLAM AGV On-Board Stack

This repository is the robot-side stack for repeatable ROS 2 AGV data
collection in the multi-robot SLAM dataset project.

The current production branch is:

```text
repository: https://github.com/osayed11/Distributed-SLAM-AGV-Code.git
branch:     ros2-migration
robot OS:   Ubuntu 22.04 on Raspberry Pi
ROS:        ROS 2 Humble
```

The design goal is simple: after a robot is flashed and provisioned, an
operator should be able to switch it on, run one readiness gate, record one
session, and validate one bag. If the robot is not ready, the tools should say
which branch failed, show the evidence, and give the next action.

## Operating Model

Every publishable run follows four gates:

```text
1. Provision robot once
   -> scripts/setup_robot_ros2.sh

2. Prove robot is ready before motion
   -> scripts/diagnostics/dataset_ready_gate.sh

3. Record with managed bringup and logging
   -> scripts/logging/start_session.sh

4. Prove the recorded artifact is usable
   -> scripts/logging/validate_ros2_bag.py
   -> scripts/diagnostics/dataset_run_audit.py for copied fleet artifacts
```

Do not treat a bag as publishable until the post-run validator/audit passes.
Pre-run readiness proves the robot state before motion; it does not prove the
bag that was later recorded.

## New Robot Setup

Run these commands on a freshly flashed robot.

### 1. Set hostname

Use the physical robot label, for example `agv110`.

```bash
sudo hostnamectl set-hostname agv110
sudo reboot
```

After reboot:

```bash
hostname
hostname -I
```

### 2. Connect Wi-Fi cleanly

Use NetworkManager. Do not manually run repeated `wpa_supplicant -B` and
`dhclient` commands; duplicate Wi-Fi/DHCP processes make SSH and ROS discovery
unreliable.

```bash
sudo nmcli device wifi connect "<LAB_WIFI_SSID>" password "<LAB_WIFI_PASSWORD>"
nmcli -t -f DEVICE,STATE,CONNECTION device
ip addr show wlan0
ping -c 3 8.8.8.8
```

Optional hostname-based SSH:

```bash
sudo apt update
sudo apt install -y avahi-daemon
sudo systemctl enable --now avahi-daemon
```

Then from the laptop:

```bash
ssh ubuntu@agv110.local
```

### 3. Clone the ROS 2 branch

```bash
git clone -b ros2-migration https://github.com/osayed11/Distributed-SLAM-AGV-Code.git ~/slam_project
cd ~/slam_project
```

For an existing robot:

```bash
cd ~/slam_project
git checkout ros2-migration
git pull --ff-only origin ros2-migration
```

### 4. Provision the robot

```bash
cd ~/slam_project
SUDO_PASSWORD=ubuntu bash scripts/setup_robot_ros2.sh agv110
```

The setup script installs ROS 2 dependencies, RealSense tools, MCAP rosbag
storage, Python MCAP parsing, the native YDLidar SDK, udev rules, LiDAR/base
permissions, and builds `agv2_ws`. It also runs a static diagnostic report at
the end.

Use these only for special cases:

```bash
bash scripts/setup_robot_ros2.sh agv110 --skip-system
bash scripts/setup_robot_ros2.sh agv110 --skip-build
bash scripts/setup_robot_ros2.sh agv110 --no-doctor
```

## Fleet Hardware Standard

All dataset robots should match this standard before collection:

```text
Camera:                         Intel RealSense D455
D455 firmware:                  5.17.0.10
USB link:                       USB 3.x / 5000 Mb/s
D455 IMU:                       BMI085
Standalone librealsense tools:  2.58.1
ROS driver package:             realsense2_camera 4.57.7
ROS node LibRealSense runtime:  2.57.7
RGB-D profile:                  640x480 at 15 Hz
D455 IMU rate:                  about 200 Hz live, >=150 Hz in recorded bags
Gyro raw topic:                 /camera/gyro/sample at about 200 Hz
Accel raw topic:                /camera/accel/sample at about 100 Hz
Bag storage:                    MCAP preferred; SQLite only with resilient evidence
```

The standard is encoded in:

```text
configs/robot_doctor_dataset_gate.json
configs/rosbag2_sensor_qos.yaml
configs/sqlite_resilient.yaml
```

## Pre-Run Readiness Gate

Run this after switching on a robot and before collecting a dataset.

```bash
cd ~/slam_project
bash scripts/diagnostics/dataset_ready_gate.sh agv110 \
  --expected-d455-serial <assigned_d455_serial> \
  --mocap-topic /optitrack/rigid_bodies/orkar_agv110 \
  --cmd-topic /agv110/cmd_vel \
  --strict-ops \
  --confirm-mechanical \
  --confirm-mocap \
  --confirm-anchors
```

Expected healthy decision:

```text
READY_TO_RECORD: true
POST_RUN_DATASET_READY: false
STATE: ready_to_record
FAILED_STAGE: none
CAUSE: pre-run gate passed; no blocking failures or pre-run warnings
```

`POST_RUN_DATASET_READY: false` is normal before recording because no bag exists
yet. If `READY_TO_RECORD` is false, do not collect publishable data. Use the
printed `FAILED_STAGE`, `CAUSE`, `EVIDENCE`, and `NEXT_ACTION`.

The report is written under:

```text
~/agv_data/diagnostics/<robot_id>_<timestamp>/
```

Keep this folder with the dataset notes.

## Recording A Session

Start recording on the robot:

```bash
cd ~/slam_project
REQUIRE_IMU=true \
REQUIRE_GT=true \
MOCAP_TOPIC=/optitrack/rigid_bodies/orkar_agv110 \
CMD_TOPIC=/agv110/cmd_vel \
bash scripts/logging/start_session.sh agv110 <scenario_name>
```

For a home test without OptiTrack:

```bash
cd ~/slam_project
REQUIRE_IMU=true \
REQUIRE_GT=false \
CMD_TOPIC=/agv110/cmd_vel \
bash scripts/logging/start_session.sh agv110 home_test
```

`start_session.sh` does this lifecycle:

```text
1. waits for clock sync before naming the session
2. records chrony and hardware snapshots
3. stops stale bringup/recording processes
4. resets only the D455 USB device and disables D455 autosuspend
5. launches ROS 2 bringup once
6. waits for /scan, /odom, RGB-D, and IMU topics
7. runs the required live RealSense gate on the active bringup
8. starts ros2 bag record with MCAP, sensor QoS overrides, and large cache
9. runs a runtime watchdog for low-bandwidth liveness checks
10. stops `ros2 bag` cleanly, stops bringup, checks post-run D455 enumeration
11. writes the manifest and RealSense fault classification
```

Session artifacts are written to `~/agv_data`:

```text
<session>/                                      ROS 2 bag directory
<session>_manifest.yaml                        run metadata and recorder settings
<session>_chrony.txt                           clock evidence
<session>_bringup.log                          launch and driver logs
<session>_hardware_pre.log                     pre-run hardware snapshot
<session>_hardware_post.log                    post-run hardware snapshot
<session>_camera_gate_pre.log                  RealSense live gate evidence
<session>_runtime_watchdog.log                 runtime liveness evidence
<session>_kernel_runtime.log                   USB/kernel evidence for this run
<session>_realsense_fault_classification.txt   camera fault classification
```

Stop recording with `Ctrl+C`. Let the script finalize; do not kill the terminal
unless the robot is unsafe.

## Post-Run Validation

Validate the newest ROS 2 bag on the robot:

```bash
cd ~/slam_project
latest_bag="$(find ~/agv_data -maxdepth 1 -type d -name 'agv110_*' | sort | tail -1)"
CMD_TOPIC=/agv110/cmd_vel \
MOCAP_TOPIC=/optitrack/rigid_bodies/orkar_agv110 \
REQUIRE_IMU=true \
REQUIRE_GT=true \
python3 scripts/logging/validate_ros2_bag.py "$latest_bag" --require-resilient-storage
```

For a home test without OptiTrack:

```bash
CMD_TOPIC=/agv110/cmd_vel \
REQUIRE_IMU=true \
REQUIRE_GT=false \
python3 scripts/logging/validate_ros2_bag.py "$latest_bag" --require-resilient-storage
```

Interpreting the result:

```text
FAIL 0, WARN 0       ideal
FAIL 0, WARN >0      review warnings; often acceptable for home/non-GT tests
FAIL >0              not publishable; fix the failed branch and rerun
```

For final copied dataset artifacts on the laptop:

```bash
python3 scripts/diagnostics/dataset_run_audit.py \
  --report 'diagnostic_reports/agv*/agv*/summary.json' \
  --bag '/path/to/copied/bags/*' \
  --manifest '/path/to/copied/manifests/*_manifest.yaml' \
  --mocap-topic /optitrack/rigid_bodies/<rigid_body_name> \
  --cmd-topic /<robot_name>/cmd_vel \
  --require-gt \
  --require-imu \
  --strict \
  --json-out diagnostic_reports/dataset_run_audits/latest/summary.json
```

## Copy Data To Laptop

ROS 2 bags are directories. Copy the whole directory plus manifest/logs.

```bash
rsync -avz ubuntu@agv110.local:/home/ubuntu/agv_data/ ~/Desktop/slam_data/agv110/
```

IP fallback:

```bash
rsync -avz ubuntu@<robot-ip>:/home/ubuntu/agv_data/ ~/Desktop/slam_data/agv110/
```

## Failure Model

The debugging pipeline is top-down and MECE. Every blocker should land in one
of these branches:

```text
1. Robot Platform
   1.1 Sensor device health
   1.2 Physical infrastructure
   1.3 Mechanical setup

2. Robot Data Stack
   2.1 OS / kernel / USB
   2.2 Drivers / launch config
   2.3 ROS data quality

3. Experiment Dataset
   3.1 Recording pipeline
   3.2 Validation pipeline
   3.3 Experiment execution
```

The diagram version is `robot_failure_modes_v3.png`.

Detailed documentation:

```text
docs/ROBOT_DIAGNOSTIC_PIPELINE.md
docs/ROBOT_DEBUG_PIPELINE_COVERAGE_AUDIT.md
docs/PUBLISHABILITY_CHECKLIST.md
docs/STAGE_1_CALIBRATION_SOP.md
```

## Common Decisions

### D455 warning but rates pass

If RealSense logs contain low-level UVC/control warning text but RGB-D and IMU
rates pass, the gate can classify it as a warning. The post-run bag validator is
authoritative for publishability.

### D455 stream or control path fails

Run the targeted fix once:

```bash
cd ~/slam_project
SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply \
  --fix d455-usb-reset \
  --fix d455-authorize-cycle
```

Then rerun the readiness gate. If the same physical-path failure repeats, use
the generated D455 A/B swap checklist to decide whether the fault follows the
camera, cable, host USB3 port, or power path.

### Wi-Fi is unstable

Check for manual Wi-Fi processes:

```bash
ps aux | grep -E 'wpa_supplicant|dhclient' | grep -v grep
nmcli -t -f DEVICE,STATE,CONNECTION device
```

Use one persistent NetworkManager connection. Reboot-test SSH before collecting
data.

### Bag has shutdown-edge gaps

The ROS 2 validator ignores a small start/stop edge window for gap checks while
still enforcing coverage and mid-run continuity. This avoids rejecting bags just
because the `ros2 bag` recorder and drivers shut down at different speeds after
`Ctrl+C`.

Mid-run gaps still fail.

## Motion Helpers

Run motion only after the recorder is active and the arena is clear.

Manual teleop:

```bash
source /opt/ros/humble/setup.bash
source ~/slam_project/agv2_ws/install/setup.bash
ros2 run myagv_teleop myagv_teleop.py
```

Mocap straight-line test:

```bash
python3 scripts/logging/drive_mocap_straight_ros2.py \
  --pose-topic /optitrack/rigid_bodies/orkar_agv110 \
  --cmd-topic /agv110/cmd_vel \
  --distance 1.0 \
  --linear 0.12 \
  --line-yaw-offset-deg 0 \
  --max-lateral-error 0.20 \
  --yes \
  --verbose
```

Mocap square:

```bash
python3 scripts/logging/drive_mocap_square_ros2.py \
  --pose-topic /optitrack/rigid_bodies/orkar_agv110 \
  --cmd-topic /agv110/cmd_vel \
  --side-length 1.0 \
  --linear 0.18 \
  --yes \
  --verbose
```

Circle scenario helper:

```bash
python3 scripts/logging/drive_circle.py \
  --radius 0.50 \
  --linear 0.16 \
  --duration 600 \
  --no-prompt \
  --verbose
```

## Code Map

Production paths:

```text
scripts/setup_robot_ros2.sh                    one-time ROS 2 robot provisioning
scripts/logging/start_session.sh               managed bringup, gate, recording, manifest
scripts/logging/validate_ros2_bag.py           ROS 2 .mcap/.db3 post-run validator
scripts/diagnostics/dataset_ready_gate.sh      read-only pre-run dataset gate
scripts/diagnostics/robot_doctor.py            evidence engine and failure classifier
scripts/diagnostics/apply_robot_doctor_fix.sh  targeted fixes, dry-run unless --apply
scripts/diagnostics/dataset_run_audit.py       final report/bag/manifest audit
scripts/diagnostics/fleet_doctor_summary.py    fleet-level report comparison
scripts/diagnostics/run_robot_doctor_remote.sh run diagnostics over SSH
scripts/diagnostics/run_fleet_doctor_remote.sh run diagnostics over SSH for many robots
scripts/diagnostics/robot_doctor_selftest.py   no-hardware regression tests
scripts/diagnostics/diagnostic_pipeline_audit.py repo coverage audit
configs/robot_doctor_dataset_gate.json         dataset gate values
configs/rosbag2_sensor_qos.yaml                rosbag2 sensor QoS overrides
configs/sqlite_resilient.yaml                  SQLite fallback storage profile
agv2_ws/src/agv_bringup/launch/bringup.launch.py ROS 2 bringup
```

Calibration and scenario helpers:

```text
scripts/calibration/extract_realsense_calib.py camera intrinsics capture
scripts/calibration/imu_static_test.py          stationary IMU characterization
scripts/diagnostics/odom_motion_test.py        ROS 2 odom/base response test
scripts/logging/drive_mocap_straight_ros2.py   mocap-feedback straight segment
scripts/logging/drive_mocap_square_ros2.py     mocap-feedback square
scripts/logging/drive_circle.py                odom-feedback circle scenario
scripts/logging/drive_square.py                odom-feedback square test
scripts/logging/drive_lawnmower.py             timed shuttle scenario helper
scripts/mocap/natnet_ros2_pose_publisher.py    direct NatNet-to-ROS2 helper
scripts/mocap/natnet_watch.py                  direct NatNet inspection helper
```

Repository layout:

```text
slam_project/
├── agv2_ws/                  active ROS 2 workspace
├── scripts/
│   ├── diagnostics/          readiness gates, failure classification, audits
│   ├── logging/              session recording, validators, motion helpers
│   ├── mocap/                NatNet/OptiTrack helper tools
│   └── calibration/          calibration extraction and static tests
├── configs/                  dataset gate, recorder, RViz configs
├── docs/                     SOPs and diagnostic documentation
├── drivers/YDLidar-SDK/      native SDK required by ydlidar_ros2_driver
└── README.md
```

## Development Checks

Run these before pushing diagnostic/logging changes:

```bash
python3 scripts/diagnostics/robot_doctor_selftest.py
python3 scripts/diagnostics/diagnostic_pipeline_audit.py
bash -n scripts/logging/start_session.sh
bash -n scripts/setup_robot_ros2.sh
bash -n scripts/diagnostics/dataset_ready_gate.sh
```

Expected result:

```text
robot_doctor_selftest.py: OK
diagnostic_pipeline_audit.py: all rows PASS
shell syntax checks: no output
```
