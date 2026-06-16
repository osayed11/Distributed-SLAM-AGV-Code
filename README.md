# AGV On-Board Stack

Robot-side ROS2 Humble stack for AGV data collection in the multi-robot SLAM dataset project.

The goal of this repo is repeatable deployment: clone or pull it on a robot, run one setup script, then collect bags with a single session command.

## Post-Flash Robot Setup

After flashing Ubuntu onto the Raspberry Pi, do the following before anything else.

### 1. Set the hostname

```bash
sudo hostnamectl set-hostname agv<N>   # e.g. agv37
sudo reboot
```

### 2. Connect to WiFi

```bash
sudo nmcli device wifi connect "<SSID>" password "<PASSWORD>"
```

Verify:

```bash
ip addr show wlan0   # confirm an IP is assigned
ping -c 3 8.8.8.8   # confirm internet access
```

### 3. Enable hostname-based SSH (no IP needed)

Install `avahi-daemon` on the robot so it advertises `agv<N>.local` over mDNS:

```bash
sudo apt update
sudo apt install -y avahi-daemon
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon
```

On your **laptop** (run once):

```bash
sudo apt install -y libnss-mdns        # Linux
# macOS already supports .local via Bonjour — no install needed
```

You can now SSH using either the hostname or IP:

```bash
ssh ubuntu@agv37.local     # hostname (preferred)
ssh ubuntu@<robot-ip>      # fallback if mDNS is unavailable
```

---

## 🚀 Quick Start

On a new robot:

### 1. Installation
On a fresh or updated robot, use one of the following methods to retrieve the stack.

```bash
git clone https://github.com/osayed11/Distributed-SLAM-AGV-Code.git ~/slam_project
cd ~/slam_project
git checkout ros2-migration
bash scripts/setup_robot.sh
```

On an updated robot:

```bash
cd ~/slam_project
git checkout ros2-migration
git pull
bash scripts/setup_robot.sh
```

`setup_robot.sh` installs expected system dependencies by default, including
`chrony`, `apriltag_ros`, ROS2 message packages, and build tools (`colcon`).
After building it validates all critical packages with `ros2 pkg list`. Use
`bash scripts/setup_robot.sh --skip-system` only when the robot is already
provisioned or has no internet access.

### 2. Ground Truth (OptiTrack)

The lab uses OptiTrack motion capture via a VRPN server on the Motive machine (`192.168.50.200:3883`).

Verify the OptiTrack stream is live before recording. The OptiTrack system publishes on ROS2 — on macOS start a pixi shell first:

```bash
pixi shell
```

Then check the topic is publishing:

```bash
ros2 topic echo /optitrack/rigid_bodies/orkar_agv1
```

If the topic is silent, check that:
1. The robot is within the OptiTrack camera capture volume
2. The rigid body is actively tracked in Motive (green indicator)

To visualise the ground truth trajectory in RViz2:

```bash
rviz2
```

In RViz2:
1. Set **Fixed Frame** to `world`
2. Click **Add → By topic → /optitrack/rigid_bodies/orkar_agv1 → Path**
3. The path will trace the robot's trajectory at ~100 Hz as it moves

**ROS2 (current)**

The mocap topic is recorded automatically. The default is `/optitrack/rigid_bodies/orkar_agv1`. Set `MOCAP_TOPIC` in the environment to override it for a different robot:

```bash
export MOCAP_TOPIC=/optitrack/rigid_bodies/orkar_agv2
```

**ROS1 (legacy)**

Install the VRPN client once per robot:

```bash
sudo apt install ros-noetic-vrpn-client-ros
```

Run it alongside `bringup.launch` in a separate terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
roslaunch vrpn_client_ros sample.launch server:=192.168.50.200
```

Verify the stream:

```bash
rostopic echo /optitrack/rigid_bodies/orkar_agv1
```

If the topic is silent, check that:
1. The robot is within the OptiTrack camera capture volume
2. The rigid body is actively tracked in Motive (green indicator)
3. VRPN streaming is enabled in Motive under **View → Data Streaming**

> The VRPN version mismatch warning (`07.33` vs `07.34`) is benign.

**Rigid body naming convention**

| Robot | Motive rigid body name | Topic |
|-------|----------------------|-------|
| AGV1  | `orkar_agv1`         | `/optitrack/rigid_bodies/orkar_agv1` |
| AGV2  | `orkar_agv2`         | `/optitrack/rigid_bodies/orkar_agv2` |
| AGV3  | `orkar_agv3`         | `/optitrack/rigid_bodies/orkar_agv3` |
| AGV4  | `orkar_agv4`         | `/optitrack/rigid_bodies/orkar_agv4` |

### 3. Record a session

Start a data collection session:

```bash
cd ~/slam_project
export REQUIRE_IMU=true
bash scripts/logging/start_session.sh agv1 square_manual
```

`start_session.sh` manages the full lifecycle:
1. Launches `bringup.launch.py` (base driver, LiDAR, camera)
2. Waits for `/scan`, `/odom`, and camera streams to stabilise
3. Starts `ros2 bag record` only after sensors are confirmed live
4. On `Ctrl+C`, stops recording cleanly → stops bringup → finalises manifest

Drive manually in another terminal:

```bash
ssh ubuntu@agv37.local          # or ssh ubuntu@<robot-ip>
source /opt/ros/humble/setup.bash
source ~/slam_project/agv2_ws/install/setup.bash
ros2 run myagv_teleop myagv_teleop.py
```

Or run an automatic motion pattern:

```bash
# Lawnmower (straight-line shuttle)
python3 scripts/logging/drive_lawnmower.py --duration 20 --linear 0.15 --cycles 3

# Circle (for concentric-circle scenarios)
python3 scripts/logging/drive_circle.py --radius 0.50 --linear 0.16 --duration 60 --no-prompt --verbose
```

Stop recording with `Ctrl+C`. Bags and manifests are written to `~/agv_data`.

## Scenario 1: Concentric Circles

For multi-robot concentric-circle runs, use the fleet launcher from the parent directory:

```bash
./launch_fleet.sh
```

Or run a single robot with epoch-based stagger timing:

```bash
T0=$(($(date +%s)+300))
python3 scripts/logging/drive_circle.py --radius 0.50 --linear 0.16 --duration 600 --start-at-epoch $T0 --start-delay 0 --no-prompt --verbose
```

Robot assignments for S1:

```text
agv1 -> radius 0.50 m -> starts at T0 +   0 s
agv2 -> radius 0.75 m -> starts at T0 +  30 s
agv3 -> radius 1.00 m -> starts at T0 +  60 s
agv4 -> radius 1.25 m -> starts at T0 +  90 s
agv5 -> radius 1.50 m -> starts at T0 + 120 s
```

## Validation

Fast audit:

```bash
cd ~/slam_project
source /opt/ros/humble/setup.bash
source ~/slam_project/agv2_ws/install/setup.bash
python3 scripts/logging/audit_bag_fast.py ~/agv_data/<bag_dir>
```

Full validator:

```bash
python3 scripts/logging/validate_bag.py ~/agv_data/<bag_dir>
```

Exit codes:

```text
0 = pass
1 = fail
2 = warning
```

## Copy Bags To Laptop

ROS2 bags are directories (containing `.db3` and `metadata.yaml`). Copy the whole directory from the laptop:

```bash
# Using hostname
scp -r ubuntu@agv37.local:/home/ubuntu/agv_data/<bag_dir>/ ~/Desktop/slam_data/
scp ubuntu@agv37.local:/home/ubuntu/agv_data/*_manifest.yaml ~/Desktop/slam_data/
scp ubuntu@agv37.local:/home/ubuntu/agv_data/*_chrony.txt ~/Desktop/slam_data/

# Using IP
scp -r ubuntu@<robot-ip>:/home/ubuntu/agv_data/<bag_dir>/ ~/Desktop/slam_data/
scp ubuntu@<robot-ip>:/home/ubuntu/agv_data/*_manifest.yaml ~/Desktop/slam_data/
scp ubuntu@<robot-ip>:/home/ubuntu/agv_data/*_chrony.txt ~/Desktop/slam_data/
```

To copy all bags at once:

```bash
rsync -avz ubuntu@agv37.local:/home/ubuntu/agv_data/ ~/Desktop/slam_data/
```

## What Is Production

Use these paths for normal robot operation:

```text
scripts/setup_robot.sh                     Build/check workspaces after clone or pull
scripts/logging/start_session.sh           Managed bringup + sensor gate + ros2 bag + manifest
scripts/logging/validate_bag.py            Full post-run publishability check
scripts/logging/audit_bag_fast.py          Fast topic/rate/gap/sync audit
scripts/logging/drive_circle.py            Concentric-circle motion controller
scripts/logging/drive_lawnmower.py         Straight-line shuttle motion controller
scripts/logging/drive_square.py            Odom-bounded square motion helper
scripts/scenarios/run_s1_concentric_robot.sh  Single-robot S1 runner with epoch stagger
agv2_ws/src/agv_bringup/launch/bringup.launch.py
agv2_ws/src/agv_bringup/launch/apriltag.launch.py
agv2_ws/src/agv_bringup/calibration/
```

Diagnostic and hardware-investigation scripts live under:

```text
scripts/diagnostics/
```

## Repository Layout

```text
slam_project/
├── agv2_ws/                 <-- ROS2 workspace (active)
│   └── src/
│       ├── agv_bringup/     <-- Master launch files + config + calibration
│       ├── myagv_odometry/  <-- Encoder/Motor feedback + base IMU
│       ├── myagv_teleop/    <-- Keyboard/PS2 control
│       └── ydlidar_ros2_driver/ <-- LiDAR drivers
├── agv_ws/                  <-- ROS1 workspace (legacy)
│   └── src/
│       ├── agv_bringup/     <-- ROS1 launch files
│       ├── myagv_odometry/  <-- ROS1 odometry
│       ├── ydlidar_ros_driver/ <-- ROS1 LiDAR driver
│       ├── realsense-ros/   <-- Camera drivers
│       ├── myagv_teleop/    <-- Keyboard/PS2 control
│       ├── myagv_navigation/<-- Navigation stack
│       ├── myagv_ps2/       <-- PS2 controller support
│       ├── myagv_urdf/      <-- Robot model
│       └── charging/        <-- Charging dock support
├── scripts/                 <-- Automation & Utility
│   ├── setup_robot.sh       <-- One-click installer (with swap, chrony, udev)
│   ├── logging/             <-- Data collection scripts
│   │   ├── start_session.sh <-- Managed lifecycle: bringup → sensor gate → ros2 bag
│   │   ├── drive_circle.py  <-- Concentric-circle motion controller
│   │   ├── drive_lawnmower.py <-- Straight-line shuttle motion controller
│   │   ├── drive_square.py  <-- Square motion controller
│   │   ├── validate_bag.py  <-- Full bag quality validator
│   │   └── audit_bag_fast.py<-- Fast topic/rate auditor
│   ├── diagnostics/         <-- Health check scripts
│   ├── calibration/         <-- Sensor calibration tools
│   └── scenarios/           <-- Scenario runner scripts
├── drivers/                 <-- Vendored SDKs
│   ├── YDLidar-SDK/         <-- YDLidar native SDK
│   └── robot_pose_ekf/      <-- EKF for sensor fusion
├── configs/                 <-- RViz configs
├── docs/                    <-- SOPs and Manuals
└── README.md
```

## Robot Runtime

Source order matters:

```bash
source /opt/ros/humble/setup.bash
source ~/slam_project/agv2_ws/install/setup.bash
```

Manual bringup without recording:

```bash
ros2 launch agv_bringup bringup.launch.py
```

AprilTag detector (optional, for offline loop-closure injection):

```bash
ENABLE_APRILTAG=true bash scripts/logging/start_session.sh agv1 corridor_loop
```

Production recording:

```bash
bash scripts/logging/start_session.sh <robot_name> <scenario_name>
```

`start_session.sh` writes:

```text
~/agv_data/<robot>_<scenario>_<timestamp>/             ROS2 bag directory
~/agv_data/<robot>_<scenario>_<timestamp>_manifest.yaml
~/agv_data/<robot>_<scenario>_<timestamp>_chrony.txt
~/agv_data/<robot>_<scenario>_<timestamp>_bringup.log
```

## Recorded Topics

Default robot bag topics:

```text
/scan                                    YDLidar X2 2D laser scans
/odom                                    Wheel odometry from base MCU
/cmd_vel                                 Velocity commands sent to base
/tf                                      Dynamic transform tree
/tf_static                               Static transforms
/camera/color/image_raw                  D455 RGB image
/camera/color/camera_info                RGB camera intrinsics
/camera/depth/camera_info                Depth camera intrinsics
/camera/aligned_depth_to_color/image_raw D455 aligned depth image
/camera/aligned_depth_to_color/camera_info Aligned depth intrinsics
/camera/extrinsics/depth_to_color        Depth-to-color extrinsics
/camera/imu                              D455 camera IMU (accel + gyro)
```

Optional topics (recorded when detectors are enabled or ground truth is present):

```text
/imu                                                   Base MCU IMU, when published
/diagnostics                                           ROS diagnostics, when published
/tag_detections                                        AprilTag detections (ENABLE_APRILTAG=true)
${MOCAP_TOPIC:-/optitrack/rigid_bodies/orkar_agv1}     OptiTrack ground truth (set MOCAP_TOPIC per robot)
```

> **Note:** On the ROS2 Humble stack, `/camera/imu` is the validated IMU stream
> and should be required for new dataset runs with `REQUIRE_IMU=true`.

## Current Validated Baseline

Live robot bag checked on 2026-04-29:

```text
bag: agv37_ros2_20260611_153804
duration: 68.7 s
/scan: 17.2 Hz
/odom: 12.7 Hz
/camera/color/image_raw: 14.9 Hz
/camera/aligned_depth_to_color/image_raw: 14.6 Hz
/camera/imu: 192.4 Hz
/tf: 12.7 Hz
overall validation: WARN, no hard failures
```

Known limitations:

```text
Occasional RGB-D frame gaps were observed on agv37; usable but flag in QA.
Ground truth is optional by default; set REQUIRE_GT=true when OptiTrack must be in-bag.
```

## Transform Tree

```text
odom
└── base_footprint
    ├── base_link          static alias, colocated
    ├── imu_link           base MCU IMU frame
    ├── laser_frame        z=0.100 m measured
    └── camera_link        CAD extrinsic from original mount
        ├── camera_color_frame
        ├── camera_depth_frame
        └── camera_aligned_depth_to_color_frame
```

Important static transforms:

```text
base_footprint -> base_link:
  xyz=(0, 0, 0), rpy=(0, 0, 0)

base_footprint -> imu_link:
  xyz=(0, 0, 0), rpy=(0, pi, pi)

base_footprint -> laser_frame:
  xyz=(0, 0, 0.100), rpy=(0, 0, 0)

base_footprint -> camera_link:
  xyz=(-0.132025, 0.000153, 0.187925)
  rpy=(pi/2, -0.007906, -pi/2)
```

## Clean Robot Run Data

On the robot:

```bash
rm -rf ~/agv_data/*/
rm -f ~/agv_data/*_manifest.yaml ~/agv_data/*_chrony.txt ~/agv_data/*_bringup.log
```

## Hardware

```text
AGV base controller: /dev/ttyACM0 (symlink /dev/myAGV via udev)
YDLiDAR X2:          /dev/ydlidar (symlink via udev, native /dev/ttyUSB0)
RealSense D455:      USB 3.x, RGB-D 640x480 @ 15 Hz plus /camera/imu
Base MCU IMU:        Optional /imu topic when exposed by the base driver
```

## Scaling To More Robots

For each robot:

1. Flash Ubuntu arm64, set hostname (`agv<N>`), connect to WiFi, install avahi-daemon (see Post-Flash setup above).
2. Clone/pull this repo to `~/slam_project`.
3. Run `bash scripts/setup_robot.sh`.
4. Assign a stable robot name, e.g. `agv1`, `agv2`, `agv3`.
5. Record with `bash scripts/logging/start_session.sh <robot_name> <scenario>`.
6. For fleet orchestration, use `launch_fleet.sh` from the parent directory.
7. Before each run, confirm chrony on robot and mocap machines if ground truth is recorded separately.
