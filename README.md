# AGV On-Board Stack

Robot-side ROS Melodic stack for AGV data collection in the multi-robot SLAM dataset project.

The goal of this repo is repeatable deployment: clone or pull it on a robot, run one setup script, then collect bags with a single session command.

## 🚀 Quick Start

On a new robot: 

### 1. Installation
On a fresh or updated robot, use one of the following methods to retrieve the stack.

**Option A: Standard Clone (Try this first)**
```bash
git clone --depth 1 [https://github.com/Gani332/agv_on-board](https://github.com/Gani332/agv_on-board) .
Rename the file to slam_project
bash scripts/setup_robot.sh
```

**Option B: Download Zip**
```bash
# Download the repository as a zip file
wget [https://github.com/Gani332/agv_on-board/archive/refs/heads/main.zip](https://github.com/Gani332/agv_on-board/archive/refs/heads/main.zip)

# Unzip the file
unzip main.zip

# Rename the resulting folder to slam_project
mv agv_on-board-main slam_project

# Remove the zip file to save space
rm main.zip

bash scripts/setup_robot.sh
```


On a updated robot:

```bash
cd ~/slam_project
git pull
bash scripts/setup_robot.sh
```

`setup_robot.sh` installs expected system dependencies by default, including
`chrony`, `apriltag_ros`, ROS message packages, rosbag, TF, and build tools. Use
`bash scripts/setup_robot.sh --skip-system` only when the robot is already
provisioned or has no internet access.

Start a data collection session:

```bash
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=false
bash scripts/logging/start_session.sh agv1 square_manual
```

Drive manually in another terminal:

```bash
ssh ubuntu@<robot-ip>
source /opt/ros/melodic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
rosrun myagv_teleop myagv_teleop.py
```

Or run a conservative automatic square:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
source /opt/ros/melodic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_square.py --side 0.75 --linear 0.22 --angular 0.28 --cycles 1
```

Stop recording with `Ctrl+C`. Bags and manifests are written to `~/agv_data`.

## Next Lab Visit Commands

Use separate terminals on the robot. Keep the robot on the floor with clear space before running motion scripts.

Terminal 1, record a straight-line bag:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=false
bash scripts/logging/start_session.sh agv1 straight_slow
```

Terminal 2, drive the straight line:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
source /opt/ros/melodic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_straight.py --distance 1.50 --speed 0.18
```

Stop Terminal 1 with `Ctrl+C`, then validate:

```bash
python3 scripts/logging/validate_bag.py $(ls -t ~/agv_data/*.bag | head -1)
python scripts/logging/audit_bag_fast.py $(ls -t ~/agv_data/*.bag | head -1)
```

Then record a square bag:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
export REQUIRE_GT=false
export REQUIRE_IMU=false
bash scripts/logging/start_session.sh agv1 square_slow
```

In another terminal:

```bash
ssh ubuntu@<robot-ip>
cd ~/slam_project
source /opt/ros/melodic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/drive_square.py --side 0.75 --linear 0.22 --angular 0.28 --cycles 1
```

Stop recording and validate again:

```bash
python3 scripts/logging/validate_bag.py $(ls -t ~/agv_data/*.bag | head -1)
python scripts/logging/audit_bag_fast.py $(ls -t ~/agv_data/*.bag | head -1)
```

## What Is Production

Use these paths for normal robot operation:

```text
scripts/setup_robot.sh                     Build/check workspaces after clone or pull
scripts/logging/start_session.sh           One-command bringup + rosbag + manifest
scripts/logging/validate_bag.py            Full post-run publishability check
scripts/logging/audit_bag_fast.py          Fast topic/rate/gap/sync audit
scripts/logging/drive_straight.py          Odom-bounded straight-line dataset helper
scripts/logging/drive_square.py            Odom-bounded square motion helper
scripts/logging/drive_forward_back.py      Odom-bounded smoke-test motion helper
agv_ws/src/agv_bringup/launch/bringup.launch
agv_ws/src/agv_bringup/launch/logging.launch
agv_ws/src/agv_bringup/launch/aruco.launch
agv_ws/src/agv_bringup/launch/aruco_bringup.launch
agv_ws/src/agv_bringup/launch/aruco_test.launch
agv_ws/src/agv_bringup/launch/apriltag.launch
agv_ws/src/agv_bringup/calibration/
```

Diagnostic and hardware-investigation scripts live under:

```text
scripts/diagnostics/
```

## Repository Layout

```text
agv_on-board/
├── myagv_ros/                  Vendor AGV base, odometry, teleop, LiDAR ROS packages
├── agv_ws/
│   └── src/
│       ├── agv_bringup/        Dataset launch files, TFs, calibration, tag config
│       └── realsense-ros/      Vendored RealSense ROS wrapper
├── scripts/
│   ├── setup_robot.sh          Build/check robot after clone or pull
│   ├── calibration/            Calibration extraction and static tests
│   ├── diagnostics/            Hardware debug scripts
│   └── logging/                Recording, validation, motion helpers
├── docs/                       SOPs and dataset checklists
├── drivers/                    Vendored third-party SDK/reference code
└── configs/                    RViz configs
```

## Robot Runtime

Source order matters:

```bash
source /opt/ros/melodic/setup.bash
source ~/slam_project/myagv_ros/devel/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
```

Manual bringup without recording:

```bash
roslaunch agv_bringup bringup.launch enable_imu:=false
```

Current ArUco marker test, using the 15 cm `DICT_ARUCO_ORIGINAL` marker id `503`:

```bash
roslaunch agv_bringup aruco_bringup.launch target_id:=503 marker_size:=0.15 publish_image:=false
```

This runs normal robot bringup plus the ArUco detector. It prints detections and publishes the target marker pose on `/aruco/target_pose` as `geometry_msgs/PoseStamped`. The pose frame is the RealSense optical frame, where `x` is right, `y` is down, and `z` is forward.

Camera-only ArUco smoke test:

```bash
roslaunch agv_bringup aruco_test.launch target_id:=503 marker_size:=0.15 publish_image:=false
```

AprilTag detector only, for real AprilTag markers:

```bash
roslaunch agv_bringup apriltag.launch
```

Production recording:

```bash
bash scripts/logging/start_session.sh <robot_name> <scenario_name>
```

`start_session.sh` writes:

```text
~/agv_data/<robot>_<scenario>_<timestamp>.bag
~/agv_data/<robot>_<scenario>_<timestamp>_manifest.yaml
~/agv_data/<robot>_<scenario>_<timestamp>_chrony.txt
```

It records with `rosbag --buffsize=2048 --lz4`, which was validated on the live robot for RGB-D + LiDAR recording without buffer overflow.

## Recorded Topics

Default robot bag topics:

```text
/scan
/odom
/cmd_vel
/tf
/tf_static
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
/camera/extrinsics/depth_to_color
/diagnostics
/aruco/target_pose
```

Optional topics are included when available:

```text
/camera/imu
/camera/accel/sample
/camera/gyro/sample
/camera/accel/imu_info
/camera/gyro/imu_info
${MOCAP_TOPIC:-/phasespace/rigids}
/mocap
```

Use:

```bash
export REQUIRE_GT=true
export MOCAP_TOPIC=/phasespace/rigids
```

when ground truth must be present in the same ROS graph. If PhaseSpace ground truth is recorded separately, keep `REQUIRE_GT=false` and save chrony status on both machines.

## Current Validated Baseline

Live robot bag checked on 2026-04-29:

```text
bag: agv1_square_manual_20260429_224111.bag
duration: 85.1 s
/scan: 17.93 Hz
/odom: 12.65 Hz
/camera/color/image_raw: 14.96 Hz
/camera/aligned_depth_to_color/image_raw: 14.96 Hz
/tf: 87.02 Hz
camera color/depth sync: 0.00 ms median, 0.00 ms max
diagnostics: 0 warnings, 0 errors
overall audit: PASS
```

This is good enough for robot-only Week 1 SLAM smoke validation.

Known limitations:

```text
RealSense D455 IMU is disabled by default. The current D455/wrapper/firmware stack publishes IMU in IMU-only mode, but not while RGB-D video is active.
ArUco target pose publishes only when the configured marker is visible; the current test marker is DICT_ARUCO_ORIGINAL id 503 with 0.15 m side length.
Ground truth is optional by default because PhaseSpace may be recorded separately on a chrony-synced machine.
```

## Transform Tree

```text
odom
└── base_footprint
    ├── base_link          static alias, colocated
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

base_footprint -> laser_frame:
  xyz=(0, 0, 0.100), rpy=(0, 0, 0)

base_footprint -> camera_link:
  xyz=(-0.132025, 0.000153, 0.187925)
  rpy=(pi/2, -0.007906, -pi/2)
```

## Validation

Fast audit:

```bash
cd ~/slam_project
source /opt/ros/melodic/setup.bash
source ~/slam_project/agv_ws/devel/setup.bash
python scripts/logging/audit_bag_fast.py ~/agv_data/<bag>.bag
```

Full validator:

```bash
python3 scripts/logging/validate_bag.py ~/agv_data/<bag>.bag
```

Exit codes:

```text
0 = pass
1 = fail
2 = warning
```

Expected warnings for the current robot-only setup:

```text
ground truth missing, unless REQUIRE_GT=true
IMU missing, unless REQUIRE_IMU=true
```

## Copy Bags To Laptop

From the laptop:

```bash
mkdir -p /Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/dataset/week1
scp ubuntu@<robot-ip>:/home/ubuntu/agv_data/*.bag \
  /Users/riyaa/Desktop/UCL_Year3/Multi-SLAM/SLAM_Code/dataset/week1/
```

## Clean Robot Run Data

On the robot:

```bash
rm -f ~/agv_data/*.bag ~/agv_data/*.bag.active ~/agv_data/*_manifest.yaml ~/agv_data/*_chrony.txt
```

## Hardware

```text
AGV base controller: /dev/ttyACM0
YDLiDAR X2:          /dev/ttyAMA0
RealSense D455:      USB 3.x, RGB-D 640x480 at 15 Hz
```

## Scaling To More Robots

For each robot:

1. Clone/pull this repo to `~/slam_project`.
2. Run `bash scripts/setup_robot.sh`.
3. Assign a stable robot name, e.g. `agv1`, `agv2`, `agv3`.
4. Record with `bash scripts/logging/start_session.sh <robot_name> <scenario>`.
5. Keep robot bags and any separate PhaseSpace logs named with the same robot/scenario/timestamp convention.
6. Before each run, confirm chrony on robot and mocap machines if ground truth is recorded separately.
