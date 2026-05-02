# Stage 1 — Sensor Calibration SOP

This document is the physical step-by-step procedure for completing Stage 1 calibration.
All output files go into `agv_ws/src/agv_bringup/calibration/` and must be committed to
the repo before any dataset runs.

Do the procedures **in order**. Each step has a pass criterion. Do not proceed to the
next step if the current one fails.

---

## 1a. Odometry Calibration

**Required tools:** tape measure, masking tape, flat floor.

### Step 1 — Linear distance calibration

1. Mark a start line on the floor with tape.
2. Mark a point exactly **2.0 m** ahead.
3. Connect to the robot and send a linear command to drive 2.0 m straight:
   ```
   rostopic pub /cmd_vel geometry_msgs/Twist \
     '{linear: {x: 0.2}, angular: {z: 0.0}}' -r 10
   ```
   Stop when visually at the 2.0 m mark.
4. Read the odometry reported distance:
   ```
   rostopic echo /odom | grep position -A3
   ```
5. Compute error: `reported_distance / actual_distance`.
6. If error > 2%, adjust `wheel_radius` in `base.yaml`:
   `new_wheel_radius = current_wheel_radius * (actual / reported)`

### Step 2 — Rotation calibration

1. Place a protractor reference mark at robot start heading.
2. Command exactly 360° rotation:
   ```
   rostopic pub /cmd_vel geometry_msgs/Twist \
     '{linear: {x: 0.0}, angular: {z: 0.5}}' -r 10
   ```
   Stop after one full visual revolution.
3. Read reported heading change from `/odom`.
4. **Pass criterion:** heading error < 2°.
5. If error > 2°, adjust `wheel_base` in `base.yaml`:
   `new_wheel_base = current_wheel_base * (actual_degrees / 360.0)`

### Step 3 — Record results

Update `calibration/extrinsics.yaml` → `odometry` section with measured values.

---

## 1b. LiDAR Extrinsic Calibration

**Required tools:** tape measure, flat wall, right-angle square.

### Step 1 — Measure physical mounting offset

1. Measure x, y, z position of the YDLiDAR X2 scan plane centre relative
   to `base_footprint` (robot centre at ground level).
   - x: forward (positive = forward)
   - y: left (positive = left)
   - z: height above ground to scan plane
2. Check the LiDAR cable exit direction:
   - Cable exits forward → `yaw = 0`
   - Cable exits backward (rotated 180°) → `yaw = pi` (current X2.launch default)
   - Cable exits to the right → `yaw = -pi/2`

### Step 2 — Update X2.launch

Edit `myagv_ros/src/ydlidar_ros_driver/launch/X2.launch`, line:
```xml
<node pkg="tf" type="static_transform_publisher" name="base_link_to_laser4"
  args="X Y Z YAW PITCH ROLL /base_footprint /laser_frame 40" />
```
Replace `X Y Z YAW PITCH ROLL` with your measurements (metres, radians).

**Note:** static_transform_publisher arg order is: `x y z yaw pitch roll parent child rate`

### Step 3 — Noise floor characterisation

1. Place the robot 1.0 m from a flat wall (use tape to mark exact distance).
2. Record 60 seconds of LiDAR data:
   ```
   rosbag record -O lidar_wall_test.bag /scan
   ```
3. Compute statistics on the distance-to-wall measurements.
   **Pass criterion:** std dev < 5 mm, no systematic angular offset > 0.5°.

### Step 4 — Update lidar.yaml

Fill in measured extrinsic values in `calibration/extrinsics.yaml` →
`base_footprint_to_laser_frame` section.

---

## 1c. RealSense D455 Camera Intrinsics

### Step 1 — Export factory calibration

Connect the D455 to the robot and run:
```bash
source ~/slam_project/agv_ws/devel/setup.bash
roslaunch agv_bringup bringup.launch
```
In a new terminal:
```bash
rostopic echo /camera/color/camera_info | head -40
```
Copy the `K` (3x3 matrix) and `D` (distortion) values into
`calibration/camera_intrinsics.yaml` → `color_camera` section.

Repeat for `/camera/depth/camera_info` → `depth_camera` section.

### Step 2 — Depth accuracy test

1. Place robot exactly **1.0 m** from a flat, texture-rich wall.
2. In `realsense-viewer` (run on a desktop with the camera), use the
   **Depth Accuracy** tool.
3. Record the reported error at 0.5 m, 1.0 m, 2.0 m.
4. Fill in `calibration/camera_intrinsics.yaml` → `depth_accuracy` section.
   **Pass criterion:** depth error < 1% at 1.0 m (< 10 mm).

### Step 3 — Optional: checkerboard intrinsic calibration

Only required if factory values give reprojection error > 0.5 px.
```bash
rosrun camera_calibration cameracalibrator.py \
  --size 9x6 --square 0.025 \
  image:=/camera/color/image_raw camera:=/camera/color
```
Save the output YAML and replace the factory values in `camera_intrinsics.yaml`.

---

## 1d. IMU Intrinsics

### Step 1 — Quick static test (minimum requirement)

1. Place robot on a stable, flat surface. Do NOT move.
2. Record 60 seconds:
   ```
   rosbag record -O imu_static.bag /camera/imu
   ```
3. Inspect gyro and accelerometer noise:
   ```python
   import rosbag, numpy as np
   msgs = [m for _, m, _ in rosbag.Bag('imu_static.bag').read_messages('/camera/imu')]
   gz = [m.angular_velocity.z for m in msgs]
   print("gyro z std:", np.std(gz), "mean:", np.mean(gz))
   ```
   **Pass criteria:**
   - Gyro drift < 0.1 deg/s (< 0.00175 rad/s)
   - Accel noise std < 0.05 m/s²

4. Fill in `calibration/imu_intrinsics.yaml` → `static_test_60s` section.

### Step 2 — Factory bias values (optional but recommended)

```bash
rs-enumerate-devices -c 2>&1 | grep -A 20 "Motion Intrinsics"
```
Copy `bias_acs` and `bias_gyr` values into `imu_intrinsics.yaml`.

---

## 1e. Camera → Base Extrinsic

### Step 1 — Physical measurement

Measure the x, y, z position of the **camera_link** origin (front face centre of the
D455) relative to `base_footprint`.
Measure mounting angles (typically 0,0,0 if mounted parallel to floor).

### Step 2 — Update extrinsics.yaml

Fill in `calibration/extrinsics.yaml` → `base_footprint_to_camera_link` section.

### Step 3 — Verify TF chain

```bash
roslaunch agv_bringup bringup.launch
rosrun tf view_frames
evince frames.pdf
```
Confirm the TF tree is continuous:
```
odom → base_footprint → base_link
base_footprint → laser_frame
base_footprint → camera_link → camera_imu_optical_frame
```
No disconnected subtrees. No TF extrapolation warnings in `rosout`.

---

## 1f. PhaseSpace Mocap Setup

### Step 1 — Define per-robot marker set

In PhaseSpace Impulse software:
1. Assign 4+ LEDs to each robot (note LED IDs).
2. Create a unique rigid body per robot.
3. Export the rigid body definition file and commit to repo.

### Step 2 — Measure mocap → base_footprint offset

1. Place robot at a known position.
2. With mocap running, record the rigid body centroid position.
3. Measure the physical offset between the rigid body centroid and
   the robot base_footprint (geometric centre at ground level).
4. Fill in `calibration/mocap_to_base.yaml`.

### Step 3 — Verify ROS driver

```bash
roslaunch phasespace_mocap phasespace.launch
rostopic hz /phasespace/rigids
```
**Pass criterion:** ≥ 100 Hz, frame IDs match expected robot IDs.

### Step 4 — Ruler accuracy test

1. Mark two points exactly 1.0 m apart on the floor.
2. Place a reflective marker at each point.
3. Read the PhaseSpace reported distance between markers.
4. **Pass criterion:** < 2 mm error.
5. Record in `calibration/mocap_to_base.yaml` → `accuracy.ruler_test_error_mm`.

---

## Completion Checklist

Before moving to Stage 2, confirm all of the following:

- [ ] `calibration/calibration_date.txt` — filled in with date, operator, robot ID, git commit
- [ ] `calibration/extrinsics.yaml` — all transforms measured, no PLACEHOLDERs
- [ ] `calibration/camera_intrinsics.yaml` — factory K/D values exported, depth tested
- [ ] `calibration/imu_intrinsics.yaml` — static test completed, bias values recorded
- [ ] `calibration/mocap_to_base.yaml` — marker set defined, offset measured
- [ ] `rosrun tf view_frames` — TF tree is continuous, no broken links
- [ ] Odometry 1m linear test — error < 2%
- [ ] Odometry 360° rotation test — error < 2°
- [ ] LiDAR wall noise floor — std dev < 5 mm
- [ ] Camera reprojection error — < 0.5 px RMS
- [ ] Depth error at 1m — < 1% (< 10 mm)
- [ ] IMU gyro drift — < 0.1 deg/s
- [ ] IMU accel noise — < 0.05 m/s²
- [ ] PhaseSpace ruler test — < 2 mm
- [ ] All calibration files committed to git
