# Stage 1 Calibration SOP

This is the ROS2 calibration procedure for dataset robots. Commit outputs under
`agv2_ws/src/agv_bringup/calibration/` before publishable runs.

Do the steps in order. Do not collect publishable data if a required calibration
step fails.

## 1. Odometry

1. Place tape marks exactly 1.0 m apart on a flat floor.
2. Start bringup:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/slam_project/agv2_ws/install/setup.bash
   ros2 launch agv_bringup bringup.launch.py agv_cmd_vel_topic:=/agv110/cmd_vel
   ```

3. In another terminal, command a slow straight pulse and stop manually:

   ```bash
   ros2 topic pub -r 10 /agv110/cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.10}, angular: {z: 0.0}}"
   ```

4. Compare physical distance against `/odom`.
5. Record any scale correction in
   `agv2_ws/src/agv_bringup/calibration/extrinsics.yaml`.

Pass criterion: measured distance and odometry agree within 2 percent.

## 2. LiDAR Extrinsics

1. Measure the YDLidar X2 scan-plane centre relative to `base_footprint`.
2. Update `base_footprint_to_laser_frame` in
   `agv2_ws/src/agv_bringup/calibration/extrinsics.yaml`.
3. Confirm the driver config is still
   `agv2_ws/src/agv_bringup/config/ydlidar_x2_agv.yaml`.
4. Start bringup and verify `/scan`:

   ```bash
   ros2 topic hz /scan
   ros2 topic echo --once /scan
   ```

Pass criterion: `/scan` is live and the physical mounting values in
`extrinsics.yaml` match the robot.

## 3. RealSense Intrinsics

Start bringup, then run:

```bash
cd ~/slam_project
python3 scripts/calibration/extract_realsense_calib.py
```

This writes
`agv2_ws/src/agv_bringup/calibration/camera_intrinsics.yaml` from live ROS2
`CameraInfo` topics.

Pass criterion: color/depth intrinsics are populated and `fx` is plausible for
the selected D455 profile.

## 4. IMU Static Test

Place the robot on a stable surface and keep it still:

```bash
cd ~/slam_project
python3 scripts/calibration/imu_static_test.py --seconds 60
```

This writes `agv2_ws/src/agv_bringup/calibration/imu_intrinsics.yaml`.

Pass criterion:

- `/camera/imu` publishes near 200 Hz.
- Gyro drift and accelerometer noise are recorded in the YAML.
- Any failed static criterion is documented before dataset collection.

## 5. Camera And Marker Mounts

Measure and update:

- `base_footprint_to_camera_link` in `extrinsics.yaml`
- `mocap_to_base_footprint` in `mocap_to_base.yaml`
- per-robot marker IDs and positions in `mocap_to_base.yaml`

For OptiTrack, verify the rigid body topic before driving:

```bash
ros2 topic hz /optitrack/rigid_bodies/<rigid_body_name>
ros2 topic echo --once /optitrack/rigid_bodies/<rigid_body_name>
```

Pass criterion: the MoCap pose changes correctly when the robot is moved by
hand, and the rigid body origin/robot base offset is documented.

## 6. Final Calibration Evidence

Before a dataset run, archive:

- committed calibration YAML files
- `dataset_ready_gate.sh` report
- `start_session.sh` manifest
- post-run `validate_ros2_bag.py` JSON or terminal output

The final publishability decision comes from `dataset_run_audit.py`, not from
manual inspection alone.
