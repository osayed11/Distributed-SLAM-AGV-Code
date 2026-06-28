#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
imu_static_test.py
------------------
Collects 60 seconds of IMU data from a stationary robot and reports:
  - Gyroscope Z drift (target: < 0.1 deg/s)
  - Accelerometer noise std dev (target: < 0.05 m/s^2)
  - Gravity alignment (accel Z should be ~9.81 m/s^2 when flat)

Run AFTER starting bringup.launch with the robot STATIONARY on a flat surface:
    roslaunch agv_bringup bringup.launch
    python scripts/calibration/imu_static_test.py

Results are printed and saved to calibration/imu_intrinsics.yaml.
"""

import os
import sys
import math
import datetime

import rospy
from sensor_msgs.msg import Imu
import yaml

IMU_TOPIC = "/camera/imu"
RECORD_SECONDS = 60
CALIB_YAML = os.path.join(
    os.path.dirname(__file__),
    "../../agv_ws/src/agv_bringup/calibration/imu_intrinsics.yaml"
)

# Pass criteria from Stage_0_Targets.md
GYRO_DRIFT_LIMIT_DEG_S  = 0.1       # deg/s
ACCEL_NOISE_LIMIT_M_S2  = 0.05      # m/s^2
GRAVITY_NOMINAL_M_S2    = 9.81
GRAVITY_TOLERANCE_M_S2  = 0.5       # warn if |az_mean - 9.81| > this


class IMUCollector:
    def __init__(self):
        self.ax = []
        self.ay = []
        self.az = []
        self.gx = []
        self.gy = []
        self.gz = []

    def callback(self, msg):
        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)
        self.gx.append(msg.angular_velocity.x)
        self.gy.append(msg.angular_velocity.y)
        self.gz.append(msg.angular_velocity.z)


def std(values):
    n = len(values)
    if n == 0:
        return 0.0
    mean = sum(values) / n
    return math.sqrt(sum((v - mean) ** 2 for v in values) / n)


def mean(values):
    return sum(values) / len(values) if values else 0.0


def main():
    rospy.init_node("imu_static_test", anonymous=True)

    # Verify topic is publishing before starting
    rospy.loginfo("Waiting for first IMU message on %s ...", IMU_TOPIC)
    try:
        rospy.wait_for_message(IMU_TOPIC, Imu, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("No IMU data on %s. Is bringup.launch running with IMU enabled?", IMU_TOPIC)
        sys.exit(1)

    collector = IMUCollector()
    sub = rospy.Subscriber(IMU_TOPIC, Imu, collector.callback)

    rospy.loginfo("Recording %d seconds of IMU data. DO NOT MOVE THE ROBOT.", RECORD_SECONDS)
    rate = rospy.Rate(1)
    for i in range(RECORD_SECONDS):
        if rospy.is_shutdown():
            break
        rospy.loginfo("  %d/%d seconds, %d samples so far...", i + 1, RECORD_SECONDS, len(collector.gz))
        rate.sleep()

    sub.unregister()

    n = len(collector.gz)
    if n < 100:
        rospy.logerr("Only %d samples collected — something is wrong. Expected ~%d.",
                     n, 200 * RECORD_SECONDS)
        sys.exit(1)

    rospy.loginfo("Collected %d samples. Computing statistics...", n)

    # Gyroscope
    gz_mean     = mean(collector.gz)
    gz_std      = std(collector.gz)
    gz_drift_deg = abs(gz_mean) * 180.0 / math.pi
    gx_std      = std(collector.gx)
    gy_std      = std(collector.gy)

    # Accelerometer
    ax_mean = mean(collector.ax)
    ay_mean = mean(collector.ay)
    az_mean = mean(collector.az)
    ax_std = std(collector.ax)
    ay_std = std(collector.ay)
    az_std = std(collector.az)
    accel_noise_max = max(ax_std, ay_std, az_std)

    # Gravity: check vector magnitude, not just Z axis.
    # The D455 may be mounted with gravity on any axis depending on orientation.
    gravity_magnitude = math.sqrt(ax_mean**2 + ay_mean**2 + az_mean**2)
    gravity_error = abs(gravity_magnitude - GRAVITY_NOMINAL_M_S2)
    # Identify dominant gravity axis (whichever has the largest absolute mean)
    gravity_axis = ["x", "y", "z"][
        [abs(ax_mean), abs(ay_mean), abs(az_mean)].index(
            max(abs(ax_mean), abs(ay_mean), abs(az_mean)))]

    # Pass / fail
    gyro_pass  = gz_drift_deg < GYRO_DRIFT_LIMIT_DEG_S
    accel_pass = accel_noise_max < ACCEL_NOISE_LIMIT_M_S2
    grav_pass  = gravity_error < GRAVITY_TOLERANCE_M_S2

    rospy.loginfo("")
    rospy.loginfo("=== IMU STATIC TEST RESULTS ===")
    rospy.loginfo("Gyro Z drift:    %.4f deg/s  (target < %.1f)  --> %s",
                  gz_drift_deg, GYRO_DRIFT_LIMIT_DEG_S, "PASS" if gyro_pass else "FAIL")
    rospy.loginfo("Accel noise max: %.4f m/s^2  (target < %.2f)  --> %s",
                  accel_noise_max, ACCEL_NOISE_LIMIT_M_S2, "PASS" if accel_pass else "FAIL")
    rospy.loginfo("Gravity magnitude: %.3f m/s^2  (nominal %.2f, dominant axis=%s) --> %s",
                  gravity_magnitude, GRAVITY_NOMINAL_M_S2, gravity_axis,
                  "PASS" if grav_pass else "CHECK MOUNTING")
    rospy.loginfo("Accel means: x=%.3f  y=%.3f  z=%.3f m/s^2",
                  ax_mean, ay_mean, az_mean)
    rospy.loginfo("Accel stds:  x=%.4f  y=%.4f  z=%.4f m/s^2",
                  ax_std, ay_std, az_std)
    rospy.loginfo("")

    if not (gyro_pass and accel_pass):
        rospy.logwarn("One or more criteria FAILED.")
        rospy.logwarn("NOTE: High accel noise is expected if base controller motors are running.")
        rospy.logwarn("For a clean static test, stop base controller or power off motors.")

    # Update imu_intrinsics.yaml
    today = datetime.date.today().isoformat()
    calib = {
        "calibration_date": today,
        "calibration_operator": os.uname()[1],
        "calibration_method": "static_test",
        "frame_id": "camera_imu_optical_frame",
        "publish_rate_hz": round(n / RECORD_SECONDS),
        "accelerometer": {
            "noise_density": None,
            "bias_instability": None,
            "scale_and_alignment": [[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]],
            "bias": [0.0, 0.0, 0.0],
            "static_test_std_m_s2": {"x": round(ax_std, 6), "y": round(ay_std, 6), "z": round(az_std, 6)},
            "static_test_mean_m_s2": {"x": round(ax_mean, 4), "y": round(ay_mean, 4), "z": round(az_mean, 4)}
        },
        "gyroscope": {
            "noise_density": None,
            "bias_instability": None,
            "scale_and_alignment": [[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]],
            "bias": [0.0, 0.0, 0.0],
            "static_test_std_rad_s": {"x": round(gx_std, 6), "y": round(gy_std, 6), "z": round(gz_std, 6)}
        },
        "imu_to_color_extrinsic": {
            "note": "Read from SDK: rs-enumerate-devices -c | grep -A20 'Motion Intrinsics'",
            "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "translation": [0.0, 0.0, 0.0]
        },
        "static_test_60s": {
            "num_samples": n,
            "gyro_drift_deg_per_s": round(gz_drift_deg, 4),
            "accel_noise_m_s2": round(accel_noise_max, 4),
            "gravity_magnitude_m_s2": round(gravity_magnitude, 4),
            "gravity_dominant_axis": gravity_axis,
            "gravity_alignment_verified": grav_pass,
            "gyro_pass": gyro_pass,
            "accel_pass": accel_pass,
            "note": "Accel noise includes motor vibration if base controller was running during test."
        }
    }

    out_path = os.path.realpath(CALIB_YAML)
    with open(out_path, "w") as f:
        yaml.dump(calib, f, default_flow_style=False)

    rospy.loginfo("Written to %s", out_path)
    overall = "PASS" if (gyro_pass and accel_pass) else "FAIL"
    rospy.loginfo("Overall Stage 1d result: %s", overall)


if __name__ == "__main__":
    main()
