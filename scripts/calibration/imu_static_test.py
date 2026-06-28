#!/usr/bin/env python3
"""Record a stationary ROS2 IMU stream and write calibration evidence."""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = ROOT / "agv2_ws/src/agv_bringup/calibration/imu_intrinsics.yaml"
GRAVITY_NOMINAL_M_S2 = 9.81
GRAVITY_TOLERANCE_M_S2 = 0.5
GYRO_DRIFT_LIMIT_DEG_S = 0.1
ACCEL_NOISE_LIMIT_M_S2 = 0.05


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def std(values: list[float]) -> float:
    if not values:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((value - avg) ** 2 for value in values) / len(values))


class ImuCollector(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("imu_static_test")
        self.ax: list[float] = []
        self.ay: list[float] = []
        self.az: list[float] = []
        self.gx: list[float] = []
        self.gy: list[float] = []
        self.gz: list[float] = []
        self.create_subscription(Imu, topic, self._on_imu, 50)

    def _on_imu(self, msg: Imu) -> None:
        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)
        self.gx.append(msg.angular_velocity.x)
        self.gy.append(msg.angular_velocity.y)
        self.gz.append(msg.angular_velocity.z)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/imu")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--min-samples", type=int, default=100)
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ImuCollector(args.topic)
    start = time.monotonic()
    next_print = start

    node.get_logger().info(f"Recording {args.seconds:.1f}s from {args.topic}. Keep the robot still.")
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_print:
                elapsed = time.monotonic() - start
                node.get_logger().info(f"{elapsed:.0f}/{args.seconds:.0f}s samples={len(node.gz)}")
                next_print = time.monotonic() + 5.0

        n = len(node.gz)
        if n < args.min_samples:
            node.get_logger().error(f"Only {n} samples collected; expected at least {args.min_samples}.")
            return 1

        ax_mean, ay_mean, az_mean = mean(node.ax), mean(node.ay), mean(node.az)
        gx_std, gy_std, gz_std = std(node.gx), std(node.gy), std(node.gz)
        ax_std, ay_std, az_std = std(node.ax), std(node.ay), std(node.az)
        gz_drift_deg = abs(mean(node.gz)) * 180.0 / math.pi
        accel_noise_max = max(ax_std, ay_std, az_std)
        gravity_magnitude = math.sqrt(ax_mean**2 + ay_mean**2 + az_mean**2)
        gravity_error = abs(gravity_magnitude - GRAVITY_NOMINAL_M_S2)
        gravity_axis = ["x", "y", "z"][
            [abs(ax_mean), abs(ay_mean), abs(az_mean)].index(max(abs(ax_mean), abs(ay_mean), abs(az_mean)))
        ]
        gyro_pass = gz_drift_deg < GYRO_DRIFT_LIMIT_DEG_S
        accel_pass = accel_noise_max < ACCEL_NOISE_LIMIT_M_S2
        gravity_pass = gravity_error < GRAVITY_TOLERANCE_M_S2

        calib = {
            "calibration_date": dt.date.today().isoformat(),
            "calibration_operator": os.uname().nodename,
            "calibration_method": "ros2_static_test",
            "frame_id": "camera_imu_optical_frame",
            "publish_rate_hz": round(n / args.seconds, 3),
            "accelerometer": {
                "noise_density": None,
                "bias_instability": None,
                "scale_and_alignment": [[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]],
                "bias": [0.0, 0.0, 0.0],
                "static_test_std_m_s2": {"x": round(ax_std, 6), "y": round(ay_std, 6), "z": round(az_std, 6)},
                "static_test_mean_m_s2": {"x": round(ax_mean, 4), "y": round(ay_mean, 4), "z": round(az_mean, 4)},
            },
            "gyroscope": {
                "noise_density": None,
                "bias_instability": None,
                "scale_and_alignment": [[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]],
                "bias": [0.0, 0.0, 0.0],
                "static_test_std_rad_s": {"x": round(gx_std, 6), "y": round(gy_std, 6), "z": round(gz_std, 6)},
            },
            "imu_to_color_extrinsic": {
                "note": "Read from rs-enumerate-devices -c when exact per-camera extrinsics are needed.",
                "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                "translation": [0.0, 0.0, 0.0],
            },
            "static_test": {
                "seconds": args.seconds,
                "num_samples": n,
                "gyro_drift_deg_per_s": round(gz_drift_deg, 4),
                "accel_noise_m_s2": round(accel_noise_max, 4),
                "gravity_magnitude_m_s2": round(gravity_magnitude, 4),
                "gravity_dominant_axis": gravity_axis,
                "gravity_alignment_verified": gravity_pass,
                "gyro_pass": gyro_pass,
                "accel_pass": accel_pass,
                "note": "Accel noise includes motor vibration if the base controller was running.",
            },
        }

        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(calib, sort_keys=False))

        node.get_logger().info(f"Wrote {output}")
        node.get_logger().info(
            f"gyro_z_drift={gz_drift_deg:.4f} deg/s accel_noise={accel_noise_max:.4f} m/s^2 "
            f"gravity={gravity_magnitude:.3f} m/s^2 axis={gravity_axis}"
        )
        if not (gyro_pass and accel_pass and gravity_pass):
            node.get_logger().warning("Static IMU criteria did not all pass; inspect the YAML evidence.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
