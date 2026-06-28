#!/usr/bin/env python3
"""Capture RealSense camera intrinsics from live ROS2 CameraInfo topics."""

from __future__ import annotations

import argparse
import datetime as dt
import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = ROOT / "agv2_ws/src/agv_bringup/calibration/camera_intrinsics.yaml"


def camera_info_to_dict(msg: CameraInfo) -> dict:
    return {
        "image_width": msg.width,
        "image_height": msg.height,
        "camera_matrix": {"rows": 3, "cols": 3, "data": list(msg.k)},
        "distortion_model": msg.distortion_model,
        "distortion_coefficients": {"rows": 1, "cols": len(msg.d), "data": list(msg.d)},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": list(msg.r)},
        "projection_matrix": {"rows": 3, "cols": 4, "data": list(msg.p)},
    }


class CameraInfoCapture(Node):
    def __init__(self, color_topic: str, depth_topic: str) -> None:
        super().__init__("extract_realsense_calib")
        self.color_msg: Optional[CameraInfo] = None
        self.depth_msg: Optional[CameraInfo] = None
        self.create_subscription(CameraInfo, color_topic, self._on_color, 10)
        self.create_subscription(CameraInfo, depth_topic, self._on_depth, 10)

    def _on_color(self, msg: CameraInfo) -> None:
        self.color_msg = msg

    def _on_depth(self, msg: CameraInfo) -> None:
        self.depth_msg = msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--color-topic", default="/camera/color/camera_info")
    parser.add_argument("--depth-topic", default="/camera/depth/camera_info")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = CameraInfoCapture(args.color_topic, args.depth_topic)
    deadline = time.monotonic() + args.timeout

    node.get_logger().info(f"Waiting for {args.color_topic} and {args.depth_topic}")
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if node.color_msg is not None and node.depth_msg is not None:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.color_msg is None or node.depth_msg is None:
            node.get_logger().error("Timed out waiting for CameraInfo topics. Is ROS2 bringup running?")
            return 1

        calib = {
            "calibration_date": dt.date.today().isoformat(),
            "calibration_operator": os.uname().nodename,
            "calibration_method": "factory_ros2_camera_info",
            "reprojection_error_px": None,
            "color_camera": camera_info_to_dict(node.color_msg),
            "depth_camera": camera_info_to_dict(node.depth_msg),
            "depth_to_color_extrinsic": {
                "note": "Read from /camera/extrinsics/depth_to_color and populate if needed.",
                "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                "translation": [0.0, 0.0, 0.0],
            },
            "depth_accuracy": {
                "error_at_0_5m_mm": None,
                "error_at_1_0m_mm": None,
                "error_at_2_0m_mm": None,
            },
        }

        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(calib, sort_keys=False))
        fx = node.color_msg.k[0]
        node.get_logger().info(f"Wrote {output}")
        node.get_logger().info(f"Color fx={fx:.1f} at {node.color_msg.width}x{node.color_msg.height}")
        if fx < 300:
            node.get_logger().warning("fx looks low for a D455; confirm the expected camera profile.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
