#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
extract_realsense_calib.py
--------------------------
Reads factory camera intrinsics from live RealSense ROS topics and writes
them to agv_ws/src/agv_bringup/calibration/camera_intrinsics.yaml.

Run AFTER starting bringup.launch:
    roslaunch agv_bringup bringup.launch
    python scripts/calibration/extract_realsense_calib.py

The script waits for one message on each camera_info topic, extracts
K (camera matrix) and D (distortion), then writes the YAML.
"""

import os
import sys
import datetime

import rospy
from sensor_msgs.msg import CameraInfo
import yaml

CALIB_YAML = os.path.join(
    os.path.dirname(__file__),
    "../../agv_ws/src/agv_bringup/calibration/camera_intrinsics.yaml"
)

COLOR_TOPIC = "/camera/color/camera_info"
DEPTH_TOPIC = "/camera/depth/camera_info"

TIMEOUT_S = 10.0


def wait_for_camera_info(topic):
    rospy.loginfo("Waiting for %s ...", topic)
    try:
        msg = rospy.wait_for_message(topic, CameraInfo, timeout=TIMEOUT_S)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for %s. Is bringup.launch running?", topic)
        sys.exit(1)
    return msg


def camera_info_to_dict(msg):
    return {
        "image_width":  msg.width,
        "image_height": msg.height,
        "camera_matrix": {
            "rows": 3, "cols": 3,
            "data": list(msg.K)
        },
        "distortion_model": msg.distortion_model,
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(msg.D),
            "data": list(msg.D)
        },
        "rectification_matrix": {
            "rows": 3, "cols": 3,
            "data": list(msg.R)
        },
        "projection_matrix": {
            "rows": 3, "cols": 4,
            "data": list(msg.P)
        }
    }


def main():
    rospy.init_node("extract_realsense_calib", anonymous=True)

    color_info = wait_for_camera_info(COLOR_TOPIC)
    depth_info = wait_for_camera_info(DEPTH_TOPIC)

    today = datetime.date.today().isoformat()
    hostname = os.uname()[1]

    calib = {
        "calibration_date": today,
        "calibration_operator": hostname,
        "calibration_method": "factory",
        "reprojection_error_px": None,
        "color_camera": camera_info_to_dict(color_info),
        "depth_camera": camera_info_to_dict(depth_info),
        "color_to_depth_extrinsic": {
            "note": "Read from /camera/extrinsics/depth_to_color — populate manually.",
            "rotation":    [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "translation": [0.015, 0.0, 0.0]
        },
        "depth_accuracy": {
            "error_at_0_5m_mm": None,
            "error_at_1_0m_mm": None,
            "error_at_2_0m_mm": None,
            "note": "Fill in from physical depth accuracy test (see STAGE_1_CALIBRATION_SOP.md 1c)"
        }
    }

    out_path = os.path.realpath(CALIB_YAML)
    with open(out_path, "w") as f:
        yaml.dump(calib, f, default_flow_style=False)

    rospy.loginfo("Written to %s", out_path)
    rospy.loginfo("Color K: %s", color_info.K)
    rospy.loginfo("Depth K: %s", depth_info.K)
    rospy.loginfo("Color D: %s", color_info.D)

    # Quick sanity: fx should be > 300 for a real camera
    fx = color_info.K[0]
    if fx < 300:
        rospy.logwarn("fx=%.1f looks wrong — is the camera actually connected?", fx)
    else:
        rospy.loginfo("PASS: fx=%.1f looks plausible for D455 at %dx%d",
                      fx, color_info.width, color_info.height)


if __name__ == "__main__":
    main()
