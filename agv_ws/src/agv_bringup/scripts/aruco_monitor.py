#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Print live ArUco marker detections with optional annotated image output."""

from __future__ import print_function

import argparse
import math
import sys
import time

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image


DICTIONARIES = {
    "original": "DICT_ARUCO_ORIGINAL",
    "aruco_original": "DICT_ARUCO_ORIGINAL",
    "4x4_50": "DICT_4X4_50",
    "4x4_100": "DICT_4X4_100",
    "4x4_250": "DICT_4X4_250",
    "4x4_1000": "DICT_4X4_1000",
    "5x5_50": "DICT_5X5_50",
    "5x5_100": "DICT_5X5_100",
    "5x5_250": "DICT_5X5_250",
    "5x5_1000": "DICT_5X5_1000",
    "6x6_50": "DICT_6X6_50",
    "6x6_100": "DICT_6X6_100",
    "6x6_250": "DICT_6X6_250",
    "6x6_1000": "DICT_6X6_1000",
    "7x7_50": "DICT_7X7_50",
    "7x7_100": "DICT_7X7_100",
    "7x7_250": "DICT_7X7_250",
    "7x7_1000": "DICT_7X7_1000",
    "apriltag_16h5": "DICT_APRILTAG_16h5",
    "apriltag_25h9": "DICT_APRILTAG_25h9",
    "apriltag_36h10": "DICT_APRILTAG_36h10",
    "apriltag_36h11": "DICT_APRILTAG_36h11",
}


class ArucoMonitor(object):
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_print = 0.0
        self.frames = 0
        self.first_frame_time = None
        self.warned_multi_pose = False

        self.dictionary = self._make_dictionary(args.dictionary)
        self.parameters = self._make_parameters()
        self.detector = self._make_detector()

        self.image_pub = None
        if args.publish_image:
            self.image_pub = rospy.Publisher(args.output_image_topic, Image, queue_size=2)
        self.pose_pub = None
        if args.publish_pose:
            self.pose_pub = rospy.Publisher(args.pose_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(args.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1)
        rospy.Subscriber(args.image_topic, Image, self.image_cb, queue_size=1)

    def _make_dictionary(self, name):
        key = name.lower()
        dict_name = DICTIONARIES.get(key, name)
        if not hasattr(cv2.aruco, dict_name):
            raise RuntimeError("OpenCV aruco dictionary not available: %s" % dict_name)
        dict_id = getattr(cv2.aruco, dict_name)
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            return cv2.aruco.getPredefinedDictionary(dict_id)
        return cv2.aruco.Dictionary_get(dict_id)

    def _make_parameters(self):
        if hasattr(cv2.aruco, "DetectorParameters"):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return params

    def _make_detector(self):
        if hasattr(cv2.aruco, "ArucoDetector"):
            return cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        return None

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D, dtype=np.float64)

    def frame_rate(self, now):
        if self.first_frame_time is None:
            return 0.0
        elapsed = now - self.first_frame_time
        if elapsed <= 0.0:
            return 0.0
        return self.frames / elapsed

    def detect_markers(self, gray):
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
            return corners, ids, rejected
        return cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters,
        )

    def estimate_poses(self, corners):
        pose_available = (
            self.camera_matrix is not None and
            self.dist_coeffs is not None and
            self.args.marker_size > 0.0 and
            hasattr(cv2.aruco, "estimatePoseSingleMarkers")
        )
        if not pose_available:
            return None, None
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.args.marker_size,
            self.camera_matrix,
            self.dist_coeffs,
        )
        return rvecs, tvecs

    def image_cb(self, msg):
        now = time.time()
        if self.first_frame_time is None:
            self.first_frame_time = now
        self.frames += 1

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detect_markers(gray)
        corners, ids = self.filter_target(corners, ids)
        rvecs = None
        tvecs = None

        if ids is not None and len(ids) > 0:
            rvecs, tvecs = self.estimate_poses(corners)
            self.publish_pose(msg.header, ids, rvecs, tvecs)
            self.print_detections(ids, rvecs, tvecs, now)
        elif now - self.last_print >= self.args.print_period:
            hz = self.frame_rate(now)
            print("no ArUco marker visible; frames=%.1fHz rejected=%d" %
                  (hz, len(rejected)))
            sys.stdout.flush()
            self.last_print = now

        if self.image_pub is not None:
            annotated = frame.copy()
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

    def filter_target(self, corners, ids):
        if ids is None or len(ids) == 0 or self.args.target_id < 0:
            return corners, ids
        ids_flat = ids.flatten()
        keep = [idx for idx, marker_id in enumerate(ids_flat)
                if int(marker_id) == self.args.target_id]
        if not keep:
            return [], None
        filtered_corners = [corners[idx] for idx in keep]
        filtered_ids = np.array([[ids_flat[idx]] for idx in keep], dtype=ids.dtype)
        return filtered_corners, filtered_ids

    def publish_pose(self, header, ids, rvecs, tvecs):
        if self.pose_pub is None or rvecs is None or tvecs is None:
            return
        if len(ids) > 1 and self.args.target_id < 0 and not self.warned_multi_pose:
            rospy.logwarn("Multiple ArUco markers visible; publishing first pose only.")
            self.warned_multi_pose = True

        pose_msg = PoseStamped()
        pose_msg.header = header
        t = tvecs[0][0]
        pose_msg.pose.position.x = float(t[0])
        pose_msg.pose.position.y = float(t[1])
        pose_msg.pose.position.z = float(t[2])

        rotation_matrix = np.identity(4)
        rotation_matrix[:3, :3] = cv2.Rodrigues(rvecs[0][0])[0]
        q = tf.transformations.quaternion_from_matrix(rotation_matrix)
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])
        self.pose_pub.publish(pose_msg)

    def print_detections(self, ids, rvecs, tvecs, now):
        if now - self.last_print < self.args.print_period:
            return

        hz = self.frame_rate(now)
        ids_flat = ids.flatten()
        print("detections=%d frame_rate=%.1fHz dictionary=%s" %
              (len(ids_flat), hz, self.args.dictionary))

        for idx, marker_id in enumerate(ids_flat):
            if tvecs is not None:
                t = tvecs[idx][0]
                distance = math.sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2])
                print("  id=%d size=%.3fm x=%.3fm y=%.3fm z=%.3fm distance=%.3fm" %
                      (marker_id, self.args.marker_size,
                       t[0], t[1], t[2], distance))
            else:
                print("  id=%d" % marker_id)
        sys.stdout.flush()
        self.last_print = now


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Monitor ArUco detections")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--output-image-topic", default="/aruco_detections_image")
    parser.add_argument("--pose-topic", default="/aruco/target_pose")
    parser.add_argument("--dictionary", default="original",
                        help="OpenCV ArUco dictionary, e.g. original, 4x4_50")
    parser.add_argument("--marker-size", type=float, default=0.15,
                        help="Marker black-square side length in metres")
    parser.add_argument("--target-id", type=int, default=-1,
                        help="Only report this marker ID; negative means all IDs")
    parser.add_argument("--print-period", type=float, default=0.5)
    parser.add_argument("--publish-image", type=str_to_bool, default=False,
                        help="Publish annotated detection images")
    parser.add_argument("--publish-pose", type=str_to_bool, default=True,
                        help="Publish the target marker pose as geometry_msgs/PoseStamped")
    args, _ = parser.parse_known_args(argv)
    return args


def str_to_bool(value):
    if isinstance(value, bool):
        return value
    value = value.lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError("expected true/false")


def main(argv):
    args = parse_args(argv)
    rospy.init_node("aruco_monitor", anonymous=True)
    ArucoMonitor(args)
    print("Listening for ArUco markers on %s using %s, size %.3fm" %
          (args.image_topic, args.dictionary, args.marker_size))
    if args.target_id >= 0:
        print("Filtering detections to marker id %d" % args.target_id)
    if args.publish_pose:
        print("Publishing target pose on %s" % args.pose_topic)
    print("Camera optical frame axes: x=right, y=down, z=forward")
    sys.stdout.flush()
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv[1:])
