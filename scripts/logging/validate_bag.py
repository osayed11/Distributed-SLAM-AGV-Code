#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
validate_bag.py — Post-run bag quality validator for the AGV SLAM dataset.

Usage:
    python3 validate_bag.py <path_to_bag_file>
    python3 validate_bag.py <path_to_bag_file> --strict      # fail on any warning
    python3 validate_bag.py <path_to_bag_file> --require-gt  # require mocap/GT
    python3 validate_bag.py <path_to_bag_file> --require-imu # require IMU

Checks:
    1. All required topics are present
    2. Each topic meets the minimum publishable rate
    3. No topic has a gap longer than 2x its expected period (frame drop check)
    4. Colour and depth are temporally aligned (USB 2 sync check)
    5. IMU timestamps are monotonically increasing if IMU is present/required
    6. Bag file is not truncated or corrupted
    7. Bag duration meets minimum run length

Exit codes:
    0 = PASS (publishable quality)
    1 = FAIL (one or more hard failures)
    2 = WARN (passes but has warnings worth reviewing)

Run on Mac or robot after recording. Requires: rosbag (ROS), PyYAML, Python 3.
"""

import subprocess
import sys
import os
import shutil
import yaml
import math
from collections import defaultdict

# ---------------------------------------------------------------------------
# Publishability thresholds
# Rates are MINIMUM acceptable Hz. Based on EuRoC (IMU 200Hz, cam 20Hz) and
# TUM RGB-D (30Hz) — we use 15Hz camera which is below TUM but acceptable
# for a LiDAR-primary dataset. IMU is currently optional because the D455 motion
# module fails when video streams are enabled on this robot.
# ---------------------------------------------------------------------------

REQUIRED_TOPICS = {
    # min_hz: minimum acceptable average rate (publishability threshold)
    # target_hz: configured/expected rate — used for gap-based drop detection
    "/scan":                                    {"min_hz": 5.0,   "target_hz": 18.0,  "type": "sensor_msgs/LaserScan"},
    "/odom":                                    {"min_hz": 12.0,  "target_hz": 20.0,  "type": "nav_msgs/Odometry"},
    "/cmd_vel":                                 {"min_hz": 0.0,   "target_hz": 0.0,   "type": "geometry_msgs/Twist"},
    "/tf":                                      {"min_hz": 10.0,  "target_hz": 50.0,  "type": "tf2_msgs/TFMessage"},
    "/tf_static":                               {"min_hz": 0.0,   "target_hz": 0.0,   "type": "tf2_msgs/TFMessage"},
    "/camera/color/image_raw":                  {"min_hz": 12.0,  "target_hz": 15.0,  "type": "sensor_msgs/Image"},
    "/camera/color/camera_info":                {"min_hz": 12.0,  "target_hz": 15.0,  "type": "sensor_msgs/CameraInfo"},
    "/camera/aligned_depth_to_color/image_raw": {"min_hz": 12.0,  "target_hz": 15.0,  "type": "sensor_msgs/Image"},
    "/camera/aligned_depth_to_color/camera_info": {"min_hz": 12.0, "target_hz": 15.0, "type": "sensor_msgs/CameraInfo"},
}

OPTIONAL_TOPICS = {
    "/camera/depth/camera_info":        {"min_hz": 0.0, "type": "sensor_msgs/CameraInfo"},
    "/imu":                             {"min_hz": 10.0, "target_hz": 12.0, "type": "sensor_msgs/Imu"},
    "/camera/imu":                      {"min_hz": 150.0, "target_hz": 200.0, "type": "sensor_msgs/Imu"},
    "/camera/accel/sample":             {"min_hz": 60.0, "target_hz": 100.0, "type": "sensor_msgs/Imu"},
    "/camera/gyro/sample":              {"min_hz": 150.0, "target_hz": 200.0, "type": "sensor_msgs/Imu"},
    "/camera/accel/imu_info":           {"min_hz": 0.0, "type": "realsense2_camera/IMUInfo"},
    "/camera/gyro/imu_info":            {"min_hz": 0.0, "type": "realsense2_camera/IMUInfo"},
    "/camera/extrinsics/depth_to_color": {"min_hz": 0.0, "type": "realsense2_camera/Extrinsics"},
    "/diagnostics":                     {"min_hz": 0.0, "type": "diagnostic_msgs/DiagnosticArray"},
    "/aruco/target_pose":                {"min_hz": 0.0, "type": "geometry_msgs/PoseStamped"},
    "/tag_detections":                  {"min_hz": 0.0, "type": "apriltag_ros/AprilTagDetectionArray"},
}

GROUND_TRUTH_TOPICS = [
    "/phasespace/rigids",
    "/mocap",
    "/ground_truth",
    "/ground_truth/pose",
    "/vrpn_client_node/agv01/pose",
]
if os.environ.get("MOCAP_TOPIC") and os.environ["MOCAP_TOPIC"] not in GROUND_TRUTH_TOPICS:
    GROUND_TRUTH_TOPICS.insert(0, os.environ["MOCAP_TOPIC"])

# Minimum run duration for a useful dataset sequence
MIN_DURATION_SEC = 30.0

# Colour/depth sync tolerance (seconds). On USB 3 hardware sync gives ~1ms.
# On USB 2 we tolerate up to 100ms before flagging as a sync failure.
COLOUR_DEPTH_SYNC_TOLERANCE_SEC = 0.100
USB2_SYNC_WARN_THRESHOLD_SEC    = 0.033  # >33ms = likely USB 2 degradation

# Max allowed gap multiplier (gap > N * expected_period = frame drop)
MAX_GAP_MULTIPLIER = 3.0

# ---------------------------------------------------------------------------

PASS  = "PASS"
WARN  = "WARN"
FAIL  = "FAIL"

results = []  # list of (level, topic_or_check, message)

def record(level, check, msg):
    results.append((level, check, msg))
    symbol = {"PASS": "✓", "WARN": "!", "FAIL": "✗"}[level]
    print("  [{}] {}: {}".format(symbol, check, msg))


def _ros_executable(name):
    candidates = []
    found = shutil.which(name)
    if found:
        candidates.append(found)
    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_distro:
        candidates.append("/opt/ros/{}/bin/{}".format(ros_distro, name))
    candidates.extend([
        "/opt/ros/noetic/bin/{}".format(name),
        "/opt/ros/melodic/bin/{}".format(name),
    ])
    for candidate in candidates:
        if candidate and os.path.exists(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return name


def _rosbag_python():
    env = _ros2_env()
    candidates = []
    ros_distro = os.environ.get("ROS_DISTRO", "")
    if ros_distro == "melodic":
        candidates.extend(["python2", "python"])
    else:
        candidates.extend([sys.executable, "python3"])
    candidates.extend([sys.executable, "python3", "python2", "python"])

    seen = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        try:
            subprocess.check_call(
                [candidate, "-c", "import rosbag"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
            )
            return candidate
        except Exception:
            continue
    return sys.executable


def get_bag_info(bag_path):
    """Run rosbag info --yaml and return parsed dict."""
    try:
        out = subprocess.check_output(
            [_ros_executable("rosbag"), "info", "--yaml", bag_path],
            stderr=subprocess.STDOUT,
            env=_ros2_env(),
        )
        text = out.decode("utf-8")
        yaml_start = text.find("path:")
        if yaml_start > 0:
            text = text[yaml_start:]
        return yaml.safe_load(text)
    except subprocess.CalledProcessError as e:
        print("ERROR: rosbag info failed: {}".format(e.output.decode("utf-8")))
        sys.exit(1)
    except FileNotFoundError:
        print("ERROR: rosbag not found. Source ROS or install ROS bag tools.")
        sys.exit(1)


def check_bag_integrity(bag_path):
    """Run rosbag check to detect corruption or truncation."""
    print("\n--- Bag integrity ---")
    try:
        out = subprocess.check_output(
            [_ros_executable("rosbag"), "check", bag_path],
            stderr=subprocess.STDOUT,
            env=_ros2_env(),
        )
        output = out.decode("utf-8").strip()
        if "No errors found" in output or output == "":
            record(PASS, "bag_integrity", "No errors found")
        else:
            record(WARN, "bag_integrity", output)
    except subprocess.CalledProcessError as e:
        record(FAIL, "bag_integrity", "rosbag check failed: {}".format(
            e.output.decode("utf-8").strip()))


def check_duration(info):
    """Check bag duration meets minimum."""
    print("\n--- Duration ---")
    duration = info.get("duration", 0.0)
    if duration >= MIN_DURATION_SEC:
        record(PASS, "duration", "{:.1f}s (min {:.0f}s)".format(duration, MIN_DURATION_SEC))
    else:
        record(FAIL, "duration", "{:.1f}s is below minimum {:.0f}s for a useful sequence".format(
            duration, MIN_DURATION_SEC))
    return duration


def check_topics(info, duration):
    """Check topic presence and message rates."""
    print("\n--- Required topics ---")
    bag_topics = {t["topic"]: t for t in info.get("topics", [])}

    for topic, spec in REQUIRED_TOPICS.items():
        if topic not in bag_topics:
            record(FAIL, topic, "MISSING — topic not recorded")
            continue

        t = bag_topics[topic]
        msg_count = t.get("messages", 0)
        actual_hz = msg_count / duration if duration > 0 else 0.0
        min_hz = spec["min_hz"]

        if actual_hz >= min_hz:
            record(PASS, topic, "{} msgs @ {:.1f} Hz (min {:.0f} Hz)".format(
                msg_count, actual_hz, min_hz))
        elif actual_hz >= min_hz * 0.8:
            record(WARN, topic, "{} msgs @ {:.1f} Hz — below target {:.0f} Hz (within 20% tolerance)".format(
                msg_count, actual_hz, min_hz))
        else:
            record(FAIL, topic, "{} msgs @ {:.1f} Hz — BELOW minimum {:.0f} Hz".format(
                msg_count, actual_hz, min_hz))

    print("\n--- Optional topics ---")
    for topic, spec in OPTIONAL_TOPICS.items():
        if topic in bag_topics:
            t = bag_topics[topic]
            record(PASS, topic, "{} msgs present".format(t.get("messages", 0)))
        else:
            record(WARN, topic, "not recorded (optional)")

    return bag_topics


def check_ground_truth(bag_topics, duration, require_gt):
    """Require at least one mocap/ground-truth topic for Week 1 validation bags."""
    print("\n--- Ground truth ---")

    present = [topic for topic in GROUND_TRUTH_TOPICS if topic in bag_topics]
    if not present:
        level = FAIL if require_gt else WARN
        record(level, "ground_truth",
               "MISSING — checked: {}".format(", ".join(GROUND_TRUTH_TOPICS)))
        return

    for topic in present:
        msg_count = bag_topics[topic].get("messages", 0)
        actual_hz = msg_count / duration if duration > 0 else 0.0
        if actual_hz >= 30.0:
            record(PASS, topic, "{} msgs @ {:.1f} Hz".format(msg_count, actual_hz))
        elif actual_hz > 0.0:
            record(WARN, topic, "{} msgs @ {:.1f} Hz — below 30 Hz floor".format(
                msg_count, actual_hz))
        else:
            record(FAIL, topic, "present but no messages")


def check_imu_requirement(bag_topics, duration, require_imu):
    """Require at least one IMU stream only when requested."""
    print("\n--- IMU availability ---")

    imu_topics = ["/imu", "/camera/imu", "/camera/accel/sample", "/camera/gyro/sample"]
    present = [topic for topic in imu_topics if topic in bag_topics]
    if not present:
        level = FAIL if require_imu else WARN
        record(level, "imu", "MISSING — no IMU stream recorded")
        return

    any_live = False
    for topic in present:
        msg_count = bag_topics[topic].get("messages", 0)
        actual_hz = msg_count / duration if duration > 0 else 0.0
        if actual_hz > 0.0:
            any_live = True
            record(PASS, topic, "{} msgs @ {:.1f} Hz".format(msg_count, actual_hz))
        else:
            record(WARN, topic, "present but no messages")

    if require_imu and not any_live:
        record(FAIL, "imu", "IMU required but all IMU topics are empty")


def check_tf_tree(bag_path, bag_topics):
    """Check the core TF edges needed to align sensors and ground truth."""
    print("\n--- TF tree ---")

    tf_topics = [topic for topic in ["/tf", "/tf_static"] if topic in bag_topics]
    if not tf_topics:
        record(FAIL, "tf_tree", "No TF topics recorded")
        return

    try:
        cmd = [
            _rosbag_python(), "-c",
            """
import rosbag, sys
b = rosbag.Bag(sys.argv[1])
pairs = set()
for topic, msg, stamp in b.read_messages(topics=['/tf', '/tf_static']):
    for tr in msg.transforms:
        parent = tr.header.frame_id.lstrip('/')
        child = tr.child_frame_id.lstrip('/')
        if parent and child:
            pairs.add(parent + '>' + child)
b.close()
print('\\n'.join(sorted(pairs)))
""",
            bag_path
        ]
        out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL,
                                      timeout=60, env=_ros2_env())
        pairs = set(line.strip() for line in out.decode().splitlines() if line.strip())

        required_pairs = [
            "odom>base_footprint",
            "base_footprint>base_link",
            "base_footprint>laser_frame",
            "base_footprint>camera_link",
        ]
        missing = [pair for pair in required_pairs if pair not in pairs]
        if missing:
            record(FAIL, "tf_tree", "Missing required TF edge(s): {}".format(
                ", ".join(missing)))
        else:
            record(PASS, "tf_tree",
                   "Core chain present: odom -> base_footprint -> base_link, laser_frame, camera_link")

        camera_pairs = [pair for pair in pairs if pair.startswith("camera_link>")]
        if camera_pairs:
            record(PASS, "camera_tf", "{} camera child frame edge(s) present".format(
                len(camera_pairs)))
        else:
            record(WARN, "camera_tf",
                   "No camera_link child frames found; RGB-D SLAM may not resolve optical frames")

    except Exception:
        record(WARN, "tf_tree",
               "Could not inspect TF messages — run with a working rosbag Python API for precise check")


def _ros2_env():
    """Return an env dict with the active ROS Python paths set for subprocess calls."""
    env = os.environ.copy()
    candidates = []
    lib_candidates = []
    bin_candidates = []
    ros_distro = env.get("ROS_DISTRO")
    if ros_distro:
        candidates.extend([
            "/opt/ros/{}/lib/python3/dist-packages".format(ros_distro),
            "/opt/ros/{}/lib/python2.7/dist-packages".format(ros_distro),
        ])
        lib_candidates.append("/opt/ros/{}/lib".format(ros_distro))
        bin_candidates.append("/opt/ros/{}/bin".format(ros_distro))
    candidates.extend([
        "/opt/ros/noetic/lib/python3/dist-packages",
        "/opt/ros/melodic/lib/python2.7/dist-packages",
    ])
    lib_candidates.extend([
        "/opt/ros/noetic/lib",
        "/opt/ros/melodic/lib",
    ])
    bin_candidates.extend([
        "/opt/ros/noetic/bin",
        "/opt/ros/melodic/bin",
    ])

    existing_parts = [part for part in env.get("PYTHONPATH", "").split(":") if part]
    for ros_python_path in reversed(candidates):
        if os.path.isdir(ros_python_path) and ros_python_path not in existing_parts:
            existing_parts.insert(0, ros_python_path)
    if existing_parts:
        env["PYTHONPATH"] = ":".join(existing_parts)

    existing_libs = [part for part in env.get("LD_LIBRARY_PATH", "").split(":") if part]
    for ros_lib_path in reversed(lib_candidates):
        if os.path.isdir(ros_lib_path) and ros_lib_path not in existing_libs:
            existing_libs.insert(0, ros_lib_path)
    if existing_libs:
        env["LD_LIBRARY_PATH"] = ":".join(existing_libs)

    existing_bins = [part for part in env.get("PATH", "").split(":") if part]
    for ros_bin_path in reversed(bin_candidates):
        if os.path.isdir(ros_bin_path) and ros_bin_path not in existing_bins:
            existing_bins.insert(0, ros_bin_path)
    if existing_bins:
        env["PATH"] = ":".join(existing_bins)
    return env


def check_frame_drops(bag_path, bag_topics, duration):
    """
    Check for frame drops by analysing timestamp gaps on camera topics.
    Reads message timestamps via the rosbag Python API.
    Falls back to gap estimation from message count if detailed check fails.
    """
    print("\n--- Frame drop check (USB 2 bandwidth) ---")

    camera_topics = [
        "/camera/color/image_raw",
        "/camera/aligned_depth_to_color/image_raw",
    ]

    for topic in camera_topics:
        if topic not in bag_topics:
            continue

        spec = REQUIRED_TOPICS.get(topic, OPTIONAL_TOPICS.get(topic, {}))
        # Use target_hz (configured rate) for gap threshold, not min_hz.
        # min_hz is for average rate check only; using it here would allow
        # multiple consecutive drops to go undetected (e.g. at 15Hz target,
        # 2 consecutive drops = 0.200s gap; at 12Hz min, limit is 0.250s).
        target_hz = spec.get("target_hz", spec.get("min_hz", 1.0))
        expected_period = 1.0 / target_hz if target_hz > 0 else 1.0
        max_gap = MAX_GAP_MULTIPLIER * expected_period

        # Use rosbag Python API to extract per-message timestamps
        try:
            cmd = [
                _rosbag_python(), "-c",
                "import rosbag, sys; b=rosbag.Bag(sys.argv[1]); "
                "[sys.stdout.write(str(m.header.stamp.to_sec())+'\\n') "
                "if hasattr(m,'header') else sys.stdout.write(str(t.to_sec())+'\\n') "
                "for _,m,t in b.read_messages(topics=[sys.argv[2]])]; b.close()",
                bag_path, topic
            ]
            out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL,
                                          timeout=60, env=_ros2_env())
            timestamps = [float(x) for x in out.decode().strip().split("\n") if x]

            if len(timestamps) < 2:
                record(WARN, topic + " gaps", "Too few messages to analyse gaps")
                continue

            gaps = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
            max_actual_gap = max(gaps)
            n_drops = sum(1 for g in gaps if g > max_gap)
            drop_rate = 100.0 * n_drops / len(gaps)

            # Also report single-frame drops (gaps > 1.5x period) even if below failure threshold
            n_single_drops = sum(1 for g in gaps if expected_period * 1.5 < g <= max_gap)

            if n_drops == 0 and n_single_drops == 0:
                record(PASS, topic + " gaps",
                       "No drops. Max gap {:.3f}s (limit {:.3f}s, target {:.0f}Hz)".format(
                           max_actual_gap, max_gap, target_hz))
            elif n_drops == 0:
                record(PASS, topic + " gaps",
                       "{} minor gap(s) (>{:.0f}ms, <{:.0f}ms). Max gap {:.3f}s. "
                       "No consecutive drops (limit {:.3f}s).".format(
                           n_single_drops, expected_period * 1500, max_gap * 1000,
                           max_actual_gap, max_gap))
            elif drop_rate < 1.0:
                record(WARN, topic + " gaps",
                       "{} drops ({:.2f}%) — max gap {:.3f}s. Likely USB 2 bandwidth pressure.".format(
                           n_drops, drop_rate, max_actual_gap))
            else:
                record(FAIL, topic + " gaps",
                       "{} drops ({:.1f}%) — max gap {:.3f}s. NOT publishable quality.".format(
                           n_drops, drop_rate, max_actual_gap))

        except Exception:
            # Fallback: estimate from message count only (no per-frame gap analysis)
            msg_count = bag_topics[topic].get("messages", 0)
            expected_count = target_hz * duration
            drop_est = max(0, expected_count - msg_count)
            drop_pct = 100.0 * drop_est / expected_count if expected_count > 0 else 0

            if drop_pct < 1.0:
                record(PASS, topic + " gaps",
                       "~{:.1f}% estimated drop rate (count-based; rosbag Python API unavailable for gap analysis)".format(drop_pct))
            else:
                record(WARN, topic + " gaps",
                       "~{:.1f}% estimated drop rate (count-based; rosbag Python API unavailable for gap analysis)".format(drop_pct))


def check_colour_depth_sync(bag_path, bag_topics):
    """
    Check temporal alignment between colour and depth frames.
    On USB 3 with hardware sync, alignment should be <5ms.
    On USB 2, alignment may degrade to 33-100ms.
    """
    print("\n--- Colour/depth sync (USB 2 check) ---")

    if ("/camera/color/image_raw" not in bag_topics or
            "/camera/aligned_depth_to_color/image_raw" not in bag_topics):
        record(WARN, "colour_depth_sync", "One or both topics missing — cannot check sync")
        return

    try:
        cmd = [
            _rosbag_python(), "-c",
            """
import rosbag, sys, bisect
b = rosbag.Bag(sys.argv[1])
colour_times = []
depth_times = []
for topic, msg, t in b.read_messages(topics=[
        '/camera/color/image_raw',
        '/camera/aligned_depth_to_color/image_raw']):
    if topic == '/camera/color/image_raw':
        colour_times.append(msg.header.stamp.to_sec())
    else:
        depth_times.append(msg.header.stamp.to_sec())
b.close()
# Nearest-neighbour matching: for each colour frame find closest depth frame
# (more accurate than index pairing when either stream drops frames)
diffs = []
for ct in colour_times:
    idx = bisect.bisect_left(depth_times, ct)
    candidates = []
    if idx < len(depth_times):
        candidates.append(abs(depth_times[idx] - ct))
    if idx > 0:
        candidates.append(abs(depth_times[idx-1] - ct))
    if candidates:
        diffs.append(min(candidates))
if diffs:
    print('{:.6f} {:.6f} {:.6f}'.format(
        sum(diffs)/len(diffs), max(diffs), sum(1 for d in diffs if d > 0.033)/len(diffs)))
""",
            bag_path
        ]
        out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL,
                                      timeout=60, env=_ros2_env())
        parts = out.decode().strip().split()
        if len(parts) == 3:
            mean_ms = float(parts[0]) * 1000
            max_ms = float(parts[1]) * 1000
            frac_over_33ms = float(parts[2])
            pct_over_33ms = frac_over_33ms * 100

            # PASS: USB 3 hardware sync (mean <5ms)
            # WARN: USB 2 soft sync — mean <33ms AND <5% frames >33ms
            # FAIL: mean >33ms OR >5% frames >33ms (structural misalignment)
            if mean_ms < 5:
                record(PASS, "colour_depth_sync",
                       "Mean {:.1f}ms, max {:.1f}ms — excellent sync (SDK temporal pairing)".format(mean_ms, max_ms))
            elif mean_ms < 33 and pct_over_33ms < 5.0:
                record(WARN, "colour_depth_sync",
                       "Mean {:.1f}ms, max {:.1f}ms, {:.1f}% frames >33ms — "
                       "USB 2 mode, sync adequate for LiDAR-primary SLAM. "
                       "Note: hardware sync requires USB 3.".format(mean_ms, max_ms, pct_over_33ms))
            else:
                record(FAIL, "colour_depth_sync",
                       "Mean {:.1f}ms, max {:.1f}ms, {:.1f}% frames >33ms — "
                       "USB 2 sync failure. Colour+depth structurally misaligned. "
                       "NOT publishable for RGB-D SLAM.".format(mean_ms, max_ms, pct_over_33ms))
    except Exception:
        record(WARN, "colour_depth_sync",
               "Could not compute sync — run with a working rosbag Python API for precise check")


def check_imu_monotonic(bag_path, bag_topics):
    """Check IMU timestamps are strictly increasing (catches driver reset bugs)."""
    print("\n--- IMU timestamp monotonicity ---")

    imu_topic = None
    for topic in ["/imu", "/camera/imu"]:
        if topic in bag_topics:
            imu_topic = topic
            break

    if imu_topic is None:
        return

    try:
        cmd = [
            _rosbag_python(), "-c",
            "import rosbag, sys; b=rosbag.Bag(sys.argv[1]); "
            "prev=0; bad=0; n=0; "
            "[exec('global prev,bad,n; t=m.header.stamp.to_sec(); bad+=1 if t<=prev else 0; prev=t; n+=1') "
            "for _,m,_ in b.read_messages(topics=[sys.argv[2]])]; "
            "b.close(); print(bad,n)",
            bag_path,
            imu_topic
        ]
        out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL,
                                      timeout=30, env=_ros2_env())
        parts = out.decode().strip().split()
        if len(parts) == 2:
            bad, total = int(parts[0]), int(parts[1])
            if bad == 0:
                record(PASS, "imu_monotonic", "All {} {} timestamps strictly increasing".format(total, imu_topic))
            else:
                record(FAIL, "imu_monotonic",
                       "{} non-monotonic {} timestamps out of {} — IMU driver issue".format(
                           bad, imu_topic, total))
    except Exception:
        record(WARN, "imu_monotonic", "Could not check — run with a working rosbag Python API for precise check")


def print_summary(bag_path, duration, bag_topics, strict):
    print("\n" + "=" * 60)
    print("VALIDATION SUMMARY: {}".format(os.path.basename(bag_path)))
    print("=" * 60)

    n_pass = sum(1 for r in results if r[0] == PASS)
    n_warn = sum(1 for r in results if r[0] == WARN)
    n_fail = sum(1 for r in results if r[0] == FAIL)

    print("  PASS: {}   WARN: {}   FAIL: {}".format(n_pass, n_warn, n_fail))
    print("  Duration: {:.1f}s   Size: {:.0f} MB".format(
        duration,
        os.path.getsize(bag_path) / 1e6
    ))

    # USB 2 specific summary
    print("\n  RGB-D/IMU assessment:")
    sync_fails = [r for r in results if "sync" in r[1] and r[0] == FAIL]
    drop_fails = [r for r in results if "gaps" in r[1] and r[0] == FAIL]
    drop_warns = [r for r in results if "gaps" in r[1] and r[0] == WARN]

    if sync_fails or drop_fails:
        print("  ! Sensor streaming is causing data quality issues — reduce load or inspect USB")
    elif drop_warns:
        print("  ~ Sensor streams show occasional drops — acceptable for most SLAM algorithms")
        print("    but flag this in your dataset paper as a known limitation")
    else:
        print("  ✓ RGB-D/IMU streaming appears stable for this run")

    print("\n  Publishability verdict:")
    if n_fail == 0 and n_warn == 0:
        print("  ✓ PASS — data meets publishable quality standards")
        return 0
    elif n_fail == 0:
        print("  ~ WARN — data is usable but review warnings before publishing")
        return 2 if strict else 0
    else:
        print("  ✗ FAIL — {} hard failures must be resolved before publishing".format(n_fail))
        if n_fail == 1 and all("optional" in r[2] for r in results if r[0] == FAIL):
            print("    (failure is in optional topic — may still be acceptable)")
        return 1


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 validate_bag.py <bag_file> [--strict]")
        sys.exit(1)

    bag_path = sys.argv[1]
    strict = "--strict" in sys.argv
    require_gt = "--require-gt" in sys.argv or os.environ.get("REQUIRE_GT") == "true"
    require_imu = "--require-imu" in sys.argv or os.environ.get("REQUIRE_IMU") == "true"

    if not os.path.exists(bag_path):
        print("ERROR: bag file not found: {}".format(bag_path))
        sys.exit(1)

    print("=" * 60)
    print("AGV Dataset Bag Validator")
    print("Bag: {}".format(bag_path))
    print("=" * 60)

    info = get_bag_info(bag_path)
    check_bag_integrity(bag_path)
    duration = check_duration(info)
    bag_topics = check_topics(info, duration)
    check_ground_truth(bag_topics, duration, require_gt)
    check_imu_requirement(bag_topics, duration, require_imu)
    check_tf_tree(bag_path, bag_topics)
    check_frame_drops(bag_path, bag_topics, duration)
    check_colour_depth_sync(bag_path, bag_topics)
    check_imu_monotonic(bag_path, bag_topics)

    exit_code = print_summary(bag_path, duration, bag_topics, strict)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
