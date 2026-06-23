#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
validate_bag.py — Post-run bag quality validator for the AGV SLAM dataset.

Supports both ROS1 (.bag file) and ROS2 (directory with metadata.yaml + .db3).
Format is auto-detected from the path.

Usage:
    python3 validate_bag.py <path_to_bag>
    python3 validate_bag.py <path_to_bag> --strict      # fail on any warning
    python3 validate_bag.py <path_to_bag> --require-gt  # require mocap/GT
    python3 validate_bag.py <path_to_bag> --require-imu # require IMU

Exit codes:
    0 = PASS   1 = FAIL   2 = WARN
"""

import subprocess
import sys
import os
import glob
import shutil
import sqlite3
import struct
import yaml
import math
import re
from collections import defaultdict

# ---------------------------------------------------------------------------
# Publishability thresholds
# ---------------------------------------------------------------------------
CMD_TOPIC = os.environ.get("CMD_TOPIC", "/cmd_vel")

REQUIRED_TOPICS = {
    "/scan":                                    {"min_hz": 5.0,   "target_hz": 18.0},
    "/odom":                                    {"min_hz": 10.0,  "target_hz": 20.0},
    CMD_TOPIC:                                  {"min_hz": 0.0,   "target_hz": 0.0},
    "/tf":                                      {"min_hz": 10.0,  "target_hz": 50.0},
    "/tf_static":                               {"min_hz": 0.0,   "target_hz": 0.0},
    "/camera/color/image_raw":                  {"min_hz": 12.0,  "target_hz": 15.0},
    "/camera/color/camera_info":                {"min_hz": 12.0,  "target_hz": 15.0},
    "/camera/aligned_depth_to_color/image_raw": {"min_hz": 12.0,  "target_hz": 15.0},
    "/camera/aligned_depth_to_color/camera_info": {"min_hz": 12.0, "target_hz": 15.0},
}

OPTIONAL_TOPICS = {
    "/camera/depth/camera_info":        {"min_hz": 0.0},
    "/camera/imu":                      {"min_hz": 150.0, "target_hz": 200.0},
    "/diagnostics":                     {"min_hz": 0.0},
    "/tag_detections":                  {"min_hz": 0.0},
}

GROUND_TRUTH_TOPICS = [
    "/optitrack/rigid_bodies/orkar_agv1",
    "/optitrack/rigid_bodies/orkar_agv2",
    "/optitrack/rigid_bodies/orkar_agv3",
    "/optitrack/rigid_bodies/orkar_agv4",
    "/gt/agv1/pose",
    "/gt/agv2/pose",
    "/gt/agv3/pose",
    "/gt/agv4/pose",
    "/phasespace/rigids",
    "/mocap",
    "/ground_truth",
    "/ground_truth/pose",
    "/vrpn_client_node/agv01/pose",
]
if os.environ.get("MOCAP_TOPIC") and os.environ["MOCAP_TOPIC"] not in GROUND_TRUTH_TOPICS:
    GROUND_TRUTH_TOPICS.insert(0, os.environ["MOCAP_TOPIC"])

MIN_DURATION_SEC = 30.0
COLOUR_DEPTH_SYNC_TOLERANCE_SEC = 0.100
USB2_SYNC_WARN_THRESHOLD_SEC = 0.033
MAX_GAP_MULTIPLIER = 3.0

PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"

results = []


def record(level, check, msg):
    results.append((level, check, msg))
    symbol = {"PASS": "✓", "WARN": "!", "FAIL": "✗"}[level]
    print("  [{}] {}: {}".format(symbol, check, msg))


# ---------------------------------------------------------------------------
# Format detection
# ---------------------------------------------------------------------------

def _is_ros2_bag(path):
    if not os.path.isdir(path):
        return False
    if os.path.exists(os.path.join(path, "metadata.yaml")):
        return True
    # Also accept bags where metadata.yaml is missing (e.g. killed mid-record)
    # as long as there are .db3 files present.
    return len(glob.glob(os.path.join(path, "*.db3"))) > 0


def _is_ros1_bag(path):
    return os.path.isfile(path) and path.endswith(".bag")


# ---------------------------------------------------------------------------
# ROS2: read metadata.yaml and SQLite3 helpers
# ---------------------------------------------------------------------------

def _get_ros2_bag_info(bag_path):
    meta_path = os.path.join(bag_path, "metadata.yaml")
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = yaml.safe_load(f)
        info = meta.get("rosbag2_bagfile_information", {})
        duration_ns = info.get("duration", {}).get("nanoseconds", 0)
        topics_raw = info.get("topics_with_message_count", [])
        topics = []
        for t in topics_raw:
            tm = t.get("topic_metadata", {})
            ros2_type = tm.get("type", "")
            display_type = ros2_type.replace("/msg/", "/") if "/msg/" in ros2_type else ros2_type
            topics.append({
                "topic": tm.get("name", ""),
                "type": display_type,
                "messages": t.get("message_count", 0),
            })
        return {"duration": duration_ns / 1e9, "topics": topics}

    # metadata.yaml missing (bag killed mid-record) — derive directly from db3 files
    counts = {}
    topic_types = {}
    min_ts = max_ts = None
    for db_file in _ros2_db_files(bag_path):
        try:
            conn = sqlite3.connect(db_file)
            id_to_name = {}
            for tid, name, typ in conn.execute("SELECT id, name, type FROM topics"):
                id_to_name[tid] = name
                topic_types[name] = typ.replace("/msg/", "/") if "/msg/" in typ else typ
                counts.setdefault(name, 0)
            for tid, ts in conn.execute("SELECT topic_id, timestamp FROM messages"):
                name = id_to_name.get(tid)
                if name:
                    counts[name] = counts.get(name, 0) + 1
                if min_ts is None or ts < min_ts:
                    min_ts = ts
                if max_ts is None or ts > max_ts:
                    max_ts = ts
            conn.close()
        except Exception:
            pass
    duration = (max_ts - min_ts) / 1e9 if min_ts and max_ts else 0.0
    topics = [{"topic": n, "type": topic_types.get(n, ""), "messages": c}
              for n, c in counts.items()]
    return {"duration": duration, "topics": topics}


def _ros2_db_files(bag_path):
    import re
    files = glob.glob(os.path.join(bag_path, "*.db3"))
    def _num(f):
        m = re.search(r"(\d+)\.db3$", os.path.basename(f))
        return int(m.group(1)) if m else 0
    return sorted(files, key=_num)


def _ros2_get_timestamps(bag_path, topic_name):
    """Return sorted list of message receive timestamps (seconds) for a topic."""
    timestamps = []
    for db_file in _ros2_db_files(bag_path):
        try:
            conn = sqlite3.connect(db_file)
            row = conn.execute(
                "SELECT id FROM topics WHERE name=?", (topic_name,)
            ).fetchone()
            if row:
                rows = conn.execute(
                    "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
                    (row[0],)
                ).fetchall()
                timestamps.extend(r[0] / 1e9 for r in rows)
            conn.close()
        except Exception:
            pass
    return sorted(timestamps)


def _ros2_get_tf_edges(bag_path):
    """
    Extract TF parent→child pairs from a ROS2 bag.
    Uses minimal CDR parsing: TFMessage starts with 4-byte CDR header, then
    uint32 array length, then each TransformStamped begins with Header
    (int32 sec, uint32 nanosec, string frame_id) followed by string child_frame_id.
    We only read frame_id and child_frame_id so we can skip the numeric fields.
    Falls back to rosbag2_py if direct parsing fails.
    """
    edges = set()
    try:
        edges = _ros2_tf_edges_cdr(bag_path)
        if edges:
            return edges
    except Exception:
        pass
    try:
        edges = _ros2_tf_edges_py(bag_path)
    except Exception:
        pass
    return edges


def _cdr_read_string(data, offset, little_endian):
    fmt = "<I" if little_endian else ">I"
    if offset + 4 > len(data):
        raise ValueError("buffer too short for string length")
    length = struct.unpack_from(fmt, data, offset)[0]
    offset += 4
    if length == 0:
        return "", offset
    s = data[offset:offset + length - 1].decode("utf-8", errors="replace")
    offset += length
    # CDR strings are padded to 4-byte boundary
    pad = (4 - (length % 4)) % 4
    offset += pad
    return s, offset


def _ros2_tf_edges_cdr(bag_path):
    edges = set()
    for db_file in _ros2_db_files(bag_path):
        conn = sqlite3.connect(db_file)
        try:
            for tf_topic in ("/tf", "/tf_static"):
                row = conn.execute(
                    "SELECT id FROM topics WHERE name=?", (tf_topic,)
                ).fetchone()
                if not row:
                    continue
                msgs = conn.execute(
                    "SELECT data FROM messages WHERE topic_id=? LIMIT 500",
                    (row[0],)
                ).fetchall()
                for (data,) in msgs:
                    try:
                        le = (data[1] == 1)
                        fmt = "<I" if le else ">I"
                        n_transforms = struct.unpack_from(fmt, data, 4)[0]
                        offset = 8
                        for _ in range(n_transforms):
                            # Header: int32 sec (4), uint32 nanosec (4), string frame_id
                            offset += 8  # skip sec + nanosec
                            frame_id, offset = _cdr_read_string(data, offset, le)
                            child_frame_id, offset = _cdr_read_string(data, offset, le)
                            # Skip Transform (translation 3xf64 + rotation 4xf64 = 56 bytes)
                            offset += 56
                            if frame_id and child_frame_id:
                                edges.add(
                                    frame_id.lstrip("/") + ">" + child_frame_id.lstrip("/")
                                )
                    except Exception:
                        continue
        finally:
            conn.close()
    return edges


def _ros2_tf_edges_py(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    edges = set()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    tf_type = get_message("tf2_msgs/msg/TFMessage")
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in ("/tf", "/tf_static"):
            continue
        msg = deserialize_message(data, tf_type)
        for tr in msg.transforms:
            p = tr.header.frame_id.lstrip("/")
            c = tr.child_frame_id.lstrip("/")
            if p and c:
                edges.add(p + ">" + c)
    return edges


# ---------------------------------------------------------------------------
# ROS1: subprocess helpers (unchanged from original)
# ---------------------------------------------------------------------------

def _ros_executable(name):
    found = shutil.which(name)
    if found:
        return found
    ros_distro = os.environ.get("ROS_DISTRO")
    candidates = []
    if ros_distro:
        candidates.append("/opt/ros/{}/bin/{}".format(ros_distro, name))
    candidates += ["/opt/ros/noetic/bin/{}".format(name),
                   "/opt/ros/melodic/bin/{}".format(name)]
    for c in candidates:
        if c and os.path.exists(c) and os.access(c, os.X_OK):
            return c
    return name


def _rosbag_python():
    env = _ros1_env()
    ros_distro = os.environ.get("ROS_DISTRO", "")
    candidates = ["python2", "python"] if ros_distro == "melodic" else [sys.executable, "python3"]
    candidates += [sys.executable, "python3", "python2", "python"]
    seen = set()
    for c in candidates:
        if c in seen:
            continue
        seen.add(c)
        try:
            subprocess.check_call([c, "-c", "import rosbag"],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                  env=env)
            return c
        except Exception:
            continue
    return sys.executable


def _ros1_env():
    env = os.environ.copy()
    ros_distro = env.get("ROS_DISTRO")
    py_paths, lib_paths, bin_paths = [], [], []
    if ros_distro:
        py_paths += ["/opt/ros/{}/lib/python3/dist-packages".format(ros_distro),
                     "/opt/ros/{}/lib/python2.7/dist-packages".format(ros_distro)]
        lib_paths.append("/opt/ros/{}/lib".format(ros_distro))
        bin_paths.append("/opt/ros/{}/bin".format(ros_distro))
    py_paths += ["/opt/ros/noetic/lib/python3/dist-packages",
                 "/opt/ros/melodic/lib/python2.7/dist-packages"]
    lib_paths += ["/opt/ros/noetic/lib", "/opt/ros/melodic/lib"]
    bin_paths += ["/opt/ros/noetic/bin", "/opt/ros/melodic/bin"]

    def _prepend(var, additions):
        existing = [p for p in env.get(var, "").split(":") if p]
        for a in reversed(additions):
            if os.path.isdir(a) and a not in existing:
                existing.insert(0, a)
        if existing:
            env[var] = ":".join(existing)

    _prepend("PYTHONPATH", py_paths)
    _prepend("LD_LIBRARY_PATH", lib_paths)
    _prepend("PATH", bin_paths)
    return env


def _get_ros1_bag_info(bag_path):
    try:
        out = subprocess.check_output(
            [_ros_executable("rosbag"), "info", "--yaml", bag_path],
            stderr=subprocess.STDOUT, env=_ros1_env())
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


def get_bag_info(bag_path):
    if _is_ros2_bag(bag_path):
        return _get_ros2_bag_info(bag_path)
    return _get_ros1_bag_info(bag_path)


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_bag_integrity(bag_path):
    print("\n--- Bag integrity ---")
    if _is_ros2_bag(bag_path):
        db_files = _ros2_db_files(bag_path)
        if not db_files:
            record(FAIL, "bag_integrity", "No .db3 files found in bag directory")
            return
        try:
            for db_file in db_files:
                conn = sqlite3.connect(db_file)
                conn.execute("SELECT count(*) FROM messages").fetchone()
                conn.execute("PRAGMA integrity_check").fetchone()
                conn.close()
            record(PASS, "bag_integrity", "SQLite3 integrity check passed ({} file(s))".format(
                len(db_files)))
        except Exception as e:
            record(FAIL, "bag_integrity", "SQLite3 error: {}".format(e))
        return

    try:
        out = subprocess.check_output(
            [_ros_executable("rosbag"), "check", bag_path],
            stderr=subprocess.STDOUT, env=_ros1_env())
        output = out.decode("utf-8").strip()
        if "No errors found" in output or output == "":
            record(PASS, "bag_integrity", "No errors found")
        else:
            record(WARN, "bag_integrity", output)
    except subprocess.CalledProcessError as e:
        record(FAIL, "bag_integrity", "rosbag check failed: {}".format(
            e.output.decode("utf-8").strip()))


def check_duration(info):
    print("\n--- Duration ---")
    duration = info.get("duration", 0.0)
    if duration >= MIN_DURATION_SEC:
        record(PASS, "duration", "{:.1f}s (min {:.0f}s)".format(duration, MIN_DURATION_SEC))
    else:
        record(FAIL, "duration", "{:.1f}s is below minimum {:.0f}s".format(
            duration, MIN_DURATION_SEC))
    return duration


def check_topics(info, duration):
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
            record(WARN, topic, "{} msgs @ {:.1f} Hz — within 20% of min {:.0f} Hz".format(
                msg_count, actual_hz, min_hz))
        else:
            record(FAIL, topic, "{} msgs @ {:.1f} Hz — BELOW minimum {:.0f} Hz".format(
                msg_count, actual_hz, min_hz))

    print("\n--- Optional topics ---")
    for topic in OPTIONAL_TOPICS:
        if topic in bag_topics:
            record(PASS, topic, "{} msgs present".format(bag_topics[topic].get("messages", 0)))
        else:
            record(WARN, topic, "not recorded (optional)")

    return bag_topics


def check_ground_truth(bag_topics, duration, require_gt):
    print("\n--- Ground truth ---")
    present = [t for t in GROUND_TRUTH_TOPICS if t in bag_topics]
    if not present:
        level = FAIL if require_gt else WARN
        record(level, "ground_truth", "MISSING — checked: {}".format(
            ", ".join(GROUND_TRUTH_TOPICS)))
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
    print("\n--- IMU availability ---")
    imu_topics = ["/imu", "/camera/imu", "/camera/accel/sample", "/camera/gyro/sample"]
    present = [t for t in imu_topics if t in bag_topics]
    if not present:
        level = FAIL if require_imu else WARN
        record(level, "imu", "MISSING — no IMU stream recorded")
        return
    any_live = False
    for topic in present:
        msg_count = bag_topics[topic].get("messages", 0)
        actual_hz = msg_count / duration if duration > 0 else 0.0
        min_hz = OPTIONAL_TOPICS.get(topic, {}).get("min_hz", 0.0)
        if actual_hz >= min_hz:
            any_live = True
            record(PASS, topic, "{} msgs @ {:.1f} Hz".format(msg_count, actual_hz))
        elif actual_hz > 0.0:
            any_live = True
            level = FAIL if require_imu else WARN
            record(level, topic, "{} msgs @ {:.1f} Hz — below required {:.0f} Hz".format(
                msg_count, actual_hz, min_hz))
        else:
            record(WARN, topic, "present but no messages")
    if require_imu and not any_live:
        record(FAIL, "imu", "IMU required but all IMU topics are empty")


def check_tf_tree(bag_path, bag_topics):
    print("\n--- TF tree ---")
    tf_topics = [t for t in ["/tf", "/tf_static"] if t in bag_topics]
    if not tf_topics:
        record(FAIL, "tf_tree", "No TF topics recorded")
        return

    if _is_ros2_bag(bag_path):
        try:
            pairs = _ros2_get_tf_edges(bag_path)
            if not pairs:
                record(WARN, "tf_tree", "Could not parse TF edges from ROS2 bag")
                return
        except Exception as e:
            record(WARN, "tf_tree", "TF edge parsing failed: {}".format(e))
            return
    else:
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
                                          timeout=60, env=_ros1_env())
            pairs = set(
                line.strip() for line in out.decode().splitlines() if line.strip()
            )
        except Exception:
            record(WARN, "tf_tree",
                   "Could not inspect TF messages — run with a working rosbag Python API")
            return

    required_pairs = [
        "odom>base_footprint",
        "base_footprint>base_link",
        "base_footprint>laser_frame",
        "base_footprint>camera_link",
    ]
    missing = [p for p in required_pairs if p not in pairs]
    if missing:
        record(FAIL, "tf_tree", "Missing required TF edge(s): {}".format(", ".join(missing)))
    else:
        record(PASS, "tf_tree",
               "Core chain present: odom→base_footprint→base_link, laser_frame, camera_link")

    camera_pairs = [p for p in pairs if p.startswith("camera_link>")]
    if camera_pairs:
        record(PASS, "camera_tf", "{} camera child frame edge(s) present".format(
            len(camera_pairs)))
    else:
        record(WARN, "camera_tf",
               "No camera_link child frames found; RGB-D SLAM may not resolve optical frames")


def check_frame_drops(bag_path, bag_topics, duration):
    print("\n--- Frame drop check ---")
    camera_topics = [
        "/camera/color/image_raw",
        "/camera/aligned_depth_to_color/image_raw",
    ]

    for topic in camera_topics:
        if topic not in bag_topics:
            continue
        spec = REQUIRED_TOPICS.get(topic, OPTIONAL_TOPICS.get(topic, {}))
        target_hz = spec.get("target_hz", spec.get("min_hz", 1.0))
        expected_period = 1.0 / target_hz if target_hz > 0 else 1.0
        max_gap_threshold = MAX_GAP_MULTIPLIER * expected_period

        if _is_ros2_bag(bag_path):
            timestamps = _ros2_get_timestamps(bag_path, topic)
        else:
            timestamps = _ros1_get_topic_timestamps(bag_path, topic)

        if not timestamps or len(timestamps) < 2:
            record(WARN, topic + " gaps", "Too few messages to analyse gaps")
            continue

        gaps = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
        max_actual_gap = max(gaps)
        n_drops = sum(1 for g in gaps if g > max_gap_threshold)
        drop_rate = 100.0 * n_drops / len(gaps)
        n_single = sum(1 for g in gaps if expected_period * 1.5 < g <= max_gap_threshold)

        if n_drops == 0 and n_single == 0:
            record(PASS, topic + " gaps",
                   "No drops. Max gap {:.3f}s (limit {:.3f}s, target {:.0f}Hz)".format(
                       max_actual_gap, max_gap_threshold, target_hz))
        elif n_drops == 0:
            record(PASS, topic + " gaps",
                   "{} minor gap(s). Max gap {:.3f}s. No consecutive drops.".format(
                       n_single, max_actual_gap))
        elif drop_rate < 1.0:
            record(WARN, topic + " gaps",
                   "{} drops ({:.2f}%) — max gap {:.3f}s.".format(
                       n_drops, drop_rate, max_actual_gap))
        else:
            record(FAIL, topic + " gaps",
                   "{} drops ({:.1f}%) — max gap {:.3f}s. NOT publishable quality.".format(
                       n_drops, drop_rate, max_actual_gap))


def check_colour_depth_sync(bag_path, bag_topics):
    print("\n--- Colour/depth sync ---")
    if ("/camera/color/image_raw" not in bag_topics or
            "/camera/aligned_depth_to_color/image_raw" not in bag_topics):
        record(WARN, "colour_depth_sync", "One or both topics missing — cannot check sync")
        return

    if _is_ros2_bag(bag_path):
        colour_times = _ros2_get_timestamps(bag_path, "/camera/color/image_raw")
        depth_times = _ros2_get_timestamps(bag_path, "/camera/aligned_depth_to_color/image_raw")
    else:
        colour_times = _ros1_get_topic_timestamps(bag_path, "/camera/color/image_raw")
        depth_times = _ros1_get_topic_timestamps(bag_path, "/camera/aligned_depth_to_color/image_raw")

    if not colour_times or not depth_times:
        record(WARN, "colour_depth_sync", "Could not read timestamps")
        return

    import bisect
    diffs = []
    for ct in colour_times:
        idx = bisect.bisect_left(depth_times, ct)
        candidates = []
        if idx < len(depth_times):
            candidates.append(abs(depth_times[idx] - ct))
        if idx > 0:
            candidates.append(abs(depth_times[idx - 1] - ct))
        if candidates:
            diffs.append(min(candidates))

    if not diffs:
        record(WARN, "colour_depth_sync", "No matching frame pairs found")
        return

    mean_ms = (sum(diffs) / len(diffs)) * 1000.0
    max_ms = max(diffs) * 1000.0
    pct_over_33ms = 100.0 * sum(1 for d in diffs if d > 0.033) / len(diffs)

    if mean_ms < 5:
        record(PASS, "colour_depth_sync",
               "Mean {:.1f}ms, max {:.1f}ms — excellent sync".format(mean_ms, max_ms))
    elif mean_ms < 33 and pct_over_33ms < 5.0:
        record(WARN, "colour_depth_sync",
               "Mean {:.1f}ms, max {:.1f}ms, {:.1f}% frames >33ms — "
               "adequate for LiDAR-primary SLAM, marginal for RGB-D SLAM.".format(
                   mean_ms, max_ms, pct_over_33ms))
    else:
        record(FAIL, "colour_depth_sync",
               "Mean {:.1f}ms, max {:.1f}ms, {:.1f}% frames >33ms — "
               "sync failure. NOT publishable for RGB-D SLAM.".format(
                   mean_ms, max_ms, pct_over_33ms))


def check_imu_monotonic(bag_path, bag_topics):
    print("\n--- IMU timestamp monotonicity ---")
    imu_topic = next(
        (t for t in ["/imu", "/camera/imu"] if t in bag_topics), None
    )
    if imu_topic is None:
        return

    if _is_ros2_bag(bag_path):
        timestamps = _ros2_get_timestamps(bag_path, imu_topic)
        if not timestamps:
            record(WARN, "imu_monotonic", "No {} messages".format(imu_topic))
            return
        bad = sum(1 for i in range(1, len(timestamps)) if timestamps[i] <= timestamps[i - 1])
        total = len(timestamps)
        if bad == 0:
            record(PASS, "imu_monotonic",
                   "All {} {} timestamps strictly increasing".format(total, imu_topic))
        else:
            record(FAIL, "imu_monotonic",
                   "{} non-monotonic {} timestamps out of {}".format(bad, imu_topic, total))
        return

    # ROS1 path
    try:
        cmd = [
            _rosbag_python(), "-c",
            "import rosbag, sys; b=rosbag.Bag(sys.argv[1]); "
            "stamps=[m.header.stamp.to_sec() for _,m,_ in b.read_messages(topics=[sys.argv[2]])]; "
            "b.close(); bad=sum(1 for i in range(1,len(stamps)) if stamps[i]<=stamps[i-1]); "
            "print(bad, len(stamps))",
            bag_path, imu_topic
        ]
        out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL,
                                      timeout=30, env=_ros1_env())
        parts = out.decode().strip().split()
        if len(parts) == 2:
            bad, total = int(parts[0]), int(parts[1])
            if bad == 0:
                record(PASS, "imu_monotonic",
                       "All {} {} timestamps strictly increasing".format(total, imu_topic))
            else:
                record(FAIL, "imu_monotonic",
                       "{} non-monotonic timestamps out of {}".format(bad, total))
    except Exception:
        record(WARN, "imu_monotonic",
               "Could not check — run with a working rosbag Python API")


def _ros1_get_topic_timestamps(bag_path, topic):
    """Get per-message timestamps from a ROS1 bag via subprocess."""
    try:
        cmd = [
            _rosbag_python(), "-c",
            "import rosbag, sys; b=rosbag.Bag(sys.argv[1]); "
            "[sys.stdout.write(str(m.header.stamp.to_sec() if hasattr(m,'header') "
            "else t.to_sec())+'\\n') "
            "for _,m,t in b.read_messages(topics=[sys.argv[2]])]; b.close()",
            bag_path, topic
        ]
        out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL,
                                      timeout=60, env=_ros1_env())
        return [float(x) for x in out.decode().strip().split("\n") if x]
    except Exception:
        return []


def check_imu_data(bag_path, bag_topics):
    """
    Check that /camera/imu has real accel and gyro data — not all zeros.

    Reads every message in the bag for accurate statistics.

    CDR offsets for sensor_msgs/msg/Imu (340 bytes, frame_id len=25):
      off       = 12 + 4 + fl + (4 - fl%4)%4   (4-byte aligned end of header)
      gyr_off   = off + 32 + 72                  (skip orientation 32B + orient_cov 72B)
      acc_off   = gyr_off + 24 + 72              (skip angular_velocity 24B + angvel_cov 72B)
    """
    print("\n--- Camera IMU data validity ---")

    if "/camera/imu" not in bag_topics:
        record(WARN, "camera_imu_data", "/camera/imu not recorded — skipping data check")
        return

    count = bag_topics["/camera/imu"].get("messages", 0)
    if count == 0:
        record(FAIL, "camera_imu_data", "/camera/imu present but contains no messages")
        return

    if _is_ros2_bag(bag_path):
        gyros = []
        accels = []
        for db_file in _ros2_db_files(bag_path):
            try:
                conn = sqlite3.connect(db_file)
                row = conn.execute(
                    "SELECT id FROM topics WHERE name='/camera/imu'"
                ).fetchone()
                if not row:
                    conn.close()
                    continue
                cursor = conn.execute(
                    "SELECT data FROM messages WHERE topic_id=?", (row[0],)
                )
                for (raw,) in cursor:
                    try:
                        d = bytes(raw)
                        e = "<" if d[1] == 1 else ">"
                        fl = struct.unpack_from(e+"I", d, 12)[0]
                        off = 12 + 4 + fl + (4 - (fl % 4)) % 4
                        gyr_off = off + 104
                        acc_off = gyr_off + 96
                        gx, gy, gz = struct.unpack_from(e+"ddd", d, gyr_off)
                        ax, ay, az = struct.unpack_from(e+"ddd", d, acc_off)
                        gyros.append((gx, gy, gz))
                        accels.append((ax, ay, az))
                    except Exception:
                        continue
                conn.close()
            except Exception:
                continue

    else:
        gyros = []
        accels = []
        try:
            cmd = [
                _rosbag_python(), "-c",
                "import rosbag,sys\nb=rosbag.Bag(sys.argv[1])\n"
                "for _,m,_ in b.read_messages(topics=['/camera/imu']):\n"
                "  print(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,"
                "m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z)\n"
                "b.close()",
                bag_path
            ]
            out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL,
                                          timeout=120, env=_ros1_env())
            for line in out.decode().strip().splitlines():
                parts = [float(x) for x in line.split()]
                if len(parts) == 6:
                    gyros.append((parts[0], parts[1], parts[2]))
                    accels.append((parts[3], parts[4], parts[5]))
        except Exception:
            pass

    if not gyros:
        record(WARN, "camera_imu_data", "Could not parse IMU messages for data check")
        return

    n = len(gyros)

    # --- Accel magnitude check ---
    accel_mags = [math.sqrt(ax**2 + ay**2 + az**2) for ax, ay, az in accels]
    mean_mag = sum(accel_mags) / n
    if 8.0 <= mean_mag <= 12.0:
        record(PASS, "camera_imu_accel",
               "Accel alive — mean |a|={:.3f} m/s² (expected ~9.8, n={})".format(mean_mag, n))
    else:
        record(FAIL, "camera_imu_accel",
               "Accel magnitude {:.3f} m/s² is outside 8–12 m/s² — "
               "sensor may be zeroed or gravity not present".format(mean_mag))

    # --- Gyro check ---
    all_axes_ok = True
    for axis, vals in zip(("x", "y", "z"), zip(*gyros)):
        vals = list(vals)
        mean_v = sum(vals) / len(vals)
        std_v = math.sqrt(sum((v - mean_v)**2 for v in vals) / len(vals))
        if all(abs(v) < 1e-9 for v in vals):
            record(FAIL, "camera_imu_gyro_{}".format(axis),
                   "Gyro {} identically zero across all {} msgs — sensor not reporting".format(
                       axis, n))
            all_axes_ok = False
        else:
            record(PASS, "camera_imu_gyro_{}".format(axis),
                   "Gyro {} alive — mean={:.5f} rad/s, std={:.5f} rad/s (n={})".format(
                       axis, mean_v, std_v, n))
    if all_axes_ok:
        record(PASS, "camera_imu_gyro",
               "All gyro axes reporting non-zero values ({} msgs)".format(n))


def _session_prefix_from_bag_path(bag_path):
    if _is_ros2_bag(bag_path):
        parent = os.path.dirname(bag_path.rstrip("/"))
        session = os.path.basename(bag_path.rstrip("/"))
        return os.path.join(parent, session)
    base = bag_path
    if base.endswith(".bag"):
        base = base[:-4]
    return base


def check_session_evidence(bag_path, require_hardware_logs=False):
    print("\n--- Session evidence files ---")

    prefix = _session_prefix_from_bag_path(bag_path)
    expected = [
        ("manifest", prefix + "_manifest.yaml"),
        ("chrony", prefix + "_chrony.txt"),
        ("hardware_pre", prefix + "_hardware_pre.log"),
        ("hardware_post", prefix + "_hardware_post.log"),
        ("runtime_watchdog_log", prefix + "_runtime_watchdog.log"),
        ("runtime_watchdog_status", prefix + "_runtime_watchdog.status"),
        ("kernel_runtime", prefix + "_kernel_runtime.log"),
        ("realsense_fault_classification", prefix + "_realsense_fault_classification.txt"),
    ]

    for label, path in expected:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            record(PASS, label, os.path.basename(path))
        else:
            required_evidence = label.startswith("hardware_") or label in (
                "runtime_watchdog_log",
                "runtime_watchdog_status",
                "kernel_runtime",
                "realsense_fault_classification",
            )
            level = FAIL if require_hardware_logs and required_evidence else WARN
            record(level, label, "missing or empty: {}".format(path))

    for label in ("hardware_pre", "hardware_post"):
        path = prefix + "_" + label + ".log"
        if not os.path.exists(path):
            continue
        try:
            with open(path, errors="replace") as f:
                text = f.read()
        except Exception as e:
            record(WARN, label, "could not read {}: {}".format(path, e))
            continue

        if "throttled=0x0" in text:
            record(PASS, label + "_power", "Pi throttling state clean")
        elif "throttled=" in text:
            record(WARN, label + "_power", "Pi throttling was nonzero; inspect {}".format(path))

        if "power/control=on" in text and "power/autosuspend=-1" in text:
            record(PASS, label + "_d455_power", "D455 runtime power management disabled")
        else:
            record(WARN, label + "_d455_power",
                   "D455 power/control or autosuspend evidence missing")

        if "Driver=uvcvideo, 5000M" in text or "speed=5000" in text:
            record(PASS, label + "_usb3", "D455/USB topology evidence shows USB3")
        else:
            record(WARN, label + "_usb3", "USB3 evidence missing")

    fault_path = prefix + "_realsense_fault_classification.txt"
    if os.path.exists(fault_path):
        try:
            with open(fault_path, errors="replace") as f:
                fault_text = f.read()
        except Exception as e:
            record(WARN, "realsense_fault_classification",
                   "could not read {}: {}".format(fault_path, e))
        else:
            match = re.search(r"^classification:\s*(\S+)", fault_text, re.MULTILINE)
            if not match:
                record(WARN, "realsense_fault_classification",
                       "classification line missing")
            else:
                classification = match.group(1)
                if classification.startswith("PASS"):
                    record(PASS, "realsense_fault_classification", classification)
                else:
                    record(FAIL, "realsense_fault_classification", classification)

    watchdog_status_path = prefix + "_runtime_watchdog.status"
    if os.path.exists(watchdog_status_path):
        try:
            with open(watchdog_status_path, errors="replace") as f:
                watchdog_status = f.readline().strip()
        except Exception as e:
            record(WARN, "runtime_watchdog",
                   "could not read {}: {}".format(watchdog_status_path, e))
        else:
            if watchdog_status.startswith("STOPPED_CLEANLY"):
                cycle_match = re.search(r"\bcycles=(\d+)", watchdog_status)
                cycles = int(cycle_match.group(1)) if cycle_match else None
                if cycles is None:
                    level = FAIL if require_hardware_logs else WARN
                    record(level, "runtime_watchdog",
                           "clean stop but cycle count missing")
                elif cycles >= 1:
                    record(PASS, "runtime_watchdog",
                           "{} runtime cycle(s) completed".format(cycles))
                else:
                    level = FAIL if require_hardware_logs else WARN
                    record(level, "runtime_watchdog",
                           "0 runtime cycles completed; run was too short to exercise watchdog")
            elif watchdog_status == "FAIL_RUNTIME_WATCHDOG":
                record(FAIL, "runtime_watchdog", watchdog_status)
            elif watchdog_status == "DISABLED":
                level = FAIL if require_hardware_logs else WARN
                record(level, "runtime_watchdog",
                       "disabled; enable for publishable ROS2 dataset runs")
            elif watchdog_status == "RUNNING":
                record(FAIL, "runtime_watchdog",
                       "still marked RUNNING; recording likely ended uncleanly")
            elif watchdog_status:
                record(WARN, "runtime_watchdog",
                       "unrecognised status: {}".format(watchdog_status))
            else:
                record(WARN, "runtime_watchdog", "empty status file")

    kernel_path = prefix + "_kernel_runtime.log"
    if os.path.exists(kernel_path):
        try:
            with open(kernel_path, errors="replace") as f:
                kernel_text = f.read()
        except Exception as e:
            record(WARN, "kernel_runtime", "could not read {}: {}".format(kernel_path, e))
        else:
            if re.search(r"UVCIOC_CTRL_QUERY|Frames didn't arrived|USB disconnect|No such device|Failed to create device",
                         kernel_text, re.IGNORECASE):
                record(WARN, "kernel_runtime",
                       "RealSense USB/UVC/HID text observed; inspect classification")
            else:
                record(PASS, "kernel_runtime", "no RealSense USB/UVC/HID fault text")


def print_summary(bag_path, duration, bag_topics, strict):
    print("\n" + "=" * 60)
    print("VALIDATION SUMMARY: {}".format(os.path.basename(bag_path.rstrip("/"))))
    print("=" * 60)

    n_pass = sum(1 for r in results if r[0] == PASS)
    n_warn = sum(1 for r in results if r[0] == WARN)
    n_fail = sum(1 for r in results if r[0] == FAIL)
    print("  PASS: {}   WARN: {}   FAIL: {}".format(n_pass, n_warn, n_fail))

    if _is_ros2_bag(bag_path):
        db_files = _ros2_db_files(bag_path)
        size_mb = sum(
            os.path.getsize(f) for f in db_files if os.path.exists(f)
        ) / 1e6
    else:
        size_mb = os.path.getsize(bag_path) / 1e6 if os.path.exists(bag_path) else 0

    print("  Duration: {:.1f}s   Size: {:.0f} MB".format(duration, size_mb))

    print("\n  RGB-D/IMU assessment:")
    sync_fails = [r for r in results if "sync" in r[1] and r[0] == FAIL]
    drop_fails = [r for r in results if "gaps" in r[1] and r[0] == FAIL]
    drop_warns = [r for r in results if "gaps" in r[1] and r[0] == WARN]

    if sync_fails or drop_fails:
        print("  ! Sensor streaming issues — reduce load or inspect USB")
    elif drop_warns:
        print("  ~ Occasional drops — acceptable but flag in dataset paper")
    else:
        print("  ✓ RGB-D/IMU streaming appears stable")

    print("\n  Publishability verdict:")
    if n_fail == 0 and n_warn == 0:
        print("  ✓ PASS — data meets publishable quality standards")
        return 0
    elif n_fail == 0:
        print("  ~ WARN — data is usable but review warnings before publishing")
        return 2 if strict else 0
    else:
        print("  ✗ FAIL — {} hard failure(s) must be resolved before publishing".format(n_fail))
        return 1


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Validate a ROS1 (.bag) or ROS2 (directory) bag for publishable quality")
    parser.add_argument("bag", help="Path to bag file or ROS2 bag directory")
    parser.add_argument("--strict", action="store_true",
                        help="Treat warnings as failures")
    parser.add_argument("--require-gt", action="store_true",
                        help="Require a ground-truth/mocap topic")
    parser.add_argument("--require-imu", action="store_true",
                        help="Require an IMU topic")
    parser.add_argument("--require-hardware-logs", action="store_true",
                        help="Fail if start_session hardware evidence logs are missing")
    args = parser.parse_args()

    bag_path = args.bag.rstrip("/")
    require_gt = args.require_gt or os.environ.get("REQUIRE_GT") == "true"
    require_imu = args.require_imu or os.environ.get("REQUIRE_IMU") == "true"

    if not os.path.exists(bag_path):
        print("ERROR: bag not found: {}".format(bag_path))
        sys.exit(1)

    if _is_ros2_bag(bag_path):
        bag_format = "ROS2 (sqlite3)"
    elif _is_ros1_bag(bag_path):
        bag_format = "ROS1"
    else:
        print("ERROR: not a recognised bag (expected .bag file or directory with metadata.yaml)")
        sys.exit(1)

    print("=" * 60)
    print("AGV Dataset Bag Validator")
    print("Bag:    {}".format(bag_path))
    print("Format: {}".format(bag_format))
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
    check_imu_data(bag_path, bag_topics)
    check_session_evidence(
        bag_path,
        args.require_hardware_logs or os.environ.get("REQUIRE_HARDWARE_LOGS") == "true",
    )

    exit_code = print_summary(bag_path, duration, bag_topics, args.strict)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
