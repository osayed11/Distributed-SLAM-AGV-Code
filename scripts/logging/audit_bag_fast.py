#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fast robot-bag audit without decoding image payloads.

Supports both ROS1 (.bag file) and ROS2 (directory with metadata.yaml + .db3).
Format is auto-detected from the path.

For ROS2 bags, timestamps come directly from the SQLite3 database (no serialisation
overhead). Full semantic content (TF edges, odom position, cmd_vel counts) requires
rosbag2_py + rclpy in the environment; when unavailable those fields are omitted and
labelled N/A rather than silently returning 0.
"""

from __future__ import print_function

import argparse
import bisect
import glob
import math
import os
import sqlite3
import struct
import sys
import yaml
from collections import defaultdict


REQUIRED_TOPICS = [
    "/scan",
    "/odom",
    "/cmd_vel",
    "/tf",
    "/tf_static",
    "/camera/color/image_raw",
    "/camera/color/camera_info",
    "/camera/depth/camera_info",
    "/camera/aligned_depth_to_color/image_raw",
    "/camera/aligned_depth_to_color/camera_info",
    "/camera/extrinsics/depth_to_color",
    "/diagnostics",
]

MIN_HZ = {
    "/scan":                                        5.0,
    "/odom":                                       10.0,
    "/tf":                                         10.0,
    "/camera/color/image_raw":                     12.0,
    "/camera/color/camera_info":                   12.0,
    "/camera/depth/camera_info":                   12.0,
    "/camera/aligned_depth_to_color/image_raw":    12.0,
    "/camera/aligned_depth_to_color/camera_info":  12.0,
}

IMU_TOPICS = ["/imu", "/camera/imu", "/camera/accel/sample", "/camera/gyro/sample"]


# ---------------------------------------------------------------------------
# Format detection
# ---------------------------------------------------------------------------

def _is_ros2_bag(path):
    return os.path.isdir(path) and os.path.exists(os.path.join(path, "metadata.yaml"))


# ---------------------------------------------------------------------------
# Shared utilities
# ---------------------------------------------------------------------------

def max_gap(values):
    if len(values) < 2:
        return 0.0
    values = sorted(values)
    return max(values[i] - values[i - 1] for i in range(1, len(values)))


def nearest_diffs(a, b):
    if not a or not b:
        return []
    b_sorted = sorted(b)
    diffs = []
    for value in a:
        idx = bisect.bisect_left(b_sorted, value)
        candidates = []
        if idx < len(b_sorted):
            candidates.append(abs(value - b_sorted[idx]))
        if idx > 0:
            candidates.append(abs(value - b_sorted[idx - 1]))
        if candidates:
            diffs.append(min(candidates))
    return diffs


def percentile(values, q):
    if not values:
        return 0.0
    values = sorted(values)
    idx = int(round((len(values) - 1) * q))
    return values[idx]


# ---------------------------------------------------------------------------
# ROS1 reader
# ---------------------------------------------------------------------------

def _stamp_to_sec_ros1(stamp):
    return stamp.secs + stamp.nsecs * 1e-9


def _msg_stamp_ros1(msg, fallback):
    header = getattr(msg, "header", None)
    if header is not None:
        s = getattr(header, "stamp", None)
        if s is not None:
            sec = _stamp_to_sec_ros1(s)
            if sec > 0.0:
                return sec
    return fallback


def _read_ros1_bag(bag_path):
    import rosbag
    counts = defaultdict(int)
    recv_times = defaultdict(list)
    header_times = defaultdict(list)
    tf_edges = set()
    nonzero_cmd_vel = 0
    odom_first = odom_last = None
    odom_max_from_start = 0.0

    with rosbag.Bag(bag_path) as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        duration = end - start

        for topic, msg, t in bag.read_messages():
            t_sec = t.to_sec()
            counts[topic] += 1
            recv_times[topic].append(t_sec)
            header_times[topic].append(_msg_stamp_ros1(msg, t_sec))

            if topic in ("/tf", "/tf_static"):
                for tr in msg.transforms:
                    p = tr.header.frame_id.lstrip("/")
                    c = tr.child_frame_id.lstrip("/")
                    if p and c:
                        tf_edges.add(p + " -> " + c)

            elif topic == "/cmd_vel":
                lin, ang = msg.linear, msg.angular
                if (abs(lin.x) + abs(lin.y) + abs(lin.z) +
                        abs(ang.x) + abs(ang.y) + abs(ang.z)) > 1e-6:
                    nonzero_cmd_vel += 1

            elif topic == "/odom":
                p = msg.pose.pose.position
                xy = (p.x, p.y)
                if odom_first is None:
                    odom_first = xy
                odom_last = xy
                odom_max_from_start = max(
                    odom_max_from_start,
                    math.hypot(xy[0] - odom_first[0], xy[1] - odom_first[1]))

    return {
        "duration": duration,
        "counts": counts,
        "recv_times": recv_times,
        "header_times": header_times,
        "tf_edges": tf_edges,
        "nonzero_cmd_vel": nonzero_cmd_vel,
        "odom_first": odom_first,
        "odom_last": odom_last,
        "odom_max_from_start": odom_max_from_start,
        "has_semantics": True,
    }


# ---------------------------------------------------------------------------
# ROS2 reader
# ---------------------------------------------------------------------------

def _ros2_db_files(bag_path):
    return sorted(glob.glob(os.path.join(bag_path, "*.db3")))


def _cdr_read_string(data, offset, little_endian):
    """Read a CDR-encoded string, return (str, new_offset)."""
    fmt = "<I" if little_endian else ">I"
    if offset + 4 > len(data):
        raise ValueError("buffer underflow")
    length = struct.unpack_from(fmt, data, offset)[0]
    offset += 4
    if length == 0:
        return "", offset
    s = data[offset:offset + length - 1].decode("utf-8", errors="replace")
    offset += length
    offset += (4 - (length % 4)) % 4  # 4-byte alignment
    return s, offset


def _ros2_tf_edges_cdr(bag_path):
    """Extract TF edges by minimal CDR parsing of TFMessage blobs."""
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
                for (data,) in conn.execute(
                        "SELECT data FROM messages WHERE topic_id=? LIMIT 500", (row[0],)):
                    try:
                        le = (data[1] == 1)
                        n = struct.unpack_from("<I" if le else ">I", data, 4)[0]
                        offset = 8
                        for _ in range(n):
                            offset += 8  # sec + nanosec
                            frame_id, offset = _cdr_read_string(data, offset, le)
                            child_id, offset = _cdr_read_string(data, offset, le)
                            offset += 56  # Transform (3+4 float64s)
                            if frame_id and child_id:
                                edges.add(
                                    frame_id.lstrip("/") + " -> " + child_id.lstrip("/")
                                )
                    except Exception:
                        continue
        finally:
            conn.close()
    return edges


def _ros2_semantics_via_rosbag2(bag_path, counts):
    """
    Attempt to read semantic content (TF, odom, cmd_vel) via rosbag2_py.
    Returns a dict of results, or None if rosbag2_py is unavailable.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return None

    tf_edges = set()
    nonzero_cmd_vel = 0
    odom_first = odom_last = None
    odom_max_from_start = 0.0

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    type_cache = {}

    def get_type(ros2_type):
        if ros2_type not in type_cache:
            type_cache[ros2_type] = get_message(ros2_type)
        return type_cache[ros2_type]

    semantic_topics = {"/tf", "/tf_static", "/cmd_vel", "/odom"}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in semantic_topics:
            continue
        try:
            msg = deserialize_message(data, get_type(topic_types[topic]))
        except Exception:
            continue

        if topic in ("/tf", "/tf_static"):
            for tr in msg.transforms:
                p = tr.header.frame_id.lstrip("/")
                c = tr.child_frame_id.lstrip("/")
                if p and c:
                    tf_edges.add(p + " -> " + c)

        elif topic == "/cmd_vel":
            lin, ang = msg.linear, msg.angular
            if (abs(lin.x) + abs(lin.y) + abs(lin.z) +
                    abs(ang.x) + abs(ang.y) + abs(ang.z)) > 1e-6:
                nonzero_cmd_vel += 1

        elif topic == "/odom":
            p = msg.pose.pose.position
            xy = (p.x, p.y)
            if odom_first is None:
                odom_first = xy
            odom_last = xy
            odom_max_from_start = max(
                odom_max_from_start,
                math.hypot(xy[0] - odom_first[0], xy[1] - odom_first[1]))

    return {
        "tf_edges": tf_edges,
        "nonzero_cmd_vel": nonzero_cmd_vel,
        "odom_first": odom_first,
        "odom_last": odom_last,
        "odom_max_from_start": odom_max_from_start,
    }


def _read_ros2_bag(bag_path):
    with open(os.path.join(bag_path, "metadata.yaml")) as f:
        meta = yaml.safe_load(f)
    info = meta.get("rosbag2_bagfile_information", {})
    duration_ns = info.get("duration", {}).get("nanoseconds", 0)
    duration = duration_ns / 1e9

    # Build topic id maps from all db files
    counts = defaultdict(int)
    recv_times = defaultdict(list)  # using bag receive timestamp (nanosec→sec)

    for db_file in _ros2_db_files(bag_path):
        conn = sqlite3.connect(db_file)
        try:
            id_to_name = {
                row[0]: row[1]
                for row in conn.execute("SELECT id, name FROM topics").fetchall()
            }
            for row in conn.execute(
                    "SELECT topic_id, timestamp FROM messages ORDER BY timestamp"):
                topic_id, ts_ns = row
                name = id_to_name.get(topic_id)
                if name:
                    counts[name] += 1
                    recv_times[name].append(ts_ns / 1e9)
        finally:
            conn.close()

    # ROS2 bags don't meaningfully distinguish recv vs header time via sqlite3
    # (the db timestamp IS the publish timestamp). Use it for both.
    header_times = {topic: list(times) for topic, times in recv_times.items()}

    # TF edges: try CDR parsing first, then rosbag2_py
    tf_edges = set()
    try:
        tf_edges = _ros2_tf_edges_cdr(bag_path)
    except Exception:
        pass

    # Semantic content via rosbag2_py
    semantics = _ros2_semantics_via_rosbag2(bag_path, counts)
    has_semantics = semantics is not None

    if has_semantics:
        # rosbag2_py gives us better TF edges (full deserialization)
        if semantics["tf_edges"]:
            tf_edges = semantics["tf_edges"]
        nonzero_cmd_vel = semantics["nonzero_cmd_vel"]
        odom_first = semantics["odom_first"]
        odom_last = semantics["odom_last"]
        odom_max_from_start = semantics["odom_max_from_start"]
    else:
        nonzero_cmd_vel = None  # unknown without deserialization
        odom_first = odom_last = None
        odom_max_from_start = None

    return {
        "duration": duration,
        "counts": counts,
        "recv_times": recv_times,
        "header_times": header_times,
        "tf_edges": tf_edges,
        "nonzero_cmd_vel": nonzero_cmd_vel,
        "odom_first": odom_first,
        "odom_last": odom_last,
        "odom_max_from_start": odom_max_from_start,
        "has_semantics": has_semantics,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Fast bag audit for ROS1 (.bag) or ROS2 (directory) bags")
    parser.add_argument("bag", help="Path to bag file or ROS2 bag directory")
    args = parser.parse_args()

    bag_path = args.bag.rstrip("/")

    if not os.path.exists(bag_path):
        print("ERROR: bag not found: {}".format(bag_path))
        sys.exit(1)

    if _is_ros2_bag(bag_path):
        data = _read_ros2_bag(bag_path)
        bag_format = "ROS2"
    else:
        try:
            data = _read_ros1_bag(bag_path)
            bag_format = "ROS1"
        except ImportError:
            print("ERROR: rosbag module not available for ROS1 bag. Source ROS1 first.")
            sys.exit(1)

    counts = data["counts"]
    recv_times = data["recv_times"]
    header_times = data["header_times"]
    tf_edges = data["tf_edges"]
    duration = data["duration"]
    has_semantics = data["has_semantics"]
    nonzero_cmd_vel = data["nonzero_cmd_vel"]
    odom_first = data["odom_first"]
    odom_last = data["odom_last"]
    odom_max_from_start = data["odom_max_from_start"]

    print("bag:    {}".format(bag_path))
    print("format: {}".format(bag_format))
    print("duration_sec: {:.3f}".format(duration))
    print("")

    print("required_topics:")
    hard_fail = False
    for topic in REQUIRED_TOPICS:
        count = counts.get(topic, 0)
        rate = count / duration if duration > 0 else 0.0
        min_hz = MIN_HZ.get(topic)
        status = "PASS"
        if count == 0:
            status = "FAIL"
            hard_fail = True
        elif min_hz is not None and rate < min_hz:
            status = "FAIL"
            hard_fail = True
        print("  {} {:48s} count={:<6d} hz={:6.2f} max_recv_gap={:6.3f}s max_stamp_gap={:6.3f}s".format(
            status, topic, count, rate,
            max_gap(recv_times[topic]),
            max_gap(header_times[topic])))

    color_stamps = header_times.get("/camera/color/image_raw", [])
    depth_stamps = header_times.get("/camera/aligned_depth_to_color/image_raw", [])
    sync_diffs = nearest_diffs(color_stamps, depth_stamps)
    print("")
    print("camera_sync:")
    if sync_diffs:
        print("  samples={} median_diff_ms={:.2f} p95_diff_ms={:.2f} max_diff_ms={:.2f}".format(
            len(sync_diffs),
            percentile(sync_diffs, 0.50) * 1000.0,
            percentile(sync_diffs, 0.95) * 1000.0,
            max(sync_diffs) * 1000.0))
    else:
        print("  samples=0 (one or both camera topics missing)")

    print("")
    print("motion:")
    if nonzero_cmd_vel is not None:
        print("  nonzero_cmd_vel_msgs={}".format(nonzero_cmd_vel))
    else:
        print("  nonzero_cmd_vel_msgs=N/A (rosbag2_py unavailable)")
    if odom_first is not None and odom_last is not None:
        print("  odom_first=({:.3f},{:.3f}) odom_last=({:.3f},{:.3f}) max_from_start_m={:.3f}".format(
            odom_first[0], odom_first[1], odom_last[0], odom_last[1],
            odom_max_from_start))
    elif counts.get("/odom", 0) > 0:
        print("  odom_position=N/A (rosbag2_py unavailable for deserialization)")

    print("")
    print("imu:")
    for topic in IMU_TOPICS:
        count = counts.get(topic, 0)
        rate = count / duration if duration > 0 else 0.0
        if count:
            print("  {:48s} count={:<6d} hz={:6.2f} max_recv_gap={:6.3f}s max_stamp_gap={:6.3f}s".format(
                topic, count, rate,
                max_gap(recv_times[topic]),
                max_gap(header_times[topic])))
        else:
            print("  {:48s} count=0".format(topic))

    print("")
    print("tf_edges:")
    if tf_edges:
        for edge in sorted(tf_edges):
            print("  {}".format(edge))
    else:
        print("  (none extracted)")

    print("")
    print("overall: {}".format("FAIL" if hard_fail else "PASS"))
    if bag_format == "ROS2" and not has_semantics:
        print("note: install rosbag2_py + rclpy for full semantic analysis "
              "(cmd_vel counts, odom position, enriched TF edges)")


if __name__ == "__main__":
    main()
