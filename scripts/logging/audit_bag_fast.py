#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Fast robot-bag audit without decoding image payloads."""

from __future__ import print_function

import argparse
import bisect
import math
from collections import defaultdict

import rosbag


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
    "/tag_detections",
]

MIN_HZ = {
    "/scan": 5.0,
    "/odom": 10.0,
    "/tf": 10.0,
    "/camera/color/image_raw": 12.0,
    "/camera/color/camera_info": 12.0,
    "/camera/depth/camera_info": 12.0,
    "/camera/aligned_depth_to_color/image_raw": 12.0,
    "/camera/aligned_depth_to_color/camera_info": 12.0,
}


def stamp_to_sec(stamp):
    return stamp.secs + stamp.nsecs * 1e-9


def msg_stamp(msg, fallback_time):
    header = getattr(msg, "header", None)
    if header is not None and getattr(header, "stamp", None) is not None:
        sec = stamp_to_sec(header.stamp)
        if sec > 0.0:
            return sec
    return fallback_time


def max_gap(values):
    if len(values) < 2:
        return 0.0
    values = sorted(values)
    return max(values[i] - values[i - 1] for i in range(1, len(values)))


def nearest_diffs(a, b):
    if not a or not b:
        return []
    b = sorted(b)
    diffs = []
    for value in a:
        idx = bisect.bisect_left(b, value)
        candidates = []
        if idx < len(b):
            candidates.append(abs(value - b[idx]))
        if idx > 0:
            candidates.append(abs(value - b[idx - 1]))
        if candidates:
            diffs.append(min(candidates))
    return diffs


def percentile(values, q):
    if not values:
        return 0.0
    values = sorted(values)
    idx = int(round((len(values) - 1) * q))
    return values[idx]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    counts = defaultdict(int)
    recv_times = defaultdict(list)
    header_times = defaultdict(list)
    tf_edges = set()
    nonzero_cmd_vel = 0
    tag_msgs_nonempty = 0
    tag_detections_total = 0
    diag_warn = 0
    diag_error = 0
    odom_first = None
    odom_last = None
    odom_max_from_start = 0.0

    with rosbag.Bag(args.bag) as bag:
        start = bag.get_start_time()
        end = bag.get_end_time()
        duration = end - start

        for topic, msg, t in bag.read_messages():
            t_sec = stamp_to_sec(t)
            counts[topic] += 1
            recv_times[topic].append(t_sec)
            header_times[topic].append(msg_stamp(msg, t_sec))

            if topic in ("/tf", "/tf_static"):
                for tr in msg.transforms:
                    parent = tr.header.frame_id.lstrip("/")
                    child = tr.child_frame_id.lstrip("/")
                    if parent and child:
                        tf_edges.add(parent + " -> " + child)

            elif topic == "/cmd_vel":
                lin = msg.linear
                ang = msg.angular
                if (abs(lin.x) + abs(lin.y) + abs(lin.z) +
                        abs(ang.x) + abs(ang.y) + abs(ang.z)) > 1e-6:
                    nonzero_cmd_vel += 1

            elif topic == "/tag_detections":
                detections = getattr(msg, "detections", [])
                if detections:
                    tag_msgs_nonempty += 1
                    tag_detections_total += len(detections)

            elif topic == "/diagnostics":
                for status in msg.status:
                    if status.level == 1:
                        diag_warn += 1
                    elif status.level >= 2:
                        diag_error += 1

            elif topic == "/odom":
                p = msg.pose.pose.position
                xy = (p.x, p.y)
                if odom_first is None:
                    odom_first = xy
                odom_last = xy
                if odom_first is not None:
                    odom_max_from_start = max(
                        odom_max_from_start,
                        math.hypot(xy[0] - odom_first[0], xy[1] - odom_first[1]))

    print("bag: {}".format(args.bag))
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
            status, topic, count, rate, max_gap(recv_times[topic]),
            max_gap(header_times[topic])))

    color_stamps = header_times["/camera/color/image_raw"]
    depth_stamps = header_times["/camera/aligned_depth_to_color/image_raw"]
    sync_diffs = nearest_diffs(color_stamps, depth_stamps)
    print("")
    print("camera_sync:")
    print("  samples={} median_diff_ms={:.2f} p95_diff_ms={:.2f} max_diff_ms={:.2f}".format(
        len(sync_diffs),
        percentile(sync_diffs, 0.50) * 1000.0,
        percentile(sync_diffs, 0.95) * 1000.0,
        max(sync_diffs) * 1000.0 if sync_diffs else 0.0))

    print("")
    print("motion:")
    print("  nonzero_cmd_vel_msgs={}".format(nonzero_cmd_vel))
    if odom_first is not None and odom_last is not None:
        print("  odom_first=({:.3f},{:.3f}) odom_last=({:.3f},{:.3f}) max_from_start_m={:.3f}".format(
            odom_first[0], odom_first[1], odom_last[0], odom_last[1],
            odom_max_from_start))

    print("")
    print("apriltag:")
    print("  messages={} nonempty_messages={} total_detections={}".format(
        counts.get("/tag_detections", 0), tag_msgs_nonempty,
        tag_detections_total))

    print("")
    print("diagnostics:")
    print("  warn_statuses={} error_statuses={}".format(diag_warn, diag_error))

    print("")
    print("tf_edges:")
    for edge in sorted(tf_edges):
        print("  {}".format(edge))

    print("")
    print("overall: {}".format("FAIL" if hard_fail else "PASS"))


if __name__ == "__main__":
    main()
