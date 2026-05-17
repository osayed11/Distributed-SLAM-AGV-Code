#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Odom-feedback forward/back shuttle for straight-line data collection.

The old version was a smoke test: it sent one fixed forward command until
Euclidean odom distance crossed a threshold, then one fixed reverse command.
That is not safe for long multi-robot straight-line runs. This controller keeps
the initial odom heading as a lane, drives to projected along-track targets,
slows near endpoints, and stops if cross-track error becomes too large.
"""

from __future__ import print_function

import argparse
import math
import signal
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


pose = None
stop_requested = False


def request_stop(signum=None, frame=None):
    global stop_requested
    stop_requested = True
    if signum is not None:
        print("\nStop requested; sending zero velocity.")


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_cb(msg):
    global pose
    p = msg.pose.pose.position
    pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))


def angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, low, high):
    return max(low, min(high, value))


def publish_zero(pub, seconds=0.8):
    msg = Twist()
    rate = rospy.Rate(20)
    end = time.time() + seconds
    while not rospy.is_shutdown() and time.time() < end:
        try:
            pub.publish(msg)
            rate.sleep()
        except rospy.ROSException:
            break
        except rospy.ROSInterruptException:
            time.sleep(0.05)


def wait_for_odom(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for /odom")
        rospy.sleep(0.05)


def wait_before_motion(pub, args):
    if args.start_at_epoch > 0.0:
        target_epoch = args.start_at_epoch + args.start_delay
        print("Waiting for scheduled start epoch %.3f" % target_epoch)
    elif args.start_delay > 0.0:
        target_epoch = time.time() + args.start_delay
        print("Waiting %.1fs before moving..." % args.start_delay)
    else:
        return True

    msg = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not stop_requested:
        remaining = target_epoch - time.time()
        if remaining <= 0.0:
            break
        pub.publish(msg)
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            time.sleep(0.05)

    late_by = time.time() - target_epoch
    if late_by > args.max_start_late:
        print("ERROR scheduled start missed by %.2fs; refusing motion" % late_by)
        return False
    return not stop_requested


def lane_state(origin_x, origin_y, lane_yaw):
    x, y, yaw = pose
    dx = x - origin_x
    dy = y - origin_y
    forward_x = math.cos(lane_yaw)
    forward_y = math.sin(lane_yaw)
    projected = dx * forward_x + dy * forward_y
    cross_track = -dx * forward_y + dy * forward_x
    return projected, cross_track, yaw


def speed_for_remaining(remaining, args):
    command = clamp(args.linear_kp * remaining, 0.0, args.speed)
    if remaining > args.min_speed_distance and command < args.min_speed:
        command = args.min_speed
    return command


def drive_leg(pub, args, origin, lane_yaw, target_along, direction, leg_index):
    origin_x, origin_y = origin
    msg = Twist()
    rate = rospy.Rate(args.rate)
    start = time.time()
    last_report = start
    completed = False
    abort_reason = ""

    while not rospy.is_shutdown() and not stop_requested:
        projected, cross_track, yaw = lane_state(origin_x, origin_y, lane_yaw)
        if direction > 0:
            remaining = target_along - projected
        else:
            remaining = projected - target_along

        heading_offset = -direction * clamp(
            args.cross_track_kp * cross_track,
            -args.max_heading_offset,
            args.max_heading_offset,
        )
        target_yaw = lane_yaw + heading_offset
        heading_error = angle_delta(target_yaw, yaw)
        elapsed = time.time() - start

        if remaining <= args.distance_tolerance:
            completed = True
            break
        if abs(cross_track) > args.max_cross_track:
            abort_reason = "cross-track %.3fm exceeded %.3fm" % (
                cross_track, args.max_cross_track)
            break
        if elapsed >= args.timeout:
            abort_reason = "timeout %.1fs, remaining %.3fm" % (
                args.timeout, remaining)
            break

        linear = direction * speed_for_remaining(remaining, args)
        if abs(heading_error) > args.heading_gate:
            linear = 0.0

        msg.linear.x = linear
        msg.angular.z = clamp(args.heading_kp * heading_error,
                              -args.max_angular, args.max_angular)
        pub.publish(msg)

        now = time.time()
        if args.verbose and now - last_report >= args.report_period:
            print(
                "leg=%d target=%.3f projected=%.3f remaining=%.3f "
                "cross=%.3f heading=%.1fdeg cmd=(%.3f, %.3f)" %
                (
                    leg_index,
                    target_along,
                    projected,
                    remaining,
                    cross_track,
                    math.degrees(heading_error),
                    msg.linear.x,
                    msg.angular.z,
                )
            )
            last_report = now

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    publish_zero(pub)
    projected, cross_track, yaw = lane_state(origin_x, origin_y, lane_yaw)
    yaw_error = angle_delta(lane_yaw, yaw)
    status = "DONE" if completed else "STOP"
    print(
        "%s leg=%d projected=%.3fm target=%.3fm cross=%.3fm yaw_error=%.1fdeg" %
        (
            status,
            leg_index,
            projected,
            target_along,
            cross_track,
            math.degrees(yaw_error),
        )
    )
    if abort_reason:
        print("  reason: %s" % abort_reason)
    return completed


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Drive repeated odom-bounded forward/back straight legs")
    parser.add_argument("--distance", type=float, default=0.20,
                        help="Along-lane distance from origin in odom metres")
    parser.add_argument("--cycles", type=int, default=1,
                        help="Number of forward/back cycles")
    parser.add_argument("--speed", type=float, default=0.10,
                        help="Maximum normalized linear command")
    parser.add_argument("--min-speed", type=float, default=0.035,
                        help="Minimum normalized command away from endpoint")
    parser.add_argument("--linear-kp", type=float, default=0.75,
                        help="P gain from remaining distance to linear command")
    parser.add_argument("--heading-kp", type=float, default=0.85,
                        help="P gain from heading error to angular command")
    parser.add_argument("--cross-track-kp", type=float, default=1.2,
                        help="Heading offset gain from lateral lane error")
    parser.add_argument("--max-angular", type=float, default=0.12,
                        help="Maximum angular command magnitude")
    parser.add_argument("--max-heading-offset-deg", type=float, default=12.0,
                        help="Max heading bias used to return to the lane")
    parser.add_argument("--heading-gate-deg", type=float, default=18.0,
                        help="Stop translating while heading error is larger")
    parser.add_argument("--max-cross-track", type=float, default=0.25,
                        help="Abort if lateral odom drift exceeds this")
    parser.add_argument("--distance-tolerance", type=float, default=0.03,
                        help="Endpoint tolerance in odom metres")
    parser.add_argument("--min-speed-distance", type=float, default=0.08,
                        help="Only enforce min speed farther than this")
    parser.add_argument("--timeout", type=float, default=8.0,
                        help="Maximum seconds per leg")
    parser.add_argument("--pause", type=float, default=1.0,
                        help="Pause between legs")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Control loop rate in Hz")
    parser.add_argument("--start-delay", type=float, default=0.0,
                        help="Seconds to wait before moving")
    parser.add_argument("--start-at-epoch", type=float, default=0.0,
                        help="Unix epoch base time; start-delay is added")
    parser.add_argument("--max-start-late", type=float, default=3.0,
                        help="Abort if scheduled start is missed by this much")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Start without pressing Enter")
    parser.add_argument("--verbose", action="store_true",
                        help="Print periodic feedback")
    parser.add_argument("--report-period", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.distance = max(0.0, args.distance)
    args.cycles = max(1, args.cycles)
    args.speed = clamp(abs(args.speed), 0.0, 1.0)
    args.min_speed = clamp(abs(args.min_speed), 0.0, args.speed)
    args.linear_kp = max(0.0, args.linear_kp)
    args.heading_kp = max(0.0, args.heading_kp)
    args.cross_track_kp = max(0.0, args.cross_track_kp)
    args.max_angular = clamp(abs(args.max_angular), 0.0, 1.0)
    args.max_heading_offset = math.radians(max(0.0, args.max_heading_offset_deg))
    args.heading_gate = math.radians(max(0.1, args.heading_gate_deg))
    args.max_cross_track = max(0.01, args.max_cross_track)
    args.distance_tolerance = max(0.0, args.distance_tolerance)
    args.min_speed_distance = max(args.distance_tolerance, args.min_speed_distance)
    args.timeout = max(0.1, args.timeout)
    args.pause = max(0.0, args.pause)
    args.rate = max(5.0, args.rate)
    args.start_delay = max(0.0, args.start_delay)
    args.start_at_epoch = max(0.0, args.start_at_epoch)
    args.max_start_late = max(0.0, args.max_start_late)
    args.report_period = max(0.5, args.report_period)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rospy.init_node("agv_forward_back_shuttle")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=10.0)
    publish_zero(pub)

    origin = (pose[0], pose[1])
    lane_yaw = pose[2]
    print("Forward/back shuttle ready:")
    print("  origin=(%.3f, %.3f) lane_yaw=%.1fdeg" %
          (origin[0], origin[1], math.degrees(lane_yaw)))
    print("  distance=%.2fm cycles=%d speed<=%.2f max_cross=%.2fm" %
          (args.distance, args.cycles, args.speed, args.max_cross_track))
    print("  Ctrl+C stops the robot.")

    if not args.no_prompt:
        try:
            raw_input("Press Enter to start, or Ctrl+C to cancel...")
        except NameError:
            input("Press Enter to start, or Ctrl+C to cancel...")

    if not wait_before_motion(pub, args):
        publish_zero(pub, seconds=1.0)
        return

    try:
        leg_index = 1
        for cycle in range(args.cycles):
            if rospy.is_shutdown() or stop_requested:
                break
            print("Cycle %d/%d: forward" % (cycle + 1, args.cycles))
            if not drive_leg(pub, args, origin, lane_yaw, args.distance,
                             1.0, leg_index):
                break
            leg_index += 1
            rospy.sleep(args.pause)

            if rospy.is_shutdown() or stop_requested:
                break
            print("Cycle %d/%d: reverse" % (cycle + 1, args.cycles))
            if not drive_leg(pub, args, origin, lane_yaw, 0.0,
                             -1.0, leg_index):
                break
            leg_index += 1
            rospy.sleep(args.pause)
    finally:
        publish_zero(pub, seconds=1.0)
        print("Forward/back shuttle finished; zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
