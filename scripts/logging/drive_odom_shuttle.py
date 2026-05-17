#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Odom-only straight-line shuttle controller.

This is intended for dataset scenarios where each robot should repeatedly move
between two endpoints on its own lane using only local odometry. It does not use
mocap, lidar obstacle checks, or inter-robot coordination.
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
last_odom_wall_time = None
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
    global pose, last_odom_wall_time
    p = msg.pose.pose.position
    pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
    last_odom_wall_time = time.time()


def angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, low, high):
    return max(low, min(high, value))


def sign_or_zero(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


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
    left_x = -forward_y
    left_y = forward_x
    along = dx * forward_x + dy * forward_y
    cross = dx * left_x + dy * left_y
    yaw_error = angle_delta(lane_yaw, yaw)
    return along, cross, yaw, yaw_error


def world_to_body(vx_world, vy_world, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    body_x = cos_yaw * vx_world + sin_yaw * vy_world
    body_y = -sin_yaw * vx_world + cos_yaw * vy_world
    return body_x, body_y


def lane_velocity_to_body(along_cmd, cross_cmd, lane_yaw, yaw):
    forward_x = math.cos(lane_yaw)
    forward_y = math.sin(lane_yaw)
    left_x = -forward_y
    left_y = forward_x
    vx_world = along_cmd * forward_x + cross_cmd * left_x
    vy_world = along_cmd * forward_y + cross_cmd * left_y
    return world_to_body(vx_world, vy_world, yaw)


def scale_xy(x, y, limit):
    mag = math.hypot(x, y)
    if mag <= limit or mag < 1e-9:
        return x, y
    scale = limit / mag
    return x * scale, y * scale


def command_for_error(along_error, cross_error, yaw_error, lane_yaw, yaw, args):
    along_cmd = clamp(args.along_kp * along_error, -args.max_speed, args.max_speed)
    if abs(along_error) > args.min_speed_distance and abs(along_cmd) < args.min_speed:
        along_cmd = sign_or_zero(along_error) * args.min_speed

    cross_cmd = clamp(-args.cross_track_kp * cross_error,
                      -args.max_lateral, args.max_lateral)

    body_x, body_y = lane_velocity_to_body(along_cmd, cross_cmd, lane_yaw, yaw)
    body_x, body_y = scale_xy(body_x, body_y, args.max_xy_command)

    if abs(yaw_error) > args.heading_gate:
        body_x = 0.0
        body_y = 0.0

    angular = clamp(args.heading_kp * yaw_error,
                    -args.max_angular, args.max_angular)
    return body_x, body_y, angular


def odom_is_stale(args):
    if last_odom_wall_time is None:
        return True
    return (time.time() - last_odom_wall_time) > args.odom_timeout


def drive_to_target(pub, args, origin, lane_yaw, target_along, leg_index):
    origin_x, origin_y = origin
    rate = rospy.Rate(args.rate)
    msg = Twist()
    start = time.time()
    last_report = start
    settle_start = None
    best_abs_along_error = None
    last_progress_time = start
    abort_reason = ""

    while not rospy.is_shutdown() and not stop_requested:
        if odom_is_stale(args):
            abort_reason = "odom stale for more than %.2fs" % args.odom_timeout
            break

        along, cross, yaw, yaw_error = lane_state(origin_x, origin_y, lane_yaw)
        along_error = target_along - along
        now = time.time()
        elapsed = now - start

        abs_along_error = abs(along_error)
        if best_abs_along_error is None or abs_along_error < best_abs_along_error - args.progress_epsilon:
            best_abs_along_error = abs_along_error
            last_progress_time = now

        position_ok = (
            abs_along_error <= args.distance_tolerance and
            abs(cross) <= args.cross_track_tolerance and
            abs(yaw_error) <= args.heading_tolerance
        )

        if position_ok:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            pub.publish(msg)
            if settle_start is None:
                settle_start = now
            if now - settle_start >= args.settle_time:
                print(
                    "DONE leg=%d target=%.3fm along=%.3fm cross=%.3fm yaw=%.1fdeg" %
                    (leg_index, target_along, along, cross, math.degrees(yaw_error))
                )
                return True
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            continue
        else:
            settle_start = None

        if abs(cross) > args.max_cross_track:
            abort_reason = "cross-track %.3fm exceeded %.3fm" % (cross, args.max_cross_track)
            break
        if abs(yaw_error) > args.max_heading_error:
            abort_reason = "heading %.1fdeg exceeded %.1fdeg" % (
                math.degrees(yaw_error), math.degrees(args.max_heading_error))
            break
        if elapsed >= args.timeout:
            abort_reason = "timeout %.1fs, along_error %.3fm" % (args.timeout, along_error)
            break
        if now - last_progress_time > args.stuck_timeout:
            abort_reason = "no odom progress for %.1fs" % args.stuck_timeout
            break

        body_x, body_y, angular = command_for_error(
            along_error, cross, yaw_error, lane_yaw, yaw, args)
        msg.linear.x = body_x
        msg.linear.y = body_y if args.use_lateral else 0.0
        msg.angular.z = angular
        pub.publish(msg)

        if args.verbose and now - last_report >= args.report_period:
            print(
                "leg=%d target=%.3f along=%.3f err=%.3f cross=%.3f "
                "yaw=%.1fdeg cmd=(%.3f, %.3f, %.3f)" %
                (
                    leg_index,
                    target_along,
                    along,
                    along_error,
                    cross,
                    math.degrees(yaw_error),
                    msg.linear.x,
                    msg.linear.y,
                    msg.angular.z,
                )
            )
            last_report = now

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    publish_zero(pub)
    along, cross, yaw, yaw_error = lane_state(origin_x, origin_y, lane_yaw)
    print(
        "STOP leg=%d target=%.3fm along=%.3fm cross=%.3fm yaw=%.1fdeg" %
        (leg_index, target_along, along, cross, math.degrees(yaw_error))
    )
    if abort_reason:
        print("  reason: %s" % abort_reason)
    return False


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Odom-only forward/back shuttle")
    parser.add_argument("--distance", type=float, default=1.0,
                        help="Endpoint distance from start along initial heading, metres")
    parser.add_argument("--cycles", type=int, default=10,
                        help="Number of out-and-back cycles")
    parser.add_argument("--max-speed", type=float, default=0.08,
                        help="Maximum normalized along-track command")
    parser.add_argument("--min-speed", type=float, default=0.025,
                        help="Minimum normalized along-track command far from endpoints")
    parser.add_argument("--max-lateral", type=float, default=0.06,
                        help="Maximum normalized lateral correction command")
    parser.add_argument("--max-xy-command", type=float, default=0.10,
                        help="Maximum combined normalized x/y command")
    parser.add_argument("--along-kp", type=float, default=0.55,
                        help="P gain from along-track error to along command")
    parser.add_argument("--cross-track-kp", type=float, default=0.75,
                        help="P gain from lateral error to lateral command")
    parser.add_argument("--heading-kp", type=float, default=0.75,
                        help="P gain from yaw error to angular command")
    parser.add_argument("--max-angular", type=float, default=0.16,
                        help="Maximum normalized angular command")
    parser.add_argument("--distance-tolerance", type=float, default=0.04,
                        help="Along-track endpoint tolerance, metres")
    parser.add_argument("--cross-track-tolerance", type=float, default=0.05,
                        help="Lateral endpoint tolerance, metres")
    parser.add_argument("--max-cross-track", type=float, default=0.18,
                        help="Abort if lateral error exceeds this, metres")
    parser.add_argument("--heading-tolerance-deg", type=float, default=5.0,
                        help="Yaw tolerance needed to finish a leg")
    parser.add_argument("--heading-gate-deg", type=float, default=15.0,
                        help="Stop translating while yaw error exceeds this")
    parser.add_argument("--max-heading-error-deg", type=float, default=40.0,
                        help="Abort if yaw error exceeds this")
    parser.add_argument("--min-speed-distance", type=float, default=0.12,
                        help="Only enforce min speed farther than this from endpoint")
    parser.add_argument("--settle-time", type=float, default=0.35,
                        help="Seconds inside endpoint tolerances before completing leg")
    parser.add_argument("--timeout", type=float, default=35.0,
                        help="Maximum seconds per leg")
    parser.add_argument("--stuck-timeout", type=float, default=8.0,
                        help="Abort if along-track error does not improve for this long")
    parser.add_argument("--progress-epsilon", type=float, default=0.01,
                        help="Minimum improvement counted as odom progress, metres")
    parser.add_argument("--odom-timeout", type=float, default=0.5,
                        help="Abort if /odom callback is stale for this long")
    parser.add_argument("--pause", type=float, default=1.0,
                        help="Pause between legs, seconds")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Control loop rate, Hz")
    parser.add_argument("--start-delay", type=float, default=0.0,
                        help="Seconds to wait before moving")
    parser.add_argument("--start-at-epoch", type=float, default=0.0,
                        help="Unix epoch base time; start-delay is added")
    parser.add_argument("--max-start-late", type=float, default=3.0,
                        help="Abort if scheduled start is missed by this much")
    parser.add_argument("--disable-lateral", action="store_true",
                        help="Do not use cmd_vel.linear.y lateral correction")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Start without pressing Enter")
    parser.add_argument("--verbose", action="store_true",
                        help="Print periodic odom feedback")
    parser.add_argument("--report-period", type=float, default=2.0)
    return parser.parse_args(argv)


def normalise_args(args):
    args.distance = max(0.0, args.distance)
    args.cycles = max(1, args.cycles)
    args.max_speed = clamp(abs(args.max_speed), 0.0, 1.0)
    args.min_speed = clamp(abs(args.min_speed), 0.0, args.max_speed)
    args.max_lateral = clamp(abs(args.max_lateral), 0.0, 1.0)
    args.max_xy_command = clamp(abs(args.max_xy_command), 0.01, 1.0)
    args.along_kp = max(0.0, args.along_kp)
    args.cross_track_kp = max(0.0, args.cross_track_kp)
    args.heading_kp = max(0.0, args.heading_kp)
    args.max_angular = clamp(abs(args.max_angular), 0.0, 1.0)
    args.distance_tolerance = max(0.0, args.distance_tolerance)
    args.cross_track_tolerance = max(0.0, args.cross_track_tolerance)
    args.max_cross_track = max(args.cross_track_tolerance, args.max_cross_track)
    args.heading_tolerance = math.radians(max(0.0, args.heading_tolerance_deg))
    args.heading_gate = math.radians(max(args.heading_tolerance_deg, args.heading_gate_deg))
    args.max_heading_error = math.radians(max(args.heading_gate_deg, args.max_heading_error_deg))
    args.min_speed_distance = max(args.distance_tolerance, args.min_speed_distance)
    args.settle_time = max(0.0, args.settle_time)
    args.timeout = max(0.1, args.timeout)
    args.stuck_timeout = max(0.1, args.stuck_timeout)
    args.progress_epsilon = max(0.0, args.progress_epsilon)
    args.odom_timeout = max(0.05, args.odom_timeout)
    args.pause = max(0.0, args.pause)
    args.rate = max(5.0, args.rate)
    args.start_delay = max(0.0, args.start_delay)
    args.start_at_epoch = max(0.0, args.start_at_epoch)
    args.max_start_late = max(0.0, args.max_start_late)
    args.report_period = max(0.5, args.report_period)
    args.use_lateral = not args.disable_lateral
    return args


def main(argv):
    args = normalise_args(parse_args(argv))

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rospy.init_node("agv_odom_shuttle")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=10.0)
    publish_zero(pub)

    origin = (pose[0], pose[1])
    lane_yaw = pose[2]
    print("Odom shuttle ready:")
    print("  origin=(%.3f, %.3f) lane_yaw=%.1fdeg" %
          (origin[0], origin[1], math.degrees(lane_yaw)))
    print("  distance=%.2fm cycles=%d max_speed=%.3f max_lateral=%.3f lateral=%s" %
          (args.distance, args.cycles, args.max_speed, args.max_lateral, args.use_lateral))
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

            print("Cycle %d/%d: out" % (cycle + 1, args.cycles))
            if not drive_to_target(pub, args, origin, lane_yaw, args.distance, leg_index):
                break
            leg_index += 1
            rospy.sleep(args.pause)

            if rospy.is_shutdown() or stop_requested:
                break

            print("Cycle %d/%d: back" % (cycle + 1, args.cycles))
            if not drive_to_target(pub, args, origin, lane_yaw, 0.0, leg_index):
                break
            leg_index += 1
            rospy.sleep(args.pause)
    finally:
        publish_zero(pub, seconds=1.0)
        print("Odom shuttle finished; zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
