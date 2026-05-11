#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Drive a continuous odom-feedback circle on /cmd_vel.

Use this for S1 concentric-circle dataset collection. Place the robot on its
ring, point it tangentially, then run with the desired radius. Ctrl+C publishes
zero velocity before exiting.
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
    end = time.time() + seconds
    rate = rospy.Rate(20)
    while time.time() < end:
        try:
            pub.publish(msg)
            rate.sleep()
        except rospy.ROSException:
            break
        except rospy.ROSInterruptException:
            time.sleep(0.05)


def wait_before_motion(pub, args):
    if args.start_at_epoch > 0.0:
        target_epoch = args.start_at_epoch + args.start_delay
        print(
            "Waiting for scheduled start epoch %.3f (base %.3f + delay %.1fs)."
            % (target_epoch, args.start_at_epoch, args.start_delay)
        )
    elif args.start_delay > 0.0:
        target_epoch = time.time() + args.start_delay
        print("Waiting %.1fs before moving..." % args.start_delay)
    else:
        return

    msg = Twist()
    rate = rospy.Rate(10)
    last_report = 0.0
    while not rospy.is_shutdown() and not stop_requested:
        remaining = target_epoch - time.time()
        if remaining <= 0.0:
            break
        pub.publish(msg)
        now = time.time()
        if args.verbose and (now - last_report >= 10.0 or remaining <= 10.0):
            print("start wait: %.1fs remaining" % remaining)
            last_report = now
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            time.sleep(0.05)

    if stop_requested:
        return

    late_by = time.time() - target_epoch
    if late_by > 1.0:
        print("Scheduled start time passed %.1fs ago; starting immediately." % late_by)


def wait_for_odom(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for /odom")
        rospy.sleep(0.05)


def circle_center(start_pose, radius, turn_sign):
    x, y, yaw = start_pose
    left_x = -math.sin(yaw)
    left_y = math.cos(yaw)
    return (
        x + turn_sign * radius * left_x,
        y + turn_sign * radius * left_y,
    )


def drive_circle(pub, args):
    start_pose = pose
    center_x, center_y = circle_center(start_pose, args.radius, args.turn_sign)
    feedforward = args.turn_sign * args.linear / args.radius
    feedforward = clamp(feedforward, -args.max_angular, args.max_angular)
    msg = Twist()
    rate = rospy.Rate(args.rate)
    start_time = time.time()
    last_report = start_time

    x, y, _ = pose
    last_theta = math.atan2(y - center_y, x - center_x)
    progress = 0.0
    max_abs_radius_error = 0.0
    max_abs_heading_error = 0.0

    while not rospy.is_shutdown() and not stop_requested:
        now = time.time()
        if args.duration > 0.0 and now - start_time >= args.duration:
            print("Duration reached; stopping circle.")
            break

        x, y, yaw = pose
        radial_x = x - center_x
        radial_y = y - center_y
        current_radius = math.hypot(radial_x, radial_y)
        if current_radius < 1e-6:
            rate.sleep()
            continue

        theta = math.atan2(radial_y, radial_x)
        progress += args.turn_sign * angle_delta(theta, last_theta)
        last_theta = theta

        radius_error = current_radius - args.radius
        max_abs_radius_error = max(max_abs_radius_error, abs(radius_error))

        tangent_yaw = theta + args.turn_sign * math.pi * 0.5
        radius_heading_offset = args.turn_sign * clamp(
            args.radius_kp * radius_error,
            -args.max_radius_heading_offset,
            args.max_radius_heading_offset,
        )
        target_yaw = tangent_yaw + radius_heading_offset
        heading_error = angle_delta(target_yaw, yaw)
        max_abs_heading_error = max(max_abs_heading_error, abs(heading_error))

        msg.linear.x = args.linear
        msg.angular.z = clamp(
            feedforward + args.heading_kp * heading_error,
            -args.max_angular,
            args.max_angular,
        )
        pub.publish(msg)

        if args.verbose and now - last_report >= args.report_period:
            laps = progress / (2.0 * math.pi)
            print(
                "circle: elapsed=%.1fs laps=%.2f radius=%.3fm error=%.3fm heading=%.1fdeg cmd=(%.3f, %.3f)"
                % (
                    now - start_time,
                    laps,
                    current_radius,
                    radius_error,
                    math.degrees(heading_error),
                    msg.linear.x,
                    msg.angular.z,
                )
            )
            last_report = now

        rate.sleep()

    publish_zero(pub)
    elapsed = time.time() - start_time
    print(
        "Circle complete: elapsed=%.1fs laps=%.2f max_radius_error=%.3fm max_heading_error=%.1fdeg"
        % (
            elapsed,
            progress / (2.0 * math.pi),
            max_abs_radius_error,
            math.degrees(max_abs_heading_error),
        )
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Drive a continuous circle on /cmd_vel")
    parser.add_argument("--radius", type=float, required=True,
                        help="Circle radius in odom metres")
    parser.add_argument("--linear", type=float, default=0.16,
                        help="Forward command, clamped to [0, 1]")
    parser.add_argument("--max-angular", type=float, default=0.45,
                        help="Maximum angular command magnitude")
    parser.add_argument("--heading-kp", type=float, default=0.75,
                        help="P gain from tangent heading error to angular command")
    parser.add_argument("--radius-kp", type=float, default=0.75,
                        help="Heading offset gain from radius error")
    parser.add_argument("--max-radius-heading-offset-deg", type=float, default=18.0,
                        help="Maximum inward/outward heading offset")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Run duration in seconds; 0 means until Ctrl+C")
    parser.add_argument("--start-delay", type=float, default=0.0,
                        help="Seconds to wait after prompt before moving")
    parser.add_argument("--start-at-epoch", type=float, default=0.0,
                        help="Unix epoch base time for scheduled start; start-delay is added")
    parser.add_argument("--counter-clockwise", action="store_true",
                        help="Drive counter-clockwise instead of clockwise")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Control loop rate in Hz")
    parser.add_argument("--report-period", type=float, default=5.0,
                        help="Verbose report period in seconds")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Start immediately without pressing Enter")
    parser.add_argument("--verbose", action="store_true",
                        help="Print periodic odom feedback")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.radius = max(0.10, args.radius)
    args.linear = clamp(abs(args.linear), 0.0, 1.0)
    args.max_angular = clamp(abs(args.max_angular), 0.02, 1.0)
    args.heading_kp = max(0.0, args.heading_kp)
    args.radius_kp = max(0.0, args.radius_kp)
    args.max_radius_heading_offset = math.radians(
        max(0.0, args.max_radius_heading_offset_deg)
    )
    args.duration = max(0.0, args.duration)
    args.start_delay = max(0.0, args.start_delay)
    args.start_at_epoch = max(0.0, args.start_at_epoch)
    args.rate = max(5.0, args.rate)
    args.report_period = max(1.0, args.report_period)
    args.turn_sign = 1.0 if args.counter_clockwise else -1.0

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rospy.init_node("agv_drive_circle")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=10.0)
    publish_zero(pub)

    direction = "counter-clockwise" if args.counter_clockwise else "clockwise"
    print("Circle drive ready:")
    print("  radius=%.2fm direction=%s linear=%.2f max_angular=%.2f duration=%.1fs" %
          (args.radius, direction, args.linear, args.max_angular, args.duration))
    if args.start_at_epoch > 0.0:
        print(
            "  start_at_epoch=%.3f start_delay=%.1fs scheduled_start=%.3f. Ctrl+C stops the robot."
            % (
                args.start_at_epoch,
                args.start_delay,
                args.start_at_epoch + args.start_delay,
            )
        )
    else:
        print("  start_delay=%.1fs. Ctrl+C stops the robot." % args.start_delay)
    if not args.no_prompt:
        try:
            raw_input("Press Enter to start, or Ctrl+C to cancel...")
        except NameError:
            input("Press Enter to start, or Ctrl+C to cancel...")

    wait_before_motion(pub, args)

    try:
        if not stop_requested:
            drive_circle(pub, args)
    finally:
        publish_zero(pub, seconds=1.0)
        print("Circle drive finished; zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
