#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Drive a conservative square using /odom feedback and /cmd_vel commands.

Run this only with the robot on the floor, clear space around it, and an
operator ready to stop it. Ctrl+C publishes zero velocity before exiting.
"""

from __future__ import print_function

import argparse
import math
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


pose = None


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_cb(msg):
    global pose
    p = msg.pose.pose.position
    pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))


def angle_delta(target, current):
    """Shortest signed angle from current to target."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, low, high):
    return max(low, min(high, value))


def clamp_with_min(value, max_abs, min_abs):
    if value == 0.0:
        return 0.0
    value = clamp(value, -max_abs, max_abs)
    if abs(value) < min_abs:
        return math.copysign(min_abs, value)
    return value


def publish_zero(pub, seconds=0.7):
    msg = Twist()
    end = time.time() + seconds
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.time() < end:
        pub.publish(msg)
        rate.sleep()


def wait_for_odom(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for /odom")
        rospy.sleep(0.05)


def drive_side(pub, side_m, max_linear, min_linear, linear_kp,
               heading_kp, max_heading_correction, tolerance_m,
               timeout_s, verbose):
    start_x, start_y, target_yaw = pose
    forward_x = math.cos(target_yaw)
    forward_y = math.sin(target_yaw)
    msg = Twist()
    rate = rospy.Rate(20)
    start = time.time()
    projected = 0.0
    cross_track = 0.0
    heading_error = 0.0

    while not rospy.is_shutdown():
        x, y, yaw = pose
        dx = x - start_x
        dy = y - start_y
        projected = dx * forward_x + dy * forward_y
        cross_track = -dx * forward_y + dy * forward_x
        remaining = side_m - projected
        heading_error = angle_delta(target_yaw, yaw)
        if remaining <= tolerance_m:
            break
        if time.time() - start > timeout_s:
            print("WARN side timeout after %.1fs; projected %.3fm cross %.3fm" %
                  (timeout_s, projected, cross_track))
            break
        msg.linear.x = clamp_with_min(linear_kp * remaining,
                                      max_linear, min_linear)
        msg.angular.z = clamp(heading_kp * heading_error,
                              -max_heading_correction,
                              max_heading_correction)
        pub.publish(msg)
        rate.sleep()

    publish_zero(pub)
    if verbose:
        print("    side feedback: projected=%.3fm cross=%.3fm heading_error=%.1fdeg elapsed=%.1fs" %
              (projected, cross_track, math.degrees(heading_error),
               time.time() - start))


def pose_delta_from(start_pose):
    start_x, start_y, start_yaw = start_pose
    x, y, yaw = pose
    dx = x - start_x
    dy = y - start_y
    forward_x = math.cos(start_yaw)
    forward_y = math.sin(start_yaw)
    along = dx * forward_x + dy * forward_y
    lateral = -dx * forward_y + dy * forward_x
    yaw_error = angle_delta(start_yaw, yaw)
    return along, lateral, yaw_error


def turn(pub, turn_rad, max_angular, min_angular, turn_kp,
         tolerance_rad, timeout_s, verbose):
    _, _, start_yaw = pose
    target_yaw = start_yaw + turn_rad
    msg = Twist()
    rate = rospy.Rate(20)
    start = time.time()
    error = turn_rad

    while not rospy.is_shutdown():
        _, _, yaw = pose
        error = angle_delta(target_yaw, yaw)
        if abs(error) <= tolerance_rad:
            break
        if time.time() - start > timeout_s:
            print("WARN turn timeout after %.1fs; remaining %.1f deg" %
                  (timeout_s, math.degrees(error)))
            break
        msg.angular.z = clamp_with_min(turn_kp * error,
                                       max_angular, min_angular)
        pub.publish(msg)
        rate.sleep()

    publish_zero(pub)
    if verbose:
        print("    turn feedback: remaining=%.1fdeg elapsed=%.1fs" %
              (math.degrees(error), time.time() - start))


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Drive a square on /cmd_vel")
    parser.add_argument("--side", type=float, default=0.50,
                        help="Target side length in odom metres")
    parser.add_argument("--linear", type=float, default=0.12,
                        help="Maximum forward command, clamped to [0, 1]")
    parser.add_argument("--min-linear", type=float, default=0.04,
                        help="Minimum forward command while a side is active")
    parser.add_argument("--angular", type=float, default=0.08,
                        help="Maximum turn command, clamped to [0, 1]")
    parser.add_argument("--min-angular", type=float, default=0.02,
                        help="Minimum turn command while a corner is active")
    parser.add_argument("--linear-kp", type=float, default=0.6,
                        help="P gain from remaining side distance to forward command")
    parser.add_argument("--heading-kp", type=float, default=0.9,
                        help="P gain from heading error to side angular correction")
    parser.add_argument("--turn-kp", type=float, default=0.45,
                        help="P gain from turn yaw error to angular command")
    parser.add_argument("--max-heading-correction", type=float, default=0.08,
                        help="Max angular correction while driving straight")
    parser.add_argument("--distance-tolerance", type=float, default=0.03,
                        help="Side completion tolerance in odom metres")
    parser.add_argument("--yaw-tolerance-deg", type=float, default=2.5,
                        help="Turn completion tolerance in degrees")
    parser.add_argument("--turn-deg", type=float, default=87.0,
                        help="Turn angle per corner")
    parser.add_argument("--cycles", type=int, default=1,
                        help="Number of squares")
    parser.add_argument("--side-timeout", type=float, default=18.0,
                        help="Max seconds per side")
    parser.add_argument("--turn-timeout", type=float, default=18.0,
                        help="Max seconds per turn")
    parser.add_argument("--pause", type=float, default=1.5,
                        help="Pause between moves")
    parser.add_argument("--clockwise", action="store_true",
                        help="Turn clockwise instead of counter-clockwise")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Start immediately without pressing Enter")
    parser.add_argument("--verbose", action="store_true",
                        help="Print final odom feedback for each side and turn")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.side = max(0.0, args.side)
    args.linear = clamp(abs(args.linear), 0.0, 1.0)
    args.min_linear = clamp(abs(args.min_linear), 0.0, args.linear)
    args.angular = clamp(abs(args.angular), 0.0, 1.0)
    args.min_angular = clamp(abs(args.min_angular), 0.0, args.angular)
    args.max_heading_correction = clamp(abs(args.max_heading_correction), 0.0, 1.0)
    args.distance_tolerance = max(0.0, args.distance_tolerance)
    yaw_tolerance = math.radians(max(0.1, args.yaw_tolerance_deg))
    turn_rad = math.radians(args.turn_deg)
    if args.clockwise:
        turn_rad = -turn_rad

    rospy.init_node("agv_drive_square")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=10.0)
    publish_zero(pub)

    print("Square drive ready:")
    print("  side=%.2fm linear<=%.2f angular<=%.2f turn=%.1fdeg cycles=%d" %
          (args.side, args.linear, args.angular, args.turn_deg, args.cycles))
    print("  tolerances: distance=%.2fm yaw=%.1fdeg" %
          (args.distance_tolerance, args.yaw_tolerance_deg))
    print("  Ctrl+C stops the robot.")
    if not args.no_prompt:
        try:
            raw_input("Press Enter to start, or Ctrl+C to cancel...")
        except NameError:
            input("Press Enter to start, or Ctrl+C to cancel...")

    try:
        for cycle in range(args.cycles):
            print("Starting square %d/%d" % (cycle + 1, args.cycles))
            cycle_start_pose = pose
            for side_idx in range(4):
                print("  side %d: forward" % (side_idx + 1))
                drive_side(pub, args.side, args.linear, args.min_linear,
                           args.linear_kp, args.heading_kp,
                           args.max_heading_correction,
                           args.distance_tolerance, args.side_timeout,
                           args.verbose)
                rospy.sleep(args.pause)
                print("  corner %d: turn" % (side_idx + 1))
                turn(pub, turn_rad, args.angular, args.min_angular,
                     args.turn_kp, yaw_tolerance, args.turn_timeout,
                     args.verbose)
                rospy.sleep(args.pause)
            along, lateral, yaw_error = pose_delta_from(cycle_start_pose)
            print("Square %d odom closure: along=%.3fm lateral=%.3fm yaw_error=%.1fdeg" %
                  (cycle + 1, along, lateral, math.degrees(yaw_error)))
    finally:
        publish_zero(pub, seconds=1.0)
        print("Square drive finished; zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
