#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Open-loop odometry response test for the myAGV base.

This intentionally commands motion for a fixed duration instead of stopping on
/odom. That makes the final /odom delta useful for checking whether odometry is
scaled plausibly against the commanded motion and a physical tape/angle mark.
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
yaw_unwrapped = None
yaw_last = None


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_cb(msg):
    global pose, yaw_unwrapped, yaw_last
    p = msg.pose.pose.position
    yaw = yaw_from_quat(msg.pose.pose.orientation)
    if yaw_unwrapped is None:
        yaw_unwrapped = yaw
    else:
        yaw_unwrapped += angle_delta(yaw, yaw_last)
    yaw_last = yaw
    pose = (p.x, p.y, yaw)


def angle_delta(end_yaw, start_yaw):
    return math.atan2(math.sin(end_yaw - start_yaw),
                      math.cos(end_yaw - start_yaw))


def clamp(value, low, high):
    return max(low, min(high, value))


def wait_for_odom(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for /odom")
        rospy.sleep(0.05)


def publish_zero(pub, seconds=1.0):
    msg = Twist()
    rate = rospy.Rate(20)
    end = time.time() + seconds
    while not rospy.is_shutdown() and time.time() < end:
        pub.publish(msg)
        rate.sleep()


def command_for_duration(pub, linear_x, angular_z, seconds):
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    rate = rospy.Rate(20)
    start = time.time()
    while not rospy.is_shutdown() and time.time() - start < seconds:
        pub.publish(msg)
        rate.sleep()
    publish_zero(pub, seconds=1.0)


def straight_test(pub, distance_m, speed_mps):
    start_x, start_y, start_yaw = pose
    start_yaw_unwrapped = yaw_unwrapped
    duration = abs(distance_m / speed_mps)
    command = math.copysign(abs(speed_mps), distance_m)

    print("STRAIGHT_START x={:.4f} y={:.4f} yaw_deg={:.2f}".format(
        start_x, start_y, math.degrees(start_yaw)))
    print("STRAIGHT_COMMAND linear_x={:.3f} duration_s={:.2f} expected_m={:.3f}".format(
        command, duration, distance_m))

    command_for_duration(pub, command, 0.0, duration)
    end_x, end_y, end_yaw = pose

    dx = end_x - start_x
    dy = end_y - start_y
    forward_x = math.cos(start_yaw)
    forward_y = math.sin(start_yaw)
    projected = dx * forward_x + dy * forward_y
    lateral = -dx * forward_y + dy * forward_x
    planar = math.hypot(dx, dy)
    yaw_drift = angle_delta(end_yaw, start_yaw)
    yaw_drift_unwrapped = yaw_unwrapped - start_yaw_unwrapped

    print("STRAIGHT_RESULT odom_projected_m={:.4f} odom_planar_m={:.4f} lateral_m={:.4f} yaw_drift_deg={:.2f} yaw_drift_unwrapped_deg={:.2f}".format(
        projected, planar, lateral, math.degrees(yaw_drift),
        math.degrees(yaw_drift_unwrapped)))
    return projected


def turn_test(pub, turn_deg, angular_rps):
    start_x, start_y, start_yaw = pose
    start_yaw_unwrapped = yaw_unwrapped
    target_rad = math.radians(turn_deg)
    duration = abs(target_rad / angular_rps)
    command = math.copysign(abs(angular_rps), target_rad)

    print("TURN_START x={:.4f} y={:.4f} yaw_deg={:.2f}".format(
        start_x, start_y, math.degrees(start_yaw)))
    print("TURN_COMMAND angular_z={:.3f} duration_s={:.2f} expected_deg={:.2f}".format(
        command, duration, turn_deg))

    command_for_duration(pub, 0.0, command, duration)
    end_x, end_y, end_yaw = pose

    dx = end_x - start_x
    dy = end_y - start_y
    yaw_delta = angle_delta(end_yaw, start_yaw)
    yaw_delta_unwrapped = yaw_unwrapped - start_yaw_unwrapped

    print("TURN_RESULT odom_yaw_delta_deg={:.2f} odom_yaw_unwrapped_deg={:.2f} position_drift_m={:.4f} dx={:.4f} dy={:.4f}".format(
        math.degrees(yaw_delta), math.degrees(yaw_delta_unwrapped),
        math.hypot(dx, dy), dx, dy))
    return math.degrees(yaw_delta_unwrapped)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Open-loop odom response test")
    parser.add_argument("--distance", type=float, default=0.50,
                        help="Nominal straight command distance in metres")
    parser.add_argument("--linear", type=float, default=0.12,
                        help="Forward command used for straight test")
    parser.add_argument("--turn-deg", type=float, default=90.0,
                        help="Nominal turn command in degrees")
    parser.add_argument("--angular", type=float, default=0.15,
                        help="Angular command used for turn test")
    parser.add_argument("--pause", type=float, default=2.0,
                        help="Pause between straight and turn tests")
    parser.add_argument("--skip-straight", action="store_true",
                        help="Do not run the straight segment")
    parser.add_argument("--skip-turn", action="store_true",
                        help="Do not run the turn segment")
    parser.add_argument("--clockwise", action="store_true",
                        help="Make the turn clockwise")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Start immediately without pressing Enter")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.linear = clamp(abs(args.linear), 0.01, 1.0)
    args.angular = clamp(abs(args.angular), 0.01, 1.0)
    if args.clockwise:
        args.turn_deg = -abs(args.turn_deg)

    rospy.init_node("agv_odom_motion_test")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=10.0)
    publish_zero(pub)

    print("Odom motion test ready:")
    if not args.skip_straight:
        print("  straight: distance={:.3f}m linear={:.3f}".format(
            args.distance, args.linear))
    if not args.skip_turn:
        print("  turn: angle={:.2f}deg angular={:.3f}".format(
            args.turn_deg, args.angular))
    print("  Ctrl+C sends zero velocity.")

    if not args.no_prompt:
        try:
            raw_input("Press Enter only when the robot has clear space...")
        except NameError:
            input("Press Enter only when the robot has clear space...")

    try:
        if not args.skip_straight:
            straight_test(pub, args.distance, args.linear)
            rospy.sleep(args.pause)
        if not args.skip_turn:
            turn_test(pub, args.turn_deg, args.angular)
    finally:
        publish_zero(pub, seconds=1.5)
        print("Odom motion test finished; zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
