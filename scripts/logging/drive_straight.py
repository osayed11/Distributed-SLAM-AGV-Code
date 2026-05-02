#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Drive one odom-bounded straight-line segment on /cmd_vel.

Use this for clean straight-line dataset sequences. The script waits for /odom,
prints a start prompt by default, drives until odometry reaches the requested
distance, then publishes zero velocity before exiting.
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


def odom_cb(msg):
    global pose
    p = msg.pose.pose.position
    pose = (p.x, p.y)


def clamp(value, low, high):
    return max(low, min(high, value))


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wait_for_odom(timeout):
    start = time.time()
    while not rospy.is_shutdown() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for /odom")
        rospy.sleep(0.05)


def publish_zero(pub, seconds=0.8):
    msg = Twist()
    rate = rospy.Rate(20)
    end = time.time() + seconds
    while not rospy.is_shutdown() and time.time() < end:
        pub.publish(msg)
        rate.sleep()


def drive(pub, target_distance, speed, timeout):
    start_pose = pose
    max_seen = 0.0
    msg = Twist()
    msg.linear.x = speed
    rate = rospy.Rate(20)
    start = time.time()

    while not rospy.is_shutdown():
        travelled = distance(pose, start_pose)
        max_seen = max(max_seen, travelled)
        if travelled >= target_distance:
            break
        if time.time() - start >= timeout:
            print("WARN timeout after %.1fs; travelled %.3fm" %
                  (timeout, travelled))
            break
        pub.publish(msg)
        rate.sleep()

    publish_zero(pub)
    travelled = distance(pose, start_pose)
    print("Straight segment complete: travelled=%.3fm max=%.3fm" %
          (travelled, max_seen))


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Drive one straight segment")
    parser.add_argument("--distance", type=float, default=1.50,
                        help="Target odometry distance in metres")
    parser.add_argument("--speed", type=float, default=0.18,
                        help="Normalized forward command, clamped to [-1, 1]")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="Maximum seconds before stopping")
    parser.add_argument("--reverse", action="store_true",
                        help="Drive backwards instead of forwards")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Start immediately without pressing Enter")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.distance = max(0.0, args.distance)
    speed = clamp(abs(args.speed), 0.0, 1.0)
    if args.reverse:
        speed = -speed

    rospy.init_node("agv_drive_straight")
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=10.0)
    publish_zero(pub)

    print("Straight drive ready:")
    print("  distance=%.2fm speed=%.2f timeout=%.1fs" %
          (args.distance, speed, args.timeout))
    print("  Ctrl+C stops the robot.")
    if not args.no_prompt:
        try:
            raw_input("Press Enter to start, or Ctrl+C to cancel...")
        except NameError:
            input("Press Enter to start, or Ctrl+C to cancel...")

    try:
        drive(pub, args.distance, speed, args.timeout)
    finally:
        publish_zero(pub, seconds=1.0)
        print("Zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
