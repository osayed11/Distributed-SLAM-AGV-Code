#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Small odom-bounded forward/back smoke test for bag validation."""

from __future__ import print_function

import argparse
import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


pose = None


def odom_cb(msg):
    global pose
    p = msg.pose.pose.position
    pose = (p.x, p.y)


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


def move(pub, speed, target_distance, timeout):
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
            print("TIMEOUT speed={:.3f} travelled={:.3f}".format(speed, travelled))
            break
        pub.publish(msg)
        rate.sleep()

    publish_zero(pub)
    travelled = distance(pose, start_pose)
    print("MOVE_DONE speed={:.3f} travelled={:.3f} max={:.3f}".format(
        speed, travelled, max_seen))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=0.20)
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--timeout", type=float, default=4.0)
    args = parser.parse_args()

    rospy.init_node("agv_forward_back_smoke_test", anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=20)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_odom(timeout=8.0)
    publish_zero(pub)

    origin = pose
    print("ORIGIN x={:.3f} y={:.3f}".format(origin[0], origin[1]))
    move(pub, abs(args.speed), args.distance, args.timeout)
    print("AFTER_FORWARD x={:.3f} y={:.3f} dist_from_origin={:.3f}".format(
        pose[0], pose[1], distance(pose, origin)))
    move(pub, -abs(args.speed), args.distance, args.timeout)
    print("FINAL x={:.3f} y={:.3f} dist_from_origin={:.3f}".format(
        pose[0], pose[1], distance(pose, origin)))
    publish_zero(pub, seconds=1.0)


if __name__ == "__main__":
    main()
