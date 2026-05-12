#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Drive one mocap-bounded straight segment on /cmd_vel.

Closes the loop on PhaseSpace ground truth (`/phasespace/rigids`) instead of
wheel odometry. Distance is the forward projection of the rigid body's
displacement onto its initial heading vector. Heading is held with a P
controller on yaw error so the robot tracks a straight line in the mocap
frame.

Run prerequisites:
  - phasespace_bringup running on the mocap PC, publishing /phasespace/rigids
  - This robot's ROS_MASTER_URI points at that mocap PC
  - The robot's rigid body id is known (--rigid-id)
"""

from __future__ import print_function

import argparse
import math
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from phasespace_msgs.msg import Rigids


state = {"x": None, "y": None, "yaw": None, "stamp": 0.0}


def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, low, high):
    return max(low, min(high, value))


def make_rigids_cb(rigid_id):
    def cb(msg):
        for r in msg.rigids:
            if r.id != rigid_id:
                continue
            # cond <= 0 means tracking lost / invalid frame for that rigid
            if r.cond <= 0.0:
                continue
            state["x"] = r.x
            state["y"] = r.y
            state["yaw"] = yaw_from_quat(r.qx, r.qy, r.qz, r.qw)
            state["stamp"] = time.time()
            return
    return cb


def wait_for_rigid(timeout):
    start = time.time()
    while not rospy.is_shutdown() and state["x"] is None:
        if time.time() - start > timeout:
            raise RuntimeError(
                "Timed out waiting for rigid on /phasespace/rigids")
        rospy.sleep(0.05)


def publish_zero(pub, seconds=0.8):
    msg = Twist()
    rate = rospy.Rate(20)
    end = time.time() + seconds
    while not rospy.is_shutdown() and time.time() < end:
        pub.publish(msg)
        rate.sleep()


def drive(pub, target_distance, speed, timeout, kp_yaw, max_yaw_rate,
          stale_thresh, yaw_sign):
    x0, y0, yaw0 = state["x"], state["y"], state["yaw"]
    cos0, sin0 = math.cos(yaw0), math.sin(yaw0)
    max_seen = 0.0
    rate = rospy.Rate(20)
    start = time.time()
    reason = "unknown"

    while not rospy.is_shutdown():
        if time.time() - state["stamp"] > stale_thresh:
            reason = "mocap_stale"
            print("WARN mocap stale > %.2fs; stopping" % stale_thresh)
            break

        dx = state["x"] - x0
        dy = state["y"] - y0
        forward = dx * cos0 + dy * sin0
        lateral = -dx * sin0 + dy * cos0
        max_seen = max(max_seen, forward)

        if forward >= target_distance:
            reason = "reached_distance"
            break
        if time.time() - start >= timeout:
            reason = "timeout"
            print("WARN timeout after %.1fs; forward=%.3fm lateral=%.3fm" %
                  (timeout, forward, lateral))
            break

        yaw_err = wrap_pi(state["yaw"] - yaw0)
        angular = clamp(-yaw_sign * kp_yaw * yaw_err,
                        -max_yaw_rate, max_yaw_rate)

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angular
        pub.publish(msg)
        rate.sleep()

    publish_zero(pub)
    dx = state["x"] - x0
    dy = state["y"] - y0
    forward = dx * cos0 + dy * sin0
    lateral = -dx * sin0 + dy * cos0
    print("Mocap segment done: reason=%s forward=%.3fm lateral=%.3fm "
          "max_forward=%.3fm yaw_drift=%.3frad" %
          (reason, forward, lateral, max_seen,
           wrap_pi(state["yaw"] - yaw0)))


def parse_args(argv):
    p = argparse.ArgumentParser(description="Drive one mocap-bounded segment")
    p.add_argument("--rigid-id", type=int, required=True,
                   help="PhaseSpace rigid body id for this robot")
    p.add_argument("--distance", type=float, default=1.0,
                   help="Target forward distance in metres")
    p.add_argument("--speed", type=float, default=0.10,
                   help="Forward linear command, clamped to [-1, 1]")
    p.add_argument("--timeout", type=float, default=30.0,
                   help="Max seconds before stopping")
    p.add_argument("--kp-yaw", type=float, default=1.2,
                   help="P gain on yaw error (rad/s per rad)")
    p.add_argument("--max-yaw-rate", type=float, default=0.4,
                   help="Saturation on angular.z (rad/s)")
    p.add_argument("--stale-thresh", type=float, default=0.3,
                   help="Stop if no fresh mocap sample for this long (s)")
    p.add_argument("--invert-yaw", action="store_true",
                   help="Flip the sign of yaw correction if the robot "
                        "veers worse with control on")
    p.add_argument("--reverse", action="store_true",
                   help="Drive backwards")
    p.add_argument("--no-prompt", action="store_true",
                   help="Start immediately without pressing Enter")
    return p.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    args.distance = max(0.0, args.distance)
    speed = clamp(abs(args.speed), 0.0, 1.0)
    if args.reverse:
        speed = -speed
    yaw_sign = -1.0 if args.invert_yaw else 1.0

    rospy.init_node("agv_drive_mocap_straight")
    rospy.Subscriber("/phasespace/rigids", Rigids,
                     make_rigids_cb(args.rigid_id), queue_size=50)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    wait_for_rigid(timeout=10.0)
    publish_zero(pub)

    print("Mocap drive ready:")
    print("  rigid_id=%d distance=%.2fm speed=%.2f timeout=%.1fs" %
          (args.rigid_id, args.distance, speed, args.timeout))
    print("  start x=%.3f y=%.3f yaw=%.3frad" %
          (state["x"], state["y"], state["yaw"]))
    print("  Ctrl+C stops the robot.")
    if not args.no_prompt:
        try:
            raw_input("Press Enter to start, or Ctrl+C to cancel...")
        except NameError:
            input("Press Enter to start, or Ctrl+C to cancel...")

    try:
        drive(pub, args.distance, speed, args.timeout,
              args.kp_yaw, args.max_yaw_rate, args.stale_thresh, yaw_sign)
    finally:
        publish_zero(pub, seconds=1.0)
        print("Zero velocity sent.")


if __name__ == "__main__":
    main(sys.argv[1:])
