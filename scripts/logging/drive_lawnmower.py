#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Simple Timed Shuttle Driver with Dual-Bias.

Drives forward for X seconds, stops, and drives back for X seconds.
Optimized for immediate execution with direction-specific bias support.
"""

from __future__ import print_function
import argparse
import signal
import sys
import time
import rospy
from geometry_msgs.msg import Twist

stop_requested = False

def request_stop(signum=None, frame=None):
    global stop_requested
    stop_requested = True
    if signum is not None:
        print("\nStop requested; sending zero velocity.")

def publish_zero(pub, seconds=1.0):
    msg = Twist()
    rate = rospy.Rate(20)
    end = time.time() + seconds
    while time.time() < end and not rospy.is_shutdown():
        pub.publish(msg)
        try: rate.sleep()
        except: break

def drive_segment(pub, args, direction, segment_index, current_bias):
    rate = rospy.Rate(args.rate)
    start_time = time.time()
    
    print("Segment %d starting (%s) for %.1fs (bias: %.4f)..." % (
        segment_index, "fwd" if direction > 0 else "rev", args.duration, current_bias))

    while not rospy.is_shutdown() and not stop_requested:
        elapsed = time.time() - start_time
        if elapsed >= args.duration:
            break

        msg = Twist()
        msg.linear.x = direction * args.linear
        msg.angular.z = current_bias
        pub.publish(msg)
        rate.sleep()

    publish_zero(pub)
    print("Segment %d done." % segment_index)

def drive_shuttle(pub, args):
    # Use rev_bias if provided, otherwise fallback to regular bias
    rev_bias = args.rev_bias if args.rev_bias is not None else args.bias
    
    for c in range(args.cycles):
        if stop_requested: break
        # Forward
        drive_segment(pub, args, 1.0, 2*c+1, args.bias)
        if stop_requested: break
        time.sleep(args.pause)
        
        # Reverse
        drive_segment(pub, args, -1.0, 2*c+2, rev_bias)
        if stop_requested: break
        time.sleep(args.pause)

def main(argv):
    parser = argparse.ArgumentParser(description="Timed Shuttle")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--linear", type=float, default=0.15)
    parser.add_argument("--bias", type=float, default=0.0, help="Angular bias for forward leg")
    parser.add_argument("--rev-bias", type=float, default=None, help="Angular bias for reverse leg (defaults to --bias)")
    parser.add_argument("--pause", type=float, default=1.5)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--no-prompt", action="store_true")
    args = parser.parse_args(argv)

    rospy.init_node("drive_shuttle_timed")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    signal.signal(signal.SIGINT, request_stop)
    
    if not args.no_prompt:
        print("Ready. Press Enter...")
        try: raw_input()
        except: input()
    
    drive_shuttle(pub, args)
    publish_zero(pub)
    print("Finished.")

if __name__ == "__main__":
    main(sys.argv[1:])