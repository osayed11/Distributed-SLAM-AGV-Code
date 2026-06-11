#!/usr/bin/env python3
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
import threading

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

_node = None
stop_requested = False


def request_stop(signum=None, frame=None):
    global stop_requested
    stop_requested = True
    if signum is not None:
        print("\nStop requested; sending zero velocity.")


def publish_zero(pub, seconds=1.0):
    msg = Twist()
    end = time.time() + seconds
    while time.time() < end:
        try:
            pub.publish(msg)
        except Exception:
            pass
        time.sleep(0.05)


def drive_segment(pub, args, direction, segment_index, current_bias):
    _dt = 1.0 / args.rate
    start_time = time.time()
    _next_iter = start_time + _dt

    print("Segment %d starting (%s) for %.1fs (bias: %.4f)..." % (
        segment_index, "fwd" if direction > 0 else "rev", args.duration, current_bias))

    while rclpy.ok() and not stop_requested:
        elapsed = time.time() - start_time
        if elapsed >= args.duration:
            break

        msg = Twist()
        msg.linear.x = direction * args.linear
        msg.angular.z = current_bias
        pub.publish(msg)
        _next_iter += _dt
        sleep_dur = _next_iter - time.time()
        if sleep_dur > 0:
            time.sleep(sleep_dur)
        elif sleep_dur < -_dt:
            _next_iter = time.time() + _dt

    publish_zero(pub)
    print("Segment %d done." % segment_index)


def drive_shuttle(pub, args):
    rev_bias = args.rev_bias if args.rev_bias is not None else args.bias

    for c in range(args.cycles):
        if stop_requested:
            break
        drive_segment(pub, args, 1.0, 2 * c + 1, args.bias)
        if stop_requested:
            break
        time.sleep(args.pause)

        drive_segment(pub, args, -1.0, 2 * c + 2, rev_bias)
        if stop_requested:
            break
        time.sleep(args.pause)


def main(argv):
    global _node

    parser = argparse.ArgumentParser(description="Timed Shuttle")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--linear", type=float, default=0.15)
    parser.add_argument("--bias", type=float, default=0.0,
                        help="Angular bias for forward leg")
    parser.add_argument("--rev-bias", type=float, default=None,
                        help="Angular bias for reverse leg (defaults to --bias)")
    parser.add_argument("--pause", type=float, default=1.5)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--no-prompt", action="store_true")
    args = parser.parse_args(argv)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    _node = rclpy.create_node("drive_shuttle_timed")

    def _spin():
        try:
            rclpy.spin(_node)
        except Exception:
            pass

    _spin_thread = threading.Thread(target=_spin, daemon=True)
    _spin_thread.start()

    pub = _node.create_publisher(Twist, "/cmd_vel", 1)

    if not args.no_prompt:
        print("Ready. Press Enter...")
        input()

    try:
        drive_shuttle(pub, args)
        publish_zero(pub)
        print("Finished.")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main(sys.argv[1:])
