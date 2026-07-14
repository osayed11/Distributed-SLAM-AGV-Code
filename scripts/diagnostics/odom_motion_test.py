#!/usr/bin/env python3
"""Open-loop ROS2 odometry response test for the myAGV base.

This is a calibration/debug tool, not a dataset collection command. It
intentionally commands fixed-duration straight/turn pulses and reports the
resulting `/odom` delta so the operator can compare odometry with tape marks or
angle marks.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.signals import SignalHandlerOptions


node = None
pose: tuple[float, float, float] | None = None
yaw_unwrapped: float | None = None
yaw_last: float | None = None
stop_requested = False


def request_stop(signum=None, frame=None) -> None:
    global stop_requested
    stop_requested = True
    if signum is not None:
        print("\nStop requested; sending zero velocity.")


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(end_yaw: float, start_yaw: float) -> float:
    return math.atan2(math.sin(end_yaw - start_yaw), math.cos(end_yaw - start_yaw))


def odom_cb(msg: Odometry) -> None:
    global pose, yaw_unwrapped, yaw_last
    p = msg.pose.pose.position
    yaw = yaw_from_quat(msg.pose.pose.orientation)
    if yaw_last is None or yaw_unwrapped is None:
        yaw_unwrapped = yaw
    else:
        yaw_unwrapped += angle_delta(yaw, yaw_last)
    yaw_last = yaw
    pose = (p.x, p.y, yaw)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wait_for_odom(timeout: float) -> None:
    start = time.time()
    while rclpy.ok() and pose is None and not stop_requested:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for odometry")
        time.sleep(0.05)


def publish_zero(pub, seconds: float = 1.0) -> None:
    msg = Twist()
    end = time.time() + seconds
    while time.time() < end:
        pub.publish(msg)
        time.sleep(0.05)


def command_for_duration(pub, linear_x: float, angular_z: float, seconds: float) -> None:
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    start = time.time()
    while rclpy.ok() and not stop_requested and time.time() - start < seconds:
        pub.publish(msg)
        time.sleep(0.05)
    publish_zero(pub, seconds=1.0)


def straight_test(pub, distance_m: float, speed_mps: float) -> float:
    if pose is None or yaw_unwrapped is None:
        raise RuntimeError("No odometry pose available")
    start_x, start_y, start_yaw = pose
    start_yaw_unwrapped = yaw_unwrapped
    duration = abs(distance_m / speed_mps)
    command = math.copysign(abs(speed_mps), distance_m)

    print(
        "STRAIGHT_START x={:.4f} y={:.4f} yaw_deg={:.2f}".format(
            start_x, start_y, math.degrees(start_yaw)
        )
    )
    print(
        "STRAIGHT_COMMAND linear_x={:.3f} duration_s={:.2f} expected_m={:.3f}".format(
            command, duration, distance_m
        )
    )

    command_for_duration(pub, command, 0.0, duration)
    if pose is None or yaw_unwrapped is None:
        raise RuntimeError("No odometry pose available after straight test")
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

    print(
        "STRAIGHT_RESULT odom_projected_m={:.4f} odom_planar_m={:.4f} "
        "lateral_m={:.4f} yaw_drift_deg={:.2f} yaw_drift_unwrapped_deg={:.2f}".format(
            projected,
            planar,
            lateral,
            math.degrees(yaw_drift),
            math.degrees(yaw_drift_unwrapped),
        )
    )
    return projected


def turn_test(pub, turn_deg: float, angular_rps: float) -> float:
    if pose is None or yaw_unwrapped is None:
        raise RuntimeError("No odometry pose available")
    start_x, start_y, start_yaw = pose
    start_yaw_unwrapped = yaw_unwrapped
    target_rad = math.radians(turn_deg)
    duration = abs(target_rad / angular_rps)
    command = math.copysign(abs(angular_rps), target_rad)

    print(
        "TURN_START x={:.4f} y={:.4f} yaw_deg={:.2f}".format(
            start_x, start_y, math.degrees(start_yaw)
        )
    )
    print(
        "TURN_COMMAND angular_z={:.3f} duration_s={:.2f} expected_deg={:.2f}".format(
            command, duration, turn_deg
        )
    )

    command_for_duration(pub, 0.0, command, duration)
    if pose is None or yaw_unwrapped is None:
        raise RuntimeError("No odometry pose available after turn test")
    end_x, end_y, end_yaw = pose

    dx = end_x - start_x
    dy = end_y - start_y
    yaw_delta = angle_delta(end_yaw, start_yaw)
    yaw_delta_unwrapped = yaw_unwrapped - start_yaw_unwrapped

    print(
        "TURN_RESULT odom_yaw_delta_deg={:.2f} odom_yaw_unwrapped_deg={:.2f} "
        "position_drift_m={:.4f} dx={:.4f} dy={:.4f}".format(
            math.degrees(yaw_delta),
            math.degrees(yaw_delta_unwrapped),
            math.hypot(dx, dy),
            dx,
            dy,
        )
    )
    return math.degrees(yaw_delta_unwrapped)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open-loop ROS2 odom response test")
    parser.add_argument("--odom-topic", default="/odom", help="Odometry topic")
    parser.add_argument("--cmd-topic", default="/cmd_vel", help="Velocity command topic")
    parser.add_argument("--distance", type=float, default=0.50, help="Straight command distance in metres")
    parser.add_argument("--linear", type=float, default=0.12, help="Straight command speed")
    parser.add_argument("--turn-deg", type=float, default=90.0, help="Turn command angle in degrees")
    parser.add_argument("--angular", type=float, default=0.15, help="Turn command angular speed")
    parser.add_argument("--pause", type=float, default=2.0, help="Pause between straight and turn tests")
    parser.add_argument("--skip-straight", action="store_true", help="Do not run the straight segment")
    parser.add_argument("--skip-turn", action="store_true", help="Do not run the turn segment")
    parser.add_argument("--clockwise", action="store_true", help="Make the turn clockwise")
    parser.add_argument("--yes", "--no-prompt", action="store_true", dest="yes", help="Start without prompt")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    global node
    args = parse_args(argv)
    args.linear = clamp(abs(args.linear), 0.01, 1.0)
    args.angular = clamp(abs(args.angular), 0.01, 1.0)
    args.pause = max(0.0, args.pause)
    if args.clockwise:
        args.turn_deg = -abs(args.turn_deg)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("agv_odom_motion_test")
    node.create_subscription(Odometry, args.odom_topic, odom_cb, 20)
    pub = node.create_publisher(Twist, args.cmd_topic, 10)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        wait_for_odom(timeout=10.0)
        publish_zero(pub)

        print("Odom motion test ready:")
        print(f"  odom_topic={args.odom_topic} cmd_topic={args.cmd_topic}")
        if not args.skip_straight:
            print(f"  straight: distance={args.distance:.3f}m linear={args.linear:.3f}")
        if not args.skip_turn:
            print(f"  turn: angle={args.turn_deg:.2f}deg angular={args.angular:.3f}")
        print("  Ctrl+C sends zero velocity.")

        if not args.yes:
            input("Press Enter only when the robot has clear space...")

        if not args.skip_straight and not stop_requested:
            straight_test(pub, args.distance, args.linear)
            time.sleep(args.pause)
        if not args.skip_turn and not stop_requested:
            turn_test(pub, args.turn_deg, args.angular)
    finally:
        try:
            publish_zero(pub, seconds=1.5)
        except Exception:
            pass
        print("Odom motion test finished; zero velocity sent.")
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
