#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive a square using ROS2 mocap PoseStamped feedback."""

import argparse
import math
import signal
import sys
import threading
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, Twist


_node = None
pose = None
last_pose_wall_time = 0.0
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


def pose_cb(msg):
    global pose, last_pose_wall_time
    p = msg.pose.position
    pose = (p.x, p.y, p.z, yaw_from_quat(msg.pose.orientation), msg.header.frame_id)
    last_pose_wall_time = time.time()


def angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, low, high):
    return max(low, min(high, value))


def publish_zero(pub, seconds=0.8):
    msg = Twist()
    end = time.time() + seconds
    while rclpy.ok() and time.time() < end:
        pub.publish(msg)
        time.sleep(0.05)


def wait_for_pose(timeout):
    start = time.time()
    while rclpy.ok() and pose is None:
        if time.time() - start > timeout:
            raise RuntimeError("Timed out waiting for mocap pose")
        time.sleep(0.05)


def line_coordinates(current_pose, origin_pose, line_yaw):
    x, y, _, _, _ = current_pose
    ox, oy, _, _, _ = origin_pose
    dx = x - ox
    dy = y - oy

    forward_x = math.cos(line_yaw)
    forward_y = math.sin(line_yaw)
    left_x = -math.sin(line_yaw)
    left_y = math.cos(line_yaw)

    along = dx * forward_x + dy * forward_y
    lateral = dx * left_x + dy * left_y
    return along, lateral


def check_pose_fresh(args):
    age = time.time() - last_pose_wall_time
    if age > args.pose_timeout:
        raise RuntimeError("mocap pose stale for %.3fs" % age)


def run_leg(pub, args, leg_index, line_yaw, target_rigid_yaw):
    origin_pose = pose
    start = time.time()
    next_tick = start
    last_report = 0.0
    best_along = -1e9
    last_progress_time = start
    max_abs_lateral = 0.0
    max_abs_heading = 0.0
    dt = 1.0 / args.rate

    print(
        "Leg %d: start=(%.3f, %.3f) line_yaw=%.1fdeg target=%.3fm"
        % (leg_index + 1, origin_pose[0], origin_pose[1], math.degrees(line_yaw), args.side_length)
    )

    while rclpy.ok() and not stop_requested:
        now = time.time()
        elapsed = now - start
        if args.dry_run and elapsed >= args.dry_run_seconds:
            print("Leg %d dry-run complete." % (leg_index + 1))
            return True
        if elapsed >= args.leg_timeout:
            raise RuntimeError("leg %d timeout %.1fs" % (leg_index + 1, args.leg_timeout))

        check_pose_fresh(args)
        along, lateral = line_coordinates(pose, origin_pose, line_yaw)
        heading_error = angle_delta(target_rigid_yaw, pose[3])
        remaining = args.side_length - along
        max_abs_lateral = max(max_abs_lateral, abs(lateral))
        max_abs_heading = max(max_abs_heading, abs(heading_error))

        if along > best_along + args.progress_epsilon:
            best_along = along
            last_progress_time = now
        elif elapsed > args.progress_grace and now - last_progress_time > args.stuck_timeout:
            raise RuntimeError("leg %d no mocap progress for %.1fs" % (leg_index + 1, args.stuck_timeout))

        if elapsed > args.progress_grace and along < -args.reverse_abort_distance:
            raise RuntimeError("leg %d moved opposite target: along=%.3fm" % (leg_index + 1, along))

        if abs(lateral) > args.max_lateral_error:
            raise RuntimeError(
                "leg %d lateral error %.3fm exceeds %.3fm"
                % (leg_index + 1, lateral, args.max_lateral_error)
            )

        if remaining <= args.position_tolerance:
            print(
                "Leg %d reached: along=%.3fm lateral=%.3fm max_lateral=%.3fm max_heading=%.1fdeg"
                % (
                    leg_index + 1,
                    along,
                    lateral,
                    max_abs_lateral,
                    math.degrees(max_abs_heading),
                )
            )
            publish_zero(pub, seconds=args.pause)
            return True

        linear = min(args.linear, max(args.min_linear, args.linear * remaining / args.slowdown_distance))
        linear = clamp(linear, 0.0, args.linear)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = clamp(args.heading_kp * heading_error, -args.max_angular, args.max_angular)
        if not args.disable_lateral:
            msg.linear.y = clamp(-args.lateral_kp * lateral, -args.max_lateral, args.max_lateral)

        if not args.dry_run:
            pub.publish(msg)

        if args.verbose and now - last_report >= args.report_period:
            print(
                "leg %d: t=%.1fs along=%.3f/%.3f lateral=%.3f heading=%.1fdeg "
                "cmd=(%.3f, %.3f, %.3f)"
                % (
                    leg_index + 1,
                    elapsed,
                    along,
                    args.side_length,
                    lateral,
                    math.degrees(heading_error),
                    msg.linear.x,
                    msg.linear.y,
                    msg.angular.z,
                )
            )
            last_report = now

        next_tick += dt
        sleep_dur = next_tick - time.time()
        if sleep_dur > 0:
            time.sleep(sleep_dur)
        elif sleep_dur < -dt:
            next_tick = time.time() + dt

    raise RuntimeError("stop requested during leg %d" % (leg_index + 1))


def run_turn(pub, args, turn_index, target_rigid_yaw):
    start = time.time()
    next_tick = start
    last_report = 0.0
    stable_count = 0
    tolerance = math.radians(args.turn_tolerance_deg)
    dt = 1.0 / args.rate

    print("Turn %d: target_rigid_yaw=%.1fdeg" % (turn_index + 1, math.degrees(target_rigid_yaw)))

    while rclpy.ok() and not stop_requested:
        now = time.time()
        elapsed = now - start
        if elapsed >= args.turn_timeout:
            raise RuntimeError("turn %d timeout %.1fs" % (turn_index + 1, args.turn_timeout))

        check_pose_fresh(args)
        error = angle_delta(target_rigid_yaw, pose[3])
        if abs(error) <= tolerance:
            stable_count += 1
            if not args.dry_run:
                pub.publish(Twist())
            if stable_count >= args.turn_stable_cycles:
                print("Turn %d reached: error=%.1fdeg" % (turn_index + 1, math.degrees(error)))
                publish_zero(pub, seconds=args.pause)
                return True
        else:
            stable_count = 0

            turn_limit = args.turn_max_angular
            slowdown_angle = math.radians(args.turn_slowdown_angle_deg)
            if slowdown_angle > tolerance:
                turn_limit = max(
                    args.min_turn_angular,
                    args.turn_max_angular * min(1.0, abs(error) / slowdown_angle),
                )

            angular = clamp(args.turn_heading_kp * error, -turn_limit, turn_limit)
            if abs(error) > tolerance and abs(angular) < args.min_turn_angular:
                angular = math.copysign(args.min_turn_angular, error)

            msg = Twist()
            msg.angular.z = angular
            if not args.dry_run:
                pub.publish(msg)

            if args.verbose and now - last_report >= args.report_period:
                print(
                    "turn %d: t=%.1fs current=%.1fdeg target=%.1fdeg error=%.1fdeg cmd_z=%.3f"
                    % (
                        turn_index + 1,
                        elapsed,
                        math.degrees(pose[3]),
                        math.degrees(target_rigid_yaw),
                        math.degrees(error),
                        msg.angular.z,
                    )
                )
                last_report = now

        next_tick += dt
        sleep_dur = next_tick - time.time()
        if sleep_dur > 0:
            time.sleep(sleep_dur)
        elif sleep_dur < -dt:
            next_tick = time.time() + dt

    raise RuntimeError("stop requested during turn %d" % (turn_index + 1))


def run_square(pub, args):
    if not args.dry_run and not args.yes:
        raise RuntimeError("Refusing to move without --yes. Use --dry-run for a non-moving check.")

    wait_for_pose(args.wait_timeout)
    start_pose = pose
    line_yaw_offset = math.radians(args.line_yaw_offset_deg)
    initial_line_yaw = start_pose[3] + line_yaw_offset
    initial_rigid_yaw = start_pose[3]
    turn_step = args.turn_sign * math.pi * 0.5

    print(
        "Mocap square ROS2: topic=%s frame=%s start=(%.3f, %.3f, %.3f) "
        "rigid_yaw=%.1fdeg line_yaw=%.1fdeg side=%.3fm linear=%.3f dry_run=%s"
        % (
            args.pose_topic,
            start_pose[4],
            start_pose[0],
            start_pose[1],
            start_pose[2],
            math.degrees(initial_rigid_yaw),
            math.degrees(initial_line_yaw),
            args.side_length,
            args.linear,
            args.dry_run,
        )
    )

    for leg_index in range(args.legs):
        line_yaw = initial_line_yaw + turn_step * leg_index
        target_rigid_yaw = line_yaw - line_yaw_offset
        run_leg(pub, args, leg_index, line_yaw, target_rigid_yaw)
        if args.dry_run:
            print("Square dry-run complete after first leg.")
            return 0
        if leg_index < args.legs - 1:
            run_turn(pub, args, leg_index, initial_rigid_yaw + turn_step * (leg_index + 1))

    publish_zero(pub)
    print("Mocap square complete.")
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Drive a ROS2 mocap-feedback square")
    parser.add_argument("--pose-topic", default="/optitrack/rigid_bodies/orkar_agv1")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--side-length", type=float, default=1.0)
    parser.add_argument("--legs", type=int, default=4)
    parser.add_argument("--linear", type=float, default=0.08)
    parser.add_argument("--min-linear", type=float, default=0.03)
    parser.add_argument("--slowdown-distance", type=float, default=0.35)
    parser.add_argument("--heading-kp", type=float, default=0.55)
    parser.add_argument("--turn-heading-kp", type=float, default=0.85)
    parser.add_argument("--max-angular", type=float, default=0.28)
    parser.add_argument("--turn-max-angular", type=float, default=0.18)
    parser.add_argument("--min-turn-angular", type=float, default=0.035)
    parser.add_argument("--turn-slowdown-angle-deg", type=float, default=25.0)
    parser.add_argument("--lateral-kp", type=float, default=0.0)
    parser.add_argument("--max-lateral", type=float, default=0.05)
    parser.add_argument("--disable-lateral", action="store_true", default=True)
    parser.add_argument("--line-yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--turn-sign", type=float, default=1.0,
                        help="1 for left/CCW turns, -1 for right/CW turns")
    parser.add_argument("--position-tolerance", type=float, default=0.03)
    parser.add_argument("--turn-tolerance-deg", type=float, default=3.0)
    parser.add_argument("--turn-stable-cycles", type=int, default=6)
    parser.add_argument("--max-lateral-error", type=float, default=0.15)
    parser.add_argument("--leg-timeout", type=float, default=25.0)
    parser.add_argument("--turn-timeout", type=float, default=12.0)
    parser.add_argument("--stuck-timeout", type=float, default=5.0)
    parser.add_argument("--progress-grace", type=float, default=2.0)
    parser.add_argument("--progress-epsilon", type=float, default=0.01)
    parser.add_argument("--reverse-abort-distance", type=float, default=0.08)
    parser.add_argument("--pose-timeout", type=float, default=0.25)
    parser.add_argument("--wait-timeout", type=float, default=8.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--pause", type=float, default=0.5)
    parser.add_argument("--report-period", type=float, default=0.5)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--dry-run-seconds", type=float, default=3.0)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    args.side_length = max(0.05, args.side_length)
    args.legs = max(1, min(4, args.legs))
    args.linear = max(0.0, min(args.linear, 0.25))
    args.min_linear = max(0.0, min(args.min_linear, args.linear))
    args.slowdown_distance = max(args.position_tolerance, args.slowdown_distance)
    args.max_angular = max(0.0, args.max_angular)
    args.turn_max_angular = max(0.0, min(args.turn_max_angular, args.max_angular))
    args.min_turn_angular = max(0.0, min(args.min_turn_angular, args.turn_max_angular))
    args.turn_slowdown_angle_deg = max(args.turn_tolerance_deg, args.turn_slowdown_angle_deg)
    args.turn_sign = 1.0 if args.turn_sign >= 0.0 else -1.0
    args.turn_stable_cycles = max(1, args.turn_stable_cycles)
    args.leg_timeout = max(1.0, args.leg_timeout)
    args.turn_timeout = max(1.0, args.turn_timeout)
    args.pose_timeout = max(0.05, args.pose_timeout)
    args.rate = max(2.0, args.rate)
    return args


def main(argv):
    global _node

    args = parse_args(argv)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    _node = rclpy.create_node("drive_mocap_square")
    _node.create_subscription(PoseStamped, args.pose_topic, pose_cb, 5)
    pub = _node.create_publisher(Twist, args.cmd_topic, 5)

    spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    spin_thread.start()

    try:
        return run_square(pub, args)
    except Exception as exc:
        publish_zero(pub)
        print("ERROR: %s" % exc)
        return 1
    finally:
        publish_zero(pub)
        try:
            _node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
