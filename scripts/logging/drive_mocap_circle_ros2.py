#!/usr/bin/env python3
"""Drive a MoCap-feedback circle with ROS 2 PoseStamped input.

This is the Scenario 1 pilot driver. It uses OptiTrack/ground-truth position
as feedback and publishes a caller-supplied, namespaced Twist command. It
refuses to move unless `--yes` is provided.
"""

import argparse
import json
import math
import os
import signal
import sys
import time
from dataclasses import asdict, dataclass
from typing import Optional, Tuple
from motion_recovery import MotionRecovery, atomic_json, read_control

ROS2_IMPORT_ERROR = None
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
except ModuleNotFoundError as exc:
    ROS2_IMPORT_ERROR = exc
    rclpy = None
    PoseStamped = None
    Twist = None

    class Node:  # type: ignore[no-redef]
        pass

    HistoryPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None


stop_requested = False
resume_generation = 0


def request_resume(signum=None, frame=None):
    del signum, frame
    global resume_generation
    resume_generation += 1


def request_stop(signum=None, frame=None):
    del signum, frame
    global stop_requested
    stop_requested = True


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def phase_speed(radius, omega, elapsed, delay, progress, gain, limit):
    """Track unwrapped clockwise/anticlockwise progress, never command reverse."""
    if elapsed < delay:
        return 0.0, -progress
    error = omega * (elapsed - delay) - progress
    return clamp(radius * (omega + gain * error), 0.0, limit), error


def circle_velocity(args, radius_error: float, heading_error: float, linear_override=None) -> Tuple[float, float]:
    linear = args.linear if linear_override is None else linear_override
    if linear_override is None and abs(radius_error) > args.slow_radius_error:
        linear = max(args.min_linear, args.linear * args.slow_linear_scale)
    if linear_override == 0:
        return 0.0, 0.0
    # Preserve the requested curvature when the radial-error guard slows motion.
    feedforward = clamp(args.turn_sign * linear / args.radius, -args.max_angular, args.max_angular)
    angular = clamp(feedforward + args.heading_kp * heading_error, -args.max_angular, args.max_angular)
    return linear, angular


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float
    frame_id: str
    stamp_sec: float
    wall_time: float


@dataclass
class CircleSummary:
    reached_duration: bool
    elapsed_sec: float
    laps: float
    radius_m: float
    center_x_m: float
    center_y_m: float
    center_source: str
    initial_radius_error_m: float
    final_radius_error_m: float
    max_radius_error_m: float
    initial_heading_error_deg: float
    max_heading_error_deg: float
    pose_samples: int
    max_pose_age_sec: float
    abort_reason: str
    active_sec: float = 0.0
    paused_sec: float = 0.0
    pause_count: int = 0
    interrupted: bool = False
    start_epoch: float = 0.0
    end_epoch: float = 0.0


class MocapCircleNode(Node):
    def __init__(self, args):
        super().__init__("drive_mocap_circle_ros2")
        self.args = args
        self.pose: Optional[PoseSample] = None
        self.pose_samples = 0
        self.pose_fault = ''
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT if args.best_effort_pose else ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(PoseStamped, args.pose_topic, self.pose_cb, qos)
        self.pub = self.create_publisher(Twist, args.cmd_topic, 10)

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, stamp_sec)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w)) - 1) > 0.1:
            self.pose_fault = 'invalid_pose'
            return
        if not msg.header.frame_id or stamp_sec <= 0:
            self.pose_fault = 'missing_pose_frame_or_stamp'
            return
        now = time.monotonic()
        if self.pose is not None:
            dt = now - self.pose.wall_time
            if msg.header.frame_id != self.pose.frame_id or stamp_sec <= self.pose.stamp_sec:
                self.pose_fault = 'pose_frame_or_timestamp_changed'
                return
            if dt <= self.args.pose_timeout and (
                math.hypot(p.x-self.pose.x, p.y-self.pose.y) > 0.03 + self.args.max_pose_speed * dt or
                abs(angle_delta(yaw_from_quat(q), self.pose.yaw)) > math.radians(5) + self.args.max_pose_yaw_rate * dt
            ):
                self.pose_fault = 'implausible_pose_jump'
                return
        self.pose_fault = ''
        self.pose = PoseSample(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            yaw=yaw_from_quat(msg.pose.orientation),
            frame_id=msg.header.frame_id,
            stamp_sec=stamp_sec,
            wall_time=now,
        )
        self.pose_samples += 1

    def publish_zero(self, seconds: float = 0.8):
        msg = Twist()
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_pose(self, timeout: float) -> PoseSample:
        start = time.monotonic()
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None and not self.pose_fault:
                return self.pose
            if time.monotonic() - start > timeout:
                raise RuntimeError(f"Timed out waiting for MoCap pose on {self.args.pose_topic}")
        raise RuntimeError("Stopped before first MoCap pose")


def compute_center(args, start: PoseSample, robot_yaw: float) -> Tuple[float, float]:
    if args.center_x is not None and args.center_y is not None:
        return args.center_x, args.center_y
    left_x = -math.sin(robot_yaw)
    left_y = math.cos(robot_yaw)
    return (
        start.x + args.turn_sign * args.radius * left_x,
        start.y + args.turn_sign * args.radius * left_y,
    )


def write_summary(path: str, summary: CircleSummary):
    if not path:
        return
    atomic_json(path, asdict(summary))


def report_state(args, state, reason='', **values):
    data = dict(run_id=args.run_id, state=state, reason=reason,
                monotonic=time.monotonic(), epoch=time.time(), **values)
    last = getattr(args, '_last_status', (None, 0.0))
    if args.status_json and (state != last[0] or data['monotonic'] - last[1] >= 0.1):
        atomic_json(args.status_json, data)
        args._last_status = (state, data['monotonic'])
    return data


def record_event(args, state, reason, publish_status=True):
    data = (report_state(args, state, reason) if publish_status else
            dict(run_id=args.run_id, state=state, reason=reason,
                 monotonic=time.monotonic(), epoch=time.time()))
    print('MOTION_STATE ' + json.dumps(data), flush=True)
    if args.events_jsonl:
        with open(args.events_jsonl, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(data) + '\n')


def check_start_pose(node, args, center_x, center_y, forward_offset):
    pose = node.pose
    if pose is None or node.pose_fault or time.monotonic() - pose.wall_time > args.pose_timeout:
        raise RuntimeError('Start gate: missing, invalid or stale pose')
    source_age = time.time() - pose.stamp_sec
    if not -args.max_future_stamp <= source_age <= args.max_source_age:
        raise RuntimeError('Start gate: source clock/latency outside bounds')
    dx, dy = pose.x-center_x, pose.y-center_y
    if abs(math.hypot(dx, dy)-args.radius) > args.max_radius_error:
        raise RuntimeError('Start gate: robot is outside the configured circle corridor')
    heading = angle_delta(math.atan2(dy, dx) + args.turn_sign*math.pi/2, pose.yaw+forward_offset)
    if abs(heading) > math.radians(args.max_start_heading_error_deg):
        raise RuntimeError('Start gate: robot is not tangent in the requested direction')


def drive(node: MocapCircleNode, args) -> CircleSummary:
    if not args.dry_run and not args.yes:
        raise RuntimeError("Refusing to move without --yes. Use --dry-run for a non-moving check.")

    start_pose = node.wait_for_pose(args.wait_timeout)
    forward_offset = math.radians(args.forward_yaw_offset_deg)
    start_robot_yaw = start_pose.yaw + forward_offset
    center_x, center_y = compute_center(args, start_pose, start_robot_yaw)
    direction = "counter-clockwise" if args.turn_sign > 0 else "clockwise"

    print(
        "Mocap circle ROS2: topic=%s cmd=%s frame=%s start=(%.3f, %.3f, %.3f) "
        "rigid_yaw=%.1fdeg robot_yaw=%.1fdeg center=(%.3f, %.3f) radius=%.3fm "
        "linear=%.3f duration=%.1fs direction=%s dry_run=%s"
        % (
            args.pose_topic,
            args.cmd_topic,
            start_pose.frame_id,
            start_pose.x,
            start_pose.y,
            start_pose.z,
            math.degrees(start_pose.yaw),
            math.degrees(start_robot_yaw),
            center_x,
            center_y,
            args.radius,
            args.linear,
            args.duration,
            direction,
            args.dry_run,
        ),
        flush=True,
    )

    node.publish_zero(0.5)
    check_start_pose(node, args, center_x, center_y, forward_offset)
    report_state(args, 'ARMED', x=node.pose.x, y=node.pose.y, pose_stamp=node.pose.stamp_sec)

    if args.start_signal_file:
        print(
            "Waiting stopped for synchronized start signal file %s"
            % args.start_signal_file,
            flush=True,
        )
        while rclpy.ok() and not stop_requested:
            command, fault = read_control(args.control_file, args.run_id, args.coordinator_timeout)
            if fault or command['mode'] == 'abort':
                raise RuntimeError(fault or 'coordinator_abort')
            report_state(args, 'ARMED', x=node.pose.x, y=node.pose.y, pose_stamp=node.pose.stamp_sec)
            try:
                with open(args.start_signal_file, "r", encoding="utf-8") as handle:
                    release_text = handle.read().strip()
                release_epoch = float(release_text)
                if release_epoch < 0.0:
                    raise ValueError("release epoch must be non-negative")
                args.start_at_epoch = release_epoch
                print(
                    "Synchronized start signal received: epoch %.3f"
                    % args.start_at_epoch,
                    flush=True,
                )
                break
            except FileNotFoundError:
                pass
            except (OSError, ValueError) as exc:
                raise RuntimeError(
                    "Invalid synchronized start signal file %s: %s"
                    % (args.start_signal_file, exc)
                ) from exc
            node.pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        if stop_requested or not rclpy.ok():
            raise RuntimeError("Stopped before synchronized start signal")

    if args.start_at_epoch > 0.0:
        print(
            "Waiting stopped for synchronized start epoch %.3f (in %.1fs)"
            % (args.start_at_epoch, max(0.0, args.start_at_epoch - time.time())),
            flush=True,
        )
        while rclpy.ok() and not stop_requested and time.time() < args.start_at_epoch:
            command, fault = read_control(args.control_file, args.run_id, args.coordinator_timeout)
            if fault or command['mode'] != 'run':
                raise RuntimeError(fault or 'start_permission_withdrawn')
            node.pub.publish(Twist())
            report_state(args, 'ARMED', x=node.pose.x, y=node.pose.y, pose_stamp=node.pose.stamp_sec)
            remaining = args.start_at_epoch - time.time()
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, remaining)))
        if stop_requested or not rclpy.ok():
            raise RuntimeError("Stopped before synchronized start")
        sample = node.pose
        if time.time() - args.start_at_epoch > args.start_late_tolerance:
            raise RuntimeError('Missed synchronized start; refusing a late independent start')
        if sample is None or node.pose_fault or time.monotonic() - sample.wall_time > args.pose_timeout:
            raise RuntimeError("MoCap pose is stale at synchronized start")
        check_start_pose(node, args, center_x, center_y, forward_offset)
        print("Synchronized start released.", flush=True)

    first = node.pose or start_pose
    last_theta = math.atan2(first.y - center_y, first.x - center_x)
    initial_radius = math.hypot(first.x - center_x, first.y - center_y)
    initial_radius_error = initial_radius - args.radius
    initial_tangent_yaw = last_theta + args.turn_sign * math.pi * 0.5
    initial_heading_error = angle_delta(initial_tangent_yaw, first.yaw + forward_offset)
    progress = 0.0
    phase_progress = 0.0
    phase_last_theta = last_theta
    max_abs_radius_error = 0.0
    max_abs_heading_error = 0.0
    final_radius_error = 0.0
    max_pose_age = 0.0
    abort_reason = ""
    reached_duration = False
    last_progress = 0.0
    last_progress_time = time.monotonic()
    start_time = time.monotonic()
    start_epoch = time.time()
    recovery = MotionRecovery(args.recovery_timeout, args.recovery_stable_sec,
                              args.max_pauses, args.max_pause_seconds)
    active_sec = 0.0
    previous_tick = start_time
    previous_state = 'RUNNING'
    record_event(args, 'RUNNING', '')
    last_report = 0.0
    msg = Twist()

    while rclpy.ok() and not stop_requested:
        cycle_started = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.0)
        now = time.monotonic()
        elapsed = now - start_time
        if recovery.state == 'RUNNING':
            active_sec += now - previous_tick
        previous_tick = now
        command, lease_fault = read_control(args.control_file, args.run_id, args.coordinator_timeout)
        if command['mode'] == 'abort':
            abort_reason = 'coordinator_abort'
            break
        if active_sec >= args.duration:
            reached_duration = True
            break

        sample = node.pose
        pose_age = now - sample.wall_time if sample else args.pose_timeout + 1
        max_pose_age = max(max_pose_age, pose_age)
        fault = lease_fault or node.pose_fault
        if not fault and pose_age > args.pose_timeout:
            fault = "mocap pose stale for %.3fs" % pose_age
        if sample is None:
            fault = fault or 'no_mocap_pose'
            # wait_for_pose established a sample; losing it is not recoverable.
            abort_reason = fault
            break
        source_age = time.time() - sample.stamp_sec
        if not fault and (source_age > args.max_source_age or source_age < -args.max_future_stamp):
            fault = 'pose_source_timestamp_out_of_bounds'

        radial_x = sample.x - center_x
        radial_y = sample.y - center_y
        current_radius = math.hypot(radial_x, radial_y)
        if current_radius < 1e-6:
            abort_reason = "robot is at circle center"
            break

        theta = math.atan2(radial_y, radial_x)
        phase_progress += args.turn_sign * angle_delta(theta, phase_last_theta)
        phase_last_theta = theta
        linear_override = None
        phase_error = 0.0
        phase_waiting = False
        if args.phase_omega > 0:
            phase_time = command.get('trajectory_time')
            if (not isinstance(phase_time, (int, float)) or isinstance(phase_time, bool)
                    or not math.isfinite(phase_time) or phase_time < 0):
                fault = fault or 'invalid_shared_trajectory_time'
                linear_override = 0.0
            else:
                linear_override, phase_error = phase_speed(args.radius, args.phase_omega,
                    phase_time, args.motion_delay, phase_progress, args.phase_kp, args.phase_max_linear)
                phase_waiting = phase_time < args.motion_delay
                if phase_time > args.motion_delay + 5 and abs(phase_error) > args.max_phase_error:
                    fault = fault or 'phase_tracking_error_exceeds_limit'
        radius_error = current_radius - args.radius
        final_radius_error = radius_error
        if abs(radius_error) > args.max_radius_error:
            fault = fault or "radius error %.3fm exceeds %.3fm" % (radius_error, args.max_radius_error)

        tangent_yaw = theta + args.turn_sign * math.pi * 0.5
        radius_heading_offset = args.turn_sign * clamp(
            args.radius_kp * radius_error,
            -args.max_radius_heading_offset,
            args.max_radius_heading_offset,
        )
        target_robot_yaw = tangent_yaw + radius_heading_offset
        robot_yaw = sample.yaw + forward_offset
        heading_error = angle_delta(target_robot_yaw, robot_yaw)
        if abs(heading_error) > math.radians(args.max_heading_error_deg):
            fault = fault or 'heading_error_exceeds_limit'
        if not node.pose_fault and pose_age <= args.pose_timeout:
            max_abs_radius_error = max(max_abs_radius_error, abs(radius_error))
            max_abs_heading_error = max(max_abs_heading_error, abs(heading_error))
        state = recovery.tick(now, fault, command['mode'] == 'run',
                              int(command['resume_generation']) if args.control_file else resume_generation)
        if state != 'RUNNING':
            # Braking takes priority over filesystem/status reporting.
            node.pub.publish(Twist())
        if state != previous_state:
            # Publish one complete status below, never a partial PAUSED snapshot.
            record_event(args, state, recovery.reason, publish_status=False)
        just_resumed = state == 'RUNNING' and previous_state == 'PAUSED'
        previous_state = state
        report_state(args, state, recovery.reason, recovery_ready=recovery.ready(now),
                     auto_resume_safe=recovery.auto_resume_safe,
                     active_sec=active_sec, radius_error_m=radius_error,
                     heading_error_deg=math.degrees(heading_error), pause_count=recovery.pause_count,
                     x=sample.x, y=sample.y, pose_stamp=sample.stamp_sec,
                     phase_error_rad=phase_error,
                     resume_generation_seen=int(command['resume_generation']))
        if state == 'ABORTED':
            abort_reason = recovery.reason
            break
        if state == 'PAUSED':
            node.pub.publish(Twist())
            last_theta = theta
            last_progress_time = now
            time.sleep(1.0 / args.rate)
            continue
        if just_resumed:
            last_theta = theta
            last_progress_time = now
        progress += args.turn_sign * angle_delta(theta, last_theta)
        last_theta = theta
        if progress > last_progress + args.progress_epsilon:
            last_progress = progress
            last_progress_time = now
        elif phase_waiting:
            last_progress_time = now
        elif not args.dry_run and active_sec > args.progress_grace and now - last_progress_time > args.stuck_timeout:
            abort_reason = "no circle progress for %.1fs" % args.stuck_timeout
            break
        max_abs_radius_error = max(max_abs_radius_error, abs(radius_error))
        max_abs_heading_error = max(max_abs_heading_error, abs(heading_error))

        msg.linear.x, msg.angular.z = circle_velocity(args, radius_error, heading_error, linear_override)
        if not args.dry_run:
            node.pub.publish(msg)

        if args.verbose and now - last_report >= args.report_period:
            print(
                "circle: t=%.1fs laps=%.2f radius=%.3f error=%+.3f heading=%+.1fdeg "
                "pose_age=%.3fs cmd=(%.3f, %.3f)"
                % (
                    elapsed,
                    progress / (2.0 * math.pi),
                    current_radius,
                    radius_error,
                    math.degrees(heading_error),
                    pose_age,
                    msg.linear.x,
                    msg.angular.z,
                ),
                flush=True,
            )
            last_report = now

        sleep_until = cycle_started + 1.0 / args.rate
        sleep_remaining = sleep_until - time.monotonic()
        if sleep_remaining > 0.0:
            time.sleep(sleep_remaining)

    if stop_requested:
        abort_reason = abort_reason or 'operator_stop'
    elapsed = time.monotonic() - start_time
    node.pub.publish(Twist())
    record_event(args, 'COMPLETE' if reached_duration and not abort_reason else 'ABORTED', abort_reason)
    node.publish_zero(1.0)
    summary = CircleSummary(
        reached_duration=reached_duration,
        elapsed_sec=elapsed,
        laps=progress / (2.0 * math.pi),
        radius_m=args.radius,
        center_x_m=center_x,
        center_y_m=center_y,
        center_source="explicit" if args.center_x is not None else "inferred",
        initial_radius_error_m=initial_radius_error,
        final_radius_error_m=final_radius_error,
        max_radius_error_m=max_abs_radius_error,
        initial_heading_error_deg=math.degrees(initial_heading_error),
        max_heading_error_deg=math.degrees(max_abs_heading_error),
        pose_samples=node.pose_samples,
        max_pose_age_sec=max_pose_age,
        abort_reason=abort_reason,
        active_sec=active_sec,
        paused_sec=recovery.total_paused(start_time + elapsed),
        pause_count=recovery.pause_count,
        interrupted=recovery.pause_count > 0,
        start_epoch=start_epoch,
        end_epoch=start_epoch + elapsed,
    )
    print(
        "Mocap circle complete: reached_duration=%s elapsed=%.1fs laps=%.2f "
        "final_radius_error=%+.3fm max_radius_error=%.3fm max_heading=%.1fdeg "
        "pose_samples=%d max_pose_age=%.3fs%s"
        % (
            summary.reached_duration,
            summary.elapsed_sec,
            summary.laps,
            summary.final_radius_error_m,
            summary.max_radius_error_m,
            summary.max_heading_error_deg,
            summary.pose_samples,
            summary.max_pose_age_sec,
            (" abort='%s'" % summary.abort_reason) if summary.abort_reason else "",
        ),
        flush=True,
    )
    write_summary(args.summary_json, summary)
    return summary


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Drive a MoCap-feedback circle using ROS 2")
    parser.add_argument(
        "--pose-topic",
        default=os.environ.get("MOCAP_TOPIC"),
        required=not bool(os.environ.get("MOCAP_TOPIC")),
        help="PoseStamped feedback topic (or set MOCAP_TOPIC).",
    )
    parser.add_argument(
        "--cmd-topic",
        default=os.environ.get("CMD_TOPIC"),
        required=not bool(os.environ.get("CMD_TOPIC")),
        help="Namespaced Twist command topic (or set CMD_TOPIC).",
    )
    parser.add_argument("--radius", type=float, default=0.5)
    parser.add_argument("--center-x", type=float, default=None, help="Marked floor center x in MoCap frame")
    parser.add_argument("--center-y", type=float, default=None, help="Marked floor center y in MoCap frame")
    parser.add_argument("--linear", type=float, default=0.10)
    parser.add_argument("--min-linear", type=float, default=0.07)
    parser.add_argument('--phase-omega', type=float, default=0.0)
    parser.add_argument('--motion-delay', type=float, default=0.0)
    parser.add_argument('--phase-kp', type=float, default=0.8)
    parser.add_argument('--phase-max-linear', type=float, default=0.20)
    parser.add_argument('--max-phase-error', type=float, default=0.15)
    parser.add_argument("--max-angular", type=float, default=0.55)
    parser.add_argument("--heading-kp", type=float, default=0.95)
    parser.add_argument("--radius-kp", type=float, default=1.20)
    parser.add_argument("--max-radius-heading-offset-deg", type=float, default=28.0)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--clockwise", action="store_true", help="Drive clockwise. Default is counter-clockwise.")
    parser.add_argument("--counter-clockwise", action="store_true", help="Drive counter-clockwise.")
    parser.add_argument("--forward-yaw-offset-deg", type=float, default=0.0,
                        help="Offset from rigid-body yaw to robot forward yaw.")
    parser.add_argument("--max-radius-error", type=float, default=0.35)
    parser.add_argument("--slow-radius-error", type=float, default=0.12)
    parser.add_argument("--slow-linear-scale", type=float, default=0.70)
    parser.add_argument("--pose-timeout", type=float, default=0.30)
    parser.add_argument('--recovery-timeout', type=float, default=15.0)
    parser.add_argument('--recovery-stable-sec', type=float, default=1.0)
    parser.add_argument('--max-pauses', type=int, default=3)
    parser.add_argument('--max-pause-seconds', type=float, default=30.0)
    parser.add_argument('--max-heading-error-deg', type=float, default=60.0)
    parser.add_argument('--max-start-heading-error-deg', type=float, default=30.0)
    parser.add_argument('--max-pose-speed', type=float, default=1.0)
    parser.add_argument('--max-pose-yaw-rate', type=float, default=3.0)
    parser.add_argument('--max-source-age', type=float, default=0.5)
    parser.add_argument('--max-future-stamp', type=float, default=0.1)
    parser.add_argument('--control-file', default='')
    parser.add_argument('--run-id', default='standalone')
    parser.add_argument('--coordinator-timeout', type=float, default=2.0)
    parser.add_argument('--start-late-tolerance', type=float, default=0.25)
    parser.add_argument('--status-json', default='')
    parser.add_argument('--events-jsonl', default='')
    parser.add_argument("--stuck-timeout", type=float, default=8.0)
    parser.add_argument("--progress-grace", type=float, default=3.0)
    parser.add_argument("--error-grace", type=float, default=3.0)
    parser.add_argument("--progress-epsilon", type=float, default=0.01)
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.add_argument(
        "--start-at-epoch",
        type=float,
        default=0.0,
        help="Remain stopped until this Unix epoch; 0 starts immediately.",
    )
    parser.add_argument(
        "--start-signal-file",
        default="",
        help="Wait stopped until this file contains a Unix release epoch.",
    )
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--report-period", type=float, default=1.0)
    parser.add_argument("--summary-json", default="")
    parser.add_argument("--best-effort-pose", action="store_true",
                        help="Use best-effort QoS for MoCap subscription.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args(argv)


def normalise_args(args):
    for key, value in vars(args).items():
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError('%s must be finite' % key)
    if (args.phase_omega < 0 or args.motion_delay < 0 or args.phase_kp <= 0
            or not 0 < args.phase_max_linear <= 0.2 or args.max_phase_error <= 0):
        raise ValueError('Invalid phase control parameters')
    if args.phase_omega > 0 and not args.control_file:
        raise ValueError('Phase control requires a fleet control file')
    for key in ('recovery_timeout', 'recovery_stable_sec', 'max_pause_seconds',
                'max_heading_error_deg', 'max_start_heading_error_deg', 'max_pose_speed', 'max_pose_yaw_rate',
                'max_source_age', 'max_future_stamp', 'coordinator_timeout', 'start_late_tolerance'):
        if getattr(args, key) <= 0:
            raise ValueError('%s must be positive' % key)
    if args.max_pauses < 0 or (args.center_x is None) != (args.center_y is None):
        raise ValueError('Invalid pause budget or incomplete circle center')
    args.radius = max(0.10, abs(args.radius))
    args.linear = clamp(abs(args.linear), 0.0, 0.50)
    args.min_linear = clamp(abs(args.min_linear), 0.0, args.linear if args.linear > 0 else 0.50)
    args.max_angular = clamp(abs(args.max_angular), 0.02, 1.5)
    args.heading_kp = max(0.0, args.heading_kp)
    args.radius_kp = max(0.0, args.radius_kp)
    args.max_radius_heading_offset = math.radians(max(0.0, args.max_radius_heading_offset_deg))
    args.duration = max(0.1, args.duration)
    args.max_radius_error = max(0.02, args.max_radius_error)
    args.slow_radius_error = max(0.0, args.slow_radius_error)
    args.slow_linear_scale = clamp(args.slow_linear_scale, 0.1, 1.0)
    args.pose_timeout = max(0.05, args.pose_timeout)
    args.stuck_timeout = max(1.0, args.stuck_timeout)
    args.progress_grace = max(0.0, args.progress_grace)
    args.error_grace = max(0.0, args.error_grace)
    args.progress_epsilon = max(0.0, args.progress_epsilon)
    args.start_at_epoch = max(0.0, args.start_at_epoch)
    args.start_signal_file = os.path.expanduser(args.start_signal_file.strip())
    args.wait_timeout = max(0.1, args.wait_timeout)
    args.rate = max(5.0, args.rate)
    args.report_period = max(0.5, args.report_period)
    if args.clockwise and args.counter_clockwise:
        raise RuntimeError("Choose only one of --clockwise or --counter-clockwise")
    args.turn_sign = -1.0 if args.clockwise else 1.0
    return args


def main(argv=None):
    args = normalise_args(parse_args(argv if argv is not None else sys.argv[1:]))
    if ROS2_IMPORT_ERROR is not None:
        print(
            "ERROR: ROS 2 Python modules are not available. "
            "Run this inside the robot ROS2 environment, e.g. source /opt/ros/humble/setup.bash.",
            file=sys.stderr,
        )
        print(f"missing import: {ROS2_IMPORT_ERROR}", file=sys.stderr)
        return 1
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGUSR1, request_resume)

    rclpy.init()
    node = MocapCircleNode(args)
    try:
        summary = drive(node, args)
        return 0 if summary.reached_duration and not summary.abort_reason else 1
    except Exception as exc:
        try:
            node.publish_zero(1.0)
        except Exception:
            pass
        try:
            record_event(args, 'ABORTED', str(exc))
        except OSError:
            pass
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
