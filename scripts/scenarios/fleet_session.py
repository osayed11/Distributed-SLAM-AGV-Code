#!/usr/bin/env python3
"""Coordinate existing S1 wrappers over persistent SSH, with robot-local leases.

No motion on startup. Every recorder/controller must be ARMED before a common
release. Faults hold the fleet; optional transient-GT recovery requires all robots
to report stable reacquisition and the separation guard to be clear.
"""

import argparse
import concurrent.futures
from collections import deque
import getpass
import itertools
import json
import math
import os
from pathlib import Path
import queue
import re
import shlex
import signal
import shutil
import socket
import subprocess
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/logging'))
from motion_recovery import atomic_json, is_transient_gt_fault


def validate_config(data):
    if not isinstance(data.get('auto_resume_gt', False), bool):
        raise ValueError('auto_resume_gt must be a boolean')
    robots = data['robots']
    if not robots or len({r['name'] for r in robots}) != len(robots):
        raise ValueError('Specify a nonempty fleet with unique robot names')
    if len({r['host'] for r in robots}) != len(robots):
        raise ValueError('Duplicate hosts are not allowed')
    if len({r['pose_topic'] for r in robots}) != len(robots):
        raise ValueError('Each robot must use its own pose topic')
    for key in ('duration', 'linear'):
        value = data[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(key + ' must be positive and finite')
    for key, default in (('startup_timeout', 90), ('recovery_timeout', 30), ('max_radius_error', 0.2),
                         ('max_pause_seconds', 30), ('recovery_stable_sec', 1.0), ('link_abort_sec', 15)):
        value = data.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(key + ' must be positive and finite')
    # Non-negative knobs: 0 pauses means any fault aborts; 0 auto-resume means operator release only.
    for key, default in (('max_pauses', 3), ('auto_resume_sec', 0)):
        value = data.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(key + ' must be non-negative and finite')
    if data.get('link_abort_sec', 15) < 3:
        raise ValueError('link_abort_sec must allow at least the 3 s worker status timeout')
    if not isinstance(data.get('runtime_imu_rearm', True), bool):
        raise ValueError('runtime_imu_rearm must be true or false')
    rearms = data.get('max_imu_rearms', 5)
    if isinstance(rearms, bool) or not isinstance(rearms, int) or rearms < 0:
        raise ValueError('max_imu_rearms must be a non-negative integer')
    if not isinstance(data.get('max_pauses', 3), int):
        raise ValueError('max_pauses must be an integer')
    if data['linear'] > 0.2 or data['duration'] > 1800:
        raise ValueError('Qualification runner limited to 0.2 m/s and 1800 s')
    omega = data.get('phase_omega', 0)
    separation = data.get('min_separation', 0)
    for value in (omega, separation):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError('Invalid phase speed or separation')
    if omega and separation < 0.55:
        raise ValueError('Coordinated phase mode requires a separation guard of at least 0.55 m')
    for robot in robots:
        if not re.fullmatch(r'agv[0-9]+', robot['name']):
            raise ValueError('Invalid robot name')
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', robot['host']) or robot['host'].startswith('-'):
            raise ValueError('Invalid SSH host')
        if not re.fullmatch(r'/[A-Za-z0-9_/]+', robot['pose_topic']):
            raise ValueError('Invalid pose topic')
        if robot['direction'] not in ('cw', 'ccw'):
            raise ValueError('direction must be cw or ccw')
        for key in ('radius', 'center_x', 'center_y', 'forward_yaw_offset_deg'):
            if not isinstance(robot[key], (int, float)) or not math.isfinite(robot[key]):
                raise ValueError(key + ' must be finite and explicit')
        if robot['radius'] < 0.1:
            raise ValueError('radius must be at least 0.1 m')
        linear = robot.get('linear', data['linear'])
        if isinstance(linear, bool) or not isinstance(linear, (int, float)) or not math.isfinite(linear) or not 0 < linear <= 0.2:
            raise ValueError('robot linear must be positive, finite and at most 0.2 m/s')
        delay = robot.get('motion_delay', 0)
        if isinstance(delay, bool) or not isinstance(delay, (int,float)) or not math.isfinite(delay) or not 0 <= delay < data['duration']:
            raise ValueError('Invalid motion delay')
        if omega and not math.isclose(linear, omega * robot['radius'], rel_tol=1e-6):
            raise ValueError('Coordinated speeds must equal phase_omega * radius')
    names = {r['name'] for r in robots}
    topics = {r['pose_topic'] for r in robots}
    for observer in data.get('observers', []):
        if not re.fullmatch(r'agv[0-9]+', observer['name']) or observer['name'] in names:
            raise ValueError('Observer names must be valid and distinct from moving robots')
        if not re.fullmatch(r'/[A-Za-z0-9_/]+', observer['pose_topic']) or observer['pose_topic'] in topics:
            raise ValueError('Observer pose topics must be valid and unique')
        names.add(observer['name'])
        topics.add(observer['pose_topic'])
    return data


def run_environment(config, robot, directory):
    linear = robot.get('linear', config['linear'])
    return dict(S1_RUN_ID=directory.name, S1_CONTROL_FILE=str(directory/'control.json'),
                S1_STATUS_JSON=str(directory/'motion.json'), S1_START_SIGNAL_FILE=str(directory/'start'),
                S1_PRECHECK='false', S1_BRINGUP_MODE='warm', S1_RECORD='true', S1_VALIDATE='false',
                S1_STORAGE_ID='mcap', S1_DURATION=str(config['duration']), S1_LINEAR=str(linear),
                S1_MIN_LINEAR=str(min(0.07, linear)),
                S1_RADIUS=str(robot['radius']), S1_CENTER_X=str(robot['center_x']),
                S1_CENTER_Y=str(robot['center_y']), S1_DIRECTION=robot['direction'],
                S1_FORWARD_YAW_OFFSET_DEG=str(robot['forward_yaw_offset_deg']),
                S1_PHASE_OMEGA=str(config.get('phase_omega', 0)),
                S1_MOTION_DELAY=str(robot.get('motion_delay', 0)),
                S1_MAX_RADIUS_ERROR=str(config.get('max_radius_error', 0.20)),
                S1_RECOVERY_TIMEOUT=str(config.get('recovery_timeout', 30)),
                S1_RECOVERY_STABLE_SEC=str(config.get('recovery_stable_sec', 1.0)),
                S1_MAX_PAUSES=str(int(config.get('max_pauses', 3))),
                S1_MAX_PAUSE_SECONDS=str(config.get('max_pause_seconds', 30)),
                S1_RUNTIME_IMU_REARM='true' if config.get('runtime_imu_rearm', True) else 'false',
                S1_MAX_IMU_REARMS=str(int(config.get('max_imu_rearms', 5))),
                MOCAP_TOPIC=robot['pose_topic'], CMD_TOPIC=f"/{robot['name']}/cmd_vel",
                REQUIRE_IMU='true', REQUIRE_GT='true')


def worker():
    """Single SSH session: supervise one wrapper and maintain a local lease file."""
    first = json.loads(sys.stdin.readline())
    config = validate_config(first['config'])
    robot = next(r for r in config['robots'] if r['name'] == first['robot'])
    if socket.gethostname().split('.')[0] != robot['name']:
        raise RuntimeError('SSH host identity does not match the configured robot')
    run_id = first['run_id']
    if not re.fullmatch(r'[a-f0-9]{32}', run_id):
        raise ValueError('Invalid run id')
    # Linux /run is tmpfs; /tmp may share the recording SD card on Ubuntu.
    runtime = Path('/run/user') / str(os.getuid())
    if not runtime.is_dir() or runtime.stat().st_uid != os.getuid():
        raise RuntimeError('Missing owned /run/user/<uid>; establish a normal SSH/PAM login first')
    directory = runtime / 'orkar-runs' / run_id
    directory.mkdir(parents=True, exist_ok=False)
    evidence = Path.home() / 'agv_data' / ('fleet_' + run_id)
    evidence.mkdir(parents=True, exist_ok=False)
    environment = run_environment(config, robot, directory)
    environment['S1_ARTIFACT_JSON'] = str(evidence/'artifacts.json')
    atomic_json(evidence/'manifest.json', dict(config=config, robot=robot, environment=environment,
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                created_epoch=time.time(), validation='PENDING'))
    control = dict(run_id=run_id, mode='hold', resume_generation=0, received_monotonic=time.monotonic())
    atomic_json(directory/'control.json', control)
    # Remove inherited S1 overrides: the manifest is the complete run configuration.
    env = {k:v for k,v in os.environ.items() if not k.startswith(('S1_', 'MOCAP_', 'CMD_TOPIC'))}
    env.update(environment)
    # Keep controller stdout off the recording SD card until motion has stopped.
    runtime_log = directory/'session.log'
    log = runtime_log.open('w')
    atomic_json(evidence/'runtime_log.json', dict(path=str(runtime_log)))
    process = subprocess.Popen(['bash', str(ROOT/'scripts/scenarios/run_s1_mocap_pilot_robot.sh'),
                                robot['name'], 'fleet_' + run_id], cwd=ROOT, env=env,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    incoming = queue.Queue()
    def reader():
        try:
            for line in sys.stdin:
                incoming.put(json.loads(line))
        finally:
            incoming.put(None)
    threading.Thread(target=reader, daemon=True).start()
    abort_started = None
    last_received = time.monotonic()
    last_report = 0.0
    def abort(signum=None, frame=None):
        nonlocal abort_started
        if abort_started is None:
            abort_started = time.monotonic()
            control.update(mode='abort', received_monotonic=abort_started)
            try:
                atomic_json(directory/'control.json', control)
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, abort)
    signal.signal(signal.SIGINT, abort)
    try:
        while process.poll() is None:
            now = time.monotonic()
            try:
                message = incoming.get(timeout=0.1)
                if message is None or message.get('run_id') != run_id:
                    abort()
                else:
                    last_received = now
                    if message['mode'] not in ('hold', 'run', 'abort'):
                        raise ValueError('Invalid mode')
                    control.update(mode=message['mode'], resume_generation=int(message['resume_generation']),
                                   received_monotonic=now, trajectory_time=message.get('trajectory_time'))
                    atomic_json(directory/'control.json', control)
                    if message['mode'] == 'abort':
                        abort()
                    elif message.get('start_epoch') and not (directory/'start').exists():
                        if not math.isfinite(message['start_epoch']) or message['start_epoch'] <= 0:
                            raise ValueError('Invalid start epoch')
                        (directory/'start.tmp').write_text(str(message['start_epoch']))
                        os.replace(directory/'start.tmp', directory/'start')
            except queue.Empty:
                pass
            # Coordinator silence: the controller already pauses when its lease
            # (coordinator_timeout) expires, so hold through short link stalls and
            # abort only once the configured link budget is gone.
            if now - last_received > float(config.get('link_abort_sec', 15)):
                abort()
            if abort_started is not None and now - abort_started > 160:
                # Last resort only after wrapper's recorder-finalisation deadlines.
                os.killpg(process.pid, signal.SIGKILL)
            if now - last_report >= 0.1:
                status = dict(state='PREPARING')
                try:
                    status = json.loads((directory/'motion.json').read_text())
                    if now - status['monotonic'] > 2 and status['state'] not in ('COMPLETE', 'ABORTED'):
                        status.update(state='ABORTED', reason='controller_status_expired')
                        abort()
                except FileNotFoundError:
                    pass
                status.update(robot=robot['name'], worker_epoch=time.time())
                print(json.dumps(status), flush=True)
                last_report = now
        rc = process.wait()
        # Recorders and motion are over and the index-based validator takes ~1 s,
        # so audit here instead of waiting for a separate --action validate round.
        # Its stdout must not reach the coordinator's JSON status channel.
        verdict, detail = 'NOT_RUN', 'no recorded artifacts'
        if (evidence/'artifacts.json').exists():
            try:
                verdict = 'TIMING_PASS' if validate_worker(run_id, log_path=evidence/'validation.log', timing_only=True) == 0 \
                    else 'REJECTED_OR_INTERRUPTED'
                detail = str(evidence/'validation.log')
            except Exception as exc:  # validation must never mask the run's own outcome
                verdict, detail = 'VALIDATION_ERROR', str(exc)[:200]
        atomic_json(evidence/'result.json', dict(exit_code=rc, validation=verdict, validation_detail=detail,
                                                  ended_epoch=time.time()))
        print(json.dumps(dict(robot=robot['name'], state='FINISHED', exit_code=rc, validation=verdict,
                              validation_detail=detail, evidence=str(evidence))), flush=True)
        return rc
    finally:
        if process.poll() is None:
            abort()
            try:
                process.wait(timeout=160)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        shutil.copyfile(runtime_log, evidence/'session.log')
        for filename in ('motion.json',):
            if (directory/filename).exists():
                (evidence/filename).write_bytes((directory/filename).read_bytes())


class LocalPoseSource:
    """Read laptop-local ROS poses without a robot/SSH round trip."""
    def __init__(self, robots, observers=()):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        self.ros = rclpy
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('orkar_fleet_separation', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.lock = threading.Lock()
        self.poses = {}
        self.observers = [r['name'] for r in observers]
        self.stop = threading.Event()
        self.subscriptions = [self.node.create_subscription(
            PoseStamped, r['pose_topic'],
            lambda msg, name=r['name']: self.receive(name, msg), qos_profile_sensor_data)
            for r in list(robots) + list(observers)]
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def receive(self, name, msg):
        if not msg.header.frame_id:
            return
        pose = dict(x=msg.pose.position.x, y=msg.pose.position.y,
                    pose_stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    pose_frame=msg.header.frame_id)
        with self.lock:
            self.poses[name] = pose

    def spin(self):
        while not self.stop.is_set() and self.context.ok():
            self.executor.spin_once(timeout_sec=0.05)

    def statuses(self, statuses):
        with self.lock:
            poses = dict(self.poses) if self.thread.is_alive() else {}
        return local_guard_statuses(statuses, poses, self.observers)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        self.executor.shutdown(timeout_sec=2)
        self.node.destroy_node()
        self.context.shutdown()


def local_guard_statuses(statuses, poses, observers=()):
    # Missing local data must fail closed, never fall back to an old SSH pose.
    states = dict(statuses)
    states.update({name: dict(state='OBSERVED') for name in observers})
    return {name: dict(state=status['state'], **poses.get(name, {}))
            for name, status in states.items()}


class SeparationGuard:
    """Conservative constant-velocity lookahead; holds, never routes around robots."""
    def __init__(self, minimum, horizon=1.5):
        self.minimum = minimum
        self.horizon = horizon
        self.previous = {}
        self.velocity = {}
        self.history = {}

    def check(self, statuses, now):
        if not self.minimum:
            return ''
        positions = {}
        frames = set()
        for name, status in statuses.items():
            if status['state'] in ('PREPARING', 'FINISHED', 'ABORTED'):
                continue
            try:
                x,y,stamp = (float(status[k]) for k in ('x','y','pose_stamp'))
                if not all(math.isfinite(v) for v in (x,y,stamp)) or not -0.1 <= now-stamp <= 0.5:
                    return 'separation_pose_stale: ' + name
            except (KeyError,TypeError,ValueError):
                return 'separation_pose_missing: ' + name
            previous = self.previous.get(name)
            if status.get('pose_frame'):
                frames.add(status['pose_frame'])
                if len(frames) > 1:
                    return 'separation_frame_mismatch'
            if previous and stamp > previous[2]:
                dt = stamp-previous[2]
                if math.hypot(x-previous[0], y-previous[1]) > 0.03 + dt:
                    return 'separation_pose_jump: ' + name
            elif previous and stamp < previous[2]:
                return 'separation_pose_clock_reversed: ' + name
            self.previous[name] = (x,y,stamp)
            history = self.history.setdefault(name, deque())
            if not history or stamp > history[-1][2]:
                history.append((x,y,stamp))
                while len(history) > 2 and history[1][2] <= stamp - 0.25:
                    history.popleft()
                span = stamp - history[0][2]
                # Avoid turning millimetre-scale marker jitter into high velocity.
                if span >= 0.2:
                    self.velocity[name] = ((x-history[0][0])/span, (y-history[0][1])/span)
            vx,vy = self.velocity.get(name,(0,0))
            if math.hypot(vx,vy)>1:
                return 'separation_pose_jump: ' + name
            age = max(0,now-stamp)
            positions[name] = (x+vx*age,y+vy*age,vx,vy)
        for a,b in itertools.combinations(positions,2):
            p,q = positions[a],positions[b]
            dx,dy,vx,vy = (p[k]-q[k] for k in range(4))
            speed2 = vx*vx+vy*vy
            t = max(0,min(self.horizon,-(dx*vx+dy*vy)/speed2)) if speed2 else 0
            if math.hypot(dx+vx*t,dy+vy*t)<self.minimum:
                return 'predicted_close_approach: '+a+' '+b
        return ''


class FleetState:
    """Coordinate manual recovery, or explicitly enabled transient-GT recovery."""
    def __init__(self, auto_resume_sec=0.0, auto_resume_gt=False):
        self.mode = 'hold'
        self.started = False
        self.generation = 0
        self.start_epoch = None
        self.trajectory_time = 0.0
        self.last_time = None
        self.auto_resume_sec = float(auto_resume_sec) if auto_resume_sec > 0 else 1.0
        self.auto_resume_gt = auto_resume_gt or auto_resume_sec > 0
        self.auto_blocked = False
        self.auto_gt_seen = False
        self.auto_ready_since = None

    def automatic_release(self, statuses, now, external_fault):
        if not self.auto_resume_gt or not self.started or self.mode == 'abort':
            return False
        if external_fault:
            if external_fault.startswith('separation_pose_stale: '):
                self.auto_gt_seen = True
            else:
                self.auto_blocked = True
        for status in statuses:
            if (status['state'] == 'PAUSED' and
                    status.get('resume_generation_seen', 0) >= self.generation):
                if status.get('auto_resume_safe') is not True:
                    self.auto_blocked = True
                if is_transient_gt_fault(status.get('reason', '')):
                    self.auto_gt_seen = True
        ready = (bool(statuses) and self.auto_gt_seen and not self.auto_blocked and not external_fault and
                 all(s['state'] == 'PAUSED' and s.get('recovery_ready') is True and
                     s.get('resume_generation_seen', 0) == self.generation and
                     isinstance(s.get('pose_stamp'), (int, float)) and
                     math.isfinite(s['pose_stamp']) and -0.1 <= now-s['pose_stamp'] <= 0.5
                     for s in statuses))
        if not ready:
            self.auto_ready_since = None
            return False
        if self.auto_ready_since is None:
            self.auto_ready_since = now
        # Robot-local stable-pose checks are followed by a fleet-wide clear window.
        return now - self.auto_ready_since >= self.auto_resume_sec

    def update(self, statuses, now, release=False, external_fault=''):
        auto_release = self.automatic_release(statuses, now, external_fault)
        release = release or auto_release
        if self.last_time is not None and self.start_epoch is not None and self.started and self.mode == 'run':
            self.trajectory_time += max(0.0, now - max(self.last_time, self.start_epoch))
        self.last_time = now
        states = [s['state'] for s in statuses]
        if any(s == 'ABORTED' or (s == 'FINISHED' and x.get('exit_code', 1) != 0)
               for s, x in zip(states, statuses)):
            self.mode = 'abort'
        elif self.mode != 'abort':
            if external_fault:
                self.mode = 'hold'
            elif not self.started and all(s == 'ARMED' for s in states):
                self.started = True
                self.start_epoch = now + 3
                self.mode = 'run'
            elif self.started and any(x['state'] == 'PAUSED' and
                                     x.get('resume_generation_seen', 0) >= self.generation for x in statuses):
                if self.mode != 'hold':
                    self.mode = 'hold'
                all_ready = all(x['state'] == 'PAUSED' and x.get('recovery_ready') for x in statuses)
                if release and all_ready:
                    self.generation += 1
                    self.mode = 'run'
                    self.auto_blocked = False
                    self.auto_gt_seen = False
                    self.auto_ready_since = None
        return dict(mode=self.mode, resume_generation=self.generation, start_epoch=self.start_epoch,
                    trajectory_time=self.trajectory_time, auto_resumed=auto_release and self.mode == 'run')


def ssh_command(args, robot, remote_args):
    prefix = ['sshpass', '-e'] if args.password else []
    return prefix + ['ssh', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=2',
                     '-o', 'ServerAliveCountMax=2', f"{args.user}@{robot['host']}",
                     ' '.join(shlex.quote(v) for v in remote_args)]


def validate_worker(run_id, log_path=None, timing_only=False):
    """Audit a finished run's bags and motion summary.

    Called automatically by the worker once the wrapper exits, and available as
    `--action validate --run-id` to re-audit later. With log_path the
    validators' output goes to that file so the worker's stdout stays a clean
    JSON status channel.
    """
    if not re.fullmatch(r'[a-f0-9]{32}', run_id):
        raise ValueError('Invalid run id')
    directory = Path.home()/'agv_data'/('fleet_' + run_id)
    artifacts = json.loads((directory/'artifacts.json').read_text())
    manifest = json.loads((directory/'manifest.json').read_text())
    import fcntl
    lock = open('/tmp/orkar_s1_motion.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('Do not run a heavy bag audit while another scenario owns the robot')
    summary = json.loads(Path(artifacts['summary']).read_text())
    env = os.environ.copy()
    env.update(CMD_TOPIC=artifacts['cmd_topic'], MOCAP_TOPIC=artifacts['pose_topic'],
               REQUIRE_GT='true', REQUIRE_IMU='true', COLOR_BAG_MIN_HZ='10', DEPTH_BAG_MIN_HZ='10',
               DEPTH_TOPIC='/camera/depth/image_rect_raw', DEPTH_INFO_TOPIC='/camera/depth/camera_info',
               IMU_TOPICS='/camera/gyro/sample /camera/accel/sample /camera/imu /imu')
    command = [sys.executable, str(ROOT/'scripts/logging/validate_ros2_bag.py'), *artifacts['bags'],
               '--strict', '--require-gt', '--require-imu', '--require-resilient-storage',
               '--window-start-epoch', str(summary['start_epoch']), '--window-duration', str(summary['elapsed_sec']),
               '--min-duration', str(manifest['config']['duration']), '--json-out', str(directory/'bag_validation.json')]
    if timing_only:
        command.append('--timing-only')
    log = open(log_path, 'w') if log_path else None
    try:
        output = dict(stdout=log, stderr=subprocess.STDOUT) if log else {}
        bag_rc = subprocess.run(command, env=env, timeout=8 if timing_only else None, **output).returncode
        motion_rc = subprocess.run([sys.executable, str(ROOT/'scripts/scenarios/validate_s1_circle_summary.py'),
                                    artifacts['summary'], '--max-radius-error',
                                    str(manifest['config'].get('max_radius_error', 0.20)),
                                    '--max-pose-age', '0.5'], timeout=5 if timing_only else None, **output).returncode
    finally:
        if log:
            log.close()
    verdict = ('TIMING_PASS' if timing_only else 'PASS') if bag_rc == 0 and motion_rc == 0 else 'REJECTED_OR_INTERRUPTED'
    atomic_json(directory/'validation.json', dict(verdict=verdict, bag_exit_code=bag_rc, motion_exit_code=motion_rc,
                                                  log=str(log_path) if log_path else None))
    if not log_path:
        print(verdict, directory)
    return 0 if bag_rc == 0 and motion_rc == 0 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--validate-worker', help=argparse.SUPPRESS)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--action', choices=['prepare', 'run', 'stop', 'validate'], default='run')
    parser.add_argument('--run-id', help='Required for deferred validation')
    parser.add_argument('--repo', default='/home/ubuntu/slam_project')
    parser.add_argument('--user', default='ubuntu')
    parser.add_argument('--password', action='store_true', help='Prompt once; requires sshpass, never saved in config')
    parser.add_argument('--yes', action='store_true', help='Operator confirms all paths are clear for motion')
    parser.add_argument('--local-poses', action='store_true',
                        help='Use laptop-local ROS poses for the separation guard (requires ROS environment)')
    parser.add_argument('--output', type=Path, default=Path('data/fleet_sessions'))
    args = parser.parse_args()
    if args.worker:
        return worker()
    if args.validate_worker:
        return validate_worker(args.validate_worker)
    if args.config is None:
        parser.error('--config is required')
    config = validate_config(json.loads(args.config.read_text()))
    if args.action == 'run' and config.get('observers') and not args.local_poses:
        parser.error('Observed stationary robots require --local-poses')
    if not re.fullmatch(r'[a-z_][a-z0-9_-]*', args.user) or not args.repo.startswith('/'):
        parser.error('Specify a valid SSH username and absolute remote repository path')
    if args.action == 'run' and not args.yes:
        parser.error('Refusing motion without --yes and clear paths')
    if args.password:
        os.environ['SSHPASS'] = getpass.getpass('Robot SSH password: ')
    if args.action != 'run':
        def service(robot):
            if args.action == 'validate':
                if not args.run_id or not re.fullmatch(r'[a-f0-9]{32}', args.run_id):
                    raise ValueError('--run-id required for validation')
                command = ['python3', args.repo+'/scripts/scenarios/fleet_session.py', '--validate-worker', args.run_id]
            else:
                command = ['python3', args.repo+'/scripts/scenarios/sensor_session.py', args.action, robot['name']]
            return subprocess.run(ssh_command(args, robot, command)).returncode
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(config['robots'])) as pool:
            return int(any(pool.map(service, config['robots'])))
    run_id = uuid.uuid4().hex
    output = args.output/run_id
    output.mkdir(parents=True)
    atomic_json(output/'manifest.json', dict(run_id=run_id, config=config, created_epoch=time.time(),
                separation_pose_source='local_ros' if args.local_poses else 'robot_status'))
    children = {}
    statuses = {r['name']: dict(state='PREPARING') for r in config['robots']}
    received = {r['name']: time.monotonic() for r in config['robots']}
    messages = queue.Queue()
    keys = queue.Queue()
    fleet = FleetState(config.get('auto_resume_sec', 0), auto_resume_gt=config.get('auto_resume_gt', False))
    link_abort_sec = float(config.get('link_abort_sec', 15))
    separation_guard = SeparationGuard(config.get('min_separation',0))
    pose_source = LocalPoseSource(config['robots'], config.get('observers', [])) if args.local_poses else None
    started = time.monotonic()
    def read_worker(name, proc):
        for line in proc.stdout:
            try:
                messages.put((name, json.loads(line)))
            except ValueError:
                messages.put((name, dict(state='ABORTED', reason='invalid_worker_protocol')))
        messages.put((name, dict(state='DISCONNECTED')))
    def read_keys():
        for line in sys.stdin:
            keys.put(line.strip().lower())
    threading.Thread(target=read_keys, daemon=True).start()
    print('r + Enter: resume after all robots are recovery-ready. q + Enter / Ctrl+C: stop.')
    if fleet.auto_resume_gt:
        print('Transient GT auto-resume enabled: all robots stable plus %.1fs clear separation.' % fleet.auto_resume_sec)
    last_states = {}
    try:
        for robot in config['robots']:
            name = robot['name']
            err = (output/(name + '_ssh.log')).open('w')
            process = subprocess.Popen(ssh_command(args, robot, ['python3', '-u', args.repo +
                                       '/scripts/scenarios/fleet_session.py', '--worker']),
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err,
                                       text=True, bufsize=1)
            err.close()
            children[name] = process
            process.stdin.write(json.dumps(dict(config=config, robot=name, run_id=run_id)) + '\n')
            process.stdin.flush()
            threading.Thread(target=read_worker, args=(name, process), daemon=True).start()
        with (output/'events.jsonl').open('a') as events:
            while True:
                now = time.monotonic()
                while not messages.empty():
                    name, status = messages.get()
                    if status['state'] == 'DISCONNECTED':
                        if statuses[name]['state'] != 'FINISHED':
                            status = dict(state='ABORTED', reason='worker_disconnected')
                        else:
                            continue
                    statuses[name] = status
                    received[name] = now
                release = False
                while not keys.empty():
                    key = keys.get()
                    if key == 'q': fleet.mode = 'abort'
                    if key == 'r': release = True
                if now - started > float(config.get('startup_timeout', 90)) and not fleet.started:
                    fleet.mode = 'abort'
                effective = []
                for name, status in statuses.items():
                    silent = now - received[name]
                    if status['state'] != 'FINISHED' and silent > link_abort_sec:
                        fleet.mode = 'abort'
                    elif status['state'] not in ('FINISHED', 'ABORTED') and silent > 3:
                        # A quiet worker is a robot we cannot see: hold the fleet,
                        # but give the link link_abort_sec to come back.
                        status = dict(status, state='PAUSED', reason='worker_link_stalled',
                                      recovery_ready=False, resume_generation_seen=fleet.generation)
                    effective.append(status)
                guard_statuses = pose_source.statuses(statuses) if pose_source else statuses
                guard_fault = separation_guard.check(guard_statuses, time.time())
                command = fleet.update(effective, time.time(), release, guard_fault)
                if command['auto_resumed']:
                    print('Fleet automatically resuming after stable GT recovery.', flush=True)
                    events.write(json.dumps(dict(epoch=time.time(), state='AUTO_RESUME',
                        resume_generation=fleet.generation, reason='stable_gt_recovery'))+'\n')
                    events.flush()
                if guard_fault and guard_fault != getattr(separation_guard,'last_fault',''):
                    print('Fleet hold: '+guard_fault,flush=True)
                    events.write(json.dumps(dict(epoch=time.time(),state='FLEET_HOLD',reason=guard_fault))+'\n')
                    events.flush()
                separation_guard.last_fault = guard_fault
                command['run_id'] = run_id
                for name, proc in children.items():
                    if proc.poll() is None:
                        try:
                            proc.stdin.write(json.dumps(command) + '\n')
                            proc.stdin.flush()
                        except BrokenPipeError:
                            fleet.mode = 'abort'
                    signature = (statuses[name]['state'], statuses[name].get('reason', ''),
                                 statuses[name].get('recovery_ready', False))
                    if signature != last_states.get(name):
                        print(name, signature, flush=True)
                        events.write(json.dumps(dict(statuses[name], epoch=time.time(), robot=name)) + '\n')
                        events.flush()
                        last_states[name] = signature
                if all(s['state'] == 'FINISHED' for s in statuses.values()):
                    break
                if fleet.mode == 'abort':
                    print('Fleet abort requested; waiting for safe recorder finalisation.', flush=True)
                    break
                time.sleep(0.05)
    except KeyboardInterrupt:
        fleet.mode = 'abort'
    finally:
        # Closing SSH stdin also causes the worker to stop; leases cover lost links.
        for proc in children.values():
            if proc.poll() is None:
                try:
                    proc.stdin.write(json.dumps(dict(run_id=run_id, mode='abort', resume_generation=fleet.generation))+'\n')
                    proc.stdin.flush()
                    proc.stdin.close()
                except (BrokenPipeError, OSError):
                    pass
        deadline = time.monotonic() + 170
        for proc in children.values():
            try:
                proc.wait(timeout=max(0.1, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=5)
        verdicts = {name: s.get('validation', 'PENDING') for name, s in statuses.items()}
        atomic_json(output/'result.json', dict(states=statuses, mode=fleet.mode, validation=verdicts))
        if pose_source:
            pose_source.close()
    print('Evidence:', output)
    for name, s in statuses.items():
        print('  %-8s %-22s %s' % (name, verdicts.get(name, 'PENDING'), s.get('validation_detail', '')), flush=True)
    print('Bags remain on robots; per-robot validator output is in each evidence directory.')
    return 1 if fleet.mode == 'abort' or any(p.returncode for p in children.values()) else 0


if __name__ == '__main__':
    sys.exit(main())
