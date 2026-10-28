#!/usr/bin/env python3
"""Coordinate existing S1 wrappers over persistent SSH, with robot-local leases.

No motion on startup. Every recorder/controller must be ARMED before a common
release. A fault holds the whole fleet; only an explicit resume can release it.
"""

import argparse
import concurrent.futures
import getpass
import json
import math
import os
from pathlib import Path
import queue
import re
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/logging'))
from motion_recovery import atomic_json


def validate_config(data):
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
    if data['linear'] > 0.2 or data['duration'] > 1800:
        raise ValueError('Qualification runner limited to 0.2 m/s and 1800 s')
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
                S1_MAX_RADIUS_ERROR=str(config.get('max_radius_error', 0.20)),
                S1_RECOVERY_TIMEOUT=str(config.get('recovery_timeout', 30)),
                S1_RECOVERY_STABLE_SEC=str(config.get('recovery_stable_sec', 1.0)),
                S1_MAX_PAUSES=str(int(config.get('max_pauses', 3))),
                S1_MAX_PAUSE_SECONDS=str(config.get('max_pause_seconds', 30)),
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
    log = (evidence/'session.log').open('w')
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
                                   received_monotonic=now)
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
            if now - last_report >= 0.25:
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
                verdict = 'PASS' if validate_worker(run_id, log_path=evidence/'validation.log') == 0 \
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
        for filename in ('motion.json',):
            if (directory/filename).exists():
                (evidence/filename).write_bytes((directory/filename).read_bytes())


class FleetState:
    """Latch faults: recovered messages alone cannot silently restart the fleet."""
    def __init__(self, auto_resume_sec=0.0):
        self.mode = 'hold'
        self.started = False
        self.generation = 0
        self.start_epoch = None
        # > 0: release automatically once every robot has been recovery-ready
        # for this long (a new fault resets it). 0: operator release only.
        self.auto_resume_sec = float(auto_resume_sec)
        self.all_ready_since = None

    def update(self, statuses, now, release=False):
        states = [s['state'] for s in statuses]
        if any(s == 'ABORTED' or (s == 'FINISHED' and x.get('exit_code', 1) != 0)
               for s, x in zip(states, statuses)):
            self.mode = 'abort'
        elif self.mode != 'abort':
            if not self.started and all(s == 'ARMED' for s in states):
                self.started = True
                self.start_epoch = now + 3
                self.mode = 'run'
            elif self.started and any(x['state'] == 'PAUSED' and
                                     x.get('resume_generation_seen', 0) >= self.generation for x in statuses):
                if self.mode != 'hold':
                    self.mode = 'hold'
                all_ready = all(x['state'] == 'PAUSED' and x.get('recovery_ready') for x in statuses)
                if not all_ready:
                    self.all_ready_since = None
                elif self.all_ready_since is None:
                    self.all_ready_since = now
                if self.auto_resume_sec > 0 and all_ready and now - self.all_ready_since >= self.auto_resume_sec:
                    release = True
                if release and all_ready:
                    self.generation += 1
                    self.mode = 'run'
                    self.all_ready_since = None
            else:
                self.all_ready_since = None
        return dict(mode=self.mode, resume_generation=self.generation, start_epoch=self.start_epoch)


def ssh_command(args, robot, remote_args):
    prefix = ['sshpass', '-e'] if args.password else []
    return prefix + ['ssh', '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=2',
                     '-o', 'ServerAliveCountMax=2', f"{args.user}@{robot['host']}",
                     ' '.join(shlex.quote(v) for v in remote_args)]


def validate_worker(run_id, log_path=None):
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
    log = open(log_path, 'w') if log_path else None
    try:
        output = dict(stdout=log, stderr=subprocess.STDOUT) if log else {}
        bag_rc = subprocess.run(command, env=env, **output).returncode
        motion_rc = subprocess.run([sys.executable, str(ROOT/'scripts/scenarios/validate_s1_circle_summary.py'),
                                    artifacts['summary'], '--max-radius-error',
                                    str(manifest['config'].get('max_radius_error', 0.20)),
                                    '--max-pose-age', '0.5'], **output).returncode
    finally:
        if log:
            log.close()
    verdict = 'PASS' if bag_rc == 0 and motion_rc == 0 else 'REJECTED_OR_INTERRUPTED'
    atomic_json(directory/'validation.json', dict(verdict=verdict, bag_exit_code=bag_rc, motion_exit_code=motion_rc,
                                                  log=str(log_path) if log_path else None))
    if not log_path:
        print(verdict, directory)
    return 0 if verdict == 'PASS' else 1


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
    parser.add_argument('--output', type=Path, default=Path('data/fleet_sessions'))
    args = parser.parse_args()
    if args.worker:
        return worker()
    if args.validate_worker:
        return validate_worker(args.validate_worker)
    if args.config is None:
        parser.error('--config is required')
    config = validate_config(json.loads(args.config.read_text()))
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
    atomic_json(output/'manifest.json', dict(run_id=run_id, config=config, created_epoch=time.time()))
    children = {}
    statuses = {r['name']: dict(state='PREPARING') for r in config['robots']}
    received = {r['name']: time.monotonic() for r in config['robots']}
    messages = queue.Queue()
    keys = queue.Queue()
    fleet = FleetState(config.get('auto_resume_sec', 0))
    link_abort_sec = float(config.get('link_abort_sec', 15))
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
    print('r + Enter: resume after all robots are recovery-ready%s. q + Enter / Ctrl+C: stop.'
          % (' (automatic after %.0f s)' % fleet.auto_resume_sec if fleet.auto_resume_sec > 0 else ''))
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
                command = fleet.update(effective, time.time(), release)
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
                time.sleep(0.25)
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
    print('Evidence:', output)
    for name, s in statuses.items():
        print('  %-8s %-22s %s' % (name, verdicts.get(name, 'PENDING'), s.get('validation_detail', '')), flush=True)
    print('Bags remain on robots; per-robot validator output is in each evidence directory.')
    return 1 if fleet.mode == 'abort' or any(p.returncode for p in children.values()) else 0


if __name__ == '__main__':
    sys.exit(main())
