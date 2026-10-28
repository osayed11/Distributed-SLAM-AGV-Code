#!/usr/bin/env python3
"""Own one persistent, non-moving sensor bringup using the system service manager."""

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def require_linger():
    result = subprocess.run(['loginctl', 'show-user', str(os.getuid()),
                             '--property=Linger', '--value'],
                            capture_output=True, text=True, check=True)
    if result.stdout.strip() != 'yes':
        raise RuntimeError('Warm drivers require persistent user IPC: run '
                           'sudo loginctl enable-linger ' + str(os.getuid()) +
                           ' before prepare; otherwise SSH logout can remove DDS shared memory')


def fingerprint(robot):
    paths = [ROOT / 'agv2_ws/install/myagv_odometry/lib/myagv_odometry/myagv_odometry_node',
             ROOT / 'scripts/network/load_ros_transport_env.sh']
    paths += sorted((ROOT / 'agv2_ws/src/agv_bringup').rglob('*.py'))
    paths += sorted((ROOT / 'agv2_ws/src/agv_bringup').rglob('*.yaml'))
    paths += sorted(Path('/etc/orkar').glob('*.env'))
    digest = hashlib.sha256()
    digest.update(str(ROOT).encode())
    for path in paths:
        digest.update(str(path).encode())
        digest.update(path.read_bytes())
    return dict(robot=robot, boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                fingerprint=digest.hexdigest(), cmd_topic=f'/{robot}/cmd_vel')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'check', 'stop', 'launch'])
    parser.add_argument('robot')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'agv[0-9]+', args.robot):
        parser.error('robot must be agv<number>')
    if socket.gethostname().split('.')[0] != args.robot:
        parser.error('Hostname does not match requested robot; refusing wrong-robot operations')
    unit = f'orkar-sensors-{args.robot}.service'
    directory = Path.home() / '.local/state/orkar' / args.robot
    log = directory / 'bringup.log'
    manifest = directory / 'sensor_session.json'
    if args.action == 'launch':
        command = ('source /opt/ros/humble/setup.bash; source scripts/network/load_ros_transport_env.sh; '
                   'source agv2_ws/install/setup.bash; '
                   'exec ros2 launch agv_bringup bringup.launch.py agv_serial_port:=/dev/ttyACM0 '
                   'agv_color_profile:=640x480x15 agv_depth_profile:=640x480x15 '
                   f'enable_sync:=false initial_reset:=false agv_cmd_vel_topic:=/{args.robot}/cmd_vel')
        os.chdir(ROOT)
        os.execvp('bash', ['bash', '-e', '-c', command])
    if args.action in ('prepare', 'stop'):
        lock = open('/tmp/orkar_s1_motion.lock', 'a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('A scenario owns the robot; stop the scenario before changing drivers')
    if args.action == 'stop':
        return subprocess.call(['sudo', '-n', 'systemctl', 'stop', unit])
    require_linger()
    expected = fingerprint(args.robot)
    active = subprocess.run(['systemctl', 'is-active', '--quiet', unit]).returncode == 0
    if active:
        if not manifest.exists() or json.loads(manifest.read_text()) != expected:
            raise RuntimeError('Sensor configuration changed; explicitly stop and prepare the session again')
        print(log)
        return 0
    if args.action == 'check':
        raise RuntimeError('No prepared sensor session; run sensor_session.py prepare ' + args.robot)
    # Do not kill somebody else's process or silently attach to an unknown stack.
    busy = subprocess.run(['pgrep', '-f', 'myagv_odometry_node|realsense2_camera_node|ros2 bag record'],
                          stdout=subprocess.DEVNULL).returncode == 0
    if busy:
        raise RuntimeError('Existing unmanaged drivers/recorders: stop their owner before preparing')
    directory.mkdir(parents=True, exist_ok=True)
    log.write_text('')
    subprocess.run(['sudo', '-n', 'systemd-run', '--unit', unit, '--collect',
                    '--property=Type=exec', '--property=KillMode=control-group',
                    '--property=Restart=no', '--property=TimeoutStopSec=15',
                    '--uid=' + str(os.getuid()), '--working-directory=' + str(ROOT),
                    '--property=StandardOutput=append:' + str(log),
                    '--property=StandardError=append:' + str(log),
                    '--setenv=HOME=' + str(Path.home()),
                    sys.executable, str(Path(__file__).resolve()), 'launch', args.robot], check=True)
    manifest.write_text(json.dumps(expected, indent=2) + '\n')
    print(log)
    print('Prepared drivers only. Fresh topic checks and recorder gates still run for every attempt.')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print('ERROR: ' + str(exc), file=sys.stderr)
        sys.exit(1)
