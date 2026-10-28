#!/usr/bin/env python3
"""Exercise the actual drive loop with a simulated clock, pose stream and publisher."""

import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import drive_mocap_circle_ros2 as circle
from motion_recovery import atomic_json
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scenarios'))
from fleet_session import FleetState


class Clock:
    def __init__(self): self.now = 0.0
    def monotonic(self): return self.now
    def time(self): return 1700000000 + self.now
    def sleep(self, duration): self.now += max(0, duration)


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


class FakeNode:
    def __init__(self, clock, outage=None, control=None, resume_at=None, shared_clock=False,
                 status=None, auto_recovery=False):
        self.clock, self.outage, self.control, self.resume_at = clock, outage, control, resume_at
        self.pose_fault = ''
        self.shared_clock = shared_clock
        self.status = status
        self.fleet = FleetState(auto_resume_gt=True) if auto_recovery else None
        if self.fleet:
            self.fleet.started = True
            self.fleet.mode = 'run'
            self.fleet.start_epoch = clock.time()
        self.pose_samples = 0
        self.commands = []
        self.pose = None
        self.pub = SimpleNamespace(publish=self.publish)
        self.tick()

    def publish(self, command):
        self.commands.append((self.clock.now, command.linear.x, command.angular.z))

    def tick(self):
        t = self.clock.now
        if self.control:
            command = dict(run_id='test', mode='run', received_monotonic=t,
                           resume_generation=int(self.resume_at is not None and t >= self.resume_at))
            if self.fleet:
                status = json.loads(self.status.read_text()) if self.status.exists() else dict(state='RUNNING')
                command.update(self.fleet.update([status], self.clock.time()))
            if self.shared_clock: command['trajectory_time'] = max(0,t-.5)
            atomic_json(self.control, command)
        if self.outage and self.outage[0] <= t < self.outage[1]:
            return
        # Fixed pose is sufficient to check permission/stop timing, not vehicle dynamics.
        self.pose = circle.PoseSample(1, 0, 0, -math.pi/2, 'world', self.clock.time(), t)
        self.pose_samples += 1

    def wait_for_pose(self, timeout):
        self.tick()
        return self.pose

    def publish_zero(self, seconds):
        end = self.clock.now + seconds
        while self.clock.now < end:
            self.publish(Twist()); self.clock.sleep(0.02); self.tick()


class DriveLoopTests(unittest.TestCase):
    def test_stop_handler_never_reenters_logging(self):
        circle.stop_requested = False
        with patch('builtins.print', side_effect=RuntimeError('reentrant stdout')) as output:
            circle.request_stop(15)
        output.assert_not_called()
        self.assertTrue(circle.stop_requested)
        circle.stop_requested = False

    def run_drive(self, outage=None, resume_at=None, phase_options=(), shared_clock=False, auto_recovery=False):
        clock = Clock()
        with tempfile.TemporaryDirectory() as directory:
            control = Path(directory)/'control.json'
            status = Path(directory)/'status.json'
            args = circle.normalise_args(circle.parse_args([
                '--pose-topic', '/gt/test', '--cmd-topic', '/test/cmd_vel', '--radius', '1',
                '--center-x', '0', '--center-y', '0', '--clockwise', '--duration', '2',
                '--stuck-timeout', '100', '--pose-timeout', '0.2', '--recovery-timeout', '3',
                '--recovery-stable-sec', '0.3', '--control-file', str(control), '--run-id', 'test',
                '--status-json', str(status),
                '--summary-json', str(Path(directory)/'summary.json'), '--yes', *phase_options]))
            node = FakeNode(clock, outage, control, resume_at, shared_clock, status, auto_recovery)
            ros = SimpleNamespace(ok=lambda: clock.now < 20,
                                  spin_once=lambda node, timeout_sec=0: (clock.sleep(timeout_sec), node.tick()))
            with patch.object(circle, 'time', clock), patch.object(circle, 'rclpy', ros), \
                 patch('motion_recovery.time', clock), \
                 patch.object(circle, 'Twist', Twist), contextlib.redirect_stdout(io.StringIO()):
                circle.stop_requested = False
                summary = circle.drive(node, args)
            saved = json.loads((Path(directory)/'summary.json').read_text())
            return summary, node.commands, saved

    def test_healthy_run_completes(self):
        summary, commands, saved = self.run_drive()
        self.assertTrue(summary.reached_duration)
        self.assertFalse(saved['interrupted'])
        self.assertEqual(commands[-1][1:], (0, 0))

    def test_phase_mode_missing_shared_clock_never_moves(self):
        summary, commands, saved = self.run_drive(phase_options=('--phase-omega','.1'))
        self.assertFalse(summary.reached_duration)
        self.assertTrue(all(v==0 and w==0 for _,v,w in commands))

    def test_phase_start_delay_keeps_both_commands_zero(self):
        summary, commands, saved = self.run_drive(
            phase_options=('--phase-omega','.1','--motion-delay','1'),shared_clock=True)
        self.assertTrue(summary.reached_duration)
        self.assertTrue(all(v==0 and w==0 for t,v,w in commands if t<1.5))
        self.assertTrue(any(v>0 for _,v,_ in commands))

    def test_dropout_stops_then_explicit_resume_preserves_active_duration(self):
        summary, commands, saved = self.run_drive(outage=(1, 1.6), resume_at=2.2)
        self.assertTrue(summary.reached_duration)
        self.assertTrue(saved['interrupted'])
        self.assertEqual(summary.pause_count, 1)
        self.assertGreater(summary.elapsed_sec, summary.active_sec + 0.5)
        self.assertTrue(all(v == 0 and w == 0 for t,v,w in commands if 1.25 < t < 2.2))
        self.assertTrue(any(v > 0 for t,v,w in commands if t > 2.2))
        self.assertEqual((summary.center_x_m, summary.center_y_m), (0, 0))

    def test_reacquisition_without_resume_aborts_stopped(self):
        summary, commands, saved = self.run_drive(outage=(1, 1.6))
        self.assertFalse(summary.reached_duration)
        self.assertIn('recovery_budget_exhausted', summary.abort_reason)
        self.assertTrue(all(v == 0 and w == 0 for t,v,w in commands if t > 1.25))

    def test_actual_drive_loop_auto_recovers_gt_and_finishes_active_duration(self):
        summary, commands, saved = self.run_drive(outage=(1, 1.6), auto_recovery=True)
        self.assertTrue(summary.reached_duration)
        self.assertEqual(summary.pause_count, 1)
        self.assertTrue(saved['interrupted'])
        self.assertGreaterEqual(summary.active_sec, 2)
        self.assertTrue(all(v == 0 and w == 0 for t,v,w in commands if 1.25 < t < 2.8))
        self.assertTrue(any(v > 0 for t,v,w in commands if t > 3))
        self.assertEqual(commands[-1][1:], (0, 0))

    def test_prolonged_dropout_aborts_stopped(self):
        summary, commands, _ = self.run_drive(outage=(1, 100))
        self.assertFalse(summary.reached_duration)
        self.assertEqual(commands[-1][1:], (0, 0))

    def test_source_clock_and_tangent_are_start_gates(self):
        clock = Clock()
        node = FakeNode(clock)
        args = circle.normalise_args(circle.parse_args(['--pose-topic','/gt/test','--cmd-topic','/test/cmd_vel',
                                                        '--radius','1','--clockwise']))
        with patch.object(circle, 'time', clock):
            circle.check_start_pose(node, args, 0, 0, 0)
            node.pose.yaw = 0
            with self.assertRaisesRegex(RuntimeError, 'tangent'):
                circle.check_start_pose(node, args, 0, 0, 0)
            node.pose.yaw = -math.pi/2
            node.pose.stamp_sec -= 2
            with self.assertRaisesRegex(RuntimeError, 'clock'):
                circle.check_start_pose(node, args, 0, 0, 0)

    def test_repeated_timestamp_does_not_freshen_pose(self):
        clock = Clock()
        node = circle.MocapCircleNode.__new__(circle.MocapCircleNode)
        node.pose = None; node.pose_samples = 0; node.pose_fault = ''
        node.args = SimpleNamespace(pose_timeout=0.3, max_pose_speed=1.0, max_pose_yaw_rate=3.0)
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1700000000, nanosec=0), frame_id='world'),
                              pose=SimpleNamespace(position=SimpleNamespace(x=1., y=0., z=0.),
                                                   orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        with patch.object(circle, 'time', clock):
            node.pose_cb(msg)
            original = node.pose
            clock.now += 0.1
            node.pose_cb(msg)
            self.assertIs(node.pose, original)
            self.assertEqual(node.pose.wall_time, 0)
            self.assertIn('timestamp', node.pose_fault)
            msg.header.stamp.nanosec = 100000000
            msg.pose.position.x = 3
            node.pose_cb(msg)
            self.assertEqual(node.pose_fault, 'implausible_pose_jump')
            self.assertIs(node.pose, original)

    def test_nan_parameters_refused(self):
        args = circle.parse_args(['--pose-topic','/gt/test','--cmd-topic','/test/cmd_vel', '--radius','nan'])
        with self.assertRaises(ValueError):
            circle.normalise_args(args)


if __name__ == '__main__':
    unittest.main()
