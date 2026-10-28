#!/usr/bin/env python3
"""Offline fault injection for recovery and fleet barriers; no ROS or robots needed."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/logging'))
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(ROOT / 'scripts/diagnostics'))
from motion_recovery import MotionRecovery, atomic_json, read_control
from fleet_session import FleetState, run_environment, validate_config
from sensor_session import require_linger
from check_live_topics import assess_stream
from validate_ros2_bag import required_specs
from watch_realsense_runtime import watch


class CollectionPolicyTests(unittest.TestCase):
    def test_rgbd_quality_minimum_is_ten_not_twelve_or_thirteen(self):
        with patch.dict('os.environ', {}, clear=True):
            specs = {s.label: s for s in required_specs()}
        for label in ('color_image', 'depth_image'):
            self.assertEqual(specs[label].min_hz, 10)
            self.assertEqual(specs[label].target_hz, 15)
        result = assess_stream([i / 11.5 for i in range(115)], 10, 10, 10, 0.5)
        self.assertTrue(result['passed'])
        self.assertTrue(result['quality_passed'])

    def test_rate_shortfall_warns_without_blocking_collection(self):
        result = assess_stream([i / 9.5 for i in range(95)], 10, 10, 10, 0.5)
        self.assertTrue(result['passed'])
        self.assertFalse(result['quality_passed'])
        self.assertIn('rate_below_quality_minimum', result['warnings'])

    def test_missing_or_currently_stalled_required_stream_blocks_launch(self):
        for samples in ([], [9.9], [0, 1, 2]):
            self.assertFalse(assess_stream(samples, 10, 10, 10, 0.5)['passed'])

    def test_recovered_sensor_gap_is_reported_not_hidden_or_launch_blocking(self):
        result = assess_stream([0, 0.1, 9.8, 9.9], 10, 10, 10, 0.5)
        self.assertTrue(result['passed'])
        self.assertIn('observed_stream_gap', result['warnings'])
        self.assertGreater(result['max_gap_sec'], 9)

    def test_duplicate_gt_rate_still_blocks(self):
        result = assess_stream([i / 200 for i in range(2000)], 10, 10, 30, 0.5, 120)
        self.assertFalse(result['passed'])

    def test_imu_log_warning_retains_evidence_without_failure_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / 'bringup.log'
            status = root / 'status.txt'
            log.write_text("iio_hid_sensor: Frames didn't arrived within the predefined interval\n")
            for warn_only, expected_rc, prefix in ((True, 0, 'WARN'), (False, 2, 'FAIL')):
                with patch('watch_realsense_runtime.capture_evidence') as capture, \
                     patch('watch_realsense_runtime.signal.signal'):
                    rc = watch(log, status, root / 'evidence', True, 0.01, warn_only=warn_only)
                self.assertEqual(rc, expected_rc)
                self.assertTrue(status.read_text().startswith(prefix + '_RUNTIME_GUARD'))
                capture.assert_called_once()


class SessionTests(unittest.TestCase):
    def test_service_refuses_logout_ipc_cleanup(self):
        with patch('sensor_session.subprocess.run') as run:
            run.return_value.stdout = 'no\n'
            with self.assertRaisesRegex(RuntimeError, 'enable-linger'):
                require_linger()
            run.return_value.stdout = 'yes\n'
            require_linger()


class RecoveryTests(unittest.TestCase):
    def test_gt_only_pause_is_eligible_but_never_releases_itself(self):
        m = MotionRecovery()
        m.tick(0, 'mocap pose stale for 0.591s')
        m.tick(1)
        self.assertTrue(m.auto_resume_safe)
        self.assertEqual(m.tick(3), 'PAUSED')

    def test_unsafe_fault_remains_ineligible_after_gt_recovers(self):
        m = MotionRecovery()
        m.tick(0, 'implausible_pose_jump')
        m.tick(.1, 'mocap pose stale for 0.591s')
        m.tick(1)
        self.assertTrue(m.ready(2))
        self.assertFalse(m.auto_resume_safe)

    def test_dropout_stops_and_never_auto_resumes(self):
        m = MotionRecovery()
        self.assertEqual(m.tick(0), 'RUNNING')
        self.assertEqual(m.tick(1, 'stale_pose'), 'PAUSED')
        self.assertEqual(m.tick(2), 'PAUSED')
        self.assertTrue(m.ready(3))
        self.assertEqual(m.tick(3), 'PAUSED')
        self.assertEqual(m.tick(4, generation=1), 'RUNNING')
        self.assertEqual(m.paused_seconds, 3)

    def test_early_resume_not_remembered(self):
        m = MotionRecovery()
        m.tick(0, 'stale')
        m.tick(1, 'stale', generation=1)
        m.tick(2, generation=1)
        self.assertEqual(m.tick(4, generation=1), 'PAUSED')
        self.assertEqual(m.tick(5, generation=2), 'RUNNING')

    def test_intermittent_recovery_requires_continuity(self):
        m = MotionRecovery()
        m.tick(0, 'stale')
        m.tick(1)
        m.tick(1.5, 'jump')
        m.tick(2)
        self.assertFalse(m.ready(2.5))
        self.assertTrue(m.ready(3))

    def test_recovery_timeout_is_latched(self):
        m = MotionRecovery(timeout=2)
        m.tick(0, 'stale')
        self.assertEqual(m.tick(2), 'ABORTED')
        self.assertEqual(m.tick(3, generation=10), 'ABORTED')

    def test_pause_count_budget(self):
        m = MotionRecovery(max_pauses=1)
        m.tick(0, 'stale'); m.tick(1); m.tick(3, generation=1)
        self.assertEqual(m.tick(4, 'stale'), 'ABORTED')

    def test_cumulative_pause_budget(self):
        m = MotionRecovery(max_pause_seconds=4)
        m.tick(0, 'stale'); m.tick(1); m.tick(3, generation=1)
        m.tick(4, 'stale', generation=1)
        self.assertEqual(m.tick(5), 'ABORTED')

    def test_fleet_hold_needs_explicit_release(self):
        m = MotionRecovery()
        m.tick(0, permit=False)
        m.tick(1, permit=False)
        self.assertTrue(m.ready(2))
        self.assertEqual(m.tick(2, permit=False, generation=1), 'PAUSED')
        self.assertEqual(m.tick(3, generation=1), 'RUNNING')

    def test_expired_or_cross_session_lease_is_not_motion_permission(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)/'control.json'
            atomic_json(p, dict(run_id='abc', mode='run', resume_generation=0, received_monotonic=5))
            with patch('motion_recovery.time.monotonic', return_value=5.5):
                self.assertEqual(read_control(p, 'abc', 1)[1], '')
                self.assertNotEqual(read_control(p, 'xyz', 1)[1], '')
            with patch('motion_recovery.time.monotonic', return_value=6):
                self.assertNotEqual(read_control(p, 'abc', 1)[1], '')
            with patch('motion_recovery.time.monotonic', return_value=4.9):
                self.assertNotEqual(read_control(p, 'abc', 1)[1], '')
            p.write_text('{')
            self.assertEqual(read_control(p, 'abc', 1)[0]['mode'], 'hold')

    def test_heartbeat_refreshed_during_read_is_not_rejected_as_future(self):
        clock = [5.0]
        def refreshed_file(*args, **kwargs):
            clock[0] = 5.02
            return json.dumps(dict(run_id='abc', mode='hold', resume_generation=0,
                                   received_monotonic=5.01))
        with patch.object(Path, 'read_text', side_effect=refreshed_file), \
                patch('motion_recovery.time.monotonic', side_effect=lambda: clock[0]):
            command, fault = read_control('/unused/control.json', 'abc', 1)
        self.assertEqual(fault, '')
        self.assertEqual(command['mode'], 'hold')


class FleetTests(unittest.TestCase):
    @staticmethod
    def recovered(now, generation=0):
        return [dict(state='PAUSED', recovery_ready=True, auto_resume_safe=True,
                     reason=reason, pose_stamp=now, resume_generation_seen=generation)
                for reason in ('mocap pose stale for 0.591s', 'fleet_pause')]

    def test_gt_auto_resume_requires_stability_and_preserves_phase_time(self):
        f = FleetState(auto_resume_gt=True)
        f.started = True; f.mode = 'run'; f.start_epoch = 0; f.last_time = 1
        self.assertEqual(f.update(self.recovered(2), 2)['mode'], 'hold')
        phase = f.trajectory_time
        self.assertEqual(f.update(self.recovered(2.5), 2.5)['mode'], 'hold')
        result = f.update(self.recovered(3), 3)
        self.assertEqual(result['mode'], 'run')
        self.assertTrue(result['auto_resumed'])
        self.assertEqual(result['resume_generation'], 1)
        self.assertEqual(result['trajectory_time'], phase)
        # Stale acknowledgement of the old generation must not trigger another release.
        self.assertEqual(f.update(self.recovered(3.1), 3.1)['resume_generation'], 1)

    def test_auto_resume_waits_for_every_robot_and_fresh_pose(self):
        for modification in ({'recovery_ready': False}, {'pose_stamp': 0},
                             {'pose_stamp': float('nan')}, {'state': 'RUNNING'},
                             {'resume_generation_seen': 9}):
            f = FleetState(auto_resume_gt=True); f.started = True
            for now in (1, 2, 3):
                statuses = self.recovered(now)
                statuses[1].update(modification)
                self.assertEqual(f.update(statuses, now)['mode'], 'hold')

    def test_auto_resume_latches_non_gt_fault_until_explicit_release(self):
        for external in ('predicted_close_approach: agv1 agv2',
                         'separation_pose_jump: agv1', 'separation_frame_mismatch'):
            f = FleetState(auto_resume_gt=True); f.started = True
            f.update(self.recovered(1), 1, external_fault=external)
            for now in (2, 4):
                self.assertEqual(f.update(self.recovered(now), now)['mode'], 'hold')
            self.assertEqual(f.update(self.recovered(5), 5, release=True)['mode'], 'run')

    def test_unsafe_or_old_controller_cannot_auto_resume(self):
        for flag in (False, None):
            f = FleetState(auto_resume_gt=True); f.started = True
            bad = self.recovered(1)
            bad[0]['auto_resume_safe'] = flag
            f.update(bad, 1)
            self.assertEqual(f.update(self.recovered(4), 4)['mode'], 'hold')

    def test_reappearing_gt_gap_restarts_clear_window(self):
        f = FleetState(auto_resume_gt=True); f.started = True
        f.update(self.recovered(1), 1)
        f.update(self.recovered(1.9), 1.9, external_fault='separation_pose_stale: agv1')
        self.assertEqual(f.update(self.recovered(2), 2)['mode'], 'hold')
        self.assertEqual(f.update(self.recovered(2.9), 2.9)['mode'], 'hold')
        self.assertEqual(f.update(self.recovered(3), 3)['mode'], 'run')

    def test_gt_auto_resume_is_opt_in_and_abort_stays_latched(self):
        f = FleetState(); f.started = True
        f.update(self.recovered(1), 1)
        self.assertEqual(f.update(self.recovered(3), 3)['mode'], 'hold')
        f = FleetState(auto_resume_gt=True); f.started = True
        f.update([dict(state='ABORTED')], 1)
        self.assertEqual(f.update(self.recovered(3), 3)['mode'], 'abort')

    def test_per_robot_speed_override_and_default(self):
        robot = dict(name='agv1', host='agv1.local', pose_topic='/gt/agv1/pose',
                     radius=1, center_x=0, center_y=0, direction='cw', forward_yaw_offset_deg=0)
        config = dict(duration=180, linear=0.08, robots=[robot])
        for override, expected, minimum in ((None, 0.08, 0.07), (0.06, 0.06, 0.06), (0.07, 0.07, 0.07)):
            selected = dict(robot, **({'linear': override} if override is not None else {}))
            validated = validate_config(dict(config, robots=[selected]))
            env = run_environment(validated, selected, Path('/tmp/test-run'))
            self.assertEqual(float(env['S1_LINEAR']), expected)
            self.assertEqual(float(env['S1_MIN_LINEAR']), minimum)
            self.assertEqual(config['linear'], 0.08)
        for invalid in (0, -0.1, 0.21, True, '0.06', None, float('nan'), float('inf')):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                validate_config(dict(config, robots=[dict(robot, linear=invalid)]))

    def test_config_rejects_duplicate_pose_or_host_and_nonfinite_limits(self):
        base = dict(duration=180, linear=0.08, robots=[dict(name='agv1', host='agv1.local',
                    pose_topic='/gt/agv1/pose', radius=1, center_x=0, center_y=0,
                    direction='cw', forward_yaw_offset_deg=0)])
        self.assertIs(validate_config(base), base)
        for modification in ({'linear': float('nan')}, {'duration': True}, {'recovery_timeout': -1}):
            with self.assertRaises(ValueError):
                validate_config(dict(base, **modification))
        duplicate = dict(base['robots'][0], name='agv2')
        with self.assertRaises(ValueError):
            validate_config(dict(base, robots=base['robots']+[duplicate]))

    def test_barrier_waits_for_all_recorders(self):
        f = FleetState()
        self.assertEqual(f.update([dict(state='ARMED'), dict(state='PREPARING')], 10)['mode'], 'hold')
        result = f.update([dict(state='ARMED'), dict(state='ARMED')], 11)
        self.assertEqual(result['mode'], 'run')
        self.assertEqual(result['start_epoch'], 14)

    def test_one_fault_holds_everyone(self):
        f = FleetState(); f.started = True; f.mode = 'run'
        result = f.update([dict(state='RUNNING'), dict(state='PAUSED')], 20)
        self.assertEqual(result['mode'], 'hold')
        self.assertEqual(f.update([dict(state='RUNNING'), dict(state='RUNNING')], 21)['mode'], 'hold')

    def test_resume_requires_all_ready_and_no_command_oscillation(self):
        f = FleetState(); f.started = True
        partial = [dict(state='PAUSED', recovery_ready=True), dict(state='PAUSED', recovery_ready=False)]
        self.assertEqual(f.update(partial, 1, True)['mode'], 'hold')
        ready = [dict(state='PAUSED', recovery_ready=True), dict(state='PAUSED', recovery_ready=True)]
        self.assertEqual(f.update(ready, 2, True)['mode'], 'run')
        self.assertEqual(f.update(ready, 2.1)['mode'], 'run')
        new_fault = [dict(state='RUNNING'), dict(state='PAUSED', resume_generation_seen=1)]
        self.assertEqual(f.update(new_fault, 3)['mode'], 'hold')

    def test_abort_latched(self):
        f = FleetState()
        self.assertEqual(f.update([dict(state='ABORTED')], 1)['mode'], 'abort')
        self.assertEqual(f.update([dict(state='ARMED')], 2, True)['mode'], 'abort')

    def test_recorder_wrapper_failure_aborts(self):
        f = FleetState(); f.started = True; f.mode = 'run'
        self.assertEqual(f.update([dict(state='FINISHED', exit_code=71), dict(state='RUNNING')], 2)['mode'], 'abort')

    def test_auto_resume_releases_only_after_all_ready_continuously(self):
        f = FleetState(auto_resume_sec=2); f.started = True; f.mode = 'run'
        ready = [dict(state='PAUSED', recovery_ready=True, auto_resume_safe=True,
                      reason='mocap pose stale for 0.6s', pose_stamp=10),
                 dict(state='PAUSED', recovery_ready=True, auto_resume_safe=True,
                      reason='fleet_pause', pose_stamp=10)]
        partial = [dict(state='PAUSED', recovery_ready=True), dict(state='PAUSED', recovery_ready=False)]
        self.assertEqual(f.update(ready, 10)['mode'], 'hold')
        for s in ready: s['pose_stamp'] = 11.5
        self.assertEqual(f.update(ready, 11.5)['mode'], 'hold')
        # A robot dropping out of readiness restarts the timer.
        self.assertEqual(f.update(partial, 11.8)['mode'], 'hold')
        # Partial status lacks safety evidence and must latch automatic recovery off.
        self.assertTrue(f.auto_blocked)
        f = FleetState(auto_resume_sec=2); f.started = True; f.mode = 'run'
        for s in ready: s['pose_stamp'] = 12
        self.assertEqual(f.update(ready, 12)['mode'], 'hold')
        for s in ready: s['pose_stamp'] = 13.5
        self.assertEqual(f.update(ready, 13.5)['mode'], 'hold')
        for s in ready: s['pose_stamp'] = 14.1
        result = f.update(ready, 14.1)
        self.assertEqual((result['mode'], result['resume_generation']), ('run', 1))
        # Default construction keeps the operator-only release.
        g = FleetState(); g.started = True; g.mode = 'run'
        g.update(ready, 0)
        self.assertEqual(g.update(ready, 1000)['mode'], 'hold')
        self.assertEqual(g.update(ready, 1001, True)['mode'], 'run')

    def test_recovery_budget_knobs_are_validated_and_forwarded(self):
        robot = dict(name='agv1', host='agv1.local', pose_topic='/gt/agv1/pose',
                     radius=1, center_x=0, center_y=0, direction='cw', forward_yaw_offset_deg=0)
        config = validate_config(dict(duration=180, linear=0.08, robots=[robot], recovery_timeout=45,
                                      max_pauses=20, max_pause_seconds=120, recovery_stable_sec=2,
                                      auto_resume_sec=3, link_abort_sec=15))
        env = run_environment(config, robot, Path('/tmp/test-run'))
        self.assertEqual((env['S1_RECOVERY_TIMEOUT'], env['S1_MAX_PAUSES'], env['S1_MAX_PAUSE_SECONDS'],
                          env['S1_RECOVERY_STABLE_SEC']), ('45', '20', '120', '2'))
        defaults = run_environment(validate_config(dict(duration=180, linear=0.08, robots=[robot])),
                                   robot, Path('/tmp/test-run'))
        self.assertEqual((defaults['S1_MAX_PAUSES'], defaults['S1_MAX_PAUSE_SECONDS']), ('3', '30'))
        for bad in ({'max_pauses': -1}, {'auto_resume_sec': -0.1}, {'link_abort_sec': 2},
                    {'max_pause_seconds': 0}, {'recovery_stable_sec': float('nan')}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                validate_config(dict(duration=180, linear=0.08, robots=[robot], **bad))


if __name__ == '__main__':
    unittest.main()
