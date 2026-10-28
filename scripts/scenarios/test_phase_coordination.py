"""Offline tests only: shared time, speed corrections and stop-only separation guard."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'logging'))
from drive_mocap_circle_ros2 import phase_speed, circle_velocity, normalise_args, parse_args
from fleet_session import FleetState, SeparationGuard, validate_config, local_guard_statuses

class PhaseTests(unittest.TestCase):
    def test_delay_commands_zero(self):
        self.assertEqual(phase_speed(.8,.1,2,10,0,.8,.2),(0,0))

    def test_behind_speeds_up_ahead_slows_down(self):
        self.assertGreater(phase_speed(.8,.1,10,0,.9,.8,.2)[0],.08)
        self.assertLess(phase_speed(.8,.1,10,0,1.1,.8,.2)[0],.08)

    def test_bounded_no_reverse_and_unwrapped_laps(self):
        self.assertEqual(phase_speed(.8,.1,0,0,1,.8,.2)[0],0)
        self.assertEqual(phase_speed(.8,.1,100,0,0,.8,.2)[0],.2)
        self.assertAlmostEqual(phase_speed(.8,.1,100,0,10,.8,.2)[0],.08)

    def test_common_clock_freezes_during_hold(self):
        fleet=FleetState()
        fleet.update([{'state':'ARMED'}],100)
        self.assertEqual(fleet.update([{'state':'RUNNING'}],104)['trajectory_time'],1)
        fleet.update([{'state':'RUNNING'}],105,external_fault='near')
        self.assertEqual(fleet.update([{'state':'PAUSED'}],110)['trajectory_time'],2)

    def test_no_resume_until_fault_clears_and_operator_releases(self):
        f=FleetState(); f.update([{'state':'ARMED'}],0)
        f.update([{'state':'RUNNING'}],4,external_fault='near')
        ready=[{'state':'PAUSED','recovery_ready':True}]
        self.assertEqual(f.update(ready,5,True,'near')['mode'],'hold')
        self.assertEqual(f.update(ready,6)['mode'],'hold')
        self.assertEqual(f.update(ready,7,True)['mode'],'run')

    def test_stop_override_has_no_heading_turn(self):
        a=normalise_args(parse_args(['--cmd-topic','/test','--pose-topic','/pose']))
        self.assertEqual(circle_velocity(a,.1,.5,0),(0,0))

    def test_speed_variation_is_corrected_in_ideal_scalar_simulation(self):
        for radius in (.8,1.5):
            for factor in (.85,1.15):
                progress=0
                for i in range(18000):
                    t=i*.01
                    speed,error=phase_speed(radius,.1,t,0,progress,.8,.2)
                    progress+=speed*factor/radius*.01
                    if t>5: self.assertLess(abs(error),.03)

class GuardTests(unittest.TestCase):
    def status(self,x,y,t): return dict(state='RUNNING',x=x,y=y,pose_stamp=t)
    def test_close_stopped_pair_holds(self):
        g=SeparationGuard(.55)
        self.assertIn('close',g.check({'a':self.status(0,0,10),'b':self.status(.4,0,10)},10))
    def test_approaching_pair_holds_before_contact(self):
        g=SeparationGuard(.55)
        g.check({'a':self.status(0,0,10),'b':self.status(1,0,10)},10)
        self.assertIn('close',g.check({'a':self.status(.1,0,10.5),'b':self.status(.9,0,10.5)},10.5))
    def test_separating_pair_does_not_hold(self):
        g=SeparationGuard(.55)
        g.check({'a':self.status(0,0,10),'b':self.status(1,0,10)},10)
        self.assertEqual(g.check({'a':self.status(-.1,0,10.5),'b':self.status(1.1,0,10.5)},10.5),'')
    def test_stale_or_invalid_pose_holds(self):
        for t in (9,math.nan,11):
            self.assertTrue(SeparationGuard(.55).check({'a':self.status(0,0,t)},10))
    def test_missing_pose_holds(self):
        self.assertTrue(SeparationGuard(.55).check({'a':{'state':'RUNNING'}},10))

    def test_local_pose_replaces_stale_robot_status(self):
        statuses={'a':self.status(0,0,1)}
        local={'a':dict(x=0,y=0,pose_stamp=10,pose_frame='world')}
        self.assertEqual(SeparationGuard(.55).check(local_guard_statuses(statuses,local),10),'')
        self.assertTrue(SeparationGuard(.55).check(local_guard_statuses(statuses,{}),10))

    def test_mixed_frames_hold(self):
        statuses={'a':dict(self.status(0,0,10),pose_frame='world'),
                  'b':dict(self.status(2,0,10),pose_frame='other')}
        self.assertEqual(SeparationGuard(.55).check(statuses,10),'separation_frame_mismatch')

    def test_observed_stationary_robot_is_protected_without_motion_status(self):
        statuses={'a':self.status(0,0,10)}
        poses={'a':dict(x=0,y=0,pose_stamp=10),'b':dict(x=.4,y=0,pose_stamp=10)}
        self.assertIn('close',SeparationGuard(.55).check(local_guard_statuses(statuses,poses,['b']),10))
        del poses['b']
        self.assertIn('missing',SeparationGuard(.55).check(local_guard_statuses(statuses,poses,['b']),10))

    def test_stationary_marker_jitter_does_not_predict_collision(self):
        g=SeparationGuard(.55)
        for i in range(300):
            t=10+i*.002
            jitter=.001 if i%2 else -.001
            self.assertEqual(g.check({'a':self.status(jitter,0,t),
                                     'b':self.status(.7-jitter,0,t)},t),'')

if __name__=='__main__': unittest.main()
