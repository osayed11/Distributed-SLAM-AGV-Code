#!/usr/bin/env python3
"""Offline tests for the fleet planner: placement geometry only, no router or robots."""

import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fleet_plan import MAX_HEADING_ERROR_DEG, decode_pose_stamped, plan  # noqa: E402

BASE = dict(duration=70, linear=0.10, robots=[
    dict(name='agv105', host='agv105.local', pose_topic='/optitrack/rigid_bodies/orkar_agv105', forward_yaw_offset_deg=0),
    dict(name='agv106', host='agv106.local', pose_topic='/optitrack/rigid_bodies/orkar_agv106', forward_yaw_offset_deg=-7),
    dict(name='agv101', host='agv101.local', pose_topic='/optitrack/rigid_bodies/orkar_agv101', forward_yaw_offset_deg=0),
    dict(name='agv103', host='agv103.local', pose_topic='/optitrack/rigid_bodies/orkar_agv103', forward_yaw_offset_deg=180),
])


def on_ring(cx, cy, r, bearing_deg, direction, offset_deg, heading_err_deg=0.0):
    b = math.radians(bearing_deg)
    tangent = b + (-math.pi / 2 if direction == 'cw' else math.pi / 2)
    yaw = tangent - math.radians(offset_deg) + math.radians(heading_err_deg)
    return (cx + r * math.cos(b), cy + r * math.sin(b), 0.3, yaw)


class PlanTests(unittest.TestCase):
    def test_lined_up_fleet_on_two_rings_plans_cleanly(self):
        cx, cy = -0.655, 0.002
        poses = {'agv105': on_ring(cx, cy, 1.0, -90, 'cw', 0), 'agv106': on_ring(cx, cy, 1.0, 90, 'cw', -7),
                 'agv101': on_ring(cx, cy, 1.7, -90, 'cw', 0), 'agv103': on_ring(cx, cy, 1.7, 90, 'cw', 180)}
        config, lines, errors = plan(BASE, poses, 'cw', rings=[1.0, 1.7])
        self.assertEqual(errors, [])
        radii = {r['name']: r['radius'] for r in config['robots']}
        self.assertEqual(radii, {'agv105': 1.0, 'agv106': 1.0, 'agv101': 1.7, 'agv103': 1.7})
        self.assertAlmostEqual(config['robots'][0]['center_x'], cx, places=2)
        self.assertAlmostEqual(config['robots'][0]['center_y'], cy, places=2)
        self.assertTrue(all(r['direction'] == 'cw' for r in config['robots']))

    def test_robot_facing_the_wrong_way_is_named_with_the_turn(self):
        poses = {'agv105': on_ring(0, 0, 1.0, 0, 'cw', 0, heading_err_deg=120),
                 'agv106': on_ring(0, 0, 1.0, 180, 'cw', -7),
                 'agv101': on_ring(0, 0, 1.7, 0, 'cw', 0), 'agv103': on_ring(0, 0, 1.7, 180, 'cw', 180)}
        config, lines, errors = plan(BASE, poses, 'cw', center=(0, 0))
        self.assertTrue(any(e.startswith('agv105') and 'turn it' in e for e in errors), errors)
        self.assertFalse(any(e.startswith('agv106') for e in errors))

    def test_two_robots_too_close_on_one_ring_is_refused(self):
        poses = {'agv105': on_ring(0, 0, 1.0, 0, 'cw', 0), 'agv106': on_ring(0, 0, 1.0, 30, 'cw', -7),
                 'agv101': on_ring(0, 0, 1.7, 0, 'cw', 0), 'agv103': on_ring(0, 0, 1.7, 180, 'cw', 180)}
        _, _, errors = plan(BASE, poses, 'cw', center=(0, 0))
        self.assertTrue(any('share the 1.00 m ring' in e for e in errors), errors)

    def test_rings_too_close_together_is_refused_and_missing_robot_reported(self):
        poses = {'agv105': on_ring(0, 0, 1.0, 0, 'cw', 0), 'agv106': on_ring(0, 0, 1.3, 180, 'cw', -7),
                 'agv101': on_ring(0, 0, 1.7, 90, 'cw', 0)}
        _, _, errors = plan(BASE, poses, 'cw', center=(0, 0))
        self.assertTrue(any('rings are 0.30 m apart' in e for e in errors), errors)
        self.assertTrue(any(e.startswith('agv103: no valid MoCap pose') for e in errors), errors)

    def test_midpoint_centre_from_a_line_of_robots(self):
        # Same geometry as the 2026-10-22 lab layout: robots on one diameter.
        cx, cy = -0.655, 0.002
        poses = {'agv101': on_ring(cx, cy, 1.7, -90, 'cw', 0), 'agv105': on_ring(cx, cy, 1.0, -90, 'cw', 0),
                 'agv106': on_ring(cx, cy, 1.0, 90, 'cw', -7), 'agv103': on_ring(cx, cy, 1.7, 90, 'cw', 180)}
        config, lines, errors = plan(BASE, poses, 'cw')
        self.assertEqual(errors, [])
        self.assertAlmostEqual(config['plan']['center'][0], cx, places=2)

    def test_cdr_pose_decode(self):
        frame = b'world\x00'
        body = struct.pack('<II', 1, 2) + struct.pack('<I', len(frame)) + frame
        body += b'\x00' * ((-(len(body))) % 8)
        body += struct.pack('<7d', 1.5, -2.0, 0.3, 0.0, 0.0, math.sin(math.radians(45)), math.cos(math.radians(45)))
        x, y, z, yaw = decode_pose_stamped(b'\x00\x01\x00\x00' + body)
        self.assertEqual((x, y, z), (1.5, -2.0, 0.3))
        self.assertAlmostEqual(math.degrees(yaw), 90.0, places=6)


if __name__ == '__main__':
    unittest.main()
