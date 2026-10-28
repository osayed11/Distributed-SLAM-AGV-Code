#!/usr/bin/env python3
"""ROS-independent regression checks for circle command curvature."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


path = Path(__file__).with_name("drive_mocap_circle_ros2.py")
spec = importlib.util.spec_from_file_location("circle_controller", path)
circle = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = circle
spec.loader.exec_module(circle)


class CircleVelocityTests(unittest.TestCase):
    def args(self, **overrides):
        values = dict(linear=0.10, min_linear=0.07, slow_linear_scale=0.7,
                      slow_radius_error=0.12, radius=0.9, turn_sign=-1,
                      max_angular=0.55, heading_kp=1.1)
        values.update(overrides)
        return SimpleNamespace(**values)

    def test_nominal_and_slow_curvature_in_both_directions(self):
        for direction in (-1, 1):
            for error, speed in ((0.0, 0.10), (-0.15, 0.07), (0.15, 0.07)):
                with self.subTest(direction=direction, error=error):
                    args = self.args(turn_sign=direction)
                    linear, angular = circle.circle_velocity(args, error, 0.0)
                    self.assertAlmostEqual(linear, speed)
                    self.assertAlmostEqual(angular, direction * speed / args.radius)
                    self.assertAlmostEqual(abs(linear / angular), args.radius)

    def test_heading_correction_remains_active_when_slow(self):
        args = self.args()
        linear, angular = circle.circle_velocity(args, -0.15, 0.1)
        self.assertAlmostEqual(angular, -linear / args.radius + args.heading_kp * 0.1)

    def test_angular_command_stays_bounded(self):
        for heading in (-10.0, 10.0):
            args = self.args()
            _, angular = circle.circle_velocity(args, 0.0, heading)
            self.assertEqual(abs(angular), args.max_angular)

    def test_minimum_speed_used_in_feedforward(self):
        args = self.args(min_linear=0.08)
        linear, angular = circle.circle_velocity(args, 0.15, 0.0)
        self.assertAlmostEqual(linear, 0.08)
        self.assertAlmostEqual(angular, -0.08 / args.radius)

    def test_no_feedforward_when_stopped(self):
        linear, angular = circle.circle_velocity(self.args(linear=0, min_linear=0), 0.15, 0)
        self.assertEqual((linear, angular), (0, 0))


if __name__ == "__main__":
    unittest.main()
