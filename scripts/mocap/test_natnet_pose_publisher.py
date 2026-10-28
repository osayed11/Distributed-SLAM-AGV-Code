#!/usr/bin/env python3
"""Offline tests for the NatNet publisher's model-definition merge; no ROS or Motive needed."""

import logging
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    from natnet_ros2_pose_publisher import NatNetPosePublisher, RigidBodyTarget
except Exception as exc:  # rclpy/natnet missing on this machine
    NatNetPosePublisher = None
    IMPORT_ERROR = exc


def make_stub(targets, name_by_id):
    stub = SimpleNamespace(targets=[RigidBodyTarget(n, '/gt/' + n) for n in targets],
                           name_by_id=dict(name_by_id), messages=[])
    logger = logging.getLogger('stub')
    stub.get_logger = lambda: SimpleNamespace(
        warning=lambda m: stub.messages.append(('warning', m)),
        info=lambda m: stub.messages.append(('info', m)))
    return stub


@unittest.skipIf(NatNetPosePublisher is None, 'publisher module not importable here')
class MergeDescriptionsTests(unittest.TestCase):
    def merge(self, targets, before, received):
        stub = make_stub(targets, before)
        NatNetPosePublisher.merge_descriptions(stub, received)
        return stub

    def test_recreated_asset_follows_its_name_and_drops_the_stale_id(self):
        stub = self.merge(['orkar_agv104'], {42: 'orkar_agv104', 46: 'orkar_agv106'},
                          {24: 'orkar_agv104', 46: 'orkar_agv106'})
        self.assertEqual(stub.name_by_id, {24: 'orkar_agv104', 46: 'orkar_agv106'})
        self.assertTrue(any('42 -> 24' in m for _, m in stub.messages))

    def test_target_missing_from_a_short_response_keeps_its_mapping(self):
        stub = self.merge(['orkar_agv104'], {42: 'orkar_agv104', 46: 'orkar_agv106'},
                          {46: 'orkar_agv106'})
        self.assertEqual(stub.name_by_id[42], 'orkar_agv104')
        self.assertEqual(stub.messages, [])

    def test_non_target_bodies_simply_follow_the_latest_response(self):
        stub = self.merge(['orkar_agv104'], {42: 'orkar_agv104', 46: 'orkar_agv106'},
                          {42: 'orkar_agv104', 47: 'orkar_agv106'})
        self.assertEqual(stub.name_by_id, {42: 'orkar_agv104', 47: 'orkar_agv106'})

    def test_target_appearing_later_is_logged_and_mapped(self):
        stub = self.merge(['orkar_agv108'], {42: 'orkar_agv104'}, {42: 'orkar_agv104', 43: 'orkar_agv108'})
        self.assertEqual(stub.name_by_id[43], 'orkar_agv108')
        self.assertTrue(any('now in the model definitions' in m for _, m in stub.messages))


if __name__ == '__main__':
    unittest.main()
