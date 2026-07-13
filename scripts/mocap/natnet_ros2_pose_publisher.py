#!/usr/bin/env python3
"""Publish an OptiTrack NatNet rigid body into ROS 2 as PoseStamped."""

import argparse
import socket
import sys
import time
from dataclasses import dataclass
from typing import Dict, List

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
except ImportError:
    print("Missing ROS 2 Python packages. Run this through the Pixi ROS 2 environment.", file=sys.stderr)
    raise

try:
    from natnet import NatNetClient
except ImportError:
    print(
        "Missing Python package 'natnet'. Install it with: python3 -m pip install natnet",
        file=sys.stderr,
    )
    raise


def guess_local_ip(server):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((server, 1510))
        return sock.getsockname()[0]
    finally:
        sock.close()


@dataclass(frozen=True)
class RigidBodyTarget:
    name: str
    topic: str


def parse_target(value: str) -> RigidBodyTarget:
    name, separator, topic = value.partition("=")
    if not separator or not name or not topic.startswith("/"):
        raise argparse.ArgumentTypeError(
            "rigid body must use NAME=/absolute/topic, for example "
            "orkar_agv103=/gt/agv103/pose"
        )
    return RigidBodyTarget(name=name, topic=topic)


class NatNetPosePublisher(Node):
    def __init__(self, args, local_ip):
        super().__init__("natnet_ros2_pose_publisher")
        self.args = args
        self.local_ip = local_ip
        self.targets: List[RigidBodyTarget] = args.targets
        self.target_by_name = {target.name: target for target in self.targets}
        self.name_by_id: Dict[int, str] = {}
        self.printed_defs = False
        self.publishers = {
            target.name: self.create_publisher(PoseStamped, target.topic, 20)
            for target in self.targets
        }
        self.published = {target.name: 0 for target in self.targets}
        self.last_status = 0.0
        self.last_modeldef_request = 0.0
        self.connected_at = 0.0
        self.last_frame_at = 0.0
        self.last_target_pose_at = 0.0

        self.client = NatNetClient(
            server_ip_address=args.server,
            local_ip_address=local_ip,
            use_multicast=args.multicast,
        )
        self.client.on_data_description_received_event.handlers.append(self.on_descriptions)
        self.client.on_data_frame_received_event.handlers.append(self.on_frame)

    def connect(self):
        self.client.connect(timeout=3.0)
        self.connected_at = time.time()
        self.request_modeldef()
        self.get_logger().info(
            "Publishing %d NatNet rigid body target(s) from %s"
            % (len(self.targets), self.args.server)
        )
        for target in self.targets:
            self.get_logger().info("  %s -> %s" % (target.name, target.topic))
        self.get_logger().info("Local interface: %s" % self.local_ip)

    def close(self):
        self.client.shutdown()

    def request_modeldef(self):
        self.client.request_modeldef()
        self.last_modeldef_request = time.time()

    def update(self):
        self.client.update_sync()
        now = time.time()
        if not self.name_by_id and now - self.last_modeldef_request >= 2.0:
            self.request_modeldef()
        if now - self.connected_at >= self.args.frame_timeout:
            if not self.last_frame_at or now - self.last_frame_at > self.args.frame_timeout:
                raise RuntimeError(
                    "NatNet frames stopped for more than %.1fs" % self.args.frame_timeout
                )
            if not self.last_target_pose_at or now - self.last_target_pose_at > self.args.frame_timeout:
                raise RuntimeError(
                    "no tracked target pose for more than %.1fs" % self.args.frame_timeout
                )

    def on_descriptions(self, desc):
        self.name_by_id = {
            int(rb.id_num): str(rb.name)
            for rb in desc.rigid_bodies
            if rb.name is not None
        }
        if self.printed_defs:
            return

        self.get_logger().info("NatNet rigid bodies:")
        for rb in desc.rigid_bodies:
            marker_count = len(rb.markers) if rb.markers is not None else 0
            self.get_logger().info(
                "  name=%s id=%s markers=%d" % (rb.name, rb.id_num, marker_count)
            )
        available_names = set(self.name_by_id.values())
        for target in self.targets:
            if target.name not in available_names:
                self.get_logger().warning(
                    "Requested rigid body '%s' is not in model definitions." % target.name
                )
        self.printed_defs = True

    def on_frame(self, frame):
        now = time.time()
        self.last_frame_at = now
        published_names = []
        stamp = self.get_clock().now().to_msg()
        for rb in frame.rigid_bodies:
            name = self.name_by_id.get(int(rb.id_num))
            if name not in self.target_by_name:
                continue
            if not rb.tracking_valid and not self.args.publish_untracked:
                continue

            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.args.frame_id
            msg.pose.position.x = float(rb.pos[0])
            msg.pose.position.y = float(rb.pos[1])
            msg.pose.position.z = float(rb.pos[2])
            msg.pose.orientation.x = float(rb.rot[0])
            msg.pose.orientation.y = float(rb.rot[1])
            msg.pose.orientation.z = float(rb.rot[2])
            msg.pose.orientation.w = float(rb.rot[3])
            self.publishers[name].publish(msg)
            self.published[name] += 1
            published_names.append(name)

        if published_names:
            self.last_target_pose_at = now
        now = time.time()
        if now - self.last_status >= self.args.status_period:
            counts = ", ".join(
                "%s=%d" % (target.name, self.published[target.name])
                for target in self.targets
            )
            self.get_logger().info("Published poses: %s" % counts)
            self.last_status = now


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Publish a NatNet rigid body as ROS 2 PoseStamped.")
    parser.add_argument("--server", default="192.168.50.200",
                        help="Motive/NatNet server IP")
    parser.add_argument("--local", default=None,
                        help="Local interface IP. Defaults to auto-detect.")
    parser.add_argument("--name", default="orkar_agv1",
                        help="Rigid body name to publish")
    parser.add_argument("--topic", default="/optitrack/rigid_bodies/orkar_agv1",
                        help="ROS 2 PoseStamped output topic")
    parser.add_argument(
        "--rigid-body",
        action="append",
        type=parse_target,
        default=[],
        metavar="NAME=/TOPIC",
        help="Rigid body/topic mapping. Repeat for a fleet. Overrides --name/--topic.",
    )
    parser.add_argument("--frame-id", default="world",
                        help="PoseStamped header frame_id")
    parser.add_argument("--multicast", action="store_true",
                        help="Use multicast data reception instead of unicast")
    parser.add_argument("--publish-untracked", action="store_true",
                        help="Publish poses even when NatNet marks tracking invalid")
    parser.add_argument("--status-period", type=float, default=2.0,
                        help="Console status print period in seconds")
    parser.add_argument("--frame-timeout", type=float, default=5.0,
                        help="Exit when NatNet or tracked target frames stop this long")
    args = parser.parse_args(argv)
    args.targets = args.rigid_body or [RigidBodyTarget(args.name, args.topic)]
    names = [target.name for target in args.targets]
    topics = [target.topic for target in args.targets]
    if len(names) != len(set(names)):
        parser.error("duplicate rigid-body name")
    if len(topics) != len(set(topics)):
        parser.error("duplicate output topic")
    if args.frame_timeout <= 0:
        parser.error("--frame-timeout must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    local_ip = args.local or guess_local_ip(args.server)

    rclpy.init(args=None)
    node = NatNetPosePublisher(args, local_ip)
    try:
        node.connect()
        while rclpy.ok():
            node.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
