#!/usr/bin/env python3
"""Publish an OptiTrack NatNet rigid body into ROS 2 as PoseStamped."""

import argparse
import socket
import sys
import time

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


class NatNetPosePublisher(Node):
    def __init__(self, args, local_ip):
        super().__init__("natnet_ros2_pose_publisher")
        self.args = args
        self.local_ip = local_ip
        self.names = []
        self.printed_defs = False
        self.published = 0
        self.last_status = 0.0
        self.last_modeldef_request = 0.0
        self.pub = self.create_publisher(PoseStamped, args.topic, 20)

        self.client = NatNetClient(
            server_ip_address=args.server,
            local_ip_address=local_ip,
            use_multicast=args.multicast,
        )
        self.client.on_data_description_received_event.handlers.append(self.on_descriptions)
        self.client.on_data_frame_received_event.handlers.append(self.on_frame)

    def connect(self):
        self.client.connect(timeout=3.0)
        self.request_modeldef()
        self.get_logger().info(
            "Publishing NatNet rigid body '%s' from %s on %s"
            % (self.args.name, self.args.server, self.args.topic)
        )
        self.get_logger().info("Local interface: %s" % self.local_ip)

    def close(self):
        self.client.shutdown()

    def request_modeldef(self):
        self.client.request_modeldef()
        self.last_modeldef_request = time.time()

    def update(self):
        self.client.update_sync()
        if not self.names and time.time() - self.last_modeldef_request >= 2.0:
            self.request_modeldef()

    def on_descriptions(self, desc):
        self.names = [rb.name for rb in desc.rigid_bodies]
        if self.printed_defs:
            return

        self.get_logger().info("NatNet rigid bodies:")
        for rb in desc.rigid_bodies:
            marker_count = len(rb.markers) if rb.markers is not None else 0
            self.get_logger().info(
                "  name=%s id=%s markers=%d" % (rb.name, rb.id_num, marker_count)
            )
        if self.args.name not in self.names:
            self.get_logger().warning(
                "Requested rigid body '%s' is not in model definitions." % self.args.name
            )
        self.printed_defs = True

    def on_frame(self, frame):
        if self.args.name not in self.names:
            return
        index = self.names.index(self.args.name)
        if index >= len(frame.rigid_bodies):
            return

        rb = frame.rigid_bodies[index]
        if not rb.tracking_valid and not self.args.publish_untracked:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.pose.position.x = float(rb.pos[0])
        msg.pose.position.y = float(rb.pos[1])
        msg.pose.position.z = float(rb.pos[2])
        msg.pose.orientation.x = float(rb.rot[0])
        msg.pose.orientation.y = float(rb.rot[1])
        msg.pose.orientation.z = float(rb.rot[2])
        msg.pose.orientation.w = float(rb.rot[3])
        self.pub.publish(msg)

        self.published += 1
        now = time.time()
        if now - self.last_status >= self.args.status_period:
            self.get_logger().info(
                "Published %d poses: %s pos=(%.3f, %.3f, %.3f) valid=%s"
                % (
                    self.published,
                    self.args.name,
                    rb.pos[0],
                    rb.pos[1],
                    rb.pos[2],
                    rb.tracking_valid,
                )
            )
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
    parser.add_argument("--frame-id", default="world",
                        help="PoseStamped header frame_id")
    parser.add_argument("--multicast", action="store_true",
                        help="Use multicast data reception instead of unicast")
    parser.add_argument("--publish-untracked", action="store_true",
                        help="Publish poses even when NatNet marks tracking invalid")
    parser.add_argument("--status-period", type=float, default=2.0,
                        help="Console status print period in seconds")
    return parser.parse_args(argv)


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
