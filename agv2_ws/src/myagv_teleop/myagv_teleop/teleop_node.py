#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keyboard teleoperation for myAGV — ROS2 port of myagv_teleop.py."""

import sys
import select
import termios
import tty
import threading

import rclpy
from geometry_msgs.msg import Twist

msg = """
Control myagv!
---------------------------
Moving around:
   u    i    o
   j    k    l
        ,

space key, k : stop
i : forward
, : backward
j : turn left
l : turn right
u : left revolve
o : right revolve

CTRL-C to quit
"""


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('myagv_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Spin callbacks in a background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    x = 0
    y = 0
    theta = 0

    try:
        print(msg)
        while True:
            key = getKey(settings)

            if key == ' ' or key == 'k':
                x = 0
                y = 0
                theta = 0
            elif key == 'i':
                x = 1
                y = 0
                theta = 0
            elif key == ',':
                x = -1
                y = 0
                theta = 0
            elif key == 'j':
                x = 0
                y = 1
                theta = 0
            elif key == 'l':
                x = 0
                y = -1
                theta = 0
            elif key == 'u':
                x = 0
                y = 0
                theta = 1
            elif key == 'o':
                x = 0
                y = 0
                theta = -1
            elif key == '\x03':
                break

            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.y = float(y)
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(theta)
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Send zero velocity before exiting
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
