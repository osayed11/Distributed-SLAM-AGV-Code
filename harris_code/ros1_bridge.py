"""
ROS1 (rospy) bridge — runs on each myAGV alongside myagv_ros.

Provides:
  - get_odom()  → dict {x, y, vx, vy, theta, omega}
  - send_cmd(vx, vy)  → publishes to /cmd_vel

Call spin_once() in the agent's control loop to process incoming callbacks.
"""
import threading
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class ROS1Bridge:
    def __init__(self, node_name: str = "swarm_agent_ros1"):
        rospy.init_node(node_name, anonymous=False)
        self._odom = {"x": 0.0, "y": 0.0, "vx": 0.0, "vy": 0.0,
                      "theta": 0.0, "omega": 0.0}
        self._lock = threading.Lock()

        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=1)
        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)
        with self._lock:
            self._odom = {
                "x":     msg.pose.pose.position.x,
                "y":     msg.pose.pose.position.y,
                "vx":    msg.twist.twist.linear.x,
                "vy":    msg.twist.twist.linear.y,
                "theta": theta,
                "omega": msg.twist.twist.angular.z,
            }

    def get_odom(self) -> dict:
        with self._lock:
            return dict(self._odom)

    def send_cmd(self, vx: float, vy: float, omega: float = 0.0) -> None:
        """Send mecanum velocity command in robot frame."""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self._cmd_pub.publish(twist)

    def spin_once(self) -> None:
        rospy.rostime.wallsleep(0)  # process pending callbacks without blocking
