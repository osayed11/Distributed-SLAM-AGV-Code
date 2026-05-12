"""
Robot-side drive-to-point runner.

Runs on the myAGV alongside myagv_ros. Reads /odom from the local ROS1
stack via ros1_bridge.py, listens for goal/estop/ctrl_stop messages from
the laptop GUI over ZMQ, runs a simple P controller (position + heading
concurrently), and publishes /cmd_vel.

Also publishes its own pose back to the laptop over ZMQ at the control
rate so the GUI can render the robot when no mocap stream is available.

Usage on the robot:
    python3 drive_runner.py --config network.yaml --id 0
"""
import argparse
import signal
import time

import yaml
import zmq
import numpy as np

from messages import pose_msg, goal_msg, unpack
from ros1_bridge import ROS1Bridge


KP_POS    = 0.7
V_MAX     = 0.25
TOL_POS   = 0.2
KP_TH     = 1.5
OMEGA_MAX = 0.6
TOL_TH    = 0.1


class DriveRunner:
    def __init__(self, robot_id: int, cfg: dict, control_hz: float = 20.0):
        self._id = robot_id
        self._dt = 1.0 / control_hz

        ctx = zmq.Context.instance()

        my_port = next(r["pub_port"] for r in cfg["robots"] if r["id"] == robot_id)
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{my_port}")

        self._sub = ctx.socket(zmq.SUB)
        self._sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['goal_pub_port']}")
        for topic in (b"goal", b"estop", b"ctrl_stop"):
            self._sub.setsockopt(zmq.SUBSCRIBE, topic)

        self._ros = ROS1Bridge(node_name=f"drive_runner_{robot_id}")

        self._goal = None         # np.array([x, y, theta]) or None
        self._goal_tol = TOL_POS
        self._paused = False
        self._running = True
        self._last_heartbeat = 0.0

        signal.signal(signal.SIGINT,  self._handle_sigint)
        signal.signal(signal.SIGTERM, self._handle_sigint)

        print(f"[drive {robot_id}] ready — pub_port={my_port}, "
              f"laptop={cfg['laptop']['ip']}:{cfg['laptop']['goal_pub_port']}")
        time.sleep(0.2)

    def _handle_sigint(self, sig, frame):
        print(f"\n[drive {self._id}] shutting down")
        self._running = False

    def _drain(self, poller):
        while self._running and dict(poller.poll(timeout=0)):
            _, raw = self._sub.recv_multipart()
            d = unpack(raw)
            t = d.get("t")
            if t == "estop":
                print(f"[drive {self._id}] ESTOP — stopping and exiting")
                self._ros.send_cmd(0.0, 0.0, 0.0)
                self._running = False
                return
            if t == "ctrl_stop":
                print(f"[drive {self._id}] soft stop — clearing goal")
                self._goal = None
                self._paused = True
                self._ros.send_cmd(0.0, 0.0, 0.0)
                continue
            if t == "goal":
                # robot_id == -1 → broadcast; otherwise target specific robot
                target = d.get("id", -1)
                if target not in (-1, self._id):
                    continue
                self._goal = np.array([d["x"], d["y"], d["theta"]])
                self._goal_tol = float(d.get("tol", TOL_POS))
                self._paused = False
                print(f"[drive {self._id}] goal updated to "
                      f"({d['x']:.2f}, {d['y']:.2f}, {d['theta']:.2f} rad) "
                      f"tol={self._goal_tol:.2f} m")

    def _compute_cmd(self, odom: dict):
        """Return (vx_body, vy_body, omega, dist, th_err)."""
        x, y, theta = odom["x"], odom["y"], odom["theta"]
        gx, gy, gth = self._goal

        dx, dy = gx - x, gy - y
        dist = float(np.hypot(dx, dy))

        v_world = np.array([KP_POS * dx, KP_POS * dy])
        mag = float(np.linalg.norm(v_world))
        if mag > V_MAX:
            v_world *= V_MAX / mag

        c, s = np.cos(theta), np.sin(theta)
        vx_b =  c * v_world[0] + s * v_world[1]
        vy_b = -s * v_world[0] + c * v_world[1]

        th_err = (gth - theta + np.pi) % (2 * np.pi) - np.pi
        omega  = float(np.clip(KP_TH * th_err, -OMEGA_MAX, OMEGA_MAX))

        if dist < self._goal_tol:
            vx_b = vy_b = 0.0
        if abs(th_err) < TOL_TH:
            omega = 0.0

        return vx_b, vy_b, omega, dist, th_err

    def run(self):
        poller = zmq.Poller()
        poller.register(self._sub, zmq.POLLIN)
        next_tick = time.monotonic()

        while self._running:
            self._drain(poller)
            if not self._running:
                break

            self._ros.spin_once()

            now = time.monotonic()
            if now >= next_tick:
                odom = self._ros.get_odom()
                self._pub.send_multipart([
                    b"pose",
                    pose_msg(self._id, odom["x"], odom["y"], odom["theta"],
                             source="odom"),
                ])

                if self._goal is None or self._paused:
                    self._ros.send_cmd(0.0, 0.0, 0.0)
                else:
                    vx_b, vy_b, omega, dist, th_err = self._compute_cmd(odom)
                    self._ros.send_cmd(vx_b, vy_b, omega)

                    if dist < self._goal_tol and abs(th_err) < TOL_TH:
                        print(f"[drive {self._id}] GOAL REACHED — "
                              f"dist={dist*100:.1f} cm, "
                              f"θ_err={np.degrees(th_err):.1f}°")
                        self._goal = None
                        self._paused = True

                    if now - self._last_heartbeat >= 5.0:
                        self._last_heartbeat = now
                        print(f"[drive {self._id}] heartbeat — "
                              f"pos=({odom['x']:.2f}, {odom['y']:.2f}) "
                              f"θ={np.degrees(odom['theta']):.1f}°  "
                              f"dist={dist*100:.1f} cm  "
                              f"cmd=({vx_b:.3f}, {vy_b:.3f}, ω={omega:.2f})")

                next_tick += self._dt

            time.sleep(max(0.0, next_tick - time.monotonic()))

        self._ros.send_cmd(0.0, 0.0, 0.0)
        print(f"[drive {self._id}] stopped")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/home/ubuntu/network.yaml")
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--control-hz", type=float, default=20.0)
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    DriveRunner(args.id, cfg, control_hz=args.control_hz).run()


if __name__ == "__main__":
    main()
