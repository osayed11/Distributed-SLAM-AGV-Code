"""
Robot-side drive-to-point runner.

Runs on the myAGV alongside myagv_ros. Subscribes to mocap pose for its
own logical id from the laptop's mocap_pub_port and uses that as the
control pose (so the world frame matches the laptop GUI's clicks).
Listens for goal/estop/ctrl_stop from goal_pub_port, runs a simple P
controller (position + heading concurrently), publishes /cmd_vel.

If mocap goes stale (server crashes or rigid drops), the controller
halts and prints a warning rather than driving on /odom alone — odom
boots at (0, 0) and isn't world-aligned, so blind odom control would
walk off in random directions.

/odom is still published over ZMQ so the GUI can show a fallback when
mocap is unavailable (display only — not used for control).

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
MOCAP_FRESH_SEC = 0.5  # halt control if no mocap pose within this window


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
        self._sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
        for topic in (b"goal", b"estop", b"ctrl_stop", b"pose"):
            self._sub.setsockopt(zmq.SUBSCRIBE, topic)

        self._ros = ROS1Bridge(node_name=f"drive_runner_{robot_id}")

        self._goal = None         # np.array([x, y, theta]) or None
        self._goal_tol = TOL_POS
        self._paused = False
        self._running = True
        self._last_heartbeat = 0.0
        self._mocap_pose = None       # (x, y, theta) — world frame from mocap
        self._mocap_ts   = 0.0
        self._mocap_warned = False    # rate-limit the "no mocap" warnings

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
            if t == "pose":
                # Only mocap-source pose for our own id is used for control.
                if d.get("id") != self._id:
                    continue
                if d.get("source", "mocap") != "mocap":
                    continue
                self._mocap_pose = (float(d["x"]), float(d["y"]), float(d["theta"]))
                self._mocap_ts = time.time()

    def _compute_cmd(self, pose):
        """pose = (x, y, theta) in world frame. Returns
        (vx_body, vy_body, omega, dist, th_err)."""
        x, y, theta = pose
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
                # Publish /odom-derived pose for the GUI fallback (display only).
                self._pub.send_multipart([
                    b"pose",
                    pose_msg(self._id, odom["x"], odom["y"], odom["theta"],
                             source="odom"),
                ])

                mocap_age = time.time() - self._mocap_ts
                mocap_fresh = (self._mocap_pose is not None
                               and mocap_age < MOCAP_FRESH_SEC)

                if self._goal is None or self._paused:
                    self._ros.send_cmd(0.0, 0.0, 0.0)
                elif not mocap_fresh:
                    # World-frame control needs world-frame pose. /odom isn't
                    # world-aligned, so halt rather than drive blind.
                    self._ros.send_cmd(0.0, 0.0, 0.0)
                    if not self._mocap_warned:
                        print(f"[drive {self._id}] no fresh mocap pose "
                              f"(age={mocap_age:.1f}s) — halting until mocap returns")
                        self._mocap_warned = True
                else:
                    if self._mocap_warned:
                        print(f"[drive {self._id}] mocap recovered — resuming")
                        self._mocap_warned = False
                    vx_b, vy_b, omega, dist, th_err = self._compute_cmd(self._mocap_pose)
                    self._ros.send_cmd(vx_b, vy_b, omega)

                    if dist < self._goal_tol and abs(th_err) < TOL_TH:
                        print(f"[drive {self._id}] GOAL REACHED — "
                              f"dist={dist*100:.1f} cm, "
                              f"θ_err={np.degrees(th_err):.1f}°")
                        self._goal = None
                        self._paused = True

                    if now - self._last_heartbeat >= 5.0:
                        self._last_heartbeat = now
                        mx, my, mth = self._mocap_pose
                        print(f"[drive {self._id}] heartbeat — "
                              f"mocap=({mx:.2f}, {my:.2f}) "
                              f"θ={np.degrees(mth):.1f}°  "
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
