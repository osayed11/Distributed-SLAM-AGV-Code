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


KP_POS    = 0.5
V_MAX     = 0.20      # halved for precision; precision over speed
TOL_POS   = 0.10      # tightened for precise endpoint
KP_TH     = 2.0
OMEGA_MAX = 0.8
TOL_TH    = 0.05      # ~3°; tighter so GOAL_REACHED waits for true alignment
MOCAP_FRESH_SEC = 1.5  # halt control if no mocap pose within this window

# Reject obviously-bad mocap poses. If a new sample disagrees with the last
# accepted one by more than these per-second rates, it's almost certainly a
# rigid-solve glitch (markers occluded, re-solved with wrong axis). The robot's
# V_MAX is 0.40 m/s, so 2.0 m/s gives ~5x slack. Yaw cap is ~170 deg/s.
MOCAP_MAX_POS_RATE = 2.0   # m/s
MOCAP_MAX_TH_RATE  = 3.0   # rad/s

# Heading fusion: use mocap absolute heading at goal lock-in, then integrate
# odom delta-theta on top of it. /odom yaw is smoother short-term than a
# poorly-tracked rigid; over a 2.5 m run odom drift is small.
USE_ODOM_HEADING = True


def _wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


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
        self._mocap_rejected = 0      # count of pose samples filtered as glitches
        # Heading-fusion snapshot: at each new goal, record (mocap_theta,
        # odom_theta). During the run, fused_theta = mocap_theta_lock +
        # (odom_theta - odom_theta_lock). Reset on each new goal.
        self._heading_lock = None     # (mocap_theta, odom_theta) or None

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
                # Snapshot heading-fusion reference: (mocap_theta at lock-in,
                # odom_theta at lock-in). Requires a recent mocap pose.
                if (USE_ODOM_HEADING and self._mocap_pose is not None
                        and time.time() - self._mocap_ts < MOCAP_FRESH_SEC):
                    odom = self._ros.get_odom()
                    self._heading_lock = (float(self._mocap_pose[2]),
                                          float(odom["theta"]))
                    print(f"[drive {self._id}] heading lock: "
                          f"mocap_θ={np.degrees(self._heading_lock[0]):+.1f}° "
                          f"odom_θ={np.degrees(self._heading_lock[1]):+.1f}°")
                else:
                    self._heading_lock = None
                print(f"[drive {self._id}] goal updated to "
                      f"({d['x']:.2f}, {d['y']:.2f}, {d['theta']:.2f} rad) "
                      f"tol={self._goal_tol:.2f} m")
            if t == "pose":
                # Only mocap-source pose for our own id is used for control.
                if d.get("id") != self._id:
                    continue
                if d.get("source", "mocap") != "mocap":
                    continue
                new_pose = (float(d["x"]), float(d["y"]), float(d["theta"]))
                # Reject impossible jumps (mocap rigid-solve glitches).
                if self._mocap_pose is not None:
                    now = time.time()
                    dt = max(now - self._mocap_ts, 1e-3)
                    if dt < 1.0:
                        dx = new_pose[0] - self._mocap_pose[0]
                        dy = new_pose[1] - self._mocap_pose[1]
                        dth = abs(_wrap_pi(new_pose[2] - self._mocap_pose[2]))
                        pos_rate = float(np.hypot(dx, dy)) / dt
                        th_rate  = dth / dt
                        if (pos_rate > MOCAP_MAX_POS_RATE
                                or th_rate > MOCAP_MAX_TH_RATE):
                            self._mocap_rejected += 1
                            if self._mocap_rejected % 10 == 1:
                                print(f"[drive {self._id}] rejected mocap "
                                      f"jump #{self._mocap_rejected} "
                                      f"(pos_rate={pos_rate:.1f} m/s, "
                                      f"th_rate={np.degrees(th_rate):.0f}°/s)")
                            continue
                self._mocap_pose = new_pose
                self._mocap_ts = time.time()

    def _control_pose(self):
        """Pose used by the controller: mocap (x, y), and either mocap θ or
        odom-fused θ depending on USE_ODOM_HEADING + whether a heading lock
        was captured at goal arrival."""
        x, y, mocap_th = self._mocap_pose
        if not USE_ODOM_HEADING or self._heading_lock is None:
            return (x, y, mocap_th)
        odom = self._ros.get_odom()
        mocap_th_lock, odom_th_lock = self._heading_lock
        fused_th = _wrap_pi(mocap_th_lock
                            + _wrap_pi(odom["theta"] - odom_th_lock))
        return (x, y, fused_th)

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
                    ctrl_pose = self._control_pose()
                    vx_b, vy_b, omega, dist, th_err = self._compute_cmd(ctrl_pose)
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
