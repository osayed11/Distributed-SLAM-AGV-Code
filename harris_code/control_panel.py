"""
Single-robot drive-to-point control panel.

Left-click on the map to place a goal. Use sliders to fine-tune X, Y, θ,
and tolerance. Hit "Send Goal" to publish over ZMQ — drive_runner on the
robot picks it up and drives there.

Robot pose comes from PhaseSpace mocap (mocap_pub.py on the laptop) if a
recent message is available; otherwise it falls back to /odom that
drive_runner publishes from each robot.

    python3 control_panel.py --config config/network.yaml
"""
import argparse
import threading
import time

import yaml
import zmq
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button, TextBox

from messages import goal_msg, estop_msg, ctrl_stop_msg, unpack


ROBOT_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:red",
                "tab:purple", "tab:brown", "tab:pink", "tab:gray"]
ARROW_LEN = 0.15
ROBOT_RADIUS = 0.2
TRAIL_LEN = 500
MOCAP_FRESH_SEC = 0.5  # use mocap pose if its timestamp is within this window


class _PoseState:
    """Pose state keyed by logical robot id (rid). Robots in network.yaml
    can have arbitrary non-contiguous ids (e.g. just [2] or [0, 2, 5]) —
    storing by rid avoids the bug where a fixed array sized by n drops
    poses whose rid >= n."""
    def __init__(self, rids):
        self.lock = threading.Lock()
        self.rids       = list(rids)
        self.mocap_pose = {rid: None for rid in self.rids}
        self.mocap_ts   = {rid: 0.0  for rid in self.rids}
        self.odom_pose  = {rid: None for rid in self.rids}
        self.odom_ts    = {rid: 0.0  for rid in self.rids}

    def best_pose(self, rid):
        """Return (pose, source) where source is 'mocap', 'odom', or None."""
        now = time.time()
        with self.lock:
            mp = self.mocap_pose.get(rid)
            if mp is not None and now - self.mocap_ts.get(rid, 0.0) < MOCAP_FRESH_SEC:
                return np.array(mp), "mocap"
            op = self.odom_pose.get(rid)
            if op is not None:
                return np.array(op), "odom"
        return None, None


def _pose_listener(state: _PoseState, cfg: dict, rids):
    valid = set(rids)
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    for r in cfg["robots"]:
        if r["id"] in valid:
            sub.connect(f"tcp://{r['ip']}:{r['pub_port']}")
    sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
    sub.setsockopt_string(zmq.SUBSCRIBE, "pose")
    while True:
        try:
            _, raw = sub.recv_multipart()
            d = unpack(raw)
        except Exception:
            continue
        if d.get("t") != "pose":
            continue
        rid = d.get("id")
        if rid not in valid:
            continue
        source = d.get("source", "mocap")
        now = time.time()
        with state.lock:
            if source == "mocap":
                state.mocap_pose[rid] = [d["x"], d["y"], d["theta"]]
                state.mocap_ts[rid]   = now
            else:
                state.odom_pose[rid] = [d["x"], d["y"], d["theta"]]
                state.odom_ts[rid]   = now


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/network.yaml")
    parser.add_argument("--n-robots", type=int, default=None,
                        help="Override robot count (default: from network.yaml)")
    parser.add_argument("--goal-tol", type=float, default=0.2)
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    # rids: the logical ids actually present in the yaml (after optional --n-robots cap).
    all_rids = [r["id"] for r in cfg["robots"]]
    n = args.n_robots if args.n_robots is not None else len(all_rids)
    rids = all_rids[:n]
    # Slice cfg["robots"] to the active ones so downstream loops see only those.
    cfg["robots"] = cfg["robots"][:n]
    state = _PoseState(rids)
    threading.Thread(target=_pose_listener, args=(state, cfg, rids), daemon=True).start()

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{cfg['laptop']['goal_pub_port']}")
    time.sleep(0.1)

    # -------------------------------------------------------------------------
    # Layout
    # -------------------------------------------------------------------------
    fig = plt.figure(figsize=(10, 9))
    fig.suptitle(f"Drive-to-Point — {n} robot{'s' if n > 1 else ''}",
                 fontsize=13, fontweight="bold")

    ax = fig.add_axes([0.05, 0.30, 0.90, 0.62])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlim(-2, 7)
    ax.set_ylim(-3, 3)

    ax_sx  = fig.add_axes([0.15, 0.22, 0.70, 0.025])
    ax_sy  = fig.add_axes([0.15, 0.17, 0.70, 0.025])
    ax_sth = fig.add_axes([0.15, 0.12, 0.70, 0.025])
    ax_st  = fig.add_axes([0.15, 0.07, 0.70, 0.025])

    sl_x   = Slider(ax_sx,  "Goal X (m)",    -5.0, 10.0, valinit=0.0, color="gold")
    sl_y   = Slider(ax_sy,  "Goal Y (m)",    -5.0,  5.0, valinit=0.0, color="gold")
    sl_th  = Slider(ax_sth, "Goal θ (rad)", -3.14,  3.14, valinit=0.0, color="gold")
    sl_tol = Slider(ax_st,  "Tolerance (m)",  0.01,  1.0, valinit=args.goal_tol, color="lightblue")

    ax_btn_stop     = fig.add_axes([0.05, 0.015, 0.26, 0.040])
    ax_btn_softstop = fig.add_axes([0.37, 0.015, 0.26, 0.040])
    ax_btn_send     = fig.add_axes([0.69, 0.015, 0.26, 0.040])
    btn_stop     = Button(ax_btn_stop,     "E-Stop",    color="salmon",      hovercolor="#cc2200")
    btn_softstop = Button(ax_btn_softstop, "Soft Stop", color="lightsalmon", hovercolor="#ff8844")
    btn_send     = Button(ax_btn_send,     "Send Goal", color="lightgreen",  hovercolor="#00cc44")

    # -------------------------------------------------------------------------
    # Map artists
    # -------------------------------------------------------------------------
    robot_circles, robot_arrows, robot_labels = [], [], []
    for i, r in enumerate(cfg["robots"]):
        rid = r["id"]
        col = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        circ = plt.Circle((0, 0), ROBOT_RADIUS, color=col, alpha=0.5, zorder=3)
        ax.add_patch(circ)
        arr = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                          arrowprops=dict(arrowstyle="->", color=col, lw=2), zorder=4)
        lbl = ax.text(0, 0, f"r{rid}", fontsize=8, ha="center", va="center",
                      color="white", fontweight="bold", zorder=5)
        robot_circles.append(circ)
        robot_arrows.append(arr)
        robot_labels.append(lbl)

    trails = [([], []) for _ in range(n)]
    trail_lines = []
    for i in range(n):
        col = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        ln, = ax.plot([], [], color=col, alpha=0.3, linewidth=1, zorder=2)
        trail_lines.append(ln)

    goal_star, = ax.plot([0.0], [0.0], marker="*", markersize=20, color="gold",
                         zorder=6, label="goal (not sent)", linestyle="None")
    goal_heading = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                               arrowprops=dict(arrowstyle="->", color="goldenrod", lw=2), zorder=6)
    tol_circle = plt.Circle((0.0, 0.0), args.goal_tol, color="gold",
                             fill=False, linestyle="--", linewidth=1.4, zorder=5)
    ax.add_patch(tol_circle)

    # Scenario overlays — one per robot, color-matched. Each consists of the
    # planned path, the waypoint dots, and a ring around the active waypoint.
    scenario_overlays = []
    for i in range(n):
        col = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        ln,   = ax.plot([], [], color=col, linestyle=":", linewidth=1.2,
                        alpha=0.7, zorder=4)
        dots, = ax.plot([], [], marker="o", markersize=6, color=col,
                        linestyle="None", alpha=0.5, zorder=4)
        cur,  = ax.plot([], [], marker="o", markersize=12, markerfacecolor="none",
                        markeredgecolor=col, markeredgewidth=2,
                        linestyle="None", zorder=5)
        scenario_overlays.append((ln, dots, cur))

    status_text = ax.text(0.02, 0.97, "click map or use sliders — then Send Goal",
                          transform=ax.transAxes, fontsize=10, va="top", ha="left",
                          bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.85))
    sent_text = ax.text(0.98, 0.97, "last sent: —",
                        transform=ax.transAxes, fontsize=9, va="top", ha="right",
                        color="gray",
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.75))
    source_text = ax.text(0.50, 0.97, "pose source: —",
                          transform=ax.transAxes, fontsize=9, va="top", ha="center",
                          color="black",
                          bbox=dict(boxstyle="round,pad=0.2", fc="lightyellow", alpha=0.85))

    ax.legend(loc="lower right", fontsize=8)

    # -------------------------------------------------------------------------
    # Goal + scenario state
    # -------------------------------------------------------------------------
    _goal = {"x": 0.0, "y": 0.0, "theta": 0.0, "tol": args.goal_tol, "sent": False}
    _block_slider = [False]
    # Multi-robot scenario state. `robots` maps robot_id → that robot's path
    # state: {waypoints: [(x,y,th), ...], idx, lap, done}.
    _scenario = {
        "active":     False,
        "name":       "",
        "robots":     {},
        "total_laps": 1,
    }
    # Per-robot SE(2) calibration from raw mocap frame → that robot's room
    # frame. Convention: at the moment of "Set Origin" each robot's nose
    # points along its own room +Y, and its position becomes (0, 0).
    # alpha_r = π/2 − mocap_theta_at_calibration_r. Each robot has an
    # independent room frame; the two are unrelated in mocap space.
    _calib = {
        r["id"]: {
            "set":          False,
            "origin_x":     0.0,
            "origin_y":     0.0,
            "origin_theta": 0.0,
            "alpha":        0.0,
        }
        for r in cfg["robots"]
    }
    # Laps state machine. One lap = drive to start → wait → drive to end.
    # Shared run params; each robot has its own phase/lap counter so robots
    # can desync (e.g. one finishes a lap faster than the other).
    _laps = {
        "active":     False,
        "total_laps": 4,
        "wait_sec":   1.0,
        "start_room": (0.0, 0.0),
        "end_room":   (0.0, 1.5),
        "robots": {
            r["id"]: {
                "current_lap":        0,
                "phase":              "idle",  # warmup_to_end / to_start /
                                               # wait_at_start / to_end / done
                "phase_t0":           0.0,
                "heading_lock_mocap": 0.0,
            }
            for r in cfg["robots"]
        },
    }

    def _mocap_to_room(mx, my, mth, rid=0):
        cal = _calib.get(rid)
        if cal is None or not cal["set"]:
            return mx, my, mth
        a = cal["alpha"]
        dx = mx - cal["origin_x"]
        dy = my - cal["origin_y"]
        c, s = np.cos(a), np.sin(a)
        rx = dx * c - dy * s
        ry = dx * s + dy * c
        rth = (mth + a + np.pi) % (2 * np.pi) - np.pi
        return rx, ry, rth

    def _room_to_mocap(rx, ry, rth, rid=0):
        cal = _calib.get(rid)
        if cal is None or not cal["set"]:
            return rx, ry, rth
        a = cal["alpha"]
        c, s = np.cos(a), np.sin(a)
        dx =  rx * c + ry * s
        dy = -rx * s + ry * c
        mx = dx + cal["origin_x"]
        my = dy + cal["origin_y"]
        mth = (rth - a + np.pi) % (2 * np.pi) - np.pi
        return mx, my, mth

    def _robot_online(i):
        """Return True if we have a recent mocap or odom pose for this robot."""
        now = time.time()
        with state.lock:
            mfresh = (not np.isnan(state.mocap_pose[i, 0])
                      and now - state.mocap_ts[i] < 1.0)
            ofresh = (not np.isnan(state.odom_pose[i, 0])
                      and now - state.odom_ts[i] < 1.0)
        return mfresh or ofresh

    def _waypoints_with_headings(positions):
        """Compute heading at each waypoint = direction toward the next
        non-coincident waypoint (handles A,B,A-style paths cleanly)."""
        N = len(positions)
        wps = []
        for i, (px, py) in enumerate(positions):
            nx, ny = px, py
            for off in range(1, N):
                cand = positions[(i + off) % N]
                if abs(cand[0] - px) > 1e-6 or abs(cand[1] - py) > 1e-6:
                    nx, ny = cand
                    break
            wps.append((px, py, float(np.arctan2(ny - py, nx - px))))
        return wps

    def _send_robot_goal(rid):
        wp = _scenario["robots"][rid]["waypoints"][_scenario["robots"][rid]["idx"]]
        # Waypoints are in room frame; the robot expects raw mocap.
        mx, my, mth = _room_to_mocap(wp[0], wp[1], wp[2])
        pub.send_multipart([
            b"goal", goal_msg(mx, my, mth, _goal["tol"], robot_id=rid),
        ])

    def _refresh_marker():
        gx, gy, gth = _goal["x"], _goal["y"], _goal["theta"]
        goal_star.set_data([gx], [gy])
        dx = np.cos(gth) * ARROW_LEN * 2
        dy = np.sin(gth) * ARROW_LEN * 2
        goal_heading.set_position((gx, gy))
        goal_heading.xy = (gx + dx, gy + dy)
        goal_heading.xytext = (gx, gy)
        tol_circle.center = (gx, gy)
        tol_circle.set_radius(_goal["tol"])
        goal_star.set_label("goal (sent)" if _goal["sent"] else "goal (pending)")
        ax.legend(loc="lower right", fontsize=8)

    def _on_map_click(event):
        if event.inaxes is not ax or event.button != 1:
            return
        _goal["x"] = event.xdata
        _goal["y"] = event.ydata
        _goal["sent"] = False
        _block_slider[0] = True
        sl_x.set_val(_goal["x"])
        sl_y.set_val(_goal["y"])
        _block_slider[0] = False
        _refresh_marker()
        fig.canvas.draw_idle()

    def _on_slider(_val):
        if _block_slider[0]:
            return
        _goal["x"]     = sl_x.val
        _goal["y"]     = sl_y.val
        _goal["theta"] = sl_th.val
        _goal["tol"]   = sl_tol.val
        _goal["sent"]  = False
        _refresh_marker()
        fig.canvas.draw_idle()

    sl_x.on_changed(_on_slider)
    sl_y.on_changed(_on_slider)
    sl_th.on_changed(_on_slider)
    sl_tol.on_changed(_on_slider)
    fig.canvas.mpl_connect("button_press_event", _on_map_click)

    def _send(_event=None):
        gx, gy, gth, gtol = _goal["x"], _goal["y"], _goal["theta"], _goal["tol"]
        # _goal stores room-frame; robot expects raw mocap.
        gx_mc, gy_mc, gth_mc = _room_to_mocap(gx, gy, gth)
        pub.send_multipart([b"goal", goal_msg(gx_mc, gy_mc, gth_mc, gtol, robot_id=-1)])
        _goal["sent"] = True
        sent_text.set_text(f"last sent: ({gx:.2f}, {gy:.2f}, {gth:.2f} rad) tol={gtol:.2f} m")
        print(f"[control_panel] sent room=({gx:.3f},{gy:.3f},{gth:.3f}) "
              f"-> mocap=({gx_mc:.3f},{gy_mc:.3f},{gth_mc:.3f}) tol={gtol:.3f}")
        _refresh_marker()
        fig.canvas.draw_idle()

    def _stop(_event=None):
        _scenario["active"] = False
        pub.send_multipart([b"estop", estop_msg()])
        print("[control_panel] ESTOP sent")
        sent_text.set_text("ESTOP sent")
        fig.canvas.draw_idle()

    def _soft_stop(_event=None):
        _scenario["active"] = False
        pub.send_multipart([b"ctrl_stop", ctrl_stop_msg()])
        print("[control_panel] soft stop sent")
        sent_text.set_text("soft stop sent")
        fig.canvas.draw_idle()

    btn_send.on_clicked(_send)
    btn_stop.on_clicked(_stop)
    btn_softstop.on_clicked(_soft_stop)

    # -------------------------------------------------------------------------
    # Scenario window — frame calibration + shuttle between two fixed points.
    # -------------------------------------------------------------------------
    sc_fig = plt.figure(figsize=(6.0, 4.8))
    sc_fig.canvas.manager.set_window_title("Shuttle")

    ax_sc_msg    = sc_fig.add_axes([0.03, 0.94, 0.48, 0.05]); ax_sc_msg.axis("off")
    ax_btn_origin = sc_fig.add_axes([0.53, 0.93, 0.20, 0.06])
    ax_sc_cancel = sc_fig.add_axes([0.76, 0.93, 0.20, 0.06])

    ax_calib_label = sc_fig.add_axes([0.03, 0.86, 0.94, 0.05]); ax_calib_label.axis("off")
    ax_title     = sc_fig.add_axes([0.03, 0.78, 0.94, 0.05]); ax_title.axis("off")

    ax_start_x   = sc_fig.add_axes([0.30, 0.69, 0.62, 0.05])
    ax_start_y   = sc_fig.add_axes([0.30, 0.61, 0.62, 0.05])
    ax_end_x     = sc_fig.add_axes([0.30, 0.51, 0.62, 0.05])
    ax_end_y     = sc_fig.add_axes([0.30, 0.43, 0.62, 0.05])

    ax_laps      = sc_fig.add_axes([0.20, 0.33, 0.20, 0.05])
    ax_wait      = sc_fig.add_axes([0.65, 0.33, 0.20, 0.05])

    ax_btn_start = sc_fig.add_axes([0.03, 0.05, 0.30, 0.22])
    ax_btn_go    = sc_fig.add_axes([0.35, 0.05, 0.30, 0.22])
    ax_btn_laps  = sc_fig.add_axes([0.67, 0.05, 0.30, 0.22])

    ax_title.text(0.5, 0.5,
                  "Shuttle (all robots) — each in its own room frame, "
                  "heading locked, pure translation",
                  fontsize=11, fontweight="bold", ha="center", va="center")

    tb_start_x = TextBox(ax_start_x, "start X (m)", initial="0.0")
    tb_start_y = TextBox(ax_start_y, "start Y (m)", initial="0.0")
    tb_end_x   = TextBox(ax_end_x,   "end X (m)",   initial="0.0")
    tb_end_y   = TextBox(ax_end_y,   "end Y (m)",   initial="2.5")
    tb_laps    = TextBox(ax_laps,    "laps",        initial="4")
    tb_wait    = TextBox(ax_wait,    "wait (s)",    initial="1.0")

    btn_go_start = Button(ax_btn_start, "Go to Start",
                          color="lightblue", hovercolor="#4488cc")
    btn_go       = Button(ax_btn_go, "Go",
                          color="lightgreen", hovercolor="#00cc44")
    btn_run_laps = Button(ax_btn_laps, "Run Laps",
                          color="plum", hovercolor="#9966cc")
    btn_set_origin = Button(ax_btn_origin, "Set Origin",
                            color="lightyellow", hovercolor="#ddaa00")

    btn_cancel = Button(ax_sc_cancel, "Cancel",
                        color="lightsalmon", hovercolor="#cc4444")
    sc_msg = ax_sc_msg.text(0.0, 0.5,
                            "1. position robot with nose along desired +Y, click Set Origin. "
                            "2. Go / Go to Start.",
                            fontsize=9, va="center", color="gray")
    calib_label = ax_calib_label.text(
        0.5, 0.5, "frame: RAW MOCAP  (no calibration yet)",
        fontsize=10, ha="center", va="center",
        color="darkred", fontweight="bold",
        bbox=dict(boxstyle="round,pad=0.3", fc="mistyrose", alpha=0.8))

    def _refresh_scenario_overlay():
        for ln, dots, cur in scenario_overlays:
            ln.set_data([], [])
            dots.set_data([], [])
            cur.set_data([], [])
        if not _scenario["active"]:
            return
        for rid, rstate in _scenario["robots"].items():
            if rid >= len(scenario_overlays):
                continue
            wps = rstate["waypoints"]
            ln, dots, cur = scenario_overlays[rid]
            xs = [w[0] for w in wps] + [wps[0][0]]
            ys = [w[1] for w in wps] + [wps[0][1]]
            ln.set_data(xs, ys)
            dots.set_data(xs[:-1], ys[:-1])
            cur_wp = wps[rstate["idx"]]
            cur.set_data([cur_wp[0]], [cur_wp[1]])

    def _cancel_scenario(_event=None):
        was_running = _scenario["active"] or _laps["active"]
        _scenario["active"] = False
        _laps["active"]     = False
        if was_running:
            pub.send_multipart([b"ctrl_stop", ctrl_stop_msg()])
            sc_msg.set_text("cancelled.")
            sc_msg.set_color("black")
            print("[control_panel] cancelled")
        else:
            sc_msg.set_text("nothing running.")
            sc_msg.set_color("gray")
        _refresh_scenario_overlay()
        sc_fig.canvas.draw_idle()
        fig.canvas.draw_idle()

    def _send_laps_target(rid, room_x, room_y):
        """Send a goal to one robot using that robot's own room frame and the
        heading lock captured at the start of this laps run."""
        rstate = _laps["robots"][rid]
        mx, my, _ = _room_to_mocap(room_x, room_y, 0.0, rid=rid)
        pub.send_multipart([
            b"goal",
            goal_msg(mx, my, rstate["heading_lock_mocap"],
                     _goal["tol"], robot_id=rid),
        ])

    def _start_laps(_event=None):
        try:
            sx = float(tb_start_x.text)
            sy = float(tb_start_y.text)
            ex = float(tb_end_x.text)
            ey = float(tb_end_y.text)
            laps = int(tb_laps.text)
            wait = float(tb_wait.text)
        except ValueError:
            sc_msg.set_text("laps: invalid numeric input")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return
        if laps < 1 or wait < 0:
            sc_msg.set_text("laps: need laps>=1 and wait>=0")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return

        # Each robot needs a fresh pose so we can capture its heading lock.
        missing = []
        for r in cfg["robots"]:
            rid = r["id"]
            if state.best_pose(rid)[0] is None:
                missing.append(rid)
        if missing:
            sc_msg.set_text(f"laps: no pose for r{missing} — can't start")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return

        # Stop any scenario in flight.
        _scenario["active"] = False
        _scenario["robots"] = {}
        _refresh_scenario_overlay()

        _laps["total_laps"] = laps
        _laps["wait_sec"]   = wait
        _laps["start_room"] = (sx, sy)
        _laps["end_room"]   = (ex, ey)

        # Init every robot's lap state. Warm up to end first so each
        # subsequent lap is a clean end → start → end round trip regardless
        # of where the robot started.
        for r in cfg["robots"]:
            rid = r["id"]
            pose_r, _ = state.best_pose(rid)
            cal = _calib.get(rid)
            if cal is not None and cal["set"]:
                heading_lock = float(cal["origin_theta"])
            else:
                heading_lock = float(pose_r[2])
            rstate = _laps["robots"][rid]
            rstate["current_lap"]        = 1
            rstate["phase"]              = "warmup_to_end"
            rstate["phase_t0"]           = 0.0
            rstate["heading_lock_mocap"] = heading_lock

        _laps["active"] = True

        # Kick off each robot's first target.
        for r in cfg["robots"]:
            _send_laps_target(r["id"], ex, ey)

        sc_msg.set_text(f"laps: warm-up → end, then {laps} round-trip(s)  "
                        f"({len(cfg['robots'])} robot(s))")
        sc_msg.set_color("black")
        sc_fig.canvas.draw_idle()
        print(f"[control_panel] laps started: {laps} lap(s), wait={wait:.1f}s, "
              f"start=({sx:.2f},{sy:.2f}), end=({ex:.2f},{ey:.2f}) "
              f"robots={[r['id'] for r in cfg['robots']]}")

    def _laps_tick():
        if not _laps["active"]:
            return ""
        tol = _goal["tol"]
        now = time.time()
        total = _laps["total_laps"]
        sx_r, sy_r = _laps["start_room"]
        ex_r, ey_r = _laps["end_room"]

        parts = []
        all_done = True

        for r in cfg["robots"]:
            rid = r["id"]
            rstate = _laps["robots"][rid]
            phase = rstate["phase"]
            cur = rstate["current_lap"]

            if phase == "done":
                parts.append(f"r{rid}:done")
                continue
            all_done = False

            pose, _src = state.best_pose(rid)
            if pose is None:
                parts.append(f"r{rid}:no_pose")
                continue
            rx, ry, _ = _mocap_to_room(pose[0], pose[1], pose[2], rid=rid)

            if phase == "warmup_to_end":
                d = float(np.hypot(rx - ex_r, ry - ey_r))
                if d <= tol:
                    rstate["phase"] = "to_start"
                    _send_laps_target(rid, sx_r, sy_r)
                    parts.append(f"r{rid}:l{cur}/{total} →start")
                else:
                    parts.append(f"r{rid}:warm→end {d*100:.0f}cm")
                continue

            if phase == "to_start":
                d = float(np.hypot(rx - sx_r, ry - sy_r))
                if d <= tol:
                    rstate["phase"]    = "wait_at_start"
                    rstate["phase_t0"] = now
                    parts.append(f"r{rid}:l{cur}/{total} @start "
                                 f"wait{_laps['wait_sec']:.1f}s")
                else:
                    parts.append(f"r{rid}:l{cur}/{total} →start {d*100:.0f}cm")
                continue

            if phase == "wait_at_start":
                elapsed = now - rstate["phase_t0"]
                if elapsed >= _laps["wait_sec"]:
                    rstate["phase"] = "to_end"
                    _send_laps_target(rid, ex_r, ey_r)
                    parts.append(f"r{rid}:l{cur}/{total} →end")
                else:
                    parts.append(f"r{rid}:l{cur}/{total} @start "
                                 f"{elapsed:.1f}/{_laps['wait_sec']:.1f}s")
                continue

            if phase == "to_end":
                d = float(np.hypot(rx - ex_r, ry - ey_r))
                if d <= tol:
                    if cur >= total:
                        rstate["phase"] = "done"
                        parts.append(f"r{rid}:done")
                    else:
                        rstate["current_lap"] = cur + 1
                        rstate["phase"]       = "to_start"
                        _send_laps_target(rid, sx_r, sy_r)
                        parts.append(f"r{rid}:l{cur+1}/{total} →start")
                else:
                    parts.append(f"r{rid}:l{cur}/{total} →end {d*100:.0f}cm")
                continue

            parts.append(f"r{rid}:phase={phase}")

        if all_done:
            _laps["active"] = False
            pub.send_multipart([b"ctrl_stop", ctrl_stop_msg()])
            sc_msg.set_text(f"laps done — {total} lap(s) on all robots.")
            sc_msg.set_color("darkgreen")
            sc_fig.canvas.draw_idle()
            print(f"[control_panel] laps complete on all robots ({total} lap(s))")
            return "[laps] all done"

        return "[laps]  " + "  |  ".join(parts)

    def _go_to(target_x_room, target_y_room, label):
        """Send the same room-frame target to every robot in the config.
        Each robot's room frame is independent — its own calibration converts
        (target_x_room, target_y_room) to that robot's mocap-frame goal.

        Heading is locked to that robot's *calibrated origin heading* (the
        mocap θ captured at Set Origin), not its current heading. This way,
        every Go actively corrects any drift accumulated from prior runs
        instead of treating drifted-θ as the new reference."""
        targets = []
        missing = []
        for r in cfg["robots"]:
            rid = r["id"]
            pose_r, _ = state.best_pose(rid)
            if pose_r is None:
                missing.append(rid)
                continue
            cal = _calib.get(rid)
            if cal is not None and cal["set"]:
                locked_mocap_heading = float(cal["origin_theta"])
            else:
                # No calibration yet — fall back to current heading so Go
                # still works pre-Set-Origin (drift won't be corrected).
                locked_mocap_heading = float(pose_r[2])
            mx, my, _ = _room_to_mocap(target_x_room, target_y_room, 0.0,
                                       rid=rid)
            targets.append((rid, mx, my, locked_mocap_heading))

        if missing:
            sc_msg.set_text(f"{label}: no pose for r{missing} — can't lock heading")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return

        # Clear any scenario or laps run so they don't fight us.
        _scenario["active"] = False
        _scenario["robots"] = {}
        _laps["active"]     = False
        _refresh_scenario_overlay()

        for rid, mx, my, locked in targets:
            pub.send_multipart([
                b"goal",
                goal_msg(mx, my, locked, _goal["tol"], robot_id=rid),
            ])

        sc_msg.set_text(f"{label}: → room ({target_x_room:.2f}, "
                        f"{target_y_room:.2f}) sent to {len(targets)} robot(s)")
        sc_msg.set_color("black")
        sc_fig.canvas.draw_idle()
        fig.canvas.draw_idle()
        print(f"[control_panel] {label}: room=({target_x_room:.2f},"
              f"{target_y_room:.2f})")
        for rid, mx, my, locked in targets:
            _, _, locked_room = _mocap_to_room(0.0, 0.0, locked, rid=rid)
            print(f"  r{rid} -> mocap=({mx:.3f},{my:.3f})  "
                  f"heading_lock={np.degrees(locked_room):.1f}° (its room)")

    def _go_to_start(_event=None):
        try:
            sx = float(tb_start_x.text)
            sy = float(tb_start_y.text)
        except ValueError:
            sc_msg.set_text("invalid start coords")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return
        _go_to(sx, sy, "go to start")

    def _go_to_end(_event=None):
        try:
            ex = float(tb_end_x.text)
            ey = float(tb_end_y.text)
        except ValueError:
            sc_msg.set_text("invalid end coords")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return
        _go_to(ex, ey, "go")

    def _set_origin(_event=None):
        # Snapshot every robot's raw mocap pose simultaneously. Each robot
        # gets its own independent room frame anchored at its current pose
        # with the nose along that frame's +Y. state.best_pose returns raw
        # mocap — the per-robot transform is applied at display time.
        snapshots = []
        missing   = []
        for r in cfg["robots"]:
            rid = r["id"]
            pose_r, _ = state.best_pose(rid)
            if pose_r is None:
                missing.append(rid)
                continue
            snapshots.append((rid,
                              float(pose_r[0]),
                              float(pose_r[1]),
                              float(pose_r[2])))
        if missing:
            sc_msg.set_text(f"set origin: no pose for r{missing} — is mocap running?")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return

        for rid, mx, my, mth in snapshots:
            cal = _calib[rid]
            cal["origin_x"]     = mx
            cal["origin_y"]     = my
            cal["origin_theta"] = mth
            cal["alpha"]        = (np.pi / 2.0) - mth   # nose along room +Y
            cal["set"]          = True

        # Reset trails — old samples were in raw mocap and would jump.
        for tx, ty in trails:
            tx.clear(); ty.clear()

        summary = "  |  ".join(
            f"r{rid}: m=({mx:+.2f},{my:+.2f}) mθ={np.degrees(mth):+.1f}°"
            for rid, mx, my, mth in snapshots
        )
        calib_label.set_text(f"frame: ROOM (per robot)  {summary}")
        calib_label.set_color("darkgreen")
        calib_label.get_bbox_patch().set_facecolor("honeydew")
        sc_msg.set_text(f"origin set for {len(snapshots)} robot(s). "
                        "each at room (0, 0) facing its own +Y.")
        sc_msg.set_color("black")
        sc_fig.canvas.draw_idle()
        fig.canvas.draw_idle()
        for rid, mx, my, mth in snapshots:
            alpha = (np.pi / 2.0) - mth
            print(f"[control_panel] r{rid} calib: "
                  f"origin_mocap=({mx:.3f},{my:.3f}) origin_theta={mth:.3f}rad "
                  f"alpha={alpha:.3f}rad ({np.degrees(alpha):+.1f}°)")

    btn_go_start.on_clicked(_go_to_start)
    btn_go.on_clicked(_go_to_end)
    btn_run_laps.on_clicked(_start_laps)
    btn_set_origin.on_clicked(_set_origin)
    btn_cancel.on_clicked(_cancel_scenario)

    def _scenario_tick():
        """Per-robot look-ahead sequencer. Each robot in _scenario['robots']
        advances independently along its own waypoint list. Scenario ends
        when *all* participating robots have completed their laps."""
        if not _scenario["active"]:
            return ""
        if not _scenario["robots"]:
            _scenario["active"] = False
            return ""

        parts = []
        all_done = True
        any_dirty = False

        for rid, rstate in _scenario["robots"].items():
            if rstate["done"]:
                parts.append(f"r{rid}:done")
                continue
            all_done = False

            pose, _ = state.best_pose(rid)
            if pose is None:
                parts.append(f"r{rid}:no_pose")
                continue

            # Waypoints stored in room frame; transform pose so distance check
            # is in the same frame as the user-defined path.
            rx, ry, _ = _mocap_to_room(pose[0], pose[1], pose[2])
            wps = rstate["waypoints"]
            idx = rstate["idx"]
            cur = wps[idx]
            dist = float(np.hypot(rx - cur[0], ry - cur[1]))

            is_last_wp   = (idx >= len(wps) - 1)
            is_final_lap = (rstate["lap"] >= _scenario["total_laps"])
            if is_last_wp and is_final_lap:
                advance = _goal["tol"]
            else:
                look = None
                for off in range(1, len(wps)):
                    cand = wps[(idx + off) % len(wps)]
                    if abs(cand[0] - cur[0]) > 1e-6 or abs(cand[1] - cur[1]) > 1e-6:
                        look = cand
                        break
                spacing = float(np.hypot(cur[0] - look[0], cur[1] - look[1])) if look else 0.0
                advance = max(_goal["tol"], 0.5 * spacing)

            if dist <= advance:
                rstate["idx"] += 1
                if rstate["idx"] >= len(wps):
                    if rstate["lap"] >= _scenario["total_laps"]:
                        rstate["done"] = True
                        parts.append(f"r{rid}:done")
                        any_dirty = True
                        continue
                    rstate["lap"] += 1
                    rstate["idx"] = 0
                _send_robot_goal(rid)
                any_dirty = True

            parts.append(f"r{rid}:l{rstate['lap']}/{_scenario['total_laps']}"
                         f"w{rstate['idx']+1}/{len(wps)} {dist*100:.0f}cm")

        if any_dirty:
            _refresh_scenario_overlay()

        if all_done:
            _scenario["active"] = False
            pub.send_multipart([b"ctrl_stop", ctrl_stop_msg()])
            sc_msg.set_text(f"done — all robots completed {_scenario['total_laps']} lap(s).")
            sc_msg.set_color("darkgreen")
            sc_fig.canvas.draw_idle()
            print("[control_panel] scenario complete")
            _refresh_scenario_overlay()
            return "[scenario done]"

        return f"[{_scenario['name']}]  " + "  ".join(parts)

    # -------------------------------------------------------------------------
    # Animation loop
    # -------------------------------------------------------------------------
    _PAD = 0.5

    def _update(_frame):
        all_x = [_goal["x"]]
        all_y = [_goal["y"]]
        active_sources = set()

        for i, r in enumerate(cfg["robots"]):
            rid = r["id"]
            pose, source = state.best_pose(rid)
            vis = pose is not None
            robot_circles[i].set_visible(vis)
            robot_arrows[i].set_visible(vis)
            robot_labels[i].set_visible(vis)
            if vis:
                # state holds raw mocap; transform to each robot's own room
                # frame so both appear at (0, 0) after Set Origin.
                x, y, theta = _mocap_to_room(pose[0], pose[1], pose[2], rid=rid)
                robot_circles[i].center = (x, y)
                robot_labels[i].set_position((x, y))
                dx = np.cos(theta) * ARROW_LEN
                dy = np.sin(theta) * ARROW_LEN
                robot_arrows[i].set_position((x, y))
                robot_arrows[i].xy = (x + dx, y + dy)
                robot_arrows[i].xytext = (x, y)
                tx, ty = trails[i]
                tx.append(x); ty.append(y)
                if len(tx) > TRAIL_LEN:
                    tx.pop(0); ty.pop(0)
                trail_lines[i].set_data(tx, ty)
                all_x.append(x); all_y.append(y)
                active_sources.add(source)

        if active_sources:
            source_text.set_text(f"pose source: {', '.join(sorted(active_sources))}")
        else:
            source_text.set_text("pose source: — (waiting)")

        scenario_status = _scenario_tick()
        laps_status     = _laps_tick()

        # Status: laps > scenario > dist-to-goal.
        # Show distance-to-goal for the first robot in the yaml (display only).
        first_rid = cfg["robots"][0]["id"] if cfg["robots"] else None
        pose0, _ = (state.best_pose(first_rid)
                    if first_rid is not None else (None, None))
        if laps_status:
            status_text.set_text(laps_status)
            status_text.get_bbox_patch().set_facecolor("lavender")
        elif scenario_status:
            status_text.set_text(scenario_status)
            status_text.get_bbox_patch().set_facecolor("lavender")
        elif pose0 is not None:
            # _goal is in room frame; transform pose to room frame for the
            # distance display so it matches what the user typed.
            rx, ry, _ = _mocap_to_room(pose0[0], pose0[1], pose0[2],
                                       rid=first_rid)
            dist = float(np.hypot(rx - _goal["x"], ry - _goal["y"]))
            pending = "" if _goal["sent"] else "  [PENDING — click Send Goal]"
            msg = f"r{first_rid} dist to goal: {dist*100:.1f} cm{pending}"
            reached = dist < _goal["tol"]
            if reached and _goal["sent"]:
                msg += "  ✓ REACHED"
            status_text.set_text(msg)
            status_text.get_bbox_patch().set_facecolor(
                "lightgreen" if (reached and _goal["sent"]) else "white"
            )
        else:
            status_text.set_text("waiting for robot pose…")

        if all_x:
            xmin, xmax = min(all_x) - _PAD, max(all_x) + _PAD
            ymin, ymax = min(all_y) - _PAD, max(all_y) + _PAD
            cur_xl, cur_yl = ax.get_xlim(), ax.get_ylim()
            ax.set_xlim(min(cur_xl[0], xmin), max(cur_xl[1], xmax))
            ax.set_ylim(min(cur_yl[0], ymin), max(cur_yl[1], ymax))

        scenario_artists = [a for tup in scenario_overlays for a in tup]
        return (robot_circles + robot_arrows + robot_labels + trail_lines +
                scenario_artists +
                [goal_star, goal_heading, status_text, sent_text, source_text])

    _anim = FuncAnimation(fig, _update, interval=50, blit=False)
    plt.show()


if __name__ == "__main__":
    main()
