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
    def __init__(self, n: int):
        self.lock = threading.Lock()
        self.mocap_pose = np.full((n, 3), np.nan)
        self.mocap_ts   = np.zeros(n)
        self.odom_pose  = np.full((n, 3), np.nan)
        self.odom_ts    = np.zeros(n)
        self.n = n

    def best_pose(self, i: int):
        """Return (pose, source) where source is 'mocap', 'odom', or None."""
        now = time.time()
        with self.lock:
            if not np.isnan(self.mocap_pose[i, 0]) and now - self.mocap_ts[i] < MOCAP_FRESH_SEC:
                return self.mocap_pose[i].copy(), "mocap"
            if not np.isnan(self.odom_pose[i, 0]):
                return self.odom_pose[i].copy(), "odom"
        return None, None


def _pose_listener(state: _PoseState, cfg: dict, n: int):
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    for r in cfg["robots"][:n]:
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
        rid = d.get("id", 0)
        if not (0 <= rid < n):
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

    n = args.n_robots if args.n_robots is not None else len(cfg["robots"])
    state = _PoseState(n)
    threading.Thread(target=_pose_listener, args=(state, cfg, n), daemon=True).start()

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
    for i in range(n):
        col = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        circ = plt.Circle((0, 0), ROBOT_RADIUS, color=col, alpha=0.5, zorder=3)
        ax.add_patch(circ)
        arr = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                          arrowprops=dict(arrowstyle="->", color=col, lw=2), zorder=4)
        lbl = ax.text(0, 0, f"r{i}", fontsize=8, ha="center", va="center",
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

    # Scenario overlay: the planned waypoint path and the currently-active waypoint.
    scenario_line,    = ax.plot([], [], color="purple", linestyle=":", linewidth=1.2,
                                alpha=0.7, zorder=4, label="path (not running)")
    scenario_dots,    = ax.plot([], [], marker="o", markersize=6, color="purple",
                                linestyle="None", alpha=0.5, zorder=4)
    scenario_current, = ax.plot([], [], marker="o", markersize=12,
                                markerfacecolor="none", markeredgecolor="purple",
                                markeredgewidth=2, linestyle="None", zorder=5)

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
    _scenario = {
        "active":   False,
        "name":     "",
        "waypoints": [],   # list of (x, y, theta)
        "idx":      0,
        "lap":      0,
        "total_laps": 1,
    }

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
        # Broadcast to all robots — robot_id=-1.
        pub.send_multipart([b"goal", goal_msg(gx, gy, gth, gtol, robot_id=-1)])
        _goal["sent"] = True
        sent_text.set_text(f"last sent: ({gx:.2f}, {gy:.2f}, {gth:.2f} rad) tol={gtol:.2f} m")
        print(f"[control_panel] sent x={gx:.3f} y={gy:.3f} theta={gth:.3f} tol={gtol:.3f}")
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
    # Scenario window — preload paths and trigger from a button
    # -------------------------------------------------------------------------
    sc_fig = plt.figure(figsize=(5.5, 5.4))
    sc_fig.canvas.manager.set_window_title("Scenarios")

    # Top bar: status message + shared Cancel button.
    ax_sc_msg    = sc_fig.add_axes([0.03, 0.94, 0.63, 0.04]); ax_sc_msg.axis("off")
    ax_sc_cancel = sc_fig.add_axes([0.70, 0.92, 0.27, 0.06])

    # Scenario 1: circle.
    ax_circ_title  = sc_fig.add_axes([0.03, 0.84, 0.94, 0.05]); ax_circ_title.axis("off")
    ax_circ_radius = sc_fig.add_axes([0.30, 0.76, 0.62, 0.05])
    ax_circ_wpts   = sc_fig.add_axes([0.30, 0.68, 0.62, 0.05])
    ax_circ_laps   = sc_fig.add_axes([0.30, 0.60, 0.62, 0.05])
    ax_circ_run    = sc_fig.add_axes([0.20, 0.50, 0.60, 0.07])

    # Scenario 2: line (round-trip A→B→A).
    ax_line_title  = sc_fig.add_axes([0.03, 0.42, 0.94, 0.05]); ax_line_title.axis("off")
    ax_line_endx   = sc_fig.add_axes([0.30, 0.32, 0.62, 0.05])
    ax_line_endy   = sc_fig.add_axes([0.30, 0.22, 0.62, 0.05])
    ax_line_laps   = sc_fig.add_axes([0.30, 0.12, 0.62, 0.05])
    ax_line_run    = sc_fig.add_axes([0.20, 0.01, 0.60, 0.07])

    ax_circ_title.text(0.5, 0.5, "Scenario 1: Circle  (center = Goal X / Y)",
                       fontsize=10, fontweight="bold", ha="center", va="center")
    ax_line_title.text(0.5, 0.5, "Scenario 2: Line  (A = Goal X / Y, B = end X / Y)",
                       fontsize=10, fontweight="bold", ha="center", va="center")

    tb_radius   = TextBox(ax_circ_radius, "radius (m)", initial="1.0")
    tb_wpts     = TextBox(ax_circ_wpts,   "waypoints",  initial="16")
    tb_laps     = TextBox(ax_circ_laps,   "laps",       initial="1")
    btn_run_circ = Button(ax_circ_run,    "Run Scenario 1 (Circle)",
                          color="lightgreen", hovercolor="#00cc44")

    tb_end_x    = TextBox(ax_line_endx,   "end X (m)",  initial="2.0")
    tb_end_y    = TextBox(ax_line_endy,   "end Y (m)",  initial="0.0")
    tb_line_laps = TextBox(ax_line_laps,  "laps",       initial="1")
    btn_run_line = Button(ax_line_run,    "Run Scenario 2 (Line)",
                          color="lightgreen", hovercolor="#00cc44")

    btn_cancel  = Button(ax_sc_cancel, "Cancel", color="lightsalmon", hovercolor="#cc4444")
    sc_msg = ax_sc_msg.text(0.0, 0.5, "tolerance = main 'Tolerance' slider.  "
                                      "1 lap of Line = A→B→A.",
                            fontsize=9, va="center", color="gray")

    def _refresh_scenario_overlay():
        wps = _scenario["waypoints"]
        if not wps or not _scenario["active"]:
            scenario_line.set_data([], [])
            scenario_dots.set_data([], [])
            scenario_current.set_data([], [])
            return
        xs = [w[0] for w in wps] + [wps[0][0]]  # close the loop
        ys = [w[1] for w in wps] + [wps[0][1]]
        scenario_line.set_data(xs, ys)
        scenario_dots.set_data(xs[:-1], ys[:-1])
        cur = wps[_scenario["idx"]]
        scenario_current.set_data([cur[0]], [cur[1]])

    def _send_current_waypoint():
        wp = _scenario["waypoints"][_scenario["idx"]]
        pub.send_multipart([
            b"goal",
            goal_msg(wp[0], wp[1], wp[2], _goal["tol"], robot_id=-1),
        ])

    def _start_circle(_event=None):
        try:
            radius = float(tb_radius.text)
            laps   = int(tb_laps.text)
            n_wpts = int(tb_wpts.text)
        except ValueError:
            sc_msg.set_text("invalid numeric input — check radius / laps / waypoints")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return
        if n_wpts < 3 or radius <= 0 or laps < 1:
            sc_msg.set_text("need radius>0, laps>=1, waypoints>=3")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return

        cx, cy = _goal["x"], _goal["y"]
        waypoints = []
        for i in range(n_wpts):
            t = 2.0 * np.pi * i / n_wpts
            # Heading = tangent to the circle at this point (CCW traversal),
            # so the front-mounted camera points along the direction of motion.
            heading = t + np.pi / 2.0
            heading = (heading + np.pi) % (2.0 * np.pi) - np.pi  # wrap to [-π, π]
            waypoints.append((cx + radius * np.cos(t),
                              cy + radius * np.sin(t),
                              heading))

        _scenario["active"]     = True
        _scenario["name"]       = f"circle r={radius:.2f}m"
        _scenario["waypoints"]  = waypoints
        _scenario["idx"]        = 0
        _scenario["lap"]        = 1
        _scenario["total_laps"] = laps

        _send_current_waypoint()
        sc_msg.set_text(f"running circle: r={radius:.2f}m, {n_wpts} waypoints, {laps} lap(s)")
        sc_msg.set_color("black")
        _refresh_scenario_overlay()
        sc_fig.canvas.draw_idle()
        fig.canvas.draw_idle()
        print(f"[control_panel] scenario started: circle "
              f"center=({cx:.2f},{cy:.2f}) r={radius:.2f} n={n_wpts} laps={laps}")

    def _cancel_scenario(_event=None):
        if not _scenario["active"]:
            sc_msg.set_text("no scenario running.")
            sc_msg.set_color("gray")
        else:
            _scenario["active"] = False
            pub.send_multipart([b"ctrl_stop", ctrl_stop_msg()])
            sc_msg.set_text(f"cancelled at waypoint {_scenario['idx']+1}.")
            sc_msg.set_color("black")
            print("[control_panel] scenario cancelled")
        _refresh_scenario_overlay()
        sc_fig.canvas.draw_idle()
        fig.canvas.draw_idle()

    def _start_line(_event=None):
        try:
            end_x = float(tb_end_x.text)
            end_y = float(tb_end_y.text)
            laps  = int(tb_line_laps.text)
        except ValueError:
            sc_msg.set_text("invalid numeric input — check end X/Y / laps")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return
        if laps < 1:
            sc_msg.set_text("need laps>=1")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return

        sx, sy = _goal["x"], _goal["y"]
        if abs(end_x - sx) < 1e-6 and abs(end_y - sy) < 1e-6:
            sc_msg.set_text("end point coincides with Goal X/Y — set a different end")
            sc_msg.set_color("red")
            sc_fig.canvas.draw_idle()
            return

        # A line only needs its endpoints. Intermediate samples just cause the
        # P controller to brake at each one. Round trip = A → B → A.
        positions = [(sx, sy), (end_x, end_y), (sx, sy)]

        # Heading at each waypoint = direction toward the *next meaningful*
        # waypoint. positions[-1] is co-located with positions[0], so for the
        # last entry we look one further ahead (positions[1]=B) — this keeps
        # the camera facing forward for the lap-to-lap transition.
        N = len(positions)
        waypoints = []
        for i, (px, py) in enumerate(positions):
            nidx = (i + 1) % N
            nx, ny = positions[nidx]
            if abs(nx - px) < 1e-6 and abs(ny - py) < 1e-6 and N > 2:
                nidx = (i + 2) % N
                nx, ny = positions[nidx]
            waypoints.append((px, py, float(np.arctan2(ny - py, nx - px))))

        _scenario["active"]     = True
        _scenario["name"]       = f"line ({sx:.2f},{sy:.2f})→({end_x:.2f},{end_y:.2f})"
        _scenario["waypoints"]  = waypoints
        _scenario["idx"]        = 0
        _scenario["lap"]        = 1
        _scenario["total_laps"] = laps

        _send_current_waypoint()
        sc_msg.set_text(f"running line: {laps} round-trip(s)")
        sc_msg.set_color("black")
        _refresh_scenario_overlay()
        sc_fig.canvas.draw_idle()
        fig.canvas.draw_idle()
        print(f"[control_panel] scenario started: line "
              f"A=({sx:.2f},{sy:.2f}) B=({end_x:.2f},{end_y:.2f}) laps={laps}")

    btn_run_circ.on_clicked(_start_circle)
    btn_run_line.on_clicked(_start_line)
    btn_cancel.on_clicked(_cancel_scenario)

    def _scenario_tick():
        """Called from the animation loop. Advances to the next waypoint when
        the robot is within a look-ahead distance of the current one (so the
        P controller never has to fully decelerate at intermediate waypoints)."""
        if not _scenario["active"]:
            return ""
        pose0, _ = state.best_pose(0)
        if pose0 is None:
            return "[scenario waiting for pose]"
        wps = _scenario["waypoints"]
        idx = _scenario["idx"]
        cur = wps[idx]
        dist = float(np.hypot(pose0[0] - cur[0], pose0[1] - cur[1]))

        # The "advance" threshold: at the very last waypoint of the very last
        # lap, fall back to the precise tolerance so the robot actually settles
        # at the endpoint. Everywhere else, look ahead half the way to the
        # next *non-coincident* waypoint — robot keeps cruising into the turn.
        is_last_wp   = (idx >= len(wps) - 1)
        is_final_lap = (_scenario["lap"] >= _scenario["total_laps"])
        if is_last_wp and is_final_lap:
            advance = _goal["tol"]
        else:
            # Find the next waypoint that isn't co-located with the current
            # one (handles the line's final-A duplicate cleanly).
            look = None
            for off in (1, 2):
                cand = wps[(idx + off) % len(wps)]
                if abs(cand[0] - cur[0]) > 1e-6 or abs(cand[1] - cur[1]) > 1e-6:
                    look = cand
                    break
            spacing = float(np.hypot(cur[0] - look[0], cur[1] - look[1])) if look else 0.0
            advance = max(_goal["tol"], 0.5 * spacing)

        if dist <= advance:
            _scenario["idx"] += 1
            if _scenario["idx"] >= len(wps):
                if _scenario["lap"] >= _scenario["total_laps"]:
                    _scenario["active"] = False
                    pub.send_multipart([b"ctrl_stop", ctrl_stop_msg()])
                    sc_msg.set_text(f"done — {_scenario['total_laps']} lap(s) completed.")
                    sc_msg.set_color("darkgreen")
                    sc_fig.canvas.draw_idle()
                    print("[control_panel] scenario complete")
                    _refresh_scenario_overlay()
                    return "[scenario done]"
                _scenario["lap"] += 1
                _scenario["idx"]  = 0
            _send_current_waypoint()
            _refresh_scenario_overlay()
        return (f"[{_scenario['name']}] lap {_scenario['lap']}/{_scenario['total_laps']} "
                f"wp {_scenario['idx']+1}/{len(wps)}  "
                f"dist={dist*100:.0f} cm  ahead@{advance*100:.0f} cm")

    # -------------------------------------------------------------------------
    # Animation loop
    # -------------------------------------------------------------------------
    _PAD = 0.5

    def _update(_frame):
        all_x = [_goal["x"]]
        all_y = [_goal["y"]]
        active_sources = set()

        for i in range(n):
            pose, source = state.best_pose(i)
            vis = pose is not None
            robot_circles[i].set_visible(vis)
            robot_arrows[i].set_visible(vis)
            robot_labels[i].set_visible(vis)
            if vis:
                x, y, theta = pose
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

        # Status: scenario takes precedence when active; otherwise show dist to goal.
        pose0, _ = state.best_pose(0)
        if scenario_status:
            status_text.set_text(scenario_status)
            status_text.get_bbox_patch().set_facecolor("lavender")
        elif pose0 is not None:
            dist = float(np.hypot(pose0[0] - _goal["x"], pose0[1] - _goal["y"]))
            pending = "" if _goal["sent"] else "  [PENDING — click Send Goal]"
            msg = f"r0 dist to goal: {dist*100:.1f} cm{pending}"
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

        return (robot_circles + robot_arrows + robot_labels + trail_lines +
                [goal_star, goal_heading, status_text, sent_text, source_text,
                 scenario_line, scenario_dots, scenario_current])

    _anim = FuncAnimation(fig, _update, interval=50, blit=False)
    plt.show()


if __name__ == "__main__":
    main()
