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
from matplotlib.widgets import Slider, Button

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
    # Goal state
    # -------------------------------------------------------------------------
    _goal = {"x": 0.0, "y": 0.0, "theta": 0.0, "tol": args.goal_tol, "sent": False}
    _block_slider = [False]

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
        pub.send_multipart([b"estop", estop_msg()])
        print("[control_panel] ESTOP sent")
        sent_text.set_text("ESTOP sent")
        fig.canvas.draw_idle()

    def _soft_stop(_event=None):
        pub.send_multipart([b"ctrl_stop", ctrl_stop_msg()])
        print("[control_panel] soft stop sent")
        sent_text.set_text("soft stop sent")
        fig.canvas.draw_idle()

    btn_send.on_clicked(_send)
    btn_stop.on_clicked(_stop)
    btn_softstop.on_clicked(_soft_stop)

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

        # Status: distance to goal for robot 0 (primary indicator)
        pose0, _ = state.best_pose(0)
        if pose0 is not None:
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
                [goal_star, goal_heading, status_text, sent_text, source_text])

    _anim = FuncAnimation(fig, _update, interval=50, blit=False)
    plt.show()


if __name__ == "__main__":
    main()
