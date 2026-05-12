#!/usr/bin/env python3
"""
Standalone mocap publisher — no ROS2 required.

Connects directly to the PhaseSpace OWL server via owl_bridge.so,
applies the lab coordinate transform, and publishes ZMQ pose messages
in the same format as the old mocap_bridge.py.

Build owl_bridge.so first:
  cd real_robot/scripts && make

Run:
  python -m real_robot.scripts.mocap_pub --config real_robot/config/network.yaml
  python -m real_robot.scripts.mocap_pub --config real_robot/config/network.yaml --server 192.168.1.25
"""
import argparse
import ctypes
import math
import os
import signal
import time

import msgpack
import yaml
import zmq

PAYLOAD_ID = -1
MAX_RIGIDS = 64
STRIDE = 9  # floats per rigid: [id, owl_x, owl_y, owl_z, qw, qx, qy, qz, cond]
STATUS_INTERVAL = 2.0  # seconds between status prints


def _load_lib():
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, "..", ".."))
    owlsock = os.path.join(repo_root, "src", "swarm_mocap", "lib", "libowlsock.so")
    ctypes.CDLL(owlsock, mode=ctypes.RTLD_GLOBAL)  # make OWL symbols globally visible
    lib = ctypes.CDLL(os.path.join(here, "owl_bridge.so"))
    lib.owl_open.restype = ctypes.c_int
    lib.owl_open.argtypes = [ctypes.c_char_p]
    lib.owl_close.restype = None
    lib.owl_poll.restype = ctypes.c_int
    lib.owl_poll.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_int]
    return lib


def _pose_msg(robot_id: int, x: float, y: float, theta: float) -> bytes:
    return msgpack.packb({"t": "pose", "id": robot_id, "ts": time.time(),
                          "x": x, "y": y, "theta": theta})


def _owl_to_ros(p):
    """
    OWL native pose[7] = [x, y, z, qw, qx, qy, qz] in mm.
    Coordinate convention (matches lab's ROS1/ROS2 node):
      ros_x =  owl_x / 1000
      ros_y = -owl_z / 1000   (axis swap)
      ros_z =  owl_y / 1000   (axis swap)
      qy_ros = -qz_owl
      qz_ros =  qy_owl
    """
    x  =  p[0] / 1000.0
    y  = -p[2] / 1000.0
    qw =  p[3]
    qx =  p[4]
    qy = -p[6]
    qz =  p[5]
    return x, y, qw, qx, qy, qz


def _yaw(qw, qx, qy, qz):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="real_robot/config/network.yaml")
    parser.add_argument("--server", default=None,
                        help="PhaseSpace server IP (overrides config)")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    server_ip = args.server or cfg.get("mocap", {}).get("server_ip", "192.168.1.71")

    id_map = {}  # phasespace_id -> logical id
    for r in cfg["robots"]:
        if "mocap_rigid_id" in r:
            id_map[r["mocap_rigid_id"]] = r["id"]
    if "payload" in cfg:
        id_map[cfg["payload"]["mocap_rigid_id"]] = PAYLOAD_ID

    print(f"Watching PhaseSpace IDs: { {ps: lid for ps, lid in id_map.items()} }")
    print(f"  (payload logical id = {PAYLOAD_ID})")

    lib = _load_lib()
    ret = lib.owl_open(server_ip.encode())
    if ret < 0:
        raise RuntimeError(f"OWL open failed (code {ret}) — server: {server_ip}")
    print(f"Connected to OWL server at {server_ip}")

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{cfg['laptop']['mocap_pub_port']}")
    print(f"Publishing on tcp://*:{cfg['laptop']['mocap_pub_port']}")
    print(f"Monitor with:  python -m real_robot.scripts.mocap_echo "
          f"--port {cfg['laptop']['mocap_pub_port']}\n")

    buf = (ctypes.c_float * (MAX_RIGIDS * STRIDE))()
    running = True
    known_ids = set()       # phasespace IDs seen so far this session
    frame_count = 0
    last_status = time.time()
    active_this_window = set()  # phasespace IDs seen in current status window

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    while running:
        n = lib.owl_poll(buf, MAX_RIGIDS)
        if n < 0:
            print("OWL error event — continuing")
            continue
        if n > 0:
            frame_count += 1

        for i in range(n):
            ps_id = int(buf[i * STRIDE])
            active_this_window.add(ps_id)

            if ps_id not in known_ids:
                known_ids.add(ps_id)
                label = f"logical {id_map[ps_id]}" if ps_id in id_map else "not in config"
                print(f"  [new] PhaseSpace rigid {ps_id} detected ({label})")

            if ps_id not in id_map:
                continue
            p = buf[i * STRIDE + 1: i * STRIDE + 8]
            x, y, qw, qx, qy, qz = _owl_to_ros(p)
            theta = _yaw(qw, qx, qy, qz)
            pub.send_multipart([b"pose", _pose_msg(id_map[ps_id], x, y, theta)])

        now = time.time()
        if now - last_status >= STATUS_INTERVAL:
            fps = frame_count / (now - last_status)
            if active_this_window:
                ids_str = ", ".join(
                    f"{ps}→{id_map[ps]}" if ps in id_map else str(ps)
                    for ps in sorted(active_this_window)
                )
            else:
                ids_str = "none"
            print(f"[{fps:.1f} Hz]  active rigids: {ids_str}")
            frame_count = 0
            last_status = now
            active_this_window.clear()

    lib.owl_close()
    print("Shutdown.")


if __name__ == "__main__":
    main()
