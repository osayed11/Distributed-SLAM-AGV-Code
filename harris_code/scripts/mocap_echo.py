#!/usr/bin/env python3
"""
Subscribe to mocap_pub's ZMQ output and print whatever pose messages come
through. Tests the layer *above* OWL — i.e. assumes mocap_pub is already
running and verifies the ZMQ publish side is reachable.

    cd harris_code
    python3 scripts/mocap_echo.py                                # localhost
    python3 scripts/mocap_echo.py --laptop 192.168.1.142         # from a robot

If you see pose lines here but the robot's drive_runner still says "no fresh
mocap pose", the problem is the robot's SUB-side connection, not mocap_pub.
"""
import argparse
import os
import sys
import time

import msgpack
import yaml
import zmq


HERE       = os.path.dirname(os.path.abspath(__file__))
HARRIS_DIR = os.path.abspath(os.path.join(HERE, ".."))
DEFAULT_CFG = os.path.join(HARRIS_DIR, "config", "network.yaml")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default=DEFAULT_CFG)
    p.add_argument("--laptop", default=None,
                   help="Override laptop IP (defaults to network.yaml's value)")
    args = p.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    ip   = args.laptop or cfg["laptop"]["ip"]
    port = cfg["laptop"]["mocap_pub_port"]

    print(f"subscribing to tcp://{ip}:{port}  (Ctrl+C to stop)\n")

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://{ip}:{port}")
    sub.setsockopt_string(zmq.SUBSCRIBE, "pose")

    last_status = time.time()
    count = 0
    seen_ids = set()
    try:
        while True:
            try:
                _, raw = sub.recv_multipart(zmq.NOBLOCK)
                d = msgpack.unpackb(raw, raw=False)
                rid = d.get("id")
                seen_ids.add(rid)
                count += 1
                print(f"  pose id={rid:3d}  x={d['x']:+.3f}m  y={d['y']:+.3f}m  "
                      f"theta={d['theta']:+.2f}rad")
            except zmq.Again:
                time.sleep(0.02)

            now = time.time()
            if now - last_status >= 2.0:
                if count == 0:
                    print(f"  …no pose messages in 2s "
                          f"(ip={ip}, port={port})")
                else:
                    print(f"  [{count/2.0:.1f} msg/s over last 2s] "
                          f"ids seen: {sorted(seen_ids)}")
                count = 0
                last_status = now
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
