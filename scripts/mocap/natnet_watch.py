#!/usr/bin/env python3
"""Print a live OptiTrack NatNet rigid-body pose."""

import argparse
import socket
import sys
import time

try:
    from natnet import NatNetClient
except ImportError:
    print(
        "Missing Python package 'natnet'. Install it with: python3 -m pip install natnet",
        file=sys.stderr,
    )
    raise


def guess_local_ip(server):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((server, 1510))
        return sock.getsockname()[0]
    finally:
        sock.close()


def fmt_tuple(values):
    return "(" + ", ".join("%.4f" % value for value in values) + ")"


def parse_args():
    parser = argparse.ArgumentParser(description="Watch one rigid body from a NatNet stream.")
    parser.add_argument("--server", default="192.168.50.200",
                        help="Motive/NatNet server IP")
    parser.add_argument("--local", default=None,
                        help="Local interface IP. Defaults to auto-detect.")
    parser.add_argument("--name", default="orkar_agv1",
                        help="Rigid body name to print")
    parser.add_argument("--period", type=float, default=0.25,
                        help="Print period in seconds")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Stop after this many seconds. 0 means run until Ctrl+C.")
    parser.add_argument("--once", action="store_true",
                        help="Exit after the first frame for the requested rigid body")
    parser.add_argument("--multicast", action="store_true",
                        help="Use multicast data reception instead of unicast")
    return parser.parse_args()


def main():
    args = parse_args()
    local_ip = args.local or guess_local_ip(args.server)

    state = {
        "names": [],
        "printed_defs": False,
        "last_print": 0.0,
        "seen_target": False,
    }

    def on_descriptions(desc):
        state["names"] = [rb.name for rb in desc.rigid_bodies]
        if state["printed_defs"]:
            return

        print("NatNet server: %s" % args.server)
        print("Local interface: %s" % local_ip)
        print("Rigid bodies:")
        for rb in desc.rigid_bodies:
            marker_count = len(rb.markers) if rb.markers is not None else 0
            print("  name=%s id=%s markers=%d" % (rb.name, rb.id_num, marker_count))
        if args.name not in state["names"]:
            print("WARN: requested rigid body '%s' is not in model definitions." % args.name)
        state["printed_defs"] = True

    def on_frame(frame):
        if args.name not in state["names"]:
            return
        index = state["names"].index(args.name)
        if index >= len(frame.rigid_bodies):
            return

        rb = frame.rigid_bodies[index]
        now = time.time()
        if now - state["last_print"] < args.period and not args.once:
            return

        state["seen_target"] = True
        state["last_print"] = now
        print(
            "%s valid=%s pos=%s rot=%s marker_error=%s"
            % (
                args.name,
                rb.tracking_valid,
                fmt_tuple(rb.pos),
                fmt_tuple(rb.rot),
                "None" if rb.marker_error is None else "%.6g" % rb.marker_error,
            )
        )

    client = NatNetClient(
        server_ip_address=args.server,
        local_ip_address=local_ip,
        use_multicast=args.multicast,
    )
    client.on_data_description_received_event.handlers.append(on_descriptions)
    client.on_data_frame_received_event.handlers.append(on_frame)

    try:
        client.connect(timeout=3.0)
        client.request_modeldef()
        start = time.time()
        while True:
            client.update_sync()
            if args.once and state["seen_target"]:
                break
            if args.duration > 0.0 and time.time() - start >= args.duration:
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        client.shutdown()


if __name__ == "__main__":
    main()
