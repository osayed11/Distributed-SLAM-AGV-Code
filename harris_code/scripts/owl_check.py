#!/usr/bin/env python3
"""
Minimal OWL sanity check — bypasses mocap_pub entirely.

Directly opens libowlsock.so + owl_bridge.so, polls for frames, and prints
whatever PhaseSpace is sending. Use this when mocap_pub shows `owl 0.0 Hz`
to confirm whether the OWL server is actually streaming.

    cd harris_code
    python3 scripts/owl_check.py                       # uses 192.168.1.25
    python3 scripts/owl_check.py --server 192.168.1.71

You'll see one of three things within a few seconds:
  - Per-frame "rigid id=X x=… y=… cond=…" lines  →  OWL streaming OK.
  - "no frames yet (Ns)"                          →  connection up but
                                                     session not started
                                                     in Master Client.
  - OSError / connect failure                     →  wrong IP or server
                                                     unreachable.
"""
import argparse
import ctypes
import os
import sys
import time


HERE       = os.path.dirname(os.path.abspath(__file__))
HARRIS_DIR = os.path.abspath(os.path.join(HERE, ".."))
REPO_ROOT  = os.path.abspath(os.path.join(HARRIS_DIR, ".."))

OWL_SOCK   = os.path.join(REPO_ROOT, "phasespace-mocap-ros",
                          "phasespace_bringup", "bin", "libowlsock.so")
OWL_BRIDGE = os.path.join(HARRIS_DIR, "owl_bridge.so")

MAX_RIGIDS = 64
STRIDE = 9  # [id, x_mm, y_mm, z_mm, qw, qx, qy, qz, cond]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--server", default="192.168.1.25",
                   help="PhaseSpace OWL server IP")
    p.add_argument("--timeout", type=float, default=30.0,
                   help="Give up after this many seconds of no frames")
    args = p.parse_args()

    if not os.path.exists(OWL_BRIDGE):
        sys.exit(f"owl_bridge.so missing — run `make` in {HARRIS_DIR}")

    ctypes.CDLL(OWL_SOCK, mode=ctypes.RTLD_GLOBAL)
    lib = ctypes.CDLL(OWL_BRIDGE)
    lib.owl_open.argtypes  = [ctypes.c_char_p]
    lib.owl_open.restype   = ctypes.c_int
    lib.owl_close.restype  = None
    lib.owl_poll.argtypes  = [ctypes.POINTER(ctypes.c_float), ctypes.c_int]
    lib.owl_poll.restype   = ctypes.c_int

    print(f"connecting to OWL at {args.server} …")
    rc = lib.owl_open(args.server.encode())
    if rc < 0:
        sys.exit(f"  owl_open failed (code {rc}) — wrong IP, server down, "
                 "or another OWL client has the stream")
    print("  connected. polling for frames (Ctrl+C to stop)…\n")

    buf = (ctypes.c_float * (MAX_RIGIDS * STRIDE))()
    seen_ids = set()
    frames = 0
    last_status = time.time()
    t_start = time.time()

    try:
        while True:
            n = lib.owl_poll(buf, MAX_RIGIDS)
            if n < 0:
                print("OWL error event")
            elif n > 0:
                frames += 1
                for i in range(n):
                    rid  = int(buf[i * STRIDE])
                    x_mm = buf[i * STRIDE + 1]
                    y_mm = buf[i * STRIDE + 2]
                    z_mm = buf[i * STRIDE + 3]
                    cond = buf[i * STRIDE + 8]
                    if rid not in seen_ids:
                        seen_ids.add(rid)
                        print(f"  [new] rigid id={rid} appeared")
                    print(f"  rigid id={rid:3d}  "
                          f"x={x_mm/1000:+.3f}m  y={y_mm/1000:+.3f}m  "
                          f"z={z_mm/1000:+.3f}m  cond={cond:.2f}")

            now = time.time()
            if now - last_status >= 2.0:
                hz = frames / (now - last_status)
                if frames == 0:
                    elapsed = now - t_start
                    print(f"  …no frames yet ({elapsed:.0f}s elapsed)")
                    if elapsed > args.timeout:
                        print(f"\nno frames in {args.timeout:.0f}s — "
                              "PhaseSpace session probably not started.")
                        break
                else:
                    print(f"  [{hz:.1f} Hz over last 2s] rigids seen so far: "
                          f"{sorted(seen_ids) if seen_ids else 'none'}")
                last_status = now
                frames = 0
    except KeyboardInterrupt:
        pass
    finally:
        lib.owl_close()
        print("\ndisconnected.")


if __name__ == "__main__":
    main()
