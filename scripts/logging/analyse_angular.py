#!/usr/bin/env python3
"""Compare commanded vs actual angular velocity from a ROS2 bag.

Usage:
    python3 analyse_angular.py <bag_path>
"""
import glob, math, os, sqlite3, struct, sys


def _db_files(bag_path):
    return sorted(glob.glob(os.path.join(bag_path, "*.db3")))


def _endian(raw):
    return "<" if bytes(raw)[1] == 1 else ">"


def parse_twist_angular_z(raw):
    d = bytes(raw)
    e = _endian(d)
    # Twist CDR: 4B header, then linear.x/y/z, angular.x/y/z (all double)
    _, _, _, _, _, az = struct.unpack_from(e + "dddddd", d, 4)
    return az


def parse_odom(raw):
    d = bytes(raw)
    e = _endian(d)
    stamp_sec  = struct.unpack_from(e + "I", d, 4)[0]
    stamp_nsec = struct.unpack_from(e + "I", d, 8)[0]
    t = stamp_sec + stamp_nsec * 1e-9

    # Skip frame_id string (length-prefixed, 4-byte aligned)
    fl = struct.unpack_from(e + "I", d, 12)[0]
    off = 12 + 4 + fl + (4 - fl % 4) % 4

    # Skip child_frame_id string
    cl = struct.unpack_from(e + "I", d, off)[0]
    off = off + 4 + cl + (4 - cl % 4) % 4

    # pose.pose: position x,y,z then orientation x,y,z,w
    px, py, _, qx, qy, qz, qw = struct.unpack_from(e + "ddddddd", d, off)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return t, yaw, px, py


def main():
    bag_path = sys.argv[1]
    db_files = _db_files(bag_path)
    if not db_files:
        sys.exit(f"No .db3 files found in {bag_path}")

    cmd_az = []
    odom_rows = []

    for db in db_files:
        conn = sqlite3.connect(db)
        row = conn.execute("SELECT id FROM topics WHERE name='/cmd_vel'").fetchone()
        if row:
            for (raw,) in conn.execute(
                "SELECT data FROM messages WHERE topic_id=?", (row[0],)
            ):
                try:
                    cmd_az.append(parse_twist_angular_z(raw))
                except Exception:
                    pass

        row = conn.execute("SELECT id FROM topics WHERE name='/odom'").fetchone()
        if row:
            for (raw,) in conn.execute(
                "SELECT data FROM messages WHERE topic_id=?", (row[0],)
            ):
                try:
                    odom_rows.append(parse_odom(raw))
                except Exception:
                    pass
        conn.close()

    # --- cmd_vel stats ---
    print(f"\ncmd_vel angular.z  (n={len(cmd_az)})")
    if cmd_az:
        mean_az = sum(cmd_az) / len(cmd_az)
        near_pos = sum(1 for a in cmd_az if abs(a - 0.45) < 0.01)
        near_neg = sum(1 for a in cmd_az if abs(a + 0.45) < 0.01)
        print(f"  mean : {mean_az:+.4f} rad/s")
        print(f"  range: {min(cmd_az):+.4f} … {max(cmd_az):+.4f} rad/s")
        print(f"  at +0.45 clamp: {near_pos} msgs ({100*near_pos/len(cmd_az):.1f}%)")
        print(f"  at -0.45 clamp: {near_neg} msgs ({100*near_neg/len(cmd_az):.1f}%)")

    # --- odom yaw-rate stats ---
    odom_rows.sort(key=lambda r: r[0])
    yaw_rates = []
    for i in range(1, len(odom_rows)):
        dt = odom_rows[i][0] - odom_rows[i - 1][0]
        if dt < 0.005 or dt > 0.5:
            continue
        dyaw = math.atan2(
            math.sin(odom_rows[i][1] - odom_rows[i - 1][1]),
            math.cos(odom_rows[i][1] - odom_rows[i - 1][1]),
        )
        yaw_rates.append(dyaw / dt)

    print(f"\nodom yaw rate  (n={len(yaw_rates)})")
    if yaw_rates:
        mean_yr = sum(yaw_rates) / len(yaw_rates)
        peak    = max(abs(r) for r in yaw_rates)
        pct95   = sorted(abs(r) for r in yaw_rates)[int(0.95 * len(yaw_rates))]
        print(f"  mean : {mean_yr:+.4f} rad/s")
        print(f"  range: {min(yaw_rates):+.4f} … {max(yaw_rates):+.4f} rad/s")
        print(f"  95th percentile magnitude: {pct95:.4f} rad/s")
        print(f"  peak magnitude:            {peak:.4f} rad/s")

    # --- verdict ---
    if cmd_az and yaw_rates:
        cmd_peak = max(abs(a) for a in cmd_az)
        odom_p95 = sorted(abs(r) for r in yaw_rates)[int(0.95 * len(yaw_rates))]
        print(f"\nVerdict")
        print(f"  max commanded angular.z : {cmd_peak:.4f} rad/s")
        print(f"  odom yaw rate p95       : {odom_p95:.4f} rad/s")
        if cmd_peak > 0.01:
            scale = odom_p95 / cmd_peak
            print(f"  implied MCU scale factor: {scale:.3f}x")
            if abs(scale - 1.0) < 0.15:
                print("  → cmd_vel angular.z is in true rad/s (no scaling)")
            else:
                print("  → MCU applies a scaling factor to angular commands")


if __name__ == "__main__":
    main()
