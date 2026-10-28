#!/usr/bin/env python3
"""Plan a fleet circle run from where the robots actually stand.

Reads each robot's latest MoCap pose through the Zenoh router's REST endpoint
(the router that is already serving the robots), derives the shared centre and
per-robot radii, checks that every robot faces its tangent and that the swept
rings do not collide, and writes a `fleet_session.py` configuration. It never
moves anything; it refuses, naming the robot and the fix, when the placement
would make the precheck fail or two robots share a path.

Only the standard library is needed: PoseStamped is decoded from the CDR bytes
the router returns.
"""

import argparse
import base64
import itertools
import json
import math
import struct
import subprocess
import sys
import urllib.request
from pathlib import Path

ROBOT_HALF_WIDTH = 0.25        # m, myAGV footprint plus marker plate
MIN_RING_SEPARATION = 0.60      # m between concentric rings (two half widths + margin)
MIN_SAME_RING_CHORD = 1.00      # m between robots sharing a ring
MAX_HEADING_ERROR_DEG = 40.0    # precheck tolerates about this much before it fails on progress
MAX_RADIAL_ERROR = 0.12         # m, robot must sit within this of the ring it is assigned to


def decode_pose_stamped(raw):
    """XCDR1 little-endian geometry_msgs/PoseStamped -> (x, y, z, yaw_rad)."""
    body = 4  # encapsulation header
    def align(offset, n):
        return offset + ((-(offset - body)) % n)
    offset = body + 8  # header.stamp sec,nanosec
    offset = align(offset, 4)
    (length,) = struct.unpack_from('<I', raw, offset)
    offset += 4 + length
    offset = align(offset, 8)
    x, y, z, qx, qy, qz, qw = struct.unpack_from('<7d', raw, offset)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return x, y, z, yaw


def fetch_pose(rest, topic, timeout=3.0):
    """Latest sample of a router-served topic via its SSE stream; None if silent."""
    url = rest.rstrip('/') + topic
    try:
        proc = subprocess.run(['curl', '-sN', '--max-time', str(timeout), '-H', 'Accept: text/event-stream', url],
                              capture_output=True, text=True)
    except FileNotFoundError:
        request = urllib.request.Request(url, headers={'Accept': 'text/event-stream'})
        with urllib.request.urlopen(request, timeout=timeout) as response:
            text = response.read().decode('utf-8', 'replace')
    else:
        text = proc.stdout
    samples = [line[5:].strip() for line in text.splitlines() if line.startswith('data:')]
    if not samples:
        return None
    payload = json.loads(samples[-1])
    value = payload.get('value')
    raw = base64.b64decode(value) if isinstance(value, str) else bytes(value)
    return decode_pose_stamped(raw) + (len(samples) / timeout,)


def fit_center(points, rings):
    """Centre minimising radial residuals to the given ring radii (robots may be on any ring)."""
    best = None
    names = list(points)
    for assignment in itertools.product(rings, repeat=len(names)):
        if len(rings) > 1 and len(set(assignment)) < min(len(rings), len(names)):
            continue
        cx = sum(points[n][0] for n in names) / len(names)
        cy = sum(points[n][1] for n in names) / len(names)
        for _ in range(300):
            gx = gy = 0.0
            for n, r in zip(names, assignment):
                x, y = points[n][0], points[n][1]
                d = math.hypot(x - cx, y - cy) or 1e-9
                gx += (d - r) * (cx - x) / d
                gy += (d - r) * (cy - y) / d
            cx -= 0.3 * gx / len(names)
            cy -= 0.3 * gy / len(names)
        residual = math.sqrt(sum((math.hypot(points[n][0] - cx, points[n][1] - cy) - r) ** 2
                                 for n, r in zip(names, assignment)) / len(names))
        if best is None or residual < best[0]:
            best = (residual, cx, cy, dict(zip(names, assignment)))
    return best


def plan(base, poses, direction, rings=None, center=None, snap_to_rings=True):
    """Return (config, report_lines, errors). Pure; poses = {name: (x, y, z, yaw)}."""
    errors, lines = [], []
    robots = {r['name']: r for r in base['robots']}
    missing = [n for n in robots if n not in poses]
    for n in missing:
        errors.append(f'{n}: no valid MoCap pose (check Motive asset / markers / publisher list)')
    present = {n: poses[n] for n in robots if n in poses}
    if len(present) < 1:
        return None, lines, errors
    assigned = {}
    if center is not None:
        cx, cy = center
        lines.append(f'centre: explicit ({cx:+.3f}, {cy:+.3f})')
    elif rings:
        residual, cx, cy, assigned = fit_center(present, rings)
        lines.append(f'centre: fitted to rings {rings} at ({cx:+.3f}, {cy:+.3f}), rms radial residual {residual:.3f} m')
        if residual > MAX_RADIAL_ERROR:
            errors.append(f'robots are not within {MAX_RADIAL_ERROR:.2f} m of the requested rings (rms {residual:.3f} m)')
    else:
        names = list(present)
        a, b = max(itertools.combinations(names, 2), key=lambda p: math.dist(present[p[0]][:2], present[p[1]][:2])) \
            if len(names) > 1 else (names[0], names[0])
        cx = (present[a][0] + present[b][0]) / 2
        cy = (present[a][1] + present[b][1]) / 2
        lines.append(f'centre: midpoint of the outermost pair {a}/{b} at ({cx:+.3f}, {cy:+.3f}) (no --rings/--center given)')
    sign = -1.0 if direction == 'cw' else 1.0
    planned = []
    for name, (x, y, z, yaw, *_rest) in present.items():
        r = math.hypot(x - cx, y - cy)
        radius = assigned.get(name, r) if (snap_to_rings and assigned) else round(r, 2)
        if abs(r - radius) > MAX_RADIAL_ERROR:
            errors.append(f'{name}: {abs(r - radius):.2f} m off its {radius:.2f} m ring (move it radially)')
        bearing = math.atan2(y - cy, x - cx)
        tangent = bearing + sign * math.pi / 2
        offset = math.radians(robots[name]['forward_yaw_offset_deg'])
        heading_error = math.degrees((tangent - (yaw + offset) + math.pi) % (2 * math.pi) - math.pi)
        if abs(heading_error) > MAX_HEADING_ERROR_DEG:
            turn = 'left (ccw)' if heading_error > 0 else 'right (cw)'
            errors.append(f'{name}: facing {abs(heading_error):.0f} deg off the {direction} tangent; turn it {turn}')
        planned.append(dict(name=name, radius=radius, measured_radius=r, bearing_deg=math.degrees(bearing),
                            heading_error_deg=heading_error, x=x, y=y))
        lines.append(f'{name:8s} r={r:.2f} m -> ring {radius:.2f}  bearing {math.degrees(bearing):+6.1f} deg  '
                     f'heading error vs {direction} tangent {heading_error:+5.1f} deg')
    # Clearance: rings far enough apart, robots on one ring far enough along it.
    for a, b in itertools.combinations(planned, 2):
        dr = abs(a['radius'] - b['radius'])
        if dr < 1e-6:
            chord = 2 * a['radius'] * abs(math.sin(math.radians(a['bearing_deg'] - b['bearing_deg']) / 2))
            if chord < MIN_SAME_RING_CHORD:
                errors.append(f"{a['name']} and {b['name']} share the {a['radius']:.2f} m ring only "
                              f"{chord:.2f} m apart (need {MIN_SAME_RING_CHORD:.1f} m)")
        elif dr < MIN_RING_SEPARATION:
            errors.append(f"{a['name']} ({a['radius']:.2f} m) and {b['name']} ({b['radius']:.2f} m) rings are "
                          f"{dr:.2f} m apart (need {MIN_RING_SEPARATION:.2f} m)")
    config = dict(base)
    config['robots'] = [dict(robots[p['name']], radius=round(p['radius'], 3), center_x=round(cx, 3),
                             center_y=round(cy, 3), direction=direction) for p in planned]
    config['plan'] = dict(center=[round(cx, 3), round(cy, 3)], direction=direction, rings=rings,
                          measured={p['name']: dict(x=round(p['x'], 3), y=round(p['y'], 3),
                                                    radius=round(p['measured_radius'], 3),
                                                    heading_error_deg=round(p['heading_error_deg'], 1))
                                    for p in planned})
    return config, lines, errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, required=True,
                        help='base fleet config: robots (name, host, pose_topic, forward_yaw_offset_deg) and run defaults')
    parser.add_argument('--output', type=Path, required=True, help='planned config to write')
    parser.add_argument('--direction', choices=['cw', 'ccw'], default=None,
                        help='circle direction for every robot (default: from the base config, else cw)')
    parser.add_argument('--rings', default='', help='comma-separated ring radii to snap robots onto, e.g. 1.0,1.7')
    parser.add_argument('--center', default='', help='explicit centre "x,y" in the MoCap frame')
    parser.add_argument('--router-rest', default='http://127.0.0.1:18000',
                        help='REST endpoint of the Zenoh MoCap router serving the robots')
    parser.add_argument('--sample-seconds', type=float, default=2.0)
    parser.add_argument('--force', action='store_true', help='write the config even with placement errors')
    args = parser.parse_args(argv)
    base = json.loads(args.config.read_text())
    direction = args.direction or next((r.get('direction') for r in base['robots'] if r.get('direction')), 'cw')
    rings = [float(v) for v in args.rings.split(',') if v.strip()] or None
    center = tuple(float(v) for v in args.center.split(',')) if args.center else None
    poses = {}
    for robot in base['robots']:
        sample = fetch_pose(args.router_rest, robot['pose_topic'], args.sample_seconds)
        if sample is not None:
            x, y, z, yaw, hz = sample
            poses[robot['name']] = (x, y, z, yaw)
            print(f"{robot['name']:8s} pose ({x:+.2f}, {y:+.2f}) yaw {math.degrees(yaw):+6.1f} deg  GT {hz:.0f} Hz")
        else:
            print(f"{robot['name']:8s} no GT at {args.router_rest}{robot['pose_topic']}")
    config, lines, errors = plan(base, poses, direction, rings, center)
    for line in lines:
        print(line)
    for error in errors:
        print('ERROR:', error)
    if config is None or (errors and not args.force):
        print('No configuration written; fix the placement (or pass --force to write anyway).')
        return 1
    args.output.write_text(json.dumps(config, indent=2) + '\n')
    print('wrote', args.output, '-', 'with errors (forced)' if errors else 'placement OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
