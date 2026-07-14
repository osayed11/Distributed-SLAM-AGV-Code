#!/usr/bin/env python3
"""Validate the short S1 motion precheck before a recorder is started."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


def finite_number(data: dict, key: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} is missing or not numeric")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{key} is not finite")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate an S1 circle precheck summary")
    parser.add_argument("summary_json")
    parser.add_argument("--max-radius-error", type=float, default=0.15)
    parser.add_argument("--min-laps", type=float, default=0.02)
    parser.add_argument("--max-pose-age", type=float, default=0.20)
    parser.add_argument("--min-pose-samples", type=int, default=30)
    args = parser.parse_args()

    path = Path(args.summary_json).expanduser()
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"FAIL precheck summary: {path}: {exc}", file=sys.stderr)
        return 1

    failures = []
    if data.get("reached_duration") is not True:
        failures.append(f"motion did not reach 5s duration: {data.get('abort_reason') or 'unknown abort'}")

    try:
        initial_radius_error = abs(finite_number(data, "initial_radius_error_m"))
        final_radius_error = abs(finite_number(data, "final_radius_error_m"))
        max_radius_error = abs(finite_number(data, "max_radius_error_m"))
        laps = finite_number(data, "laps")
        max_pose_age = finite_number(data, "max_pose_age_sec")
        pose_samples = int(finite_number(data, "pose_samples"))
    except ValueError as exc:
        failures.append(str(exc))
    else:
        if initial_radius_error > args.max_radius_error:
            failures.append(
                f"initial radius error {initial_radius_error:.3f}m exceeds {args.max_radius_error:.3f}m"
            )
        if final_radius_error > args.max_radius_error:
            failures.append(
                f"final radius error {final_radius_error:.3f}m exceeds {args.max_radius_error:.3f}m"
            )
        if max_radius_error > args.max_radius_error:
            failures.append(
                f"max radius error {max_radius_error:.3f}m exceeds {args.max_radius_error:.3f}m"
            )
        if laps < args.min_laps:
            failures.append(
                f"direction-normalised progress {laps:.3f} laps is below {args.min_laps:.3f}; "
                "check tangent orientation and circle direction"
            )
        if max_pose_age > args.max_pose_age:
            failures.append(
                f"GT max age {max_pose_age:.3f}s exceeds {args.max_pose_age:.3f}s"
            )
        if pose_samples < args.min_pose_samples:
            failures.append(
                f"only {pose_samples} GT samples received; require at least {args.min_pose_samples}"
            )

    if failures:
        for failure in failures:
            print(f"FAIL S1 precheck: {failure}", file=sys.stderr)
        return 1

    print(
        "PASS S1 precheck: "
        f"laps={laps:.3f} radius_initial={initial_radius_error:.3f}m "
        f"radius_final={final_radius_error:.3f}m radius_max={max_radius_error:.3f}m "
        f"GT_samples={pose_samples} GT_max_age={max_pose_age:.3f}s"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
