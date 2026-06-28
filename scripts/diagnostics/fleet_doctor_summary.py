#!/usr/bin/env python3
"""Summarize multiple robot_doctor summary.json files as a fleet table."""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

from validate_robot_doctor_report import validate_report


def expand_inputs(patterns: Iterable[str]) -> List[Path]:
    paths: List[Path] = []
    for pattern in patterns:
        matches = glob.glob(str(Path(pattern).expanduser()))
        if matches:
            paths.extend(Path(match) for match in matches)
        else:
            paths.append(Path(pattern).expanduser())
    return sorted(set(paths))


def failure_codes(report: Dict[str, object]) -> str:
    codes = []
    for check in report.get("checks", []):
        if check.get("status") == "FAIL":
            codes.append("{}:{}".format(check.get("code", "?"), check.get("check", "?")))
    return ", ".join(codes) if codes else "-"


def primary_blocker(report: Dict[str, object]) -> str:
    decision = report.get("decision", {})
    if not isinstance(decision, dict):
        return "-"
    blocker = decision.get("primary_blocker")
    if not isinstance(blocker, dict):
        return "-"
    return "{}:{}:{}".format(
        blocker.get("status", "?"),
        blocker.get("code", "?"),
        blocker.get("check", "?"),
    )


def short_sha(value: object) -> str:
    text = str(value or "")
    return text[:12] if text else "-"


def unique_values(rows: Sequence[Dict[str, object]], key: str) -> List[object]:
    return sorted({row.get(key, "") for row in rows})


def fleet_gate_errors(rows: Sequence[Dict[str, object]], args: argparse.Namespace) -> List[str]:
    errors: List[str] = []
    checks = [
        ("gate_id", args.require_same_gate),
        ("gate_version", args.require_same_gate),
        ("config_sha", args.require_same_config),
        ("repo_commit", args.require_same_commit),
    ]
    for key, enabled in checks:
        if not enabled:
            continue
        values = unique_values(rows, key)
        if len(values) > 1:
            errors.append(f"{key} differs across reports: {values}")
    if args.require_clean_repo:
        dirty = [row for row in rows if str(row.get("repo_dirty")) != "false"]
        if dirty:
            robots = ", ".join(str(row.get("robot_id", "?")) for row in dirty)
            errors.append(f"repo_dirty is not false for: {robots}")
    if args.require_configured_gate:
        missing = [
            row for row in rows
            if row.get("gate_id") in {"", "-"}
            or row.get("gate_version") in {"", "-"}
            or row.get("config_sha") in {"", "-"}
        ]
        if missing:
            robots = ", ".join(str(row.get("robot_id", "?")) for row in missing)
            errors.append(f"configured gate/config hash missing for: {robots}")
    return errors


def fleet_readiness_errors(rows: Sequence[Dict[str, object]], args: argparse.Namespace) -> List[str]:
    errors: List[str] = []
    bad_reports = [row for row in rows if not row.get("report_ok")]
    if bad_reports:
        robots = ", ".join(str(row.get("robot_id", "?")) for row in bad_reports)
        errors.append(f"invalid robot_doctor report structure for: {robots}")
    failed_reports = [row for row in rows if int(row.get("fail", 0) or 0) > 0]
    if failed_reports:
        robots = ", ".join(str(row.get("robot_id", "?")) for row in failed_reports)
        errors.append(f"robot_doctor FAIL checks present for: {robots}")
    if args.require_dataset_ready:
        not_ready = [row for row in rows if not row.get("dataset_ready")]
        if not_ready:
            robots = ", ".join(str(row.get("robot_id", "?")) for row in not_ready)
            errors.append(f"dataset_ready is not true for: {robots}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize robot_doctor summary.json files.")
    parser.add_argument("summary_json", nargs="+", help="summary.json path or glob")
    parser.add_argument("--json-out", help="write aggregate JSON")
    parser.add_argument(
        "--require-dataset-ready",
        action="store_true",
        help="exit nonzero unless every report has dataset_ready=true",
    )
    parser.add_argument(
        "--require-same-gate",
        action="store_true",
        help="exit nonzero unless gate_id and gate_version match across reports",
    )
    parser.add_argument(
        "--require-same-config",
        action="store_true",
        help="exit nonzero unless config_sha256 matches across reports",
    )
    parser.add_argument(
        "--require-same-commit",
        action="store_true",
        help="exit nonzero unless repo_state.commit matches across reports",
    )
    parser.add_argument(
        "--require-clean-repo",
        action="store_true",
        help="exit nonzero unless every report has repo_state.dirty=false",
    )
    parser.add_argument(
        "--require-configured-gate",
        action="store_true",
        help="exit nonzero unless every report has gate_id, gate_version, and config_sha256",
    )
    parser.add_argument(
        "--check-evidence",
        action="store_true",
        help="also require each report's evidence paths to exist locally or under the copied report root",
    )
    parser.add_argument(
        "--strict-fleet",
        action="store_true",
        help="equivalent to requiring dataset-ready, same gate/config/commit, and clean repo state",
    )
    args = parser.parse_args()
    if args.strict_fleet:
        args.require_dataset_ready = True
        args.require_same_gate = True
        args.require_same_config = True
        args.require_same_commit = True
        args.require_clean_repo = True
        args.require_configured_gate = True
        args.check_evidence = True

    rows = []
    for path in expand_inputs(args.summary_json):
        if path.is_dir():
            path = path / "summary.json"
        try:
            report = json.loads(path.read_text())
        except Exception as exc:
            print(f"WARN: cannot read {path}: {exc}", file=sys.stderr)
            continue
        report_ok, report_errors = validate_report(report, check_evidence=args.check_evidence, summary_json=path)
        counts = report.get("counts", {})
        loaded_config = report.get("loaded_config", {}) if isinstance(report.get("loaded_config"), dict) else {}
        repo_state = report.get("repo_state", {}) if isinstance(report.get("repo_state"), dict) else {}
        rows.append(
            {
                "robot_id": report.get("robot_id", path.parent.name),
                "verdict": report.get("verdict", "?"),
                "state": report.get("decision", {}).get("state", "?") if isinstance(report.get("decision"), dict) else "?",
                "report_ok": report_ok,
                "can_run_tests": report.get("can_run_tests", False),
                "dataset_ready": report.get("dataset_ready", False),
                "gate_id": loaded_config.get("gate_id", "-"),
                "gate_version": loaded_config.get("gate_version", "-"),
                "config_sha": short_sha(report.get("config_sha256", "")),
                "repo_commit": repo_state.get("commit", "-"),
                "repo_dirty": repo_state.get("dirty", "-"),
                "pass": counts.get("PASS", 0),
                "warn": counts.get("WARN", 0),
                "fail": counts.get("FAIL", 0),
                "primary": primary_blocker(report),
                "failures": failure_codes(report),
                "report_errors": report_errors,
                "summary": str(path),
            }
        )

    if not rows:
        print("No readable robot_doctor reports found.", file=sys.stderr)
        return 1

    headers = [
        "robot_id",
        "verdict",
        "state",
        "report_ok",
        "can_run_tests",
        "dataset_ready",
        "gate_id",
        "gate_version",
        "config_sha",
        "repo_commit",
        "repo_dirty",
        "pass",
        "warn",
        "fail",
        "primary",
        "failures",
    ]
    widths = {header: len(header) for header in headers}
    for row in rows:
        for header in headers:
            widths[header] = max(widths[header], len(str(row[header])))

    print(" | ".join(header.ljust(widths[header]) for header in headers))
    print("-+-".join("-" * widths[header] for header in headers))
    for row in rows:
        print(" | ".join(str(row[header]).ljust(widths[header]) for header in headers))

    if args.json_out:
        out = Path(args.json_out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(rows, indent=2, sort_keys=True) + "\n")

    readiness_errors = fleet_readiness_errors(rows, args)
    gate_errors = fleet_gate_errors(rows, args)
    if readiness_errors or gate_errors:
        print("", file=sys.stderr)
        print("Fleet audit failures:", file=sys.stderr)
        for error in readiness_errors + gate_errors:
            print(f"  - {error}", file=sys.stderr)
    return 1 if readiness_errors or gate_errors else 0


if __name__ == "__main__":
    sys.exit(main())
