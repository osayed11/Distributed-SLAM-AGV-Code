#!/usr/bin/env python3
"""Create a valid robot_doctor failure report when remote execution aborts."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/diagnostics"))

from robot_doctor import (  # noqa: E402
    FAIL,
    FAILURE_TREE,
    REPORT_SCHEMA_VERSION,
    ROBOT_DOCTOR_VERSION,
    CheckResult,
    git_info,
    summarize_decision,
)


def build_report(args: argparse.Namespace) -> dict:
    out_dir = Path(args.output_dir).expanduser()
    logs = sorted(str(path) for path in (out_dir / "logs").glob("*.txt")) if (out_dir / "logs").exists() else []
    evidence = logs[:20]
    remote_rc = str(args.remote_rc)
    if remote_rc == "255":
        code = "3.3"
        check_name = "remote_ssh_interrupted"
        next_action = "check Wi-Fi signal, robot power, and SSH stability; inspect partial logs before rerunning robot_doctor"
    else:
        code = "3.2"
        check_name = "robot_doctor_execution"
        next_action = "inspect copied logs; rerun after fixing the first hardware/transport failure or diagnostic crash"
    check = CheckResult(
        code=code,
        status=FAIL,
        check=check_name,
        summary=(
            "remote robot_doctor did not produce summary.json "
            f"(remote_rc={remote_rc}, remote_report={args.remote_report})"
        ),
        evidence=evidence,
        next_action=next_action,
    )
    decision = summarize_decision([check], args.profile)
    counts = {"PASS": 0, "WARN": 0, "FAIL": 1, "INFO": 0}
    counts_by_code = {code: {"PASS": 0, "WARN": 0, "FAIL": 0, "INFO": 0} for code in FAILURE_TREE}
    counts_by_code[check.code]["FAIL"] = 1
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "tool": "robot_doctor",
        "tool_version": ROBOT_DOCTOR_VERSION,
        "robot_id": args.robot_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "profile": args.profile,
        "config_path": "",
        "config_sha256": "",
        "loaded_config": {},
        "effective_gate": {"profile": args.profile, "synthesized_remote_failure": True},
        "repo_state": git_info(ROOT),
        "can_run_tests": decision["can_run_tests"],
        "dataset_ready": decision["dataset_ready"],
        "verdict": decision["verdict"],
        "decision": decision,
        "output_dir": str(out_dir),
        "failure_tree": FAILURE_TREE,
        "counts": counts,
        "counts_by_code": counts_by_code,
        "checks": [asdict(check)],
        "commands": [],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--remote-report", required=True)
    parser.add_argument("--remote-rc", required=True)
    parser.add_argument("--profile", default="dataset")
    args = parser.parse_args()

    out_dir = Path(args.output_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    report = build_report(args)
    (out_dir / "summary.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    (out_dir / "summary.md").write_text(
        "\n".join(
            [
                "# Synthesized Robot Doctor Failure",
                "",
                f"- robot_id: `{args.robot_id}`",
                f"- remote_report: `{args.remote_report}`",
                f"- remote_rc: `{args.remote_rc}`",
                "",
                "The remote diagnostic did not complete far enough to write its own summary.",
                "Inspect copied logs, then rerun robot_doctor.",
                "",
            ]
        )
    )
    print(out_dir / "summary.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
