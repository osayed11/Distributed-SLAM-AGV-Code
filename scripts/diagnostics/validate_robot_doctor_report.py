#!/usr/bin/env python3
"""Validate the structure and decision semantics of robot_doctor summary.json."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


VALID_CODES = {"1.1", "1.2", "1.3", "2.1", "2.2", "2.3", "3.1", "3.2", "3.3"}
VALID_STATUSES = {"PASS", "WARN", "FAIL", "INFO"}
SUPPORTED_SCHEMA_VERSION = "1.0"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _count_statuses(checks: Iterable[Dict[str, object]]) -> Dict[str, int]:
    counts = {status: 0 for status in VALID_STATUSES}
    for check in checks:
        counts[str(check.get("status", ""))] = counts.get(str(check.get("status", "")), 0) + 1
    return counts


def resolve_evidence_path(
    evidence_path: str,
    *,
    summary_json: Optional[Path] = None,
    report_output_dir: str = "",
) -> Optional[Path]:
    """Resolve evidence paths after reports have been copied off a robot."""

    if not evidence_path:
        return None
    direct = Path(evidence_path).expanduser()
    if direct.exists():
        return direct
    if summary_json is None:
        return None

    local_root = summary_json.expanduser().resolve().parent
    remote_root = str(report_output_dir or "").rstrip("/")
    if remote_root and evidence_path == remote_root:
        return local_root if local_root.exists() else None
    if remote_root and evidence_path.startswith(remote_root + "/"):
        candidate = local_root / evidence_path[len(remote_root) + 1 :]
        return candidate if candidate.exists() else None
    if not direct.is_absolute():
        candidate = local_root / evidence_path
        return candidate if candidate.exists() else None
    return None


def validate_report(
    report: Dict[str, object],
    *,
    check_evidence: bool = False,
    summary_json: Optional[Path] = None,
) -> Tuple[bool, List[str]]:
    errors: List[str] = []

    required_top = [
        "schema_version",
        "tool",
        "tool_version",
        "robot_id",
        "created_at",
        "profile",
        "verdict",
        "can_run_tests",
        "dataset_ready",
        "decision",
        "config_path",
        "config_sha256",
        "loaded_config",
        "effective_gate",
        "repo_state",
        "output_dir",
        "failure_tree",
        "counts",
        "counts_by_code",
        "checks",
        "commands",
    ]
    for key in required_top:
        if key not in report:
            errors.append(f"missing top-level key: {key}")

    if report.get("schema_version") != SUPPORTED_SCHEMA_VERSION:
        errors.append(
            f"unsupported schema_version: {report.get('schema_version')!r}; expected {SUPPORTED_SCHEMA_VERSION!r}"
        )
    if report.get("tool") != "robot_doctor":
        errors.append(f"unexpected tool: {report.get('tool')!r}")
    if not isinstance(report.get("tool_version"), str) or not report.get("tool_version"):
        errors.append("tool_version must be a non-empty string")

    config_path = report.get("config_path", "")
    config_sha = report.get("config_sha256", "")
    if not isinstance(config_path, str):
        errors.append("config_path must be a string")
        config_path = ""
    if not isinstance(config_sha, str):
        errors.append("config_sha256 must be a string")
        config_sha = ""
    if config_path and not config_sha:
        errors.append("config_sha256 is required when config_path is set")
    if config_sha and not re.fullmatch(r"[0-9a-f]{64}", config_sha):
        errors.append("config_sha256 must be a lowercase 64-character SHA256 hex digest")
    if config_path and config_sha:
        path = Path(str(config_path)).expanduser()
        if path.exists():
            actual_sha = sha256_file(path)
            if actual_sha != config_sha:
                errors.append(f"config_sha256 mismatch: reported={config_sha} actual={actual_sha}")

    loaded_config = report.get("loaded_config", {})
    if not isinstance(loaded_config, dict):
        errors.append("loaded_config must be an object")
        loaded_config = {}
    for key in ["gate_id", "gate_version"]:
        if key in loaded_config and (not isinstance(loaded_config.get(key), str) or not loaded_config.get(key)):
            errors.append(f"loaded_config.{key} must be a non-empty string when present")

    effective_gate = report.get("effective_gate", {})
    if not isinstance(effective_gate, dict):
        errors.append("effective_gate must be an object")
        effective_gate = {}
    if not effective_gate:
        errors.append("effective_gate must not be empty")
    if "profile" in effective_gate and effective_gate.get("profile") != report.get("profile"):
        errors.append(
            f"effective_gate.profile mismatch: reported={effective_gate.get('profile')!r} top-level={report.get('profile')!r}"
        )
    if "required_topic" in effective_gate and not isinstance(effective_gate.get("required_topic"), list):
        errors.append("effective_gate.required_topic must be a list when present")

    repo_state = report.get("repo_state", {})
    if not isinstance(repo_state, dict):
        errors.append("repo_state must be an object")
        repo_state = {}
    for key in ["branch", "commit", "dirty", "status_short"]:
        if key not in repo_state:
            errors.append(f"repo_state missing key: {key}")
        elif not isinstance(repo_state.get(key), str):
            errors.append(f"repo_state.{key} must be a string")

    report_output_dir = report.get("output_dir", "")
    if not isinstance(report_output_dir, str):
        errors.append("output_dir must be a string")
        report_output_dir = ""

    failure_tree = report.get("failure_tree", {})
    if not isinstance(failure_tree, dict):
        errors.append("failure_tree must be an object")
        failure_tree = {}
    missing_codes = sorted(VALID_CODES - set(failure_tree))
    if missing_codes:
        errors.append("failure_tree missing codes: " + ", ".join(missing_codes))

    checks = report.get("checks", [])
    if not isinstance(checks, list):
        errors.append("checks must be a list")
        checks = []

    for index, check in enumerate(checks):
        if not isinstance(check, dict):
            errors.append(f"checks[{index}] must be an object")
            continue
        code = check.get("code")
        status = check.get("status")
        if code not in VALID_CODES:
            errors.append(f"checks[{index}] has unknown code: {code}")
        if status not in VALID_STATUSES:
            errors.append(f"checks[{index}] has invalid status: {status}")
        for key in ["check", "summary"]:
            if not isinstance(check.get(key), str) or not check.get(key):
                errors.append(f"checks[{index}] missing non-empty string: {key}")
        if status in {"FAIL", "WARN"}:
            next_action = check.get("next_action", "")
            if not isinstance(next_action, str) or not next_action.strip():
                errors.append(f"checks[{index}] {status} check missing non-empty next_action")
        evidence = check.get("evidence", [])
        if evidence is not None and not isinstance(evidence, list):
            errors.append(f"checks[{index}].evidence must be a list")
        elif check_evidence:
            for evidence_path in evidence:
                if (
                    isinstance(evidence_path, str)
                    and evidence_path
                    and resolve_evidence_path(
                        evidence_path,
                        summary_json=summary_json,
                        report_output_dir=report_output_dir,
                    )
                    is None
                ):
                    errors.append(
                        "evidence path does not exist locally or under copied report root: "
                        + evidence_path
                    )

    reported_counts = report.get("counts", {})
    if not isinstance(reported_counts, dict):
        errors.append("counts must be an object")
        reported_counts = {}
    actual_counts = _count_statuses(checks)
    for status in VALID_STATUSES:
        if int(reported_counts.get(status, 0) or 0) != actual_counts.get(status, 0):
            errors.append(
                f"count mismatch for {status}: reported={reported_counts.get(status, 0)} actual={actual_counts.get(status, 0)}"
            )

    counts_by_code = report.get("counts_by_code", {})
    if isinstance(counts_by_code, dict):
        expected_by_code = {
            code: {status: 0 for status in VALID_STATUSES}
            for code in VALID_CODES
        }
        for check in checks:
            code = check.get("code")
            status = check.get("status")
            if code in expected_by_code and status in VALID_STATUSES:
                expected_by_code[code][status] += 1
        for code, expected_counts in expected_by_code.items():
            reported = counts_by_code.get(code, {})
            if not isinstance(reported, dict):
                errors.append(f"counts_by_code[{code}] must be an object")
                continue
            for status, expected in expected_counts.items():
                if int(reported.get(status, 0) or 0) != expected:
                    errors.append(
                        f"counts_by_code mismatch for {code}/{status}: reported={reported.get(status, 0)} actual={expected}"
                    )
    else:
        errors.append("counts_by_code must be an object")

    decision = report.get("decision", {})
    if not isinstance(decision, dict):
        errors.append("decision must be an object")
        decision = {}

    failures = [check for check in checks if check.get("status") == "FAIL"]
    warnings = [check for check in checks if check.get("status") == "WARN"]
    if failures:
        expected_state = "blocked"
        expected_verdict = "FAIL"
        expected_can_run_tests = False
        expected_dataset_ready = False
        expected_primary_status = "FAIL"
    elif warnings:
        expected_state = "review"
        expected_verdict = "WARN"
        expected_can_run_tests = True
        expected_dataset_ready = False
        expected_primary_status = "WARN"
    else:
        expected_state = "ready"
        expected_verdict = "PASS"
        expected_can_run_tests = True
        expected_dataset_ready = report.get("profile") == "dataset"
        expected_primary_status = None

    expected_values = {
        "state": expected_state,
        "verdict": expected_verdict,
        "can_run_tests": expected_can_run_tests,
        "dataset_ready": expected_dataset_ready,
    }
    for key, expected in expected_values.items():
        if decision.get(key) != expected:
            errors.append(f"decision.{key} mismatch: reported={decision.get(key)!r} expected={expected!r}")
        if key in report and report.get(key) != expected:
            errors.append(f"top-level {key} mismatch: reported={report.get(key)!r} expected={expected!r}")

    primary = decision.get("primary_blocker")
    if expected_primary_status is None:
        if primary is not None:
            errors.append("decision.primary_blocker should be null for ready reports")
    else:
        if not isinstance(primary, dict):
            errors.append("decision.primary_blocker must be an object for blocked/review reports")
        elif primary.get("status") != expected_primary_status:
            errors.append(
                f"decision.primary_blocker.status mismatch: reported={primary.get('status')} expected={expected_primary_status}"
            )

    blockers = decision.get("blockers", [])
    if not isinstance(blockers, list):
        errors.append("decision.blockers must be a list")
    else:
        expected_blocker_count = len(failures) if failures else len(warnings)
        if len(blockers) != expected_blocker_count:
            errors.append(
                f"decision.blockers length mismatch: reported={len(blockers)} expected={expected_blocker_count}"
            )

    return not errors, errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate robot_doctor summary.json consistency.")
    parser.add_argument("summary_json", nargs="+")
    parser.add_argument("--check-evidence", action="store_true", help="also require evidence paths to exist on this machine")
    args = parser.parse_args()

    ok_all = True
    for path_arg in args.summary_json:
        path = Path(path_arg).expanduser()
        if path.is_dir():
            path = path / "summary.json"
        try:
            report = json.loads(path.read_text())
        except Exception as exc:
            print(f"FAIL {path}: cannot read JSON: {exc}")
            ok_all = False
            continue
        ok, errors = validate_report(report, check_evidence=args.check_evidence, summary_json=path)
        if ok:
            print(f"PASS {path}")
        else:
            ok_all = False
            print(f"FAIL {path}")
            for error in errors:
                print(f"  - {error}")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
