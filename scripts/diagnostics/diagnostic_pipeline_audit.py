#!/usr/bin/env python3
"""Acceptance audit for the AGV diagnostic pipeline itself.

This is a lightweight static guardrail. It does not prove a robot is healthy;
it proves the repository still contains the pieces needed to make robot
failures diagnosable: the doctor, report validator, remote wrapper, fleet
summary, bag validators, docs, config, and regression coverage.
"""

from __future__ import annotations

import argparse
import json
import stat
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/diagnostics"))

from robot_doctor import FAILURE_TREE  # noqa: E402


PASS = "PASS"
FAIL = "FAIL"


@dataclass
class AuditResult:
    status: str
    check: str
    summary: str


def read(path: Path) -> str:
    return path.read_text(errors="replace")


def executable(path: Path) -> bool:
    try:
        mode = path.stat().st_mode
    except OSError:
        return False
    return bool(mode & (stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH))


class Audit:
    def __init__(self) -> None:
        self.results: list[AuditResult] = []
        self.text_cache: dict[Path, str] = {}

    def add(self, status: str, check: str, summary: str) -> None:
        self.results.append(AuditResult(status, check, summary))

    def path(self, relative: str) -> Path:
        return ROOT / relative

    def text(self, relative: str) -> str:
        path = self.path(relative)
        if path not in self.text_cache:
            self.text_cache[path] = read(path)
        return self.text_cache[path]

    def require_files(self) -> None:
        files = [
            "scripts/diagnostics/robot_doctor.py",
            "scripts/diagnostics/robot_doctor.sh",
            "scripts/diagnostics/dataset_ready_gate.sh",
            "scripts/diagnostics/validate_robot_doctor_report.py",
            "scripts/diagnostics/fleet_doctor_summary.py",
            "scripts/diagnostics/run_robot_doctor_remote.sh",
            "scripts/diagnostics/run_fleet_doctor_remote.sh",
            "scripts/diagnostics/apply_robot_doctor_fix.sh",
            "scripts/diagnostics/dataset_run_audit.py",
            "scripts/diagnostics/synthesize_robot_doctor_failure.py",
            "scripts/diagnostics/diagnostic_pipeline_audit.py",
            "scripts/setup_robot_ros2.sh",
            "scripts/logging/validate_ros2_bag.py",
            "configs/robot_doctor_dataset_gate.json",
            "configs/robot_doctor_sensor_logging_gate.json",
            "configs/sqlite_resilient.yaml",
            "docs/ROBOT_DIAGNOSTIC_PIPELINE.md",
            "docs/ROBOT_DEBUG_PIPELINE_COVERAGE_AUDIT.md",
            "robot_failure_modes_v3.png",
        ]
        missing = [path for path in files if not self.path(path).exists()]
        if missing:
            self.add(FAIL, "required_files", "missing required files: " + ", ".join(missing))
        else:
            self.add(PASS, "required_files", f"{len(files)} required files present")

        executable_files = [
            "scripts/diagnostics/robot_doctor.py",
            "scripts/diagnostics/robot_doctor.sh",
            "scripts/diagnostics/dataset_ready_gate.sh",
            "scripts/diagnostics/run_robot_doctor_remote.sh",
            "scripts/diagnostics/run_fleet_doctor_remote.sh",
            "scripts/diagnostics/apply_robot_doctor_fix.sh",
            "scripts/diagnostics/dataset_run_audit.py",
            "scripts/diagnostics/diagnostic_pipeline_audit.py",
            "scripts/setup_robot_ros2.sh",
            "scripts/logging/validate_ros2_bag.py",
        ]
        not_exec = [path for path in executable_files if not executable(self.path(path))]
        if not_exec:
            self.add(FAIL, "executable_bits", "not executable: " + ", ".join(not_exec))
        else:
            self.add(PASS, "executable_bits", f"{len(executable_files)} executable entrypoints are executable")

    def require_failure_tree(self) -> None:
        expected_codes = {"1.1", "1.2", "1.3", "2.1", "2.2", "2.3", "3.1", "3.2", "3.3"}
        actual_codes = set(FAILURE_TREE)
        if actual_codes != expected_codes:
            self.add(
                FAIL,
                "failure_tree_codes",
                f"expected {sorted(expected_codes)}, found {sorted(actual_codes)}",
            )
        else:
            self.add(PASS, "failure_tree_codes", "failure tree has the expected 3x3 code structure")

        for code, meta in FAILURE_TREE.items():
            if not meta.get("name") or not meta.get("parent") or not meta.get("examples"):
                self.add(FAIL, "failure_tree_metadata", f"{code} missing name/parent/examples metadata")
                return
        self.add(PASS, "failure_tree_metadata", "every failure-tree code has metadata")

    def require_source_patterns(
        self,
        check: str,
        patterns: Sequence[str],
        files: Sequence[str],
    ) -> None:
        haystack = "\n".join(self.text(file) for file in files if self.path(file).exists())
        missing = [pattern for pattern in patterns if pattern not in haystack]
        if missing:
            self.add(FAIL, check, "missing patterns: " + ", ".join(missing))
        else:
            self.add(PASS, check, f"{len(patterns)} required patterns present")

    def require_branch_coverage(self) -> None:
        files = [
            "scripts/diagnostics/robot_doctor.py",
            "scripts/diagnostics/synthesize_robot_doctor_failure.py",
        ]
        required_by_branch = {
            "1.1": ["d455_enumeration", "d455_firmware", "d455_imu_hid", "realsense_motion_stream_gate", "serial_devices"],
            "1.2": [
                "power_throttle",
                "d455_usb_autosuspend_delay",
                "kernel_usb_autosuspend_elpg",
                "kernel_usb_overcurrent",
                "kernel_usb_disconnect",
                "d455_physical_swap_evidence",
            ],
            "1.3": ["mechanical_operator_check", "odom_mocap_sanity"],
            "2.1": [
                "d455_usb_speed",
                "d455_uvc_binding",
                "device_permissions",
                "kernel_uvc_errors",
                "kernel_xhci_errors",
                "realsense_stream_transport",
                "viewer_passes_ros2_fails",
            ],
            "2.2": [
                "realsense_tools",
                "librealsense_version",
                "realsense_ros_driver_version",
                "d455_infra_fps_cap",
                "dataset_bringup_context",
                "native_ros2_stack",
            ],
            "2.3": ["topic_present", "topic_rate", "ros_graph_skipped"],
            "3.1": ["disk_free", "stale_recorder"],
            "3.2": [
                "bag_validation",
                "bag_validation_missing",
                "realsense_stream_test",
                "robot_doctor_execution",
                "diagnostic_lock",
                "require_resilient_storage",
            ],
            "3.3": [
                "clock_sync",
                "chrony_offset",
                "wifi_management",
                "mocap_topic",
                "anchors_operator_check",
                "remote_ssh_interrupted",
                "dds_discovery",
                "dds_discovery_server",
            ],
        }
        for code, patterns in required_by_branch.items():
            self.require_source_patterns(f"branch_{code}_coverage", patterns, files)

    def require_dataset_gate(self) -> None:
        path = self.path("configs/robot_doctor_dataset_gate.json")
        try:
            config = json.loads(path.read_text())
        except Exception as exc:
            self.add(FAIL, "dataset_gate_json", f"cannot parse dataset gate config: {exc}")
            return
        required_values = {
            "profile": "dataset",
            "require_gt": True,
            "require_imu": True,
            "expect_camera": True,
            "strict_versions": True,
            "expected_d455_firmware": "5.17.0.10",
            "expected_librealsense": "2.58.1",
            "expected_realsense_ros_driver": "4.57.7",
            "expected_realsense_ros_librealsense": "2.57.7",
            "stream_test_motion": True,
            "d455_motion_test_seconds": 10,
            "max_clock_offset_ms": 1.0,
            "expect_native_ros2": True,
            "require_odom_mocap_sanity": True,
            "odom_mocap_max_error_ratio": 0.1,
            "require_resilient_storage": True,
        }
        mismatches = [
            f"{key}={config.get(key)!r}"
            for key, expected in required_values.items()
            if config.get(key) != expected
        ]
        if mismatches:
            self.add(FAIL, "dataset_gate_values", "unexpected gate values: " + ", ".join(mismatches))
        else:
            self.add(PASS, "dataset_gate_values", "dataset gate pins expected required values")

        required_topics = set(config.get("required_topic", []))
        expected_topics = {
            "/scan",
            "/odom",
            "/tf",
            "/camera/color/image_raw",
            "/camera/aligned_depth_to_color/image_raw",
        }
        missing = sorted(expected_topics - required_topics)
        if missing:
            self.add(FAIL, "dataset_gate_topics", "missing required topics: " + ", ".join(missing))
        else:
            self.add(PASS, "dataset_gate_topics", "dataset gate includes required core topics")

        if int(config.get("stream_test_seconds", 0) or 0) < 60:
            self.add(FAIL, "dataset_gate_stream_duration", "stream_test_seconds below 60")
        else:
            self.add(PASS, "dataset_gate_stream_duration", "stream gate duration is at least 60 seconds")

    def require_sensor_logging_gate(self) -> None:
        path = self.path("configs/robot_doctor_sensor_logging_gate.json")
        try:
            config = json.loads(path.read_text())
        except Exception as exc:
            self.add(FAIL, "sensor_logging_gate_json", f"cannot parse sensor logging gate config: {exc}")
            return

        required_values = {
            "profile": "preflight",
            "require_gt": False,
            "require_imu": True,
            "expect_camera": True,
            "strict_versions": True,
            "expected_d455_firmware": "5.17.0.10",
            "expected_librealsense": "2.58.1",
            "expected_realsense_ros_driver": "4.57.7",
            "expected_realsense_ros_librealsense": "2.57.7",
            "stream_test_motion": True,
            "d455_motion_test_seconds": 10,
            "expect_native_ros2": True,
            "require_odom_mocap_sanity": False,
        }
        mismatches = [
            f"{key}={config.get(key)!r}"
            for key, expected in required_values.items()
            if config.get(key) != expected
        ]
        if mismatches:
            self.add(FAIL, "sensor_logging_gate_values", "unexpected gate values: " + ", ".join(mismatches))
        else:
            self.add(PASS, "sensor_logging_gate_values", "sensor logging gate pins expected no-GT values")

        required_topics = set(config.get("required_topic", []))
        expected_topics = {
            "/scan",
            "/odom",
            "/tf",
            "/camera/color/image_raw",
            "/camera/aligned_depth_to_color/image_raw",
        }
        missing = sorted(expected_topics - required_topics)
        if missing:
            self.add(FAIL, "sensor_logging_gate_topics", "missing required topics: " + ", ".join(missing))
        else:
            self.add(PASS, "sensor_logging_gate_topics", "sensor logging gate includes required core topics")

    def require_report_and_remote_guards(self) -> None:
        self.require_source_patterns(
            "report_validator_guards",
            [
                "resolve_evidence_path",
                "schema_version",
                "counts_by_code",
                "decision.primary_blocker",
                "next_action",
                "config_sha256",
                "repo_state",
            ],
            ["scripts/diagnostics/validate_robot_doctor_report.py"],
        )
        self.require_source_patterns(
            "remote_wrapper_guards",
            [
                "validate_robot_doctor_report.py\" --check-evidence",
                "synthesize_robot_doctor_failure.py",
                "remote_wrapper_failure.txt",
                "RUN_REMOTE_SELFTEST",
                "diagnostic_pipeline_audit.py",
                "dataset_run_audit.py",
                "setup_robot_ros2.sh",
                "dataset_ready_gate.sh",
            ],
            ["scripts/diagnostics/run_robot_doctor_remote.sh"],
        )
        self.require_source_patterns(
            "dataset_ready_gate_guards",
            [
                "mode: read-only",
                "validate_robot_doctor_report.py",
                "READY_TO_RECORD:",
                "POST_RUN_DATASET_READY:",
                "FAILED_STAGE:",
                "NEXT_ACTION:",
                "--expected-d455-serial",
                "dataset_ready",
                "no fixes",
            ],
            ["scripts/diagnostics/dataset_ready_gate.sh"],
        )
        self.require_source_patterns(
            "doctor_process_guards",
            [
                "preexec_fn=os.setsid",
                "os.killpg",
                "ROBOT_DOCTOR_LOCK_DIR",
                "diagnostic_lock",
                "--lock-timeout-seconds",
                "format_operator_decision",
                "FAILED_STAGE:",
            ],
            ["scripts/diagnostics/robot_doctor.py"],
        )
        self.require_source_patterns(
            "remediation_script_guards",
            [
                "d455-autosuspend",
                "d455-uvc-bind",
                "d455-usb-reset",
                "d455-authorize-cycle",
                "authorized",
                "autosuspend_delay_ms",
            ],
            ["scripts/diagnostics/apply_robot_doctor_fix.sh"],
        )
        self.require_source_patterns(
            "fleet_summary_guards",
            [
                "--check-evidence",
                "require_dataset_ready",
                "require_same_gate",
                "require_same_config",
                "require_same_commit",
                "require_clean_repo",
                "require_configured_gate",
            ],
            ["scripts/diagnostics/fleet_doctor_summary.py"],
        )
        self.require_source_patterns(
            "dataset_run_audit_guards",
            [
                "robot_doctor summary.json validation",
                "report_dataset_ready",
                "bag_validation",
                "manifest_complete",
                "robot_artifact_match",
                "manifest_bag_supplied",
                "bag_manifest_match",
                "fleet_same_gate",
                "validate_ros2_bag.py",
                "--mocap-topic",
                "MOCAP_TOPIC",
                "--cmd-topic",
            ],
            ["scripts/diagnostics/dataset_run_audit.py"],
        )

    def require_bag_validator_guards(self) -> None:
        self.require_source_patterns(
            "ros2_bag_validator_guards",
            [
                "required_specs",
                "ground_truth_topics",
                "imu_topics",
                "major_gaps",
                "coverage",
                "REQUIRED_TOPICS",
                "--require-gt",
                "--require-imu",
                "find_mcap_files",
                "metadata_yaml",
                "timestamp_monotonic",
                "storage_resilience",
                "sqlite_resilient",
                "journal_mode=wal",
                "--require-resilient-storage",
            ],
            ["scripts/logging/validate_ros2_bag.py"],
        )
    def require_docs_and_tests(self) -> None:
        self.require_source_patterns(
            "diagnostic_docs",
            [
                "d455_physical_swap_evidence",
                "remote_ssh_interrupted",
                "--check-evidence",
                "--strict-fleet",
                "dataset_ready=true",
                "validate_ros2_bag.py",
                "operator_d455_swap_checklist.md",
                "diagnostic_lock",
                "process group",
                "dataset_run_audit.py",
                "viewer_passes_ros2_fails",
                "d455_infra_fps_cap",
                "chrony_offset",
                "native_ros2_stack",
                "odom_mocap_sanity",
                "dds_discovery",
                "storage_resilience",
                "ROS_DISCOVERY_SERVER",
                "sqlite_resilient",
                "MCAP",
                "READY:",
                "FAILED_STAGE:",
            ],
            ["docs/ROBOT_DIAGNOSTIC_PIPELINE.md", "docs/ROBOT_DEBUG_PIPELINE_COVERAGE_AUDIT.md"],
        )
        self.require_source_patterns(
            "regression_tests",
            [
                "test_copied_remote_evidence_paths_resolve_under_local_report_root",
                "test_realsense_stream_classifier_zero_frames_is_usb_kernel",
                "test_synthesized_remote_failure_report_validates",
                "test_truncated_required_stream_fails_coverage",
                "test_run_fleet_wrapper_passes_doctor_args_once",
                "test_timeout_kills_child_process_group",
                "test_diagnostic_lock_blocks_second_doctor",
                "test_realsense_depth_isolation_zero_frames_is_usb_kernel",
                "test_dataset_run_audit_manifest_complete",
                "test_dataset_run_audit_reports_missing_fail",
                "test_dataset_run_audit_rejects_report_manifest_robot_mismatch",
                "test_dataset_run_audit_rejects_unmatched_bag_and_manifest",
                "test_chrony_tracking_parser_enforces_sub_ms_gate",
                "test_realsense_motion_isolation_zero_frames_is_d455_imu_failure",
                "test_viewer_passes_ros2_fails_named_failure_class",
                "test_d455_infra_fps_cap_detects_15hz_cap_when_higher_fps_requested",
                "test_ros2_validator_classifies_mcap_read_failure",
                "test_ros2_validator_warns_when_metadata_yaml_missing",
                "test_non_monotonic_storage_timestamp_fails",
                "test_operator_decision_block_matches_target_shape",
                "test_native_ros2_classifier_flags_bridge_when_expected",
                "test_odom_mocap_sanity_classifier_fails_slip",
                "test_dds_discovery_classifier_requires_expected_namespaces",
                "test_ros2_validator_requires_resilient_sqlite_when_configured",
            ],
            ["scripts/diagnostics/robot_doctor_selftest.py"],
        )

    def run(self) -> list[AuditResult]:
        self.require_files()
        self.require_failure_tree()
        self.require_branch_coverage()
        self.require_dataset_gate()
        self.require_sensor_logging_gate()
        self.require_report_and_remote_guards()
        self.require_bag_validator_guards()
        self.require_docs_and_tests()
        return self.results


def print_table(results: Iterable[AuditResult]) -> None:
    rows = list(results)
    widths = {
        "status": max(len("status"), *(len(row.status) for row in rows)),
        "check": max(len("check"), *(len(row.check) for row in rows)),
    }
    print(f"{'status'.ljust(widths['status'])} | {'check'.ljust(widths['check'])} | summary")
    print(f"{'-' * widths['status']}-+-{'-' * widths['check']}-+--------")
    for row in rows:
        print(f"{row.status.ljust(widths['status'])} | {row.check.ljust(widths['check'])} | {row.summary}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-out", help="write audit results as JSON")
    args = parser.parse_args()

    audit = Audit()
    results = audit.run()
    print_table(results)
    if args.json_out:
        out = Path(args.json_out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps([asdict(result) for result in results], indent=2, sort_keys=True) + "\n")
    return 1 if any(result.status == FAIL for result in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
