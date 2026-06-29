#!/usr/bin/env python3
"""No-hardware regression tests for the AGV diagnostic pipeline."""

from __future__ import annotations

import json
import os
import sqlite3
import subprocess
import sys
import tempfile
import time
import unittest
from dataclasses import asdict
from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parents[2]
VALIDATOR = ROOT / "scripts/logging/validate_ros2_bag.py"
sys.path.insert(0, str(ROOT / "scripts/diagnostics"))

from robot_doctor import (  # noqa: E402
    CheckResult,
    FAILURE_TREE,
    REPORT_SCHEMA_VERSION,
    ROBOT_DOCTOR_VERSION,
    USB_RESET_EVENT_PATTERNS,
    XHCI_PATTERNS,
    Doctor,
    CommandResult,
    apply_gate_config,
    build_parser,
    format_operator_decision,
    summarize_decision,
)
from classify_realsense_fault import classify as classify_realsense_fault  # noqa: E402
from dataset_run_audit import audit_artifact_consistency, audit_manifests, audit_reports  # noqa: E402
from fleet_doctor_summary import fleet_gate_errors, fleet_readiness_errors  # noqa: E402
from validate_robot_doctor_report import resolve_evidence_path, validate_report  # noqa: E402


TOPICS = [
    ("/scan", "sensor_msgs/msg/LaserScan", 18),
    ("/odom", "nav_msgs/msg/Odometry", 20),
    ("/cmd_vel", "geometry_msgs/msg/Twist", 1),
    ("/tf", "tf2_msgs/msg/TFMessage", 50),
    ("/tf_static", "tf2_msgs/msg/TFMessage", 1),
    ("/camera/color/image_raw", "sensor_msgs/msg/Image", 15),
    ("/camera/color/camera_info", "sensor_msgs/msg/CameraInfo", 15),
    ("/camera/aligned_depth_to_color/image_raw", "sensor_msgs/msg/Image", 15),
    ("/camera/aligned_depth_to_color/camera_info", "sensor_msgs/msg/CameraInfo", 15),
    ("/imu", "sensor_msgs/msg/Imu", 100),
    ("/camera/imu", "sensor_msgs/msg/Imu", 200),
    ("/camera/gyro/sample", "sensor_msgs/msg/Imu", 200),
    ("/camera/accel/sample", "sensor_msgs/msg/Imu", 100),
    ("/optitrack/rigid_bodies/agv", "geometry_msgs/msg/PoseStamped", 50),
]


def make_report(checks: list[CheckResult], *, profile: str = "dataset") -> dict:
    counts = {status: 0 for status in ["PASS", "WARN", "FAIL", "INFO"]}
    by_code = {
        code: {status: 0 for status in ["PASS", "WARN", "FAIL", "INFO"]}
        for code in FAILURE_TREE
    }
    for check in checks:
        counts[check.status] += 1
        by_code[check.code][check.status] += 1
    decision = summarize_decision(checks, profile)
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "tool": "robot_doctor",
        "tool_version": ROBOT_DOCTOR_VERSION,
        "robot_id": "agvtest",
        "created_at": "2026-06-27T00:00:00+00:00",
        "profile": profile,
        "verdict": decision["verdict"],
        "can_run_tests": decision["can_run_tests"],
        "dataset_ready": decision["dataset_ready"],
        "decision": decision,
        "config_path": "",
        "config_sha256": "",
        "loaded_config": {},
        "effective_gate": {"profile": profile, "required_topic": ["/scan", "/odom"]},
        "repo_state": {
            "branch": "test",
            "commit": "0000000",
            "dirty": "false",
            "status_short": "",
        },
        "output_dir": "/tmp/robot_doctor_test",
        "failure_tree": FAILURE_TREE,
        "counts": counts,
        "counts_by_code": by_code,
        "checks": [asdict(check) for check in checks],
        "commands": [],
    }


def write_ros2_bag(
    bag_dir: Path,
    *,
    duration_sec: float = 10.0,
    missing_topic: str = "",
    missing_topics: Optional[set[str]] = None,
    empty_topic: str = "",
    scan_major_gap: bool = False,
    scan_bounded_gap: bool = False,
    scan_shutdown_gap: bool = False,
    truncated_topic: str = "",
    non_monotonic_topic: str = "",
    topic_hz_overrides: Optional[dict[str, float]] = None,
) -> None:
    bag_dir.mkdir(parents=True, exist_ok=True)
    db_path = bag_dir / "test_0.db3"
    conn = sqlite3.connect(str(db_path))
    cur = conn.cursor()
    cur.execute(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
        "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)"
    )
    cur.execute(
        "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, "
        "timestamp INTEGER NOT NULL, data BLOB NOT NULL)"
    )
    start_ns = 1_000_000_000
    msg_id = 1
    topic_id = 1
    missing = set(missing_topics or set())
    if missing_topic:
        missing.add(missing_topic)
    hz_overrides = topic_hz_overrides or {}
    for name, msg_type, default_hz in TOPICS:
        if name in missing:
            continue
        hz = hz_overrides.get(name, default_hz)
        cur.execute("INSERT INTO topics VALUES (?,?,?,?,?)", (topic_id, name, msg_type, "cdr", ""))
        if name == empty_topic:
            topic_id += 1
            continue
        topic_duration = duration_sec * 0.5 if name == truncated_topic else duration_sec
        n_msgs = max(1, int(topic_duration * hz))
        for i in range(n_msgs):
            if n_msgs == 1:
                offset_ns = 0
            else:
                offset_ns = int(i * (topic_duration * 1e9 / (n_msgs - 1)))
            if scan_major_gap and name == "/scan" and i > n_msgs // 2:
                offset_ns += 350_000_000
            if scan_bounded_gap and name == "/scan" and i > n_msgs // 2:
                offset_ns += 130_000_000
            if scan_shutdown_gap and name == "/scan" and i > n_msgs - max(3, int(hz)):
                offset_ns += 350_000_000
            if non_monotonic_topic == name and i == n_msgs // 2:
                offset_ns = 0
            cur.execute(
                "INSERT INTO messages VALUES (?,?,?,?)",
                (msg_id, topic_id, start_ns + offset_ns, b"0"),
            )
            msg_id += 1
        topic_id += 1
    conn.commit()
    conn.close()


def run_validator(bag_dir: Path, extra_env: Optional[dict[str, str]] = None) -> tuple[int, dict]:
    report = bag_dir / "report.json"
    env = os.environ.copy()
    env["MOCAP_TOPIC"] = "/optitrack/rigid_bodies/agv"
    if extra_env:
        env.update(extra_env)
    proc = subprocess.run(
        [
            sys.executable,
            str(VALIDATOR),
            str(bag_dir),
            "--require-gt",
            "--require-imu",
            "--min-duration",
            "5",
            "--json-out",
            str(report),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    return proc.returncode, json.loads(report.read_text())


class ValidateRos2BagTests(unittest.TestCase):
    def test_healthy_synthetic_bag_has_no_failures(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "healthy"
            write_ros2_bag(bag)
            rc, report = run_validator(bag)
            self.assertEqual(rc, 0)
            self.assertEqual(report["counts"]["fail"], 0)
            self.assertIn(report["verdict"], {"PASS", "WARN"})

    def test_missing_required_topic_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "missing_scan"
            write_ros2_bag(bag, missing_topic="/scan")
            rc, report = run_validator(bag)
            self.assertEqual(rc, 1)
            failures = [item["check"] for item in report["results"] if item["level"] == "FAIL"]
            self.assertIn("scan", failures)

    def test_ros2_validator_accepts_raw_imu_when_fused_imu_empty(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "raw_imu_only"
            write_ros2_bag(bag, missing_topics={"/imu"}, empty_topic="/camera/imu")
            rc, report = run_validator(bag)
            self.assertEqual(rc, 0)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertFalse(failures)
            warnings = [item for item in report["results"] if item["level"] == "WARN"]
            self.assertTrue(any(item["check"] == "/camera/imu" for item in warnings))
            self.assertTrue(any(item["check"] == "/camera/gyro/sample" and item["level"] == "PASS" for item in report["results"]))
            self.assertTrue(any(item["check"] == "/camera/accel/sample" and item["level"] == "PASS" for item in report["results"]))

    def test_major_gap_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "scan_gap"
            write_ros2_bag(bag, scan_major_gap=True)
            rc, report = run_validator(bag)
            self.assertEqual(rc, 1)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertTrue(any(item["check"] == "scan_gaps" for item in failures))

    def test_bounded_scan_gap_is_warning_not_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "scan_bounded_gap"
            write_ros2_bag(bag, scan_bounded_gap=True)
            rc, report = run_validator(bag)
            self.assertEqual(rc, 0)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertFalse(any(item["check"] == "scan_gaps" for item in failures))
            warnings = [item for item in report["results"] if item["level"] == "WARN"]
            self.assertTrue(any(item["check"] == "scan_gaps" for item in warnings))

    def test_default_full_system_rgbd_bag_rate_policy(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "full_system_rgbd_rates"
            write_ros2_bag(
                bag,
                topic_hz_overrides={
                    "/camera/color/image_raw": 11.0,
                    "/camera/color/camera_info": 11.0,
                    "/camera/aligned_depth_to_color/image_raw": 13.0,
                    "/camera/aligned_depth_to_color/camera_info": 13.0,
                },
            )
            rc, report = run_validator(bag)
            self.assertEqual(rc, 0)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertFalse(failures)

    def test_rgbd_bag_rate_policy_can_be_tightened(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "strict_rgbd_rates"
            write_ros2_bag(
                bag,
                topic_hz_overrides={
                    "/camera/color/image_raw": 11.0,
                    "/camera/color/camera_info": 11.0,
                    "/camera/aligned_depth_to_color/image_raw": 13.0,
                    "/camera/aligned_depth_to_color/camera_info": 13.0,
                },
            )
            rc, report = run_validator(bag, {"RGBD_BAG_MIN_HZ": "12", "CAMERA_INFO_MIN_HZ": "12"})
            self.assertEqual(rc, 1)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertTrue(any(item["check"] == "color_image" for item in failures))

    def test_shutdown_edge_gap_does_not_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "scan_shutdown_gap"
            write_ros2_bag(bag, scan_shutdown_gap=True)
            rc, report = run_validator(bag)
            self.assertEqual(rc, 0)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertFalse(any(item["check"] == "scan_gaps" for item in failures))
            passes = [item for item in report["results"] if item["level"] == "PASS"]
            self.assertTrue(any(item["check"] == "scan_gaps" and "edge gap" in item["message"] for item in passes))

    def test_truncated_required_stream_fails_coverage(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "scan_truncated"
            write_ros2_bag(bag, truncated_topic="/scan")
            rc, report = run_validator(bag)
            self.assertEqual(rc, 1)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertTrue(any(item["check"] == "scan_coverage" for item in failures))

    def test_non_monotonic_storage_timestamp_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "non_monotonic"
            write_ros2_bag(bag, non_monotonic_topic="/odom")
            rc, report = run_validator(bag)
            self.assertEqual(rc, 1)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertTrue(any(item["check"] == "timestamp_monotonic" for item in failures))

    def test_extra_required_topic_from_env_fails_when_missing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "missing_custom_required"
            write_ros2_bag(bag)
            rc, report = run_validator(bag, {"REQUIRED_TOPICS": "/custom/required"})
            self.assertEqual(rc, 1)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertTrue(any(item["check"] == "/custom/required" for item in failures))

    def test_ros2_validator_warns_when_metadata_yaml_missing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "missing_metadata"
            write_ros2_bag(bag)
            rc, report = run_validator(bag)
            self.assertEqual(rc, 0)
            warnings = [item for item in report["results"] if item["level"] == "WARN"]
            self.assertTrue(any(item["check"] == "metadata_yaml" for item in warnings))

    def test_ros2_validator_requires_resilient_sqlite_when_configured(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "plain_sqlite"
            write_ros2_bag(bag)
            rc, report = run_validator(bag, {"REQUIRE_RESILIENT_STORAGE": "true"})
            self.assertEqual(rc, 1)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertTrue(any(item["check"] == "storage_resilience" for item in failures))

    def test_ros2_validator_accepts_sqlite_resilient_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "resilient_sqlite"
            write_ros2_bag(bag)
            (bag.parent / f"{bag.name}_manifest.yaml").write_text(
                "storage_config_uri: sqlite_resilient.yaml\njournal_mode=WAL\n"
            )
            rc, report = run_validator(bag, {"REQUIRE_RESILIENT_STORAGE": "true"})
            self.assertEqual(rc, 0)
            failures = [item for item in report["results"] if item["level"] == "FAIL"]
            self.assertFalse(any(item["check"] == "storage_resilience" for item in failures))

    def test_ros2_validator_classifies_mcap_read_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "empty.mcap"
            bag.write_bytes(b"")
            report = Path(tmp) / "report.json"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(VALIDATOR),
                    str(bag),
                    "--min-duration",
                    "0",
                    "--json-out",
                    str(report),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self.assertEqual(proc.returncode, 1, proc.stdout)
            data = json.loads(report.read_text())
            failures = [item for item in data["results"] if item["level"] == "FAIL"]
            self.assertTrue(any(item["check"] == "bag_integrity" for item in failures), data)


class RobotDoctorParserTests(unittest.TestCase):
    def test_clock_parser_requires_ntp_yes(self) -> None:
        self.assertTrue(Doctor.clock_synchronized("NTPSynchronized=yes\nTimezone=Europe/London\n"))
        self.assertFalse(Doctor.clock_synchronized("NTPSynchronized=no\nTimezone=Europe/London\n"))
        self.assertFalse(Doctor.clock_synchronized("Timezone=Europe/London\n"))

    def test_chrony_tracking_parser_enforces_sub_ms_gate(self) -> None:
        ok_text = (
            "Reference ID    : 192.168.50.1\n"
            "System time     : 0.000123456 seconds slow of NTP time\n"
            "Last offset     : +0.000234567 seconds\n"
            "RMS offset      : 0.000345678 seconds\n"
            "Leap status     : Normal\n"
        )
        status, summary, _ = Doctor.classify_chrony_tracking(ok_text, 1.0, "dataset")
        self.assertEqual(status, "PASS")
        self.assertIn("<= 1.000 ms", summary)

        stepped_text = ok_text.replace("0.000234567", "1975798.750000000").replace("0.000345678", "1975798.750000000")
        status, summary, _ = Doctor.classify_chrony_tracking(stepped_text, 1.0, "dataset")
        self.assertEqual(status, "PASS")
        self.assertIn("historical", summary)

        bad_text = ok_text.replace("0.000123456", "0.002000000")
        status, summary, next_action = Doctor.classify_chrony_tracking(bad_text, 1.0, "dataset")
        self.assertEqual(status, "FAIL")
        self.assertIn("exceeds", summary)
        self.assertIn("chrony", next_action.lower())

    def test_ping_parser_requires_successful_return_code_and_zero_loss(self) -> None:
        self.assertTrue(Doctor.ping_succeeded(0, "2 packets transmitted, 2 received, 0% packet loss"))
        self.assertFalse(Doctor.ping_succeeded(1, "2 packets transmitted, 2 received, 0% packet loss"))
        self.assertFalse(Doctor.ping_succeeded(0, "2 packets transmitted, 0 received, 100% packet loss"))

    def test_wifi_classifier_detects_manual_processes(self) -> None:
        text = "NMCLI\nwlan0          wifi      connected     VM3090788\nPROCESSES\nwpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf\ndhclient wlan0\n"
        code, status, check, summary, _ = Doctor.classify_wifi_management(text, "dataset")
        self.assertEqual((code, status, check), ("3.3", "FAIL", "wifi_management"))
        self.assertIn("manual dhclient", summary)

    def test_wifi_classifier_accepts_networkmanager(self) -> None:
        text = "PROCESSES\n/usr/sbin/NetworkManager --no-daemon\nNMCLI\nwlan0          wifi      connected     VM3090788\n"
        code, status, check, _, _ = Doctor.classify_wifi_management(text, "preflight")
        self.assertEqual((code, status, check), ("3.3", "PASS", "wifi_management"))

    def test_native_ros2_classifier_flags_bridge_when_expected(self) -> None:
        text = "ENV\nROS_DISTRO=humble\nROS_MASTER_URI=http://localhost:11311\nPROCESSES\n123 ros1_bridge dynamic_bridge\n"
        code, status, check, summary, next_action = Doctor.classify_native_ros2_stack(
            text,
            "ros2",
            True,
            "dataset",
        )
        self.assertEqual((code, status, check), ("2.2", "FAIL", "native_ros2_stack"))
        self.assertIn("ROS1", summary)
        self.assertIn("bridge", next_action)

    def test_native_ros2_classifier_accepts_native_ros2(self) -> None:
        text = "ENV\nROS_DISTRO=humble\nCOMMANDS\n/opt/ros/humble/bin/ros2\nPROCESSES\n"
        self.assertEqual(
            Doctor.classify_native_ros2_stack(text, "ros2", True, "dataset"),
            ("2.2", "PASS", "native_ros2_stack", "native ROS2 stack proven with no ROS1 bridge evidence", ""),
        )

    def test_dds_discovery_classifier_requires_expected_namespaces(self) -> None:
        text = "/agv100/driver\n/agv101/camera\n"
        code, status, check, summary, _ = Doctor.classify_dds_discovery(
            text,
            ["/agv100", "/agv101", "/agv102"],
            "",
            "dataset",
        )
        self.assertEqual((code, status, check), ("3.3", "FAIL", "dds_discovery"))
        self.assertIn("/agv102", summary)

    def test_dds_discovery_classifier_passes_all_namespaces(self) -> None:
        text = "/agv100/driver\n/agv101/camera\n"
        code, status, check, summary, _ = Doctor.classify_dds_discovery(
            text,
            ["/agv100", "/agv101"],
            "192.168.50.100:11811",
            "dataset",
        )
        self.assertEqual((code, status, check), ("3.3", "PASS", "dds_discovery"))
        self.assertIn("discovery server", summary)

    def test_odom_mocap_sanity_classifier_passes_within_threshold(self) -> None:
        self.assertEqual(
            Doctor.classify_odom_mocap_sanity(
                {"odom_distance_m": 1.04, "mocap_distance_m": 1.0},
                0.10,
                "dataset",
            )[0:3],
            ("1.3", "PASS", "odom_mocap_sanity"),
        )

    def test_odom_mocap_sanity_classifier_fails_slip(self) -> None:
        code, status, check, summary, next_action = Doctor.classify_odom_mocap_sanity(
            {"odom_distance_m": 1.25, "mocap_distance_m": 1.0},
            0.10,
            "dataset",
        )
        self.assertEqual((code, status, check), ("1.3", "FAIL", "odom_mocap_sanity"))
        self.assertIn("25.0%", summary)
        self.assertIn("wheel odometry", next_action)

    def test_usb_power_classifier_detects_d455_autosuspend(self) -> None:
        text = "USB_DEVICE=/sys/bus/usb/devices/2-1\nidVendor=8086\nidProduct=0b5c\nproduct=Intel RealSense D455\nspeed=5000\npower_control=auto\npower_autosuspend_delay_ms=2000\nCMDLINE\nquiet splash\n"
        results = Doctor.classify_usb_power_policy(text, "dataset")
        self.assertIn(("2.1", "WARN", "d455_usb_autosuspend", "D455 USB autosuspend is enabled (power/control=auto)", "disable autosuspend for the D455 before long dataset runs"), results)
        self.assertTrue(any(item[0:3] == ("1.2", "FAIL", "d455_usb_autosuspend_delay") for item in results))

    def test_usb_power_classifier_accepts_negative_delay_with_auto_control(self) -> None:
        text = "USB_DEVICE=/sys/bus/usb/devices/2-1\nidVendor=8086\nidProduct=0b5c\nproduct=Intel RealSense D455\nspeed=5000\npower_control=auto\npower_autosuspend_delay_ms=-1\nCMDLINE\nusbcore.quirks=8086:0b5c:kn\n"
        results = Doctor.classify_usb_power_policy(text, "dataset")
        self.assertIn(("2.1", "PASS", "d455_usb_autosuspend", "D455 USB autosuspend disabled (power/control=auto, autosuspend_delay_ms=-1)", ""), results)
        self.assertTrue(any(item[0:3] == ("1.2", "PASS", "d455_usb_autosuspend_delay") for item in results))

    def test_usb_power_classifier_accepts_d455_power_on_and_quirk(self) -> None:
        text = "USB_DEVICE=/sys/bus/usb/devices/2-1\nidVendor=8086\nidProduct=0b5c\nproduct=Intel RealSense D455\nspeed=5000\npower_control=on\npower_autosuspend_delay_ms=-1\nCMDLINE\nusbcore.quirks=8086:0b5c:kn\n"
        results = Doctor.classify_usb_power_policy(text)
        self.assertIn(("2.1", "PASS", "d455_usb_autosuspend", "D455 USB autosuspend disabled (power/control=on)", ""), results)
        self.assertIn(("1.2", "PASS", "d455_usb_autosuspend_delay", "D455 autosuspend_delay_ms=-1", ""), results)
        self.assertIn(("2.1", "PASS", "d455_usb_boot_quirk", "D455 usbcore quirk is present in kernel cmdline", ""), results)

    def test_d455_uvc_binding_classifier_accepts_bound_interfaces(self) -> None:
        text = (
            "D455_DEVICE=2-2\n"
            "interface=2-2:1.0 class=0e subclass=01 driver=uvcvideo name=Depth\n"
            "interface=2-2:1.1 class=0e subclass=02 driver=uvcvideo name=Depth\n"
            "interface=2-2:1.5 class=03 subclass=00 driver=usbhid name=HID\n"
        )
        results = Doctor.classify_d455_uvc_binding(text)
        self.assertIn(("2.1", "PASS", "d455_uvc_binding", "D455 video interfaces are bound to uvcvideo", ""), results)

    def test_d455_uvc_binding_classifier_detects_unbound_interfaces(self) -> None:
        text = (
            "D455_DEVICE=2-2\n"
            "interface=2-2:1.0 class=0e subclass=01 driver=none name=Depth\n"
            "interface=2-2:1.1 class=0e subclass=02 driver=none name=Depth\n"
            "interface=2-2:1.5 class=03 subclass=00 driver=usbhid name=HID\n"
        )
        results = Doctor.classify_d455_uvc_binding(text)
        self.assertEqual(results[0][0:3], ("2.1", "FAIL", "d455_uvc_binding"))
        self.assertIn("2-2:1.0=none", results[0][3])
        self.assertIn("d455-uvc-bind", results[0][4])

    def test_d455_imu_hid_accepts_iio_motion_devices(self) -> None:
        text = (
            "IIO=/sys/bus/iio/devices/iio:device0 "
            "PATH=/sys/devices/platform/scb/usb2/2-2/2-2:1.5/"
            "0003:8086:0B5C.0003/HID-SENSOR-200073.1.auto/iio:device0\n"
            "NAME=accel_3d\n"
        )
        self.assertEqual(
            Doctor.classify_d455_imu_hid(text, require_imu=True),
            ("1.1", "PASS", "d455_imu_hid", "D455 IIO motion sensor path is visible", ""),
        )

    def test_d455_imu_hid_fails_when_required_and_absent(self) -> None:
        code, status, check, summary, next_action = Doctor.classify_d455_imu_hid(
            "HIDRAW_DEVICES\nIIO_DEVICES\n",
            require_imu=True,
        )
        self.assertEqual((code, status, check), ("1.1", "FAIL", "d455_imu_hid"))
        self.assertIn("HID/IIO", summary)
        self.assertIn("standalone motion", next_action)

    def test_realsense_usb_speed_parser(self) -> None:
        text = "T: Bus=02 Lev=01 Prnt=01 Port=01 Cnt=01 Dev#= 2 Spd=5000\nP: Vendor=8086 ProdID=0b5c\n"
        self.assertEqual(Doctor.parse_d455_speed(text), 5000)

    def test_realsense_firmware_parser(self) -> None:
        text = "Device Name                   Serial Number       Firmware Version\nIntel RealSense D455          123                 5.17.0.10\n"
        self.assertEqual(Doctor.parse_realsense_firmware(text), "5.17.0.10")

    def test_realsense_serial_parser(self) -> None:
        summary = "Device Name                   Serial Number       Firmware Version\nIntel RealSense D455          333422300768        5.17.0.10\n"
        full = "Name: Intel RealSense D455\nSerial Number: 333422300768\nFirmware: 5.17.0.10\n"
        self.assertEqual(Doctor.parse_realsense_serial(summary), "333422300768")
        self.assertEqual(Doctor.parse_realsense_serial(full), "333422300768")

    def test_realsense_ros_driver_version_parser(self) -> None:
        dpkg = "ros-humble-realsense2-camera\t4.57.7-1jammy.20260601\n"
        self.assertEqual(Doctor.parse_realsense_ros_driver_version(dpkg), "4.57.7")
        package_xml = "<package><name>realsense2_camera</name><version>4.57.7</version></package>"
        self.assertEqual(Doctor.parse_realsense_ros_driver_version(package_xml), "4.57.7")

    def test_standalone_realsense_tool_parser_ignores_ros_section(self) -> None:
        text = (
            "STANDALONE_RS_TOOLS\n"
            "STANDALONE_PKG_CONFIG\n"
            "DPKG\n"
            "ros-humble-realsense2-camera\t4.57.7-4jammy\n"
            "ROS_REALSENSE_PACKAGE\n"
            "/opt/ros/humble/bin/rs-enumerate-devices\n"
        )
        self.assertEqual(Doctor.parse_standalone_realsense_tool(text), "")

    def test_standalone_realsense_tool_parser_accepts_non_ros_tool(self) -> None:
        text = (
            "STANDALONE_RS_TOOLS\n"
            "/usr/bin/rs-enumerate-devices\n"
            "STANDALONE_PKG_CONFIG\n"
        )
        self.assertEqual(
            Doctor.parse_standalone_realsense_tool(text),
            "/usr/bin/rs-enumerate-devices",
        )

    def test_standalone_librealsense_parser_ignores_ros_driver_versions(self) -> None:
        text = (
            "STANDALONE_RS_TOOLS\n"
            "STANDALONE_PKG_CONFIG\n"
            "2.58.1\n"
            "DPKG\n"
            "librealsense2\t2.58.1-0~realsense.8235\n"
            "ros-humble-realsense2-camera\t4.57.7-4jammy.20260423.183825\n"
            "ROS_REALSENSE_PACKAGE\n"
            "<package><name>realsense2_camera</name><version>4.57.7</version></package>\n"
        )
        self.assertEqual(Doctor.parse_lrs_versions(text), ["2.58.1"])

    def test_realsense_setup_provenance_parsers(self) -> None:
        text = (
            "DPKG\n"
            "librealsense2:arm64\t2.58.1-0~realsense.8235\n"
            "librealsense2-utils:arm64\t2.58.1-0~realsense.8235\n"
            "APT_HOLDS\n"
            "librealsense2\n"
            "librealsense2-utils\n"
            "REALSENSE_APT_SOURCES\n"
            "FILE:/etc/apt/sources.list.d/librealsense.list\n"
            "deb [trusted=yes] https://librealsense.intel.com/Debian/apt-repo jammy main\n"
            "FILE:/etc/apt/sources.list.d/old.list.disabled-by-setup-20260627\n"
            "deb [signed-by=/old/key] https://librealsense.intel.com/Debian/apt-repo jammy main\n"
            "PYREALSENSE2\n"
            "2.58.1.10581\n"
            "ROS_REALSENSE_PACKAGE\n"
        )
        self.assertEqual(
            Doctor.parse_installed_realsense_packages(text),
            ["librealsense2", "librealsense2-utils"],
        )
        self.assertEqual(Doctor.parse_apt_holds(text), ["librealsense2", "librealsense2-utils"])
        self.assertEqual(Doctor.parse_pyrealsense2_status(text), ("2.58.1.10581", ""))
        status, summary, _ = Doctor.classify_realsense_apt_sources(text)
        self.assertEqual(status, "INFO")
        self.assertIn("trusted=yes", summary)

    def test_realsense_setup_provenance_check_runs(self) -> None:
        text = (
            "DPKG\n"
            "librealsense2:arm64\t2.58.1-0~realsense.8235\n"
            "librealsense2-dev:arm64\t2.58.1-0~realsense.8235\n"
            "librealsense2-utils:arm64\t2.58.1-0~realsense.8235\n"
            "librealsense2-udev-rules:arm64\t2.58.1-0~realsense.8235\n"
            "APT_HOLDS\n"
            "librealsense2\n"
            "librealsense2-dev\n"
            "librealsense2-utils\n"
            "librealsense2-udev-rules\n"
            "REALSENSE_APT_SOURCES\n"
            "FILE:/etc/apt/sources.list.d/librealsense.list\n"
            "deb [signed-by=/key] https://librealsense.intel.com/Debian/apt-repo jammy main\n"
            "PYREALSENSE2\n"
            "2.58.1.10581\n"
            "ROS_REALSENSE_PACKAGE\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.check_realsense_setup_provenance(text, ["/tmp/evidence.log"], "2.58.1")
            checks = [item.check for item in doctor.results]
            self.assertIn("realsense_apt_source", checks)
            self.assertIn("realsense_package_holds", checks)
            self.assertIn("realsense_python_binding", checks)
            self.assertTrue(all(item.status == "PASS" for item in doctor.results))

    def test_realsense_apt_source_duplicate_warns(self) -> None:
        text = (
            "REALSENSE_APT_SOURCES\n"
            "FILE:/etc/apt/sources.list.d/a.list\n"
            "deb [trusted=yes] https://librealsense.intel.com/Debian/apt-repo jammy main\n"
            "FILE:/etc/apt/sources.list.d/b.list\n"
            "deb [signed-by=/key] https://librealsense.intel.com/Debian/apt-repo jammy main\n"
            "PYREALSENSE2\n"
        )
        status, summary, next_action = Doctor.classify_realsense_apt_sources(text)
        self.assertEqual(status, "WARN")
        self.assertIn("multiple", summary)
        self.assertIn("disable duplicate", next_action)

    def test_pyrealsense2_import_error_parser(self) -> None:
        text = "PYREALSENSE2\nIMPORT_ERROR:ModuleNotFoundError:No module named pyrealsense2\nROS_REALSENSE_PACKAGE\n"
        self.assertEqual(
            Doctor.parse_pyrealsense2_status(text),
            ("", "ModuleNotFoundError:No module named pyrealsense2"),
        )

    def test_realsense_ros_librealsense_runtime_parser(self) -> None:
        text = "RealSense ROS v4.57.7\nBuilt with LibRealSense v2.57.7\nRunning with LibRealSense v2.57.7\n"
        self.assertEqual(Doctor.parse_realsense_ros_librealsense_versions(text), ("2.57.7", "2.57.7"))

    def test_realsense_error_parser(self) -> None:
        self.assertTrue(Doctor.has_realsense_error("Failed to query (GET_CUR) UVC control 4"))
        self.assertFalse(Doctor.has_realsense_error("Intel RealSense D455 ready"))

    def test_intentional_usb_reset_is_not_xhci_failure(self) -> None:
        line = "Jun 27 22:58:27 agv2 kernel: usb 2-2: reset SuperSpeed USB device number 2 using xhci_hcd"
        self.assertEqual(Doctor.match_patterns(line, XHCI_PATTERNS), [])
        self.assertEqual(Doctor.match_patterns(line, USB_RESET_EVENT_PATTERNS), [line])

    def test_librealsense_visibility_is_os_kernel_usb_branch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--output-root", tmp])
            args.expected_librealsense = "2.58.1"
            args.expected_realsense_ros_driver = "4.57.7"

            class FakeDoctor(Doctor):
                def run(self, label: str, command: str, timeout: int = 15) -> CommandResult:
                    outputs = {
                        "realsense_versions": (
                            "STANDALONE_RS_TOOLS\n"
                            "/usr/bin/rs-enumerate-devices\n"
                            "STANDALONE_PKG_CONFIG\n"
                            "2.58.1\n"
                            "DPKG\n"
                            "librealsense2:arm64\t2.58.1-0~realsense.8235\n"
                            "ros-humble-realsense2-camera\t4.57.7-4jammy\n"
                            "APT_HOLDS\n"
                            "librealsense2\n"
                            "REALSENSE_APT_SOURCES\n"
                            "FILE:/etc/apt/sources.list.d/librealsense.list\n"
                            "deb [trusted=yes] https://librealsense.intel.com/Debian/apt-repo jammy main\n"
                            "PYREALSENSE2\n"
                            "2.58.1.10581\n"
                            "ROS_REALSENSE_PACKAGE\n"
                        ),
                        "rs_enumerate_summary": "No device detected. Is it plugged in?\n",
                        "rs_enumerate_controls": "set_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed\n",
                    }
                    output = outputs.get(label, "")
                    log_path = self.log_dir / f"fake_{label}.txt"
                    log_path.write_text(f"$ {command}\n\n{output}")
                    result = CommandResult(label, command, 1 if label != "realsense_versions" else 0, 0.0, False, str(log_path))
                    self.commands.append(result)
                    return result

            doctor = FakeDoctor(args)
            doctor.check_realsense()
            matches = [item for item in doctor.results if item.check == "d455_rs_enumerate"]
            self.assertEqual(len(matches), 1)
            self.assertEqual((matches[0].code, matches[0].status), ("2.1", "FAIL"))

    def test_realsense_stream_classifier_missing_dependency(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_stream_result(
            {"error": "missing_pyrealsense2"},
            seconds=10,
            fps=15,
        )
        self.assertEqual((code, status, check), ("2.2", "FAIL", "realsense_python_binding"))

    def test_realsense_stream_classifier_timeout_is_usb_kernel(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_stream_result(
            {"color": 150, "depth": 150, "timeouts": 1},
            seconds=10,
            fps=15,
        )
        self.assertEqual((code, status, check), ("2.1", "FAIL", "realsense_stream_timeouts"))

    def test_realsense_stream_exception_with_uvc_is_usb_kernel(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_stream_result(
            {"error": "stream_exception", "detail": "UVCIOC_CTRL_QUERY failed: Connection timed out"},
            seconds=10,
            fps=15,
        )
        self.assertEqual((code, status, check), ("2.1", "FAIL", "realsense_stream_exception"))

    def test_unparseable_realsense_stream_timeout_is_usb_kernel(self) -> None:
        code, status, check, _, _ = Doctor.classify_unparseable_realsense_stream(
            "TIMEOUT after 35s\nUVCIOC_CTRL_QUERY failed",
            timed_out=True,
        )
        self.assertEqual((code, status, check), ("2.1", "FAIL", "realsense_stream_transport"))

    def test_unparseable_realsense_stream_without_transport_error_is_validation_failure(self) -> None:
        code, status, check, _, _ = Doctor.classify_unparseable_realsense_stream(
            "unexpected python output",
            timed_out=False,
        )
        self.assertEqual((code, status, check), ("3.2", "FAIL", "realsense_stream_test"))

    def test_realsense_stream_classifier_low_rate_is_validation_failure(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_stream_result(
            {"color": 50, "depth": 150, "timeouts": 0},
            seconds=10,
            fps=15,
        )
        self.assertEqual((code, status, check), ("3.2", "FAIL", "realsense_stream_rate"))

    def test_realsense_stream_classifier_zero_frames_is_usb_kernel(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_stream_result(
            {"color": 0, "depth": 0, "timeouts": 0},
            seconds=10,
            fps=15,
        )
        self.assertEqual((code, status, check), ("2.1", "FAIL", "realsense_stream_no_frames"))

    def test_realsense_depth_isolation_zero_frames_is_usb_kernel(self) -> None:
        code, status, check, summary, _ = Doctor.classify_realsense_single_stream_result(
            "depth",
            {"color": 0, "depth": 0, "timeouts": 5},
            seconds=5,
            fps=15,
        )
        self.assertEqual((code, status, check), ("2.1", "FAIL", "realsense_depth_stream"))
        self.assertIn("no usable frames", summary)

    def test_realsense_motion_isolation_zero_frames_is_d455_imu_failure(self) -> None:
        code, status, check, summary, _ = Doctor.classify_realsense_single_stream_result(
            "motion",
            {"gyro": 0, "accel": 0, "timeouts": 0},
            seconds=5,
            fps=15,
        )
        self.assertEqual((code, status, check), ("2.1", "FAIL", "realsense_motion_stream_isolation"))
        self.assertIn("no usable IMU frames", summary)

    def test_realsense_color_isolation_passes(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_single_stream_result(
            "color",
            {"color": 70, "depth": 0, "timeouts": 0},
            seconds=5,
            fps=15,
        )
        self.assertEqual((code, status, check), ("3.2", "PASS", "realsense_color_stream"))

    def test_realsense_stream_classifier_low_motion_rate_fails(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_stream_result(
            {"color": 150, "depth": 150, "gyro": 100, "accel": 100, "timeouts": 0},
            seconds=10,
            fps=15,
            motion=True,
        )
        self.assertEqual((code, status, check), ("3.2", "FAIL", "realsense_motion_stream_rate"))

    def test_realsense_stream_classifier_pass(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_stream_result(
            {"color": 150, "depth": 150, "timeouts": 0},
            seconds=10,
            fps=15,
        )
        self.assertEqual((code, status, check), ("3.2", "PASS", "realsense_stream_test"))

    def test_realsense_stream_classifier_motion_pass(self) -> None:
        code, status, check, _, _ = Doctor.classify_realsense_stream_result(
            {"color": 150, "depth": 150, "gyro": 1800, "accel": 900, "timeouts": 0},
            seconds=10,
            fps=15,
            motion=True,
        )
        self.assertEqual((code, status, check), ("3.2", "PASS", "realsense_stream_test"))

    def test_viewer_passes_ros2_fails_named_failure_class(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--output-root", tmp, "--ros", "ros2"])
            doctor = Doctor(args)
            doctor.ros_mode = "ros2"
            doctor.topic_types = {"/scan": "sensor_msgs/msg/LaserScan"}
            doctor.results.append(CheckResult("3.2", "PASS", "realsense_stream_test", "standalone ok"))
            doctor.check_realsense_ros2_failure_class()
            matches = [item for item in doctor.results if item.check == "viewer_passes_ros2_fails"]
            self.assertEqual((matches[-1].code, matches[-1].status), ("2.1", "FAIL"))
            self.assertIn("ROS2 camera image topics are absent", matches[-1].summary)

    def test_d455_infra_fps_cap_detects_15hz_cap_when_higher_fps_requested(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--output-root", tmp, "--ros", "ros2", "--camera-fps", "30"])

            class FakeDoctor(Doctor):
                def measure_topic_rate(self, topic: str, seconds: int) -> Optional[float]:
                    return 15.0

            doctor = FakeDoctor(args)
            doctor.ros_mode = "ros2"
            doctor.topic_types = {"/camera/infra1/image_rect_raw": "sensor_msgs/msg/Image"}
            doctor.check_d455_infra_fps_cap()
            matches = [item for item in doctor.results if item.check == "d455_infra_fps_cap"]
            self.assertEqual((matches[-1].code, matches[-1].status), ("2.2", "FAIL"))
            self.assertIn("depth_module.enable_auto_exposure", matches[-1].next_action)

    def test_topic_hz_parser_uses_median_not_shutdown_tail(self) -> None:
        text = """
average rate: 14.996
average rate: 14.974
average rate: 14.983
average rate: 14.999
average rate: 14.985
average rate: 10.696
"""
        self.assertAlmostEqual(Doctor.parse_topic_hz(text) or 0.0, 14.984, places=3)


class RealSenseFaultClassifierTests(unittest.TestCase):
    def test_bounded_rgbd_gap_is_pass_with_stream_warnings(self) -> None:
        text = """
PASS color stream: /camera/color/image_raw 14.638 Hz
WARN color stream steady max gap: 0.655s exceeds warning 0.25s but is <= hard 0.75s after window 40
PASS aligned depth stream: /camera/aligned_depth_to_color/image_raw 14.986 Hz
WARN aligned depth stream steady max gap: 0.651s exceeds warning 0.25s but is <= hard 0.75s after window 40
PASS camera imu stream: /camera/imu 200.066 Hz
PASS camera imu stream steady max gap: 0.011s <= warning 0.10s after window 80
PASS RealSense runtime log: no UVC/control timeout text observed
speed=5000
"""
        classification, evidence, limitations = classify_realsense_fault(text)
        self.assertEqual(classification, "PASS_WITH_STREAM_WARNINGS")
        self.assertTrue(any("bounded warning gaps" in item for item in evidence))
        self.assertTrue(any("Final bag validation" in item for item in limitations))

    def test_hard_rgbd_gap_remains_stream_gap_failure(self) -> None:
        text = """
PASS color stream: /camera/color/image_raw 14.638 Hz
FAIL color stream steady max gap: 1.250s exceeds hard 0.75s after window 40
PASS aligned depth stream: /camera/aligned_depth_to_color/image_raw 14.986 Hz
PASS camera imu stream: /camera/imu 200.066 Hz
PASS camera imu stream steady max gap: 0.011s <= warning 0.10s after window 80
speed=5000
"""
        classification, _, _ = classify_realsense_fault(text)
        self.assertEqual(classification, "REALSENSE_STREAM_GAP_FAILURE")

    def test_raw_imu_fallback_allows_fused_imu_gap(self) -> None:
        text = """
PASS color stream: /camera/color/image_raw 14.638 Hz
PASS aligned depth stream: /camera/aligned_depth_to_color/image_raw 14.986 Hz
FAIL camera imu stream steady max gap: 2.046s exceeds hard 0.10s after window 80
WARN camera imu stream: fused /camera/imu failed; checking raw gyro+accel fallback
PASS camera gyro stream: /camera/gyro/sample 199.900 Hz
PASS camera gyro stream steady max gap: 0.010s <= warning 0.10s after window 80
PASS camera accel stream: /camera/accel/sample 99.900 Hz
PASS camera accel stream steady max gap: 0.020s <= warning 0.10s after window 40
PASS camera imu fallback: raw gyro+accel streams satisfy IMU gate
speed=5000
"""
        classification, evidence, _ = classify_realsense_fault(text)
        self.assertEqual(classification, "PASS_WITH_STREAM_WARNINGS")
        self.assertTrue(any("IMU/HID topics produced rate data" in item for item in evidence))


class RobotDoctorDecisionTests(unittest.TestCase):
    def test_operator_decision_block_matches_target_shape(self) -> None:
        report = make_report(
            [
                CheckResult(
                    "2.1",
                    "FAIL",
                    "kernel_uvc_errors",
                    "D455 enumerates OK but UVC -110 timeout during rs-motion test",
                    evidence=["rs-enumerate-devices.log", "kernel_usb_logs.log"],
                    next_action="power-cycle, swap cable, mark USB host suspect if repeatable",
                )
            ],
            profile="dataset",
        )
        text = format_operator_decision(report)
        self.assertIn("READY: false", text)
        self.assertIn("FAILED_STAGE: 2.1 OS / kernel / USB", text)
        self.assertIn("CAUSE: D455 enumerates OK but UVC -110 timeout during rs-motion test", text)
        self.assertIn("EVIDENCE:", text)
        self.assertIn("rs-enumerate-devices.log", text)
        self.assertIn("NEXT_ACTION: power-cycle, swap cable, mark USB host suspect if repeatable", text)

    def test_failure_blocks_tests_and_dataset(self) -> None:
        decision = summarize_decision(
            [
                CheckResult("2.2", "FAIL", "realsense_tools", "missing tools", next_action="install tools"),
                CheckResult("3.3", "WARN", "mocap_operator_check", "not confirmed"),
            ]
        )
        self.assertEqual(decision["state"], "blocked")
        self.assertFalse(decision["can_run_tests"])
        self.assertFalse(decision["dataset_ready"])
        self.assertEqual(decision["primary_blocker"]["check"], "realsense_tools")
        self.assertEqual(decision["recommendation"], "install tools")

    def test_root_cause_failure_beats_downstream_missing_topic_symptom(self) -> None:
        decision = summarize_decision(
            [
                CheckResult("2.2", "FAIL", "dataset_bringup_context", "required topics missing: /scan"),
                CheckResult(
                    "1.2",
                    "FAIL",
                    "ydlidar_scan_frame_timeout",
                    "YDLidar serial opens and scan command is issued, but no scan frames arrive after scan start",
                    next_action="check LiDAR motor power",
                ),
                CheckResult("2.3", "FAIL", "topic_present", "/scan missing"),
            ]
        )
        self.assertEqual(decision["primary_blocker"]["check"], "ydlidar_scan_frame_timeout")
        self.assertEqual(decision["primary_blocker"]["code"], "1.2")
        self.assertEqual(decision["recommendation"], "check LiDAR motor power")

    def test_warning_allows_tests_but_blocks_dataset(self) -> None:
        decision = summarize_decision(
            [
                CheckResult("1.3", "WARN", "mechanical_operator_check", "not confirmed"),
                CheckResult("3.1", "PASS", "disk_free", "ok"),
            ]
        )
        self.assertEqual(decision["state"], "review")
        self.assertTrue(decision["can_run_tests"])
        self.assertFalse(decision["dataset_ready"])
        self.assertEqual(decision["primary_blocker"]["check"], "mechanical_operator_check")

    def test_clean_report_is_dataset_ready(self) -> None:
        decision = summarize_decision([CheckResult("3.1", "PASS", "disk_free", "ok")], "dataset")
        self.assertEqual(decision["state"], "ready")
        self.assertTrue(decision["can_run_tests"])
        self.assertTrue(decision["dataset_ready"])
        self.assertIsNone(decision["primary_blocker"])

    def test_clean_static_report_is_not_dataset_ready(self) -> None:
        decision = summarize_decision([CheckResult("3.1", "PASS", "disk_free", "ok")], "static")
        self.assertEqual(decision["state"], "ready")
        self.assertTrue(decision["can_run_tests"])
        self.assertFalse(decision["dataset_ready"])


class RobotDoctorConfigTests(unittest.TestCase):
    def test_config_applies_when_cli_does_not_override(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "gate.json"
            config.write_text(
                json.dumps(
                    {
                        "profile": "dataset",
                        "require_gt": True,
                        "require_bag": True,
                        "confirm_d455_camera_swap": True,
                        "confirm_d455_cable_swap": True,
                        "confirm_d455_host_port_swap": True,
                        "d455_swap_notes": "/tmp/d455_swap_notes.md",
                        "expected_d455_serial": "333422300768",
                        "expected_d455_firmware": "5.17.0.10",
                        "expected_realsense_ros_driver": "4.57.7",
                        "expected_realsense_ros_librealsense": "2.57.7",
                        "expect_native_ros2": True,
                        "expected_robot_namespace": ["/agv100", "/agv101"],
                        "require_odom_mocap_sanity": True,
                        "odom_mocap_sanity_json": "/tmp/odom_mocap.json",
                        "odom_mocap_max_error_ratio": 0.1,
                        "require_resilient_storage": True,
                        "required_topic": ["/scan", "/odom"],
                    }
                )
            )
            argv = ["agvtest", "--config", str(config)]
            args = build_parser().parse_args(argv)
            loaded = apply_gate_config(args, argv)
            self.assertEqual(loaded["profile"], "dataset")
            self.assertEqual(args.profile, "dataset")
            self.assertTrue(args.require_gt)
            self.assertTrue(args.require_bag)
            self.assertTrue(args.confirm_d455_camera_swap)
            self.assertTrue(args.confirm_d455_cable_swap)
            self.assertTrue(args.confirm_d455_host_port_swap)
            self.assertEqual(args.d455_swap_notes, "/tmp/d455_swap_notes.md")
            self.assertEqual(args.expected_d455_serial, "333422300768")
            self.assertEqual(args.expected_d455_firmware, "5.17.0.10")
            self.assertEqual(args.expected_realsense_ros_driver, "4.57.7")
            self.assertEqual(args.expected_realsense_ros_librealsense, "2.57.7")
            self.assertTrue(args.expect_native_ros2)
            self.assertEqual(args.expected_robot_namespace, ["/agv100", "/agv101"])
            self.assertTrue(args.require_odom_mocap_sanity)
            self.assertEqual(args.odom_mocap_sanity_json, "/tmp/odom_mocap.json")
            self.assertEqual(args.odom_mocap_max_error_ratio, 0.1)
            self.assertTrue(args.require_resilient_storage)
            self.assertEqual(args.required_topic, ["/scan", "/odom"])

    def test_cli_overrides_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "gate.json"
            config.write_text(json.dumps({"profile": "dataset", "stream_test_seconds": 90}))
            argv = ["agvtest", "--config", str(config), "--profile", "static", "--stream-test-seconds", "5"]
            args = build_parser().parse_args(argv)
            apply_gate_config(args, argv)
            self.assertEqual(args.profile, "static")
            self.assertEqual(args.stream_test_seconds, 5)

    def test_required_topic_does_not_override_default_rate_threshold(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--output-root", tmp])
            args.required_topic = ["/scan", "/custom/topic"]
            doctor = Doctor(args)
            doctor.topic_types = {"/scan": "sensor_msgs/msg/LaserScan"}
            specs = doctor.live_topic_specs()
            self.assertEqual(specs["/scan"]["min_hz"], 5.0)
            self.assertEqual(specs["/custom/topic"]["min_hz"], 0.0)

    def test_dataset_bringup_context_flags_missing_existing_graph(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--profile", "dataset", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.topic_types = {"/rosout": "rcl_interfaces/msg/Log"}
            doctor.check_dataset_bringup_context()
            self.assertEqual(len(doctor.results), 1)
            self.assertEqual(
                (doctor.results[0].code, doctor.results[0].status, doctor.results[0].check),
                ("2.2", "FAIL", "dataset_bringup_context"),
            )

    def test_dataset_no_ros_is_partial_gate_warning(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--profile", "dataset", "--no-ros", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.ros_mode = "ros2"
            doctor.ros_live_checks()
            self.assertEqual(len(doctor.results), 1)
            self.assertEqual(
                (doctor.results[0].code, doctor.results[0].status, doctor.results[0].check),
                ("2.3", "WARN", "ros_graph_skipped"),
            )

    def test_dataset_without_bag_is_partial_gate_warning(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--profile", "dataset", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.validate_bag()
            self.assertEqual(len(doctor.results), 1)
            self.assertEqual(
                (doctor.results[0].code, doctor.results[0].status, doctor.results[0].check),
                ("3.2", "WARN", "bag_validation_missing"),
            )

    def test_require_bag_without_bag_is_hard_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(
                ["agvtest", "--profile", "dataset", "--require-bag", "--output-root", tmp]
            )
            doctor = Doctor(args)
            doctor.validate_bag()
            self.assertEqual(len(doctor.results), 1)
            self.assertEqual(
                (doctor.results[0].code, doctor.results[0].status, doctor.results[0].check),
                ("3.2", "FAIL", "bag_validation_missing"),
            )

    def test_dataset_bringup_context_passes_existing_graph_with_topics(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--profile", "dataset", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.topic_types = {
                "/scan": "sensor_msgs/msg/LaserScan",
                "/odom": "nav_msgs/msg/Odometry",
                "/tf": "tf2_msgs/msg/TFMessage",
                "/camera/color/image_raw": "sensor_msgs/msg/Image",
                "/camera/color/camera_info": "sensor_msgs/msg/CameraInfo",
                "/camera/aligned_depth_to_color/image_raw": "sensor_msgs/msg/Image",
                "/camera/aligned_depth_to_color/camera_info": "sensor_msgs/msg/CameraInfo",
            }
            doctor.check_dataset_bringup_context()
            self.assertEqual(len(doctor.results), 1)
            self.assertEqual(
                (doctor.results[0].code, doctor.results[0].status, doctor.results[0].check),
                ("2.2", "PASS", "dataset_bringup_context"),
            )

    def test_ydlidar_scan_frame_timeout_is_classified_from_bringup_log(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--profile", "dataset", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.bringup_log = doctor.log_dir / "bringup_command.log"
            doctor.bringup_log.write_text(
                "YDLidar SDK initializing\n"
                "LiDAR successfully connected\n"
                "[YDLIDAR]:Lidar running correctly ! The health status: good\n"
                "[CYdLidar] Successed to start scan mode, Elapsed time 1061 ms\n"
                "timout count: 1\n"
                "timout count: 2\n"
                "[CYdLidar] Failed to turn on the Lidar, because the lidar is [Operation timed out].\n"
            )
            doctor.topic_types = {"/odom": "nav_msgs/msg/Odometry"}
            doctor.check_ydlidar_bringup_classification()
            self.assertEqual(len(doctor.results), 1)
            self.assertEqual(
                (doctor.results[0].code, doctor.results[0].status, doctor.results[0].check),
                ("1.2", "FAIL", "ydlidar_scan_frame_timeout"),
            )


class RobotDoctorProcessTests(unittest.TestCase):
    def test_diagnostic_lock_blocks_second_doctor(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            old_lock_dir = os.environ.get("ROBOT_DOCTOR_LOCK_DIR")
            os.environ["ROBOT_DOCTOR_LOCK_DIR"] = tmp
            try:
                args1 = build_parser().parse_args(["agvtest", "--output-root", tmp])
                args2 = build_parser().parse_args(
                    ["agvtest", "--output-root", tmp, "--lock-timeout-seconds", "0"]
                )
                doctor1 = Doctor(args1)
                doctor2 = Doctor(args2)
                self.assertTrue(doctor1.acquire_lock())
                self.assertFalse(doctor2.acquire_lock())
                self.assertEqual(doctor2.results[-1].check, "diagnostic_lock")
            finally:
                try:
                    doctor1.release_lock()
                except Exception:
                    pass
                if old_lock_dir is None:
                    os.environ.pop("ROBOT_DOCTOR_LOCK_DIR", None)
                else:
                    os.environ["ROBOT_DOCTOR_LOCK_DIR"] = old_lock_dir

    def test_timeout_kills_child_process_group(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--output-root", tmp])
            doctor = Doctor(args)
            marker = f"robot_doctor_timeout_child_{os.getpid()}_{time.time_ns()}"
            command = (
                "python3 - <<'PY'\n"
                "import subprocess, time\n"
                f"subprocess.Popen(['python3', '-c', 'import time; time.sleep(30)', '{marker}'])\n"
                "time.sleep(30)\n"
                "PY"
            )
            result = doctor.run("timeout_process_group", command, timeout=1)
            self.assertTrue(result.timed_out)
            time.sleep(0.5)
            ps = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                check=False,
            )
            leftovers = [line for line in ps.stdout.splitlines() if marker in line]
            for line in leftovers:
                pid = line.strip().split(None, 1)[0]
                subprocess.run(["kill", "-9", pid], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.assertEqual(leftovers, [])


class RobotDoctorD455PhysicalEvidenceTests(unittest.TestCase):
    def test_no_d455_physical_failure_does_not_create_swap_check(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.results.append(CheckResult("3.1", "PASS", "disk_free", "ok"))
            doctor.check_d455_physical_swap_evidence()
            self.assertFalse(any(item.check == "d455_physical_swap_evidence" for item in doctor.results))
            self.assertFalse((doctor.out_dir / "operator_d455_swap_checklist.md").exists())

    def test_d455_physical_failure_requests_swap_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.results.append(
                CheckResult("2.1", "FAIL", "realsense_control_query", "control query failed", ["controls.log"])
            )
            doctor.check_d455_physical_swap_evidence()
            matches = [item for item in doctor.results if item.check == "d455_physical_swap_evidence"]
            self.assertEqual(len(matches), 1)
            self.assertEqual((matches[0].code, matches[0].status), ("1.2", "WARN"))
            self.assertIn("camera", matches[0].summary)
            self.assertTrue((doctor.out_dir / "operator_d455_swap_checklist.md").exists())

    def test_d455_physical_failure_is_strict_ops_failure_without_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--strict-ops", "--output-root", tmp])
            doctor = Doctor(args)
            doctor.results.append(CheckResult("2.1", "FAIL", "realsense_stream_transport", "stream timed out"))
            doctor.check_d455_physical_swap_evidence()
            match = [item for item in doctor.results if item.check == "d455_physical_swap_evidence"][0]
            self.assertEqual(match.status, "FAIL")

    def test_d455_swap_confirmations_pass(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(
                [
                    "agvtest",
                    "--output-root",
                    tmp,
                    "--confirm-d455-camera-swap",
                    "--confirm-d455-cable-swap",
                    "--confirm-d455-host-port-swap",
                ]
            )
            doctor = Doctor(args)
            doctor.results.append(CheckResult("1.2", "FAIL", "kernel_usb_disconnect", "disconnect seen"))
            doctor.check_d455_physical_swap_evidence()
            match = [item for item in doctor.results if item.check == "d455_physical_swap_evidence"][0]
            self.assertEqual((match.code, match.status), ("1.2", "PASS"))

    def test_d455_swap_notes_pass_as_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            notes = Path(tmp) / "swap_notes.md"
            notes.write_text("camera/cable/host A/B swap evidence\n")
            args = build_parser().parse_args(["agvtest", "--output-root", tmp, "--d455-swap-notes", str(notes)])
            doctor = Doctor(args)
            doctor.results.append(CheckResult("2.1", "FAIL", "d455_rs_enumerate", "not visible"))
            doctor.check_d455_physical_swap_evidence()
            match = [item for item in doctor.results if item.check == "d455_physical_swap_evidence"][0]
            self.assertEqual(match.status, "PASS")
            self.assertIn(str(notes), match.evidence)


class RobotDoctorReportValidationTests(unittest.TestCase):
    def test_valid_report_passes(self) -> None:
        report = make_report(
            [
                CheckResult("3.1", "PASS", "disk_free", "ok"),
                CheckResult(
                    "1.3",
                    "WARN",
                    "mechanical_operator_check",
                    "not confirmed",
                    next_action="complete mechanical checklist before publishable collection",
                ),
            ]
        )
        ok, errors = validate_report(report)
        self.assertTrue(ok, errors)

    def test_doctor_report_without_config_uses_empty_config_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            args = build_parser().parse_args(["agvtest", "--profile", "static", "--output-root", tmp])
            args.loaded_config = {}
            doctor = Doctor(args)
            doctor.results.append(CheckResult("3.1", "PASS", "disk_free", "ok"))
            summary_json = doctor.write_reports()[0]
            report = json.loads(summary_json.read_text())
            self.assertEqual(report["config_path"], "")
            ok, errors = validate_report(report, summary_json=summary_json)
            self.assertTrue(ok, errors)

    def test_clean_static_report_validates_but_is_not_dataset_ready(self) -> None:
        report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")], profile="static")
        ok, errors = validate_report(report)
        self.assertTrue(ok, errors)
        self.assertTrue(report["can_run_tests"])
        self.assertFalse(report["dataset_ready"])

    def test_bad_count_fails(self) -> None:
        report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")])
        report["counts"]["PASS"] = 99
        ok, errors = validate_report(report)
        self.assertFalse(ok)
        self.assertTrue(any("count mismatch" in error for error in errors))

    def test_bad_decision_fails(self) -> None:
        report = make_report(
            [CheckResult("2.2", "FAIL", "realsense_tools", "missing", next_action="install tools")]
        )
        report["decision"]["state"] = "ready"
        ok, errors = validate_report(report)
        self.assertFalse(ok)
        self.assertTrue(any("decision.state mismatch" in error for error in errors))

    def test_fail_warn_checks_require_next_action(self) -> None:
        report = make_report([CheckResult("2.2", "FAIL", "realsense_tools", "missing")])
        ok, errors = validate_report(report)
        self.assertFalse(ok)
        self.assertTrue(any("next_action" in error for error in errors))

    def test_bad_schema_fails(self) -> None:
        report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")])
        report["schema_version"] = "0.0"
        ok, errors = validate_report(report)
        self.assertFalse(ok)
        self.assertTrue(any("unsupported schema_version" in error for error in errors))

    def test_missing_reproducibility_fields_fail(self) -> None:
        report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")])
        report.pop("effective_gate")
        report["repo_state"] = {}
        ok, errors = validate_report(report)
        self.assertFalse(ok)
        self.assertTrue(any("effective_gate" in error for error in errors))
        self.assertTrue(any("repo_state missing key" in error for error in errors))

    def test_bad_config_hash_fails(self) -> None:
        report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")])
        report["config_path"] = "configs/robot_doctor_dataset_gate.json"
        report["config_sha256"] = "not-a-sha"
        ok, errors = validate_report(report)
        self.assertFalse(ok)
        self.assertTrue(any("config_sha256" in error for error in errors))

    def test_copied_remote_evidence_paths_resolve_under_local_report_root(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            local_report_dir = Path(tmp) / "agv102_20260627_120000"
            log_dir = local_report_dir / "logs"
            log_dir.mkdir(parents=True)
            log = log_dir / "001_system_identity.txt"
            log.write_text("hostname\n")
            summary = local_report_dir / "summary.json"
            remote_root = "/home/ubuntu/agv_data/diagnostics/agv102_20260627_120000"
            remote_evidence = remote_root + "/logs/001_system_identity.txt"
            report = make_report(
                [CheckResult("3.1", "PASS", "disk_free", "ok", evidence=[remote_evidence])]
            )
            report["output_dir"] = remote_root
            summary.write_text(json.dumps(report) + "\n")

            self.assertEqual(
                resolve_evidence_path(remote_evidence, summary_json=summary, report_output_dir=remote_root),
                log.resolve(),
            )
            ok, errors = validate_report(report, check_evidence=True, summary_json=summary)
            self.assertTrue(ok, errors)

    def test_missing_copied_remote_evidence_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            local_report_dir = Path(tmp) / "agv102_20260627_120000"
            local_report_dir.mkdir(parents=True)
            summary = local_report_dir / "summary.json"
            remote_root = "/home/ubuntu/agv_data/diagnostics/agv102_20260627_120000"
            report = make_report(
                [
                    CheckResult(
                        "3.1",
                        "PASS",
                        "disk_free",
                        "ok",
                        evidence=[remote_root + "/logs/missing.txt"],
                    )
                ]
            )
            report["output_dir"] = remote_root
            summary.write_text(json.dumps(report) + "\n")

            ok, errors = validate_report(report, check_evidence=True, summary_json=summary)
            self.assertFalse(ok)
            self.assertTrue(any("evidence path" in error for error in errors))

    def test_synthesized_remote_failure_report_validates(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "failed_remote"
            (out / "logs").mkdir(parents=True)
            (out / "logs" / "001_example.txt").write_text("remote failure evidence\n")
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts/diagnostics/synthesize_robot_doctor_failure.py"),
                    "--robot-id",
                    "agvtest",
                    "--output-dir",
                    str(out),
                    "--remote-report",
                    "/home/ubuntu/agv_data/diagnostics/agvtest_failed",
                    "--remote-rc",
                    "255",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout)
            report = json.loads((out / "summary.json").read_text())
            ok, errors = validate_report(report)
            self.assertTrue(ok, errors)
            self.assertEqual(report["decision"]["primary_blocker"]["check"], "remote_ssh_interrupted")
            self.assertEqual(report["decision"]["primary_blocker"]["code"], "3.3")

    def test_synthesized_non_ssh_remote_failure_report_validates(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "failed_remote"
            (out / "logs").mkdir(parents=True)
            (out / "logs" / "001_example.txt").write_text("remote failure evidence\n")
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts/diagnostics/synthesize_robot_doctor_failure.py"),
                    "--robot-id",
                    "agvtest",
                    "--output-dir",
                    str(out),
                    "--remote-report",
                    "/home/ubuntu/agv_data/diagnostics/agvtest_failed",
                    "--remote-rc",
                    "1",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout)
            report = json.loads((out / "summary.json").read_text())
            ok, errors = validate_report(report)
            self.assertTrue(ok, errors)
            self.assertEqual(report["decision"]["primary_blocker"]["check"], "robot_doctor_execution")
            self.assertEqual(report["decision"]["primary_blocker"]["code"], "3.2")


class DatasetRunAuditTests(unittest.TestCase):
    def test_dataset_run_audit_reports_missing_fail(self) -> None:
        items = audit_reports([], require_ready=True, require_configured_gate=True)
        self.assertEqual(len(items), 1)
        self.assertEqual((items[0].status, items[0].check), ("FAIL", "reports_present"))

    def test_dataset_run_audit_accepts_ready_report(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            report_dir = Path(tmp) / "report"
            report_dir.mkdir()
            report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")], profile="dataset")
            report["output_dir"] = str(report_dir)
            (report_dir / "summary.json").write_text(json.dumps(report) + "\n")
            items = audit_reports([report_dir], require_ready=True, require_configured_gate=False)
            self.assertEqual([item for item in items if item.status == "FAIL"], [])
            self.assertTrue(any(item.check == "report_dataset_ready" for item in items))

    def test_dataset_run_audit_manifest_complete(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag = root / "run_0.db3"
            bag.write_bytes(b"sqlite placeholder")
            manifest = root / "run_manifest.yaml"
            manifest.write_text(
                "\n".join(
                    [
                        "session_id: run",
                        "robot_id: agv100",
                        "scenario: square",
                        "date: 2026-06-27",
                        "time_start: '10:00:00'",
                        "time_end: '10:01:00'",
                        "bag_file: run_0.db3",
                        "duration_sec: 60",
                        "bag_size_mb: 1",
                    ]
                )
            )
            items = audit_manifests([manifest], require_manifest=True)
            self.assertFalse([item for item in items if item.status == "FAIL"])
            self.assertTrue(any(item.check == "manifest_complete" for item in items))

    def test_dataset_run_audit_rejects_report_manifest_robot_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            report_dir = root / "report"
            report_dir.mkdir()
            report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")], profile="dataset")
            report["robot_id"] = "agv100"
            report["output_dir"] = str(report_dir)
            summary = report_dir / "summary.json"
            summary.write_text(json.dumps(report) + "\n")

            bag = root / "agv101_square_20260627_120000"
            bag.mkdir()
            manifest = root / "agv101_square_20260627_120000_manifest.yaml"
            manifest.write_text(
                "\n".join(
                    [
                        "session_id: agv101_square_20260627_120000",
                        "robot_id: agv101",
                        "scenario: square",
                        "date: 2026-06-27",
                        "time_start: '12:00:00'",
                        "time_end: '12:01:00'",
                        "bag_file: agv101_square_20260627_120000",
                        "duration_sec: 60",
                        "bag_size_mb: 1",
                    ]
                )
            )
            items = audit_artifact_consistency([summary], [bag], [manifest])
            failures = [item for item in items if item.status == "FAIL"]
            self.assertTrue(any(item.check == "robot_artifact_match" for item in failures), items)

    def test_dataset_run_audit_rejects_unmatched_bag_and_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag = root / "agv100_other_20260627_120000"
            bag.mkdir()
            manifest = root / "agv100_square_20260627_120000_manifest.yaml"
            manifest.write_text(
                "\n".join(
                    [
                        "session_id: agv100_square_20260627_120000",
                        "robot_id: agv100",
                        "scenario: square",
                        "date: 2026-06-27",
                        "time_start: '12:00:00'",
                        "time_end: '12:01:00'",
                        "bag_file: agv100_square_20260627_120000",
                        "duration_sec: 60",
                        "bag_size_mb: 1",
                    ]
                )
            )
            items = audit_artifact_consistency([], [bag], [manifest])
            failures = [item for item in items if item.status == "FAIL"]
            self.assertTrue(any(item.check == "manifest_bag_supplied" for item in failures), items)
            self.assertTrue(any(item.check == "bag_manifest_match" for item in failures), items)


class FleetDoctorSummaryTests(unittest.TestCase):
    def test_shell_wrappers_are_syntax_valid(self) -> None:
        scripts = [
            ROOT / "scripts/diagnostics/apply_robot_doctor_fix.sh",
            ROOT / "scripts/diagnostics/robot_doctor.sh",
            ROOT / "scripts/diagnostics/run_robot_doctor_remote.sh",
            ROOT / "scripts/diagnostics/run_fleet_doctor_remote.sh",
            ROOT / "scripts/setup_robot_ros2.sh",
            ROOT / "scripts/logging/start_session.sh",
        ]
        for script in scripts:
            with self.subTest(script=str(script)):
                proc = subprocess.run(
                    ["bash", "-n", str(script)],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                self.assertEqual(proc.returncode, 0, proc.stdout)

    def test_diagnostic_pipeline_acceptance_audit_passes(self) -> None:
        proc = subprocess.run(
            [sys.executable, str(ROOT / "scripts/diagnostics/diagnostic_pipeline_audit.py")],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.assertEqual(proc.returncode, 0, proc.stdout)
        self.assertIn("failure_tree_codes", proc.stdout)

    def test_dataset_run_audit_passes_synthetic_ros2_collection(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            bag_dir = tmp_path / "agv100_synthetic_20260627_120000"
            write_ros2_bag(bag_dir, duration_sec=5.0)

            report_dir = tmp_path / "report"
            report_dir.mkdir()
            report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")], profile="dataset")
            report["robot_id"] = "agv100"
            report["loaded_config"] = {"gate_id": "test_gate", "gate_version": "1.0.0"}
            report["config_sha256"] = "a" * 64
            report["output_dir"] = str(report_dir)
            summary = report_dir / "summary.json"
            summary.write_text(json.dumps(report) + "\n")

            manifest = tmp_path / "agv100_synthetic_20260627_120000_manifest.yaml"
            manifest.write_text(
                "\n".join(
                    [
                        "session_id: agv100_synthetic_20260627_120000",
                        "robot_id: agv100",
                        "scenario: synthetic",
                        "date: 2026-06-27",
                        "time_start: 12:00:00",
                        "time_end: 12:00:05",
                        f"bag_file: {bag_dir.name}",
                        "duration_sec: 5",
                        "bag_size_mb: 1",
                        "",
                    ]
                )
            )
            json_out = tmp_path / "dataset_audit.json"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts/diagnostics/dataset_run_audit.py"),
                    "--report",
                    str(summary),
                    "--bag",
                    str(bag_dir),
                    "--manifest",
                    str(manifest),
                    "--min-duration",
                    "4",
                    "--mocap-topic",
                    "/optitrack/rigid_bodies/agv",
                    "--require-gt",
                    "--require-imu",
                    "--json-out",
                    str(json_out),
                    "--output-dir",
                    str(tmp_path / "audit_logs"),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout)
            audit = json.loads(json_out.read_text())
            self.assertIn(audit["verdict"], {"PASS", "WARN"})
            self.assertFalse(any(item["status"] == "FAIL" for item in audit["items"]))

    def test_dataset_run_audit_manifest_complete(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            report_dir = tmp_path / "report"
            report_dir.mkdir()
            report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")], profile="dataset")
            report["loaded_config"] = {"gate_id": "test_gate", "gate_version": "1.0.0"}
            report["config_sha256"] = "a" * 64
            report["output_dir"] = str(report_dir)
            summary = report_dir / "summary.json"
            summary.write_text(json.dumps(report) + "\n")

            manifest = tmp_path / "bad_manifest.yaml"
            manifest.write_text(
                "\n".join(
                    [
                        "session_id: agv100_bad",
                        "robot_id: agv100",
                        "scenario: synthetic",
                        "date: 2026-06-27",
                        "time_start: 12:00:00",
                        "time_end: ~",
                        "bag_file: missing_run",
                        "duration_sec: ~",
                        "bag_size_mb: ~",
                        "",
                    ]
                )
            )
            proc = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts/diagnostics/dataset_run_audit.py"),
                    "--report",
                    str(summary),
                    "--manifest",
                    str(manifest),
                    "--no-require-bag",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self.assertEqual(proc.returncode, 1, proc.stdout)
            self.assertIn("manifest_complete", proc.stdout)

    def test_dataset_run_audit_reports_missing_fail(self) -> None:
        proc = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/diagnostics/dataset_run_audit.py"),
                "--no-require-bag",
                "--no-require-manifest",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.assertEqual(proc.returncode, 1, proc.stdout)
        self.assertIn("reports_present", proc.stdout)

    def test_fleet_gate_errors_detect_mixed_config(self) -> None:
        rows = [
            {
                "robot_id": "agv100",
                "gate_id": "gate",
                "gate_version": "1",
                "config_sha": "aaa",
                "repo_commit": "111",
                "repo_dirty": "false",
            },
            {
                "robot_id": "agv101",
                "gate_id": "gate",
                "gate_version": "1",
                "config_sha": "bbb",
                "repo_commit": "111",
                "repo_dirty": "false",
            },
        ]
        args = type(
            "Args",
            (),
            {
                "require_same_gate": True,
                "require_same_config": True,
                "require_same_commit": True,
                "require_clean_repo": True,
                "require_configured_gate": True,
            },
        )()
        errors = fleet_gate_errors(rows, args)
        self.assertTrue(any("config_sha" in error for error in errors))
        self.assertFalse(any("gate_id" in error for error in errors))

    def test_fleet_gate_errors_detect_dirty_repo(self) -> None:
        rows = [
            {
                "robot_id": "agv100",
                "gate_id": "gate",
                "gate_version": "1",
                "config_sha": "aaa",
                "repo_commit": "111",
                "repo_dirty": "true",
            }
        ]
        args = type(
            "Args",
            (),
            {
                "require_same_gate": False,
                "require_same_config": False,
                "require_same_commit": False,
                "require_clean_repo": True,
                "require_configured_gate": False,
            },
        )()
        errors = fleet_gate_errors(rows, args)
        self.assertTrue(any("agv100" in error for error in errors))

    def test_fleet_gate_errors_detect_missing_configured_gate(self) -> None:
        rows = [
            {
                "robot_id": "agv100",
                "gate_id": "-",
                "gate_version": "-",
                "config_sha": "-",
                "repo_commit": "111",
                "repo_dirty": "false",
            }
        ]
        args = type(
            "Args",
            (),
            {
                "require_same_gate": False,
                "require_same_config": False,
                "require_same_commit": False,
                "require_clean_repo": False,
                "require_configured_gate": True,
            },
        )()
        errors = fleet_gate_errors(rows, args)
        self.assertTrue(any("configured gate" in error and "agv100" in error for error in errors))

    def test_fleet_readiness_errors_detect_not_ready(self) -> None:
        rows = [
            {
                "robot_id": "agv100",
                "report_ok": True,
                "fail": 0,
                "dataset_ready": False,
            }
        ]
        args = type("Args", (), {"require_dataset_ready": True})()
        errors = fleet_readiness_errors(rows, args)
        self.assertTrue(any("dataset_ready" in error and "agv100" in error for error in errors))

    def test_run_fleet_wrapper_passes_doctor_args_once(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            hosts = tmp_path / "hosts.txt"
            hosts.write_text("agv100 127.0.0.1\n")
            output = tmp_path / "out"
            fake_report = tmp_path / "summary_template.json"
            report = make_report([CheckResult("3.1", "PASS", "disk_free", "ok")])
            report["loaded_config"] = {"gate_id": "test_gate", "gate_version": "1.0.0"}
            report["config_sha256"] = "a" * 64
            fake_report.write_text(json.dumps(report) + "\n")
            fake_remote = tmp_path / "fake_remote.sh"
            fake_remote.write_text(
                """#!/usr/bin/env bash
set -euo pipefail
robot="$1"
shift
out="${LOCAL_OUTPUT_ROOT}/fake_${robot}"
mkdir -p "${out}"
printf '%s\n' "$@" > "${LOCAL_OUTPUT_ROOT}/remote_args.txt"
cp "${FAKE_REPORT}" "${out}/summary.json"
"""
            )
            fake_remote.chmod(0o755)
            env = os.environ.copy()
            env["FLEET_REMOTE_WRAPPER"] = str(fake_remote)
            env["LOCAL_OUTPUT_ROOT"] = str(output)
            env["FAKE_REPORT"] = str(fake_report)
            proc = subprocess.run(
                [
                    "bash",
                    str(ROOT / "scripts/diagnostics/run_fleet_doctor_remote.sh"),
                    str(hosts),
                    "--strict-fleet",
                    "--",
                    "--config",
                    "configs/robot_doctor_dataset_gate.json",
                    "--profile",
                    "preflight",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
            )
            self.assertEqual(proc.returncode, 0, proc.stdout)
            args_seen = (output / "remote_args.txt").read_text().splitlines()
            self.assertEqual(
                args_seen,
                [
                    "127.0.0.1",
                    "--",
                    "--config",
                    "configs/robot_doctor_dataset_gate.json",
                    "--profile",
                    "preflight",
                ],
            )
            self.assertTrue((output / "fleet_summary.json").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
