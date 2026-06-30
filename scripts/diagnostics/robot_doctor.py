#!/usr/bin/env python3
"""Unified AGV diagnostic pipeline mapped to robot_failure_modes_v3.png.

Run this on a robot before dataset collection, after a failed run, or against a
bag. It writes one evidence directory containing command logs, a JSON report,
and a short human summary. Every check is tagged with the failure-tree code so
the next action is driven by evidence instead of guessing.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"
INFO = "INFO"
REPORT_SCHEMA_VERSION = "1.0"
ROBOT_DOCTOR_VERSION = "1.0.0"


FAILURE_TREE: Dict[str, Dict[str, str]] = {
    "1.1": {
        "name": "Sensor device health",
        "parent": "1 Robot platform",
        "examples": "D455/LiDAR/IMU/base MCU identity, firmware, calibration",
    },
    "1.2": {
        "name": "Physical infrastructure",
        "parent": "1 Robot platform",
        "examples": "USB cables/ports, power, brownout, powered hub, charging state",
    },
    "1.3": {
        "name": "Mechanical setup",
        "parent": "1 Robot platform",
        "examples": "Sensor mounts, MoCap markers, wheels, chassis, slip/alignment",
    },
    "2.1": {
        "name": "OS / kernel / USB",
        "parent": "2 Robot data stack",
        "examples": "USB speed, enumeration, UVC/xHCI errors, /dev permissions",
    },
    "2.2": {
        "name": "Drivers / launch config",
        "parent": "2 Robot data stack",
        "examples": "librealsense, ROS drivers, FPS/res/alignment, remaps, serial ports",
    },
    "2.3": {
        "name": "ROS data quality",
        "parent": "2 Robot data stack",
        "examples": "Topics/types/QoS, rates, gaps, TF/frame IDs, timestamps",
    },
    "3.1": {
        "name": "Recording pipeline",
        "parent": "3 Experiment dataset",
        "examples": "Bag start/stop, topic inclusion, disk, corruption, recorder crash",
    },
    "3.2": {
        "name": "Validation pipeline",
        "parent": "3 Experiment dataset",
        "examples": "Pre-run gate, runtime watchdog, post-run validator, metadata",
    },
    "3.3": {
        "name": "Experiment execution",
        "parent": "3 Experiment dataset",
        "examples": "MoCap/GT/anchors, network/sync, scenario path, operator error",
    },
}


DEFAULT_TOPIC_SPECS = {
    "/scan": {"min_hz": 5.0, "target_hz": 18.0},
    "/odom": {"min_hz": 12.0, "target_hz": 20.0},
    "/tf": {"min_hz": 10.0, "target_hz": 50.0},
    "/camera/color/image_raw": {"min_hz": 12.0, "target_hz": 15.0},
    "/camera/color/camera_info": {"min_hz": 0.0, "target_hz": 0.0},
    "/camera/aligned_depth_to_color/image_raw": {"min_hz": 12.0, "target_hz": 15.0},
    "/camera/aligned_depth_to_color/camera_info": {"min_hz": 0.0, "target_hz": 0.0},
}


UVC_PATTERNS = [
    r"Failed to query.*UVC",
    r"UVCIOC_CTRL_QUERY",
    r"uvcvideo.*(?:timeout|error|-110)",
    r"Frames didn't arrive",
    r"set_xu.*failed",
]

XHCI_PATTERNS = [
    r"xhci.*(?:timeout|error|reset|stopped)",
    r"xHCI not responding to stop endpoint command",
    r"CLEAR_HALT for active endpoint",
    r"usb .*device descriptor read.*error",
]

USB_RESET_EVENT_PATTERNS = [
    r"usb .*reset (?:SuperSpeed|high-speed|full-speed|low-speed) USB device",
]

USB_DISCONNECT_PATTERNS = [
    r"USB disconnect",
    r"device not accepting address",
]

USB_AUTOSUSPEND_PATTERNS = [
    r"tegra-xusb.*entering ELPG",
    r"usb_suspend_both.*status 0",
]

USB2_FALLBACK_PATTERNS = [
    r"new high-speed USB device",
]

USB_OVERCURRENT_PATTERNS = [
    r"System throttled due to over-current",
    r"over-current",
    r"overcurrent",
]

D455_PHYSICAL_FAILURE_CHECKS = {
    "d455_enumeration",
    "d455_imu_hid",
    "d455_usb_speed",
    "d455_usb_autosuspend",
    "d455_usb_autosuspend_delay",
    "d455_uvc_binding",
    "d455_rs_enumerate",
    "kernel_uvc_errors",
    "kernel_xhci_errors",
    "kernel_usb_autosuspend_elpg",
    "kernel_usb_overcurrent",
    "kernel_usb_disconnect",
    "realsense_control_query",
    "realsense_color_stream",
    "realsense_depth_stream",
    "realsense_motion_stream_gate",
    "realsense_motion_stream_isolation",
    "realsense_stream_exception",
    "realsense_stream_no_frames",
    "realsense_stream_timeouts",
    "realsense_stream_transport",
}


CONFIG_FLAG_MAP = {
    "profile": ["--profile"],
    "output_root": ["--output-root"],
    "ros": ["--ros"],
    "no_ros": ["--no-ros"],
    "bringup_cmd": ["--bringup-cmd"],
    "bringup_wait": ["--bringup-wait"],
    "live_seconds": ["--live-seconds"],
    "bag": ["--bag"],
    "require_bag": ["--require-bag"],
    "bag_validation_timeout": ["--bag-validation-timeout"],
    "require_gt": ["--require-gt"],
    "require_imu": ["--require-imu"],
    "mocap_topic": ["--mocap-topic"],
    "cmd_topic": ["--cmd-topic"],
    "required_topic": ["--required-topic"],
    "expect_native_ros2": ["--expect-native-ros2"],
    "expected_robot_namespace": ["--expected-robot-namespace"],
    "require_odom_mocap_sanity": ["--require-odom-mocap-sanity"],
    "odom_mocap_sanity_json": ["--odom-mocap-sanity-json"],
    "odom_mocap_max_error_ratio": ["--odom-mocap-max-error-ratio"],
    "require_resilient_storage": ["--require-resilient-storage"],
    "min_free_gb": ["--min-free-gb"],
    "expect_camera": ["--expect-camera", "--no-expect-camera"],
    "expected_d455_serial": ["--expected-d455-serial"],
    "expected_d455_firmware": ["--expected-d455-firmware"],
    "expected_librealsense": ["--expected-librealsense"],
    "expected_realsense_ros_driver": ["--expected-realsense-ros-driver"],
    "expected_realsense_ros_librealsense": ["--expected-realsense-ros-librealsense"],
    "strict_versions": ["--strict-versions"],
    "stream_test_seconds": ["--stream-test-seconds"],
    "stream_test_motion": ["--stream-test-motion"],
    "d455_motion_test_seconds": ["--d455-motion-test-seconds"],
    "camera_width": ["--camera-width"],
    "camera_height": ["--camera-height"],
    "camera_fps": ["--camera-fps"],
    "max_clock_offset_ms": ["--max-clock-offset-ms"],
    "strict_ops": ["--strict-ops"],
    "confirm_mechanical": ["--confirm-mechanical"],
    "confirm_mocap": ["--confirm-mocap"],
    "confirm_anchors": ["--confirm-anchors"],
    "confirm_d455_camera_swap": ["--confirm-d455-camera-swap"],
    "confirm_d455_cable_swap": ["--confirm-d455-cable-swap"],
    "confirm_d455_host_port_swap": ["--confirm-d455-host-port-swap"],
    "d455_swap_notes": ["--d455-swap-notes"],
    "lock_timeout_seconds": ["--lock-timeout-seconds"],
}


@dataclass
class CommandResult:
    label: str
    command: str
    rc: int
    elapsed_sec: float
    timed_out: bool
    log: str


@dataclass
class CheckResult:
    code: str
    status: str
    check: str
    summary: str
    evidence: List[str] = field(default_factory=list)
    next_action: str = ""


ROOT_CAUSE_CHECK_PRIORITY = {
    "ydlidar_uart_alias": 0,
    "ydlidar_scan_frame_timeout": 1,
    "ydlidar_serial_bind": 2,
    "ydlidar_device_health": 3,
    "realsense_stream_transport": 10,
    "realsense_stream_no_frames": 11,
    "realsense_control_query": 12,
    "kernel_usb_disconnect": 13,
    "kernel_usb_overcurrent": 14,
    "kernel_xhci_errors": 15,
    "kernel_uvc_errors": 16,
}

DOWNSTREAM_SYMPTOM_CHECKS = {
    "dataset_bringup_context",
    "bringup_wait",
    "topic_present",
    "topic_rate",
}


def primary_blocker(results: Sequence[CheckResult]) -> Optional[CheckResult]:
    if not results:
        return None

    def sort_key(indexed: Tuple[int, CheckResult]) -> Tuple[int, int, int]:
        index, item = indexed
        if item.check in ROOT_CAUSE_CHECK_PRIORITY:
            return (0, ROOT_CAUSE_CHECK_PRIORITY[item.check], index)
        if item.check in DOWNSTREAM_SYMPTOM_CHECKS:
            return (2, 0, index)
        return (1, 0, index)

    return min(enumerate(results), key=sort_key)[1]


def summarize_decision(results: Sequence[CheckResult], profile: str = "dataset") -> Dict[str, object]:
    hard_failures = [item for item in results if item.status == FAIL]
    warnings = [item for item in results if item.status == WARN]
    blockers = hard_failures if hard_failures else warnings

    if hard_failures:
        state = "blocked"
        verdict = "FAIL"
        can_run_tests = False
        dataset_ready = False
        primary = primary_blocker(hard_failures)
        recommendation = primary.next_action or "Fix the first hard failure and rerun robot_doctor."
    elif warnings:
        state = "review"
        verdict = "WARN"
        can_run_tests = True
        dataset_ready = False
        primary = primary_blocker(warnings)
        recommendation = primary.next_action or "Review warnings and rerun robot_doctor before publishable collection."
    else:
        state = "ready"
        verdict = "PASS"
        can_run_tests = True
        primary = None
        if profile == "dataset":
            dataset_ready = True
            recommendation = "Robot meets the configured dataset diagnostic gate."
        else:
            dataset_ready = False
            recommendation = f"{profile} diagnostic passed; run the dataset profile before publishable collection."

    return {
        "state": state,
        "verdict": verdict,
        "can_run_tests": can_run_tests,
        "dataset_ready": dataset_ready,
        "primary_blocker": asdict(primary) if primary else None,
        "blockers": [asdict(item) for item in blockers],
        "recommendation": recommendation,
    }


def format_operator_decision(report: Dict[str, object]) -> str:
    decision = report.get("decision", {}) if isinstance(report.get("decision"), dict) else {}
    primary = decision.get("primary_blocker") if isinstance(decision, dict) else None
    dataset_ready = bool(report.get("dataset_ready", False))

    if isinstance(primary, dict):
        code = str(primary.get("code", "unknown"))
        stage_name = FAILURE_TREE.get(code, {}).get("name", "unknown")
        failed_stage = f"{code} {stage_name}"
        cause = str(primary.get("summary", "") or "unknown")
        evidence = primary.get("evidence", [])
        next_action = str(primary.get("next_action", "") or "rerun robot_doctor after fixing the failed stage")
    else:
        failed_stage = "none"
        cause = str(decision.get("recommendation", "no blockers")) if isinstance(decision, dict) else "no blockers"
        evidence = []
        next_action = "ready for the configured gate" if dataset_ready else "run the dataset profile before publishable collection"

    lines = [
        f"READY: {str(dataset_ready).lower()}",
        f"FAILED_STAGE: {failed_stage}",
        f"CAUSE: {cause}",
        "EVIDENCE:",
    ]
    if isinstance(evidence, list) and evidence:
        lines.extend(f"  - {item}" for item in evidence)
    else:
        lines.append("  - none")
    lines.append(f"NEXT_ACTION: {next_action}")
    return "\n".join(lines)


def cli_supplied_flags(argv: Sequence[str]) -> set:
    flags = set()
    for arg in argv:
        if not arg.startswith("--"):
            continue
        flags.add(arg.split("=", 1)[0])
    return flags


def load_gate_config(config_path: str) -> Dict[str, object]:
    if not config_path:
        return {}
    path = Path(config_path).expanduser()
    with path.open("r") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"diagnostic config must be a JSON object: {path}")
    return data


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def git_info(root: Path) -> Dict[str, str]:
    def git_cmd(command: str) -> str:
        try:
            out = subprocess.check_output(
                ["bash", "-lc", f"git -C {shell_quote(root)} {command}"],
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=4,
            )
            return out.strip()
        except Exception:
            return "unavailable"

    status = git_cmd("status --short")
    return {
        "branch": git_cmd("branch --show-current"),
        "commit": git_cmd("rev-parse --short HEAD"),
        "dirty": str(bool(status and status != "unavailable")).lower(),
        "status_short": status,
    }


def normalize_config_key(key: str) -> str:
    return key.replace("-", "_")


def apply_gate_config(args: argparse.Namespace, argv: Sequence[str]) -> Dict[str, object]:
    config = load_gate_config(getattr(args, "config", ""))
    if not config:
        return {}

    supplied = cli_supplied_flags(argv)
    for raw_key, value in config.items():
        key = normalize_config_key(raw_key)
        if key not in CONFIG_FLAG_MAP:
            continue
        if any(flag in supplied for flag in CONFIG_FLAG_MAP[key]):
            continue
        if key in {"required_topic", "expected_robot_namespace"}:
            if isinstance(value, str):
                setattr(args, key, split_topics(value))
            elif isinstance(value, list):
                setattr(args, key, [str(item) for item in value])
            continue
        setattr(args, key, value)
    return config


def effective_gate_config(args: argparse.Namespace) -> Dict[str, object]:
    keys = sorted(CONFIG_FLAG_MAP)
    return {key: getattr(args, key) for key in keys if hasattr(args, key)}


def slug(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("_")
    return value[:80] or "command"


def shell_quote(path: Path) -> str:
    return "'" + str(path).replace("'", "'\"'\"'") + "'"


def split_topics(value: str) -> List[str]:
    return [item.strip() for item in value.replace(",", " ").split() if item.strip()]


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


class Doctor:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.root = Path(__file__).resolve().parents[2]
        self.start_epoch = int(time.time())
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = Path(args.output_root).expanduser() / f"{args.robot_id}_{self.timestamp}"
        self.log_dir = self.out_dir / "logs"
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.results: List[CheckResult] = []
        self.commands: List[CommandResult] = []
        self.ros_mode = "none"
        self.ros_setup_prefix = ""
        self.topic_types: Dict[str, str] = {}
        self.bringup_process: Optional[subprocess.Popen] = None
        self.bringup_log: Optional[Path] = None
        self.lock_handle = None

    def add(
        self,
        code: str,
        status: str,
        check: str,
        summary: str,
        evidence: Optional[Sequence[str]] = None,
        next_action: str = "",
    ) -> None:
        self.results.append(
            CheckResult(
                code=code,
                status=status,
                check=check,
                summary=summary,
                evidence=list(evidence or []),
                next_action=next_action,
            )
        )
        marker = {PASS: "OK", WARN: "WARN", FAIL: "FAIL", INFO: "INFO"}[status]
        print(f"[{marker}] {code} {check}: {summary}")
        if next_action and status in {FAIL, WARN}:
            print(f"      next: {next_action}")

    def run(self, label: str, command: str, timeout: int = 15) -> CommandResult:
        start = time.time()
        log_path = self.log_dir / f"{len(self.commands) + 1:03d}_{slug(label)}.txt"
        timed_out = False
        proc = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=str(self.root),
            preexec_fn=os.setsid,
        )
        try:
            output, _ = proc.communicate(timeout=timeout)
            rc = proc.returncode
        except subprocess.TimeoutExpired as exc:
            timed_out = True
            rc = 124
            output = exc.stdout or ""
            if isinstance(output, bytes):
                output = output.decode("utf-8", errors="replace")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass
            try:
                tail, _ = proc.communicate(timeout=3)
                output += tail or ""
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                tail, _ = proc.communicate()
                output += tail or ""
            output += f"\nTIMEOUT after {timeout}s\n"
        elapsed = time.time() - start
        log_path.write_text(f"$ {command}\n\n{output}", errors="replace")
        result = CommandResult(label, command, rc, elapsed, timed_out, str(log_path))
        self.commands.append(result)
        return result

    def acquire_lock(self) -> bool:
        if getattr(self.args, "no_lock", False):
            return True
        lock_name = slug(self.args.robot_id)
        lock_path = Path(os.environ.get("ROBOT_DOCTOR_LOCK_DIR", "/tmp")) / f"robot_doctor_{lock_name}.lock"
        self.lock_handle = lock_path.open("w")
        deadline = time.time() + float(getattr(self.args, "lock_timeout_seconds", 5))
        while True:
            try:
                fcntl.flock(self.lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.lock_handle.seek(0)
                self.lock_handle.truncate()
                self.lock_handle.write(f"pid={os.getpid()} robot_id={self.args.robot_id} out_dir={self.out_dir}\n")
                self.lock_handle.flush()
                return True
            except BlockingIOError:
                if time.time() >= deadline:
                    self.add(
                        "3.2",
                        FAIL,
                        "diagnostic_lock",
                        f"another robot_doctor process holds {lock_path}",
                        next_action="wait for the existing diagnostic to finish or kill the stale robot_doctor process before rerunning",
                    )
                    try:
                        self.lock_handle.close()
                    except Exception:
                        pass
                    self.lock_handle = None
                    return False
                time.sleep(0.25)

    def release_lock(self) -> None:
        if self.lock_handle is None:
            return
        try:
            fcntl.flock(self.lock_handle.fileno(), fcntl.LOCK_UN)
        except Exception:
            pass
        try:
            self.lock_handle.close()
        except Exception:
            pass
        self.lock_handle = None

    def ros_cmd(self, inner: str, timeout: int = 20, label: Optional[str] = None) -> CommandResult:
        command = f"{self.ros_setup_prefix}{inner}"
        return self.run(label or inner.split()[0], command, timeout=timeout)

    def start_bringup(self) -> None:
        if not self.args.bringup_cmd:
            return
        self.bringup_log = self.log_dir / "bringup_command.log"
        command = f"{self.ros_setup_prefix}{self.args.bringup_cmd}"
        self.bringup_log.write_text(f"$ {command}\n\n")
        log_handle = self.bringup_log.open("a")
        self.bringup_process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            cwd=str(self.root),
            preexec_fn=os.setsid,
            text=True,
        )
        self.add(
            "2.2",
            INFO,
            "bringup_start",
            f"started bringup command pid={self.bringup_process.pid}",
            [str(self.bringup_log)],
        )
        self.wait_for_bringup_topics()

    def wait_for_bringup_topics(self) -> None:
        deadline = time.time() + max(0, int(self.args.bringup_wait))
        if self.args.profile == "dataset":
            required = [
                topic
                for topic, spec in self.live_topic_specs().items()
                if spec.get("min_hz", 0.0) > 0
            ]
        else:
            required = ["/scan", "/odom", "/tf"]
        required.extend(topic for topic in self.args.required_topic if topic not in required)
        required = list(dict.fromkeys(required))
        last_seen: Dict[str, str] = {}
        while time.time() < deadline:
            result = self.ros_cmd(
                "(ros2 topic list -t --no-daemon --spin-time 2 2>&1 || ros2 topic list -t 2>&1)",
                timeout=8,
                label="bringup_wait_topic_list",
            )
            last_seen = self.parse_topic_list(self.command_output(result))
            missing = [topic for topic in required if topic not in last_seen]
            if not missing:
                self.add(
                    "2.2",
                    PASS,
                    "bringup_wait",
                    f"required bringup topics visible: {', '.join(required)}",
                    [result.log],
                )
                return
            time.sleep(2.0)
        missing = [topic for topic in required if topic not in last_seen]
        self.add(
            "2.2",
            WARN,
            "bringup_wait",
            f"bringup wait reached {self.args.bringup_wait}s; still missing: {', '.join(missing)}",
            next_action="inspect bringup_command.log if required topics are still missing in later ROS graph checks",
        )

    def stop_bringup(self) -> None:
        if self.bringup_process is None:
            return
        if self.bringup_process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.bringup_process.pid), signal.SIGINT)
            self.bringup_process.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(self.bringup_process.pid), signal.SIGTERM)
            except Exception:
                pass

    def command_output(self, result: CommandResult) -> str:
        try:
            text = Path(result.log).read_text(errors="replace")
        except Exception:
            return ""
        return text.split("\n\n", 1)[-1]

    def detect_ros(self) -> None:
        requested = self.args.ros
        if requested != "auto":
            self.ros_mode = requested
        else:
            distro = os.environ.get("ROS_DISTRO", "")
            if distro in {"humble", "foxy", "galactic", "iron", "jazzy", "rolling"}:
                self.ros_mode = "ros2"
            elif shutil.which("ros2") or Path("/opt/ros/humble/setup.bash").exists():
                self.ros_mode = "ros2"
            else:
                self.ros_mode = "none"

        parts: List[str] = []
        if self.ros_mode == "ros2":
            for setup in [
                os.environ.get("ROS_SETUP", ""),
                "/opt/ros/humble/setup.bash",
                "/opt/ros/jazzy/setup.bash",
                str(Path.home() / "slam_project/install/setup.bash"),
                str(Path.home() / "slam_project/agv2_ws/install/setup.bash"),
                str(self.root / "install/setup.bash"),
                str(self.root / "agv2_ws/install/setup.bash"),
            ]:
                if setup and Path(setup).exists():
                    parts.append(f"source {shell_quote(Path(setup))}; ")
        self.ros_setup_prefix = "".join(dict.fromkeys(parts))
        self.add(
            "2.2",
            INFO,
            "ros_environment",
            f"detected {self.ros_mode}",
            [],
        )

    def inventory(self) -> None:
        for label, command, timeout in [
            (
                "system_identity",
                "hostname; hostname -I; date --iso-8601=seconds; uname -a; "
                "sed -n '1,12p' /etc/os-release 2>/dev/null || true; "
                "echo ROS_DISTRO=${ROS_DISTRO:-unset}; echo ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}",
                8,
            ),
            ("process_snapshot", "ps -eo pid,ppid,stat,comm,args --sort=comm | sed -n '1,180p'", 8),
            ("memory_cpu", "uptime; free -h; lscpu 2>/dev/null | sed -n '1,40p'", 8),
            ("disk", "df -h / /home 2>/dev/null || df -h", 8),
            ("network", "ip -br addr; ip route; nmcli dev status 2>/dev/null || true; iw dev wlan0 link 2>/dev/null || true", 10),
            ("time_sync", "timedatectl status --no-pager 2>/dev/null || true; chronyc tracking 2>/dev/null || true; chronyc sources -v 2>/dev/null || true", 10),
        ]:
            self.run(label, command, timeout=timeout)
        self.check_disk()
        self.check_clock()
        self.check_network()
        self.check_wifi_management()
        self.check_native_ros2_stack()

    def check_native_ros2_stack(self) -> None:
        result = self.run(
            "native_ros2_stack",
            "echo ENV; "
            "printenv | grep -E '^(ROS_DISTRO|ROS_DOMAIN_ID|ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME)=' || true; "
            "echo COMMANDS; command -v ros2 || true; "
            "echo PROCESSES; "
            "ps -eo pid=,comm=,args= | awk '$2 !~ /^(bash|sh|awk|grep|pgrep)$/ && "
            "$0 ~ /(roscore|rosmaster|ros1_bridge|dynamic_bridge|parameter_bridge)/ {print}' || true; "
            "echo PACKAGES; dpkg -l 2>/dev/null | grep -E 'ros-.*ros1-bridge|ros1_bridge' || true",
            timeout=10,
        )
        code, status, check, summary, next_action = self.classify_native_ros2_stack(
            self.command_output(result),
            self.ros_mode,
            bool(self.args.expect_native_ros2),
            self.args.profile,
        )
        self.add(code, status, check, summary, [result.log], next_action)

    @staticmethod
    def classify_native_ros2_stack(
        text: str,
        ros_mode: str,
        expect_native_ros2: bool,
        profile: str,
    ) -> Tuple[str, str, str, str, str]:
        bridge_or_ros1 = bool(
            re.search(r"\b(roscore|rosmaster|ros1_bridge|dynamic_bridge|parameter_bridge)\b", text)
            or re.search(r"^ROS_MASTER_URI=", text, flags=re.MULTILINE)
        )
        ros2_available = ros_mode == "ros2" or bool(re.search(r"(^|/)ros2\b|ROS_DISTRO=(humble|foxy|galactic|iron|jazzy|rolling)", text))
        if not expect_native_ros2:
            if bridge_or_ros1:
                return (
                    "2.2",
                    WARN,
                    "native_ros2_stack",
                    "ROS1 bridge or ROS1 environment evidence is present; native ROS2 was not required by this gate",
                    "if this robot is part of the ROS2 dataset fleet, rerun with --expect-native-ros2 and remove the bridge path",
                )
            return ("2.2", INFO, "native_ros2_stack", "native ROS2 expectation not enabled for this run", "")
        if not ros2_available:
            return (
                "2.2",
                FAIL if profile == "dataset" else WARN,
                "native_ros2_stack",
                "native ROS2 environment is not proven",
                "boot the ROS2 image or source the ROS2 workspace before using this robot in the ROS2 fleet",
            )
        if bridge_or_ros1:
            return (
                "2.2",
                FAIL if profile == "dataset" else WARN,
                "native_ros2_stack",
                "ROS1 bridge/process/environment evidence found on a robot expected to be native ROS2",
                "remove ros1_bridge/ROS_MASTER_URI from the dataset path, or add an explicit bridge failure branch and latency gate",
            )
        return ("2.2", PASS, "native_ros2_stack", "native ROS2 stack proven with no ROS1 bridge evidence", "")

    def check_disk(self) -> None:
        usage = shutil.disk_usage(str(Path.home()))
        free_gb = usage.free / (1024**3)
        if free_gb < self.args.min_free_gb:
            self.add(
                "3.1",
                FAIL,
                "disk_free",
                f"{free_gb:.1f} GB free below {self.args.min_free_gb:.1f} GB",
                next_action="free space in ~/agv_data or use a larger SD card before recording",
            )
        else:
            self.add("3.1", PASS, "disk_free", f"{free_gb:.1f} GB free")

    def check_clock(self) -> None:
        result = self.run(
            "clock_machine_state",
            "timedatectl show -p NTPSynchronized -p SystemClockSynchronized -p Timezone 2>/dev/null || true",
            timeout=6,
        )
        text = self.command_output(result)
        if self.clock_synchronized(text):
            self.add("3.3", PASS, "clock_sync", "system clock reports synchronized", [result.log])
        else:
            status = FAIL if self.args.profile == "dataset" else WARN
            self.add(
                "3.3",
                status,
                "clock_sync",
                "system clock synchronization not proven",
                [result.log],
                "start/repair chrony or NTP before multi-robot dataset collection",
            )
        chrony = self.run("chrony_tracking_gate", "chronyc tracking 2>&1 || true", timeout=8)
        status, summary, next_action = self.classify_chrony_tracking(
            self.command_output(chrony),
            self.args.max_clock_offset_ms,
            self.args.profile,
        )
        self.add("3.3", status, "chrony_offset", summary, [chrony.log], next_action)

    @staticmethod
    def clock_synchronized(text: str) -> bool:
        values = {}
        for line in text.splitlines():
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip().lower()
        if "NTPSynchronized" in values:
            return values["NTPSynchronized"] == "yes"
        if "SystemClockSynchronized" in values:
            return values["SystemClockSynchronized"] == "yes"
        return False

    @staticmethod
    def classify_chrony_tracking(text: str, max_offset_ms: float, profile: str) -> Tuple[str, str, str]:
        if "command not found" in text or "chronyc:" in text and "command not found" in text:
            status = FAIL if profile == "dataset" else WARN
            return (
                status,
                "chronyc tracking is unavailable",
                "install/start chrony and rerun before multi-robot dataset collection",
            )
        if not text.strip():
            status = FAIL if profile == "dataset" else WARN
            return (
                status,
                "chrony tracking produced no output",
                "start chrony and verify NTP server reachability before recording",
            )
        leap = re.search(r"Leap status\s*:\s*(.+)", text, flags=re.IGNORECASE)
        if leap and "normal" not in leap.group(1).lower():
            status = FAIL if profile == "dataset" else WARN
            return (
                status,
                f"chrony leap status is not normal: {leap.group(1).strip()}",
                "repair chrony source selection; dataset timestamps are not trustworthy",
            )

        offsets_sec: Dict[str, float] = {}
        for label in ["System time", "Last offset", "RMS offset"]:
            match = re.search(rf"{re.escape(label)}\s*:\s*([+-]?[0-9.]+)\s+seconds", text, flags=re.IGNORECASE)
            if match:
                offsets_sec[label] = abs(float(match.group(1)))
        if not offsets_sec:
            status = FAIL if profile == "dataset" else WARN
            return (
                status,
                "chrony offset could not be parsed",
                "capture `chronyc tracking` output and verify offset is below the dataset threshold",
            )

        if "System time" in offsets_sec:
            system_ms = offsets_sec["System time"] * 1000.0
            historical_ms = max(offsets_sec.values()) * 1000.0
            if system_ms <= max_offset_ms:
                summary = f"chrony system offset {system_ms:.3f} ms <= {max_offset_ms:.3f} ms"
                if historical_ms > max_offset_ms:
                    summary += f"; historical last/RMS offset still settling up to {historical_ms:.3f} ms"
                return (PASS, summary, "")
            status = FAIL if profile == "dataset" else WARN
            return (
                status,
                f"chrony system offset {system_ms:.3f} ms exceeds {max_offset_ms:.3f} ms",
                "repair NTP/chrony topology and rerun before collecting publishable multi-robot data",
            )

        max_seen_ms = max(offsets_sec.values()) * 1000.0
        if max_seen_ms <= max_offset_ms:
            return (PASS, f"chrony parsed offset {max_seen_ms:.3f} ms <= {max_offset_ms:.3f} ms", "")
        status = FAIL if profile == "dataset" else WARN
        return (
            status,
            f"chrony parsed offset {max_seen_ms:.3f} ms exceeds {max_offset_ms:.3f} ms",
            "repair NTP/chrony topology and rerun before collecting publishable multi-robot data",
        )

    def check_network(self) -> None:
        ip_result = self.run("network_ping_ip", "ping -c 2 -W 2 8.8.8.8", timeout=8)
        dns_result = self.run("network_ping_dns", "ping -c 2 -W 2 google.com", timeout=8)
        ip_ok = self.ping_succeeded(ip_result.rc, self.command_output(ip_result))
        dns_ok = self.ping_succeeded(dns_result.rc, self.command_output(dns_result))
        if ip_ok and dns_ok:
            self.add("3.3", PASS, "network_internet", "IP and DNS connectivity passed", [ip_result.log, dns_result.log])
        else:
            details = []
            if not ip_ok:
                details.append("IP ping failed")
            if not dns_ok:
                details.append("DNS ping failed")
            self.add(
                "3.3",
                WARN,
                "network_internet",
                "; ".join(details),
                [ip_result.log, dns_result.log],
                "check Wi-Fi profile, signal, DNS, and DHCP if setup/pull/SSH is unreliable",
            )

    @staticmethod
    def ping_succeeded(return_code: int, text: str) -> bool:
        return return_code == 0 and bool(re.search(r"\b0(?:\.0)?% packet loss\b", text))

    def platform_checks(self) -> None:
        self.check_power_state()
        self.check_usb_inventory()
        self.check_usb_power_policy()
        self.check_dev_permissions()
        self.check_d455_imu_hid()
        self.check_realsense()
        self.check_serial_devices()
        self.check_kernel_logs()
        self.write_mechanical_checklist()
        self.check_odom_mocap_sanity()

    def check_wifi_management(self) -> None:
        result = self.run(
            "wifi_management",
            "echo SERVICES; "
            "systemctl is-active NetworkManager 2>/dev/null || true; "
            "systemctl is-active systemd-networkd 2>/dev/null || true; "
            "systemctl is-active wpa_supplicant@wlan0 2>/dev/null || true; "
            "echo NMCLI; nmcli dev status 2>/dev/null || true; "
            "echo PROCESSES; ps -eo args | grep -E 'wpa_supplicant|dhclient|NetworkManager|systemd-networkd' | grep -v grep || true; "
            "echo NETPLAN; sudo -n sed -n '1,120p' /etc/netplan/*.yaml 2>/dev/null || sed -n '1,120p' /etc/netplan/*.yaml 2>/dev/null || true",
            timeout=10,
        )
        code, status, check, summary, next_action = self.classify_wifi_management(
            self.command_output(result),
            self.args.profile,
        )
        self.add(code, status, check, summary, [result.log], next_action)

    @staticmethod
    def classify_wifi_management(text: str, profile: str) -> Tuple[str, str, str, str, str]:
        manual_dhclient = bool(re.search(r"\bdhclient\b[^\n]*\bwlan0\b", text))
        manual_wpa = bool(
            re.search(r"wpa_supplicant\s+-B[^\n]*\bwlan0\b", text)
            or re.search(r"wpa_supplicant[^\n]*-i\s*wlan0[^\n]*wpa_supplicant\.conf", text)
        )
        network_manager = "NetworkManager" in text and re.search(r"\bwlan0\s+wifi\s+connected\b", text)
        networkd_yaml_wifi = (
            re.search(r"\bactive\b", text)
            and "systemd-networkd" in text
            and re.search(r"renderer:\s*networkd", text)
            and re.search(r"wifis:\s*\n\s*wlan0:", text)
            and re.search(r"access-points:", text)
        )
        networkd_runtime_wifi = (
            re.search(r"\bactive\b", text)
            and "systemd-networkd" in text
            and "/run/netplan/wpa-wlan0.conf" in text
        )
        if manual_dhclient or manual_wpa:
            status = FAIL if profile == "dataset" else WARN
            details = []
            if manual_dhclient:
                details.append("manual dhclient on wlan0")
            if manual_wpa:
                details.append("manual wpa_supplicant on wlan0")
            return (
                "3.3",
                status,
                "wifi_management",
                "conflicting Wi-Fi management detected: " + ", ".join(details),
                "use one persistent NetworkManager/netplan path; stop manual wpa_supplicant/dhclient and reboot-test SSH",
            )
        if network_manager:
            return ("3.3", PASS, "wifi_management", "wlan0 is managed by NetworkManager without manual DHCP/WPA conflicts", "")
        if networkd_yaml_wifi or networkd_runtime_wifi:
            return ("3.3", PASS, "wifi_management", "wlan0 is managed by persistent netplan/systemd-networkd Wi-Fi without manual DHCP/WPA conflicts", "")
        return (
            "3.3",
            WARN,
            "wifi_management",
            "could not prove stable persistent Wi-Fi management",
            "verify Wi-Fi persistence with NetworkManager or netplan, reboot, and SSH reconnect before lab collection",
        )

    def check_power_state(self) -> None:
        result = self.run(
            "power_thermal",
            "vcgencmd get_throttled 2>/dev/null || true; "
            "vcgencmd measure_temp 2>/dev/null || true; "
            "for f in /sys/class/power_supply/*/{status,capacity,voltage_now,current_now}; do [ -f \"$f\" ] && echo \"$f=$(cat \"$f\")\"; done 2>/dev/null || true",
            timeout=8,
        )
        text = self.command_output(result)
        match = re.search(r"throttled=0x([0-9a-fA-F]+)", text)
        if not match:
            self.add(
                "1.2",
                WARN,
                "power_throttle",
                "could not read Raspberry Pi throttle state",
                [result.log],
                "install/repair vcgencmd or capture power evidence manually before publishable collection",
            )
            return
        value = int(match.group(1), 16)
        current_bits = value & 0xF
        historical_bits = value & 0xF0000
        if current_bits:
            self.add(
                "1.2",
                FAIL,
                "power_throttle",
                f"current undervoltage/throttle bits set: 0x{value:x}",
                [result.log],
                "fix power source, battery, charging state, or USB load before collecting data",
            )
        elif historical_bits:
            self.add(
                "1.2",
                WARN,
                "power_throttle",
                f"historical undervoltage/throttle observed: 0x{value:x}",
                [result.log],
                "power-cycle and retest; if it returns, treat robot power path as suspect",
            )
        else:
            self.add("1.2", PASS, "power_throttle", "no current or historical throttle bits", [result.log])

    def check_usb_inventory(self) -> None:
        lsusb = self.run("lsusb", "lsusb; echo; lsusb -t 2>/dev/null || true", timeout=10)
        usb_devices = self.run("usb_devices", "usb-devices 2>/dev/null || true", timeout=10)
        text = self.command_output(lsusb) + "\n" + self.command_output(usb_devices)

        if "8086:0b5c" in text or "RealSense" in text or "D455" in text:
            self.add("1.1", PASS, "d455_enumeration", "D455 appears in USB inventory", [lsusb.log, usb_devices.log])
            speed = self.parse_d455_speed(self.command_output(usb_devices))
            if speed is None:
                self.add(
                    "2.1",
                    WARN,
                    "d455_usb_speed",
                    "could not prove D455 USB speed",
                    [usb_devices.log],
                    "verify the D455 is on USB3 with lsusb -t or rerun after reseating the camera",
                )
            elif speed >= 5000:
                self.add("2.1", PASS, "d455_usb_speed", f"D455 reports USB {speed} Mb/s", [usb_devices.log])
            else:
                self.add(
                    "2.1",
                    FAIL,
                    "d455_usb_speed",
                    f"D455 reports USB {speed} Mb/s, expected 5000 Mb/s",
                    [usb_devices.log],
                    "reseat cable, use Pi USB3 port, try known-good cable/camera, then retest",
                )
            self.check_d455_uvc_binding()
        elif self.args.expect_camera:
            self.add(
                "1.1",
                FAIL,
                "d455_enumeration",
                "expected D455 but it is absent from USB inventory",
                [lsusb.log, usb_devices.log],
                "check camera cable, port, power, or swap camera/cable to isolate hardware",
            )
        else:
            self.add("1.1", INFO, "d455_enumeration", "camera not expected for this run")

    def check_d455_uvc_binding(self) -> None:
        result = self.run(
            "d455_uvc_binding",
            "for d in /sys/bus/usb/devices/*; do "
            "[ -f \"$d/idVendor\" ] || continue; "
            "[ -f \"$d/idProduct\" ] || continue; "
            "[ \"$(cat \"$d/idVendor\" 2>/dev/null)\" = \"8086\" ] || continue; "
            "[ \"$(cat \"$d/idProduct\" 2>/dev/null)\" = \"0b5c\" ] || continue; "
            "echo D455_DEVICE=$(basename \"$d\"); "
            "for i in \"$d\":*; do "
            "[ -d \"$i\" ] || continue; "
            "cls=$(cat \"$i/bInterfaceClass\" 2>/dev/null || true); "
            "sub=$(cat \"$i/bInterfaceSubClass\" 2>/dev/null || true); "
            "name=$(cat \"$i/interface\" 2>/dev/null || true); "
            "driver=none; [ -L \"$i/driver\" ] && driver=$(basename \"$(readlink \"$i/driver\")\"); "
            "echo interface=$(basename \"$i\") class=$cls subclass=$sub driver=$driver name=$name; "
            "done; "
            "done",
            timeout=10,
        )
        for code, status, check, summary, next_action in self.classify_d455_uvc_binding(
            self.command_output(result)
        ):
            self.add(code, status, check, summary, [result.log], next_action)

    def check_usb_power_policy(self) -> None:
        result = self.run(
            "usb_power_policy",
            "for d in /sys/bus/usb/devices/*; do "
            "[ -f \"$d/idVendor\" ] || continue; "
            "echo USB_DEVICE=$d; "
            "echo idVendor=$(cat \"$d/idVendor\" 2>/dev/null); "
            "echo idProduct=$(cat \"$d/idProduct\" 2>/dev/null); "
            "echo product=$(cat \"$d/product\" 2>/dev/null || true); "
            "echo speed=$(cat \"$d/speed\" 2>/dev/null || true); "
            "echo power_control=$(cat \"$d/power/control\" 2>/dev/null || true); "
            "echo power_autosuspend=$(cat \"$d/power/autosuspend\" 2>/dev/null || true); "
            "echo power_autosuspend_delay_ms=$(cat \"$d/power/autosuspend_delay_ms\" 2>/dev/null || true); "
            "done; echo CMDLINE; cat /proc/cmdline 2>/dev/null || true",
            timeout=10,
        )
        for code, status, check, summary, next_action in self.classify_usb_power_policy(self.command_output(result), self.args.profile):
            self.add(code, status, check, summary, [result.log], next_action)

    @staticmethod
    def classify_usb_power_policy(text: str, profile: str = "preflight") -> List[Tuple[str, str, str, str, str]]:
        results: List[Tuple[str, str, str, str, str]] = []
        blocks = re.split(r"\n(?=USB_DEVICE=)", text)
        d455_block = ""
        for block in blocks:
            if "idVendor=8086" in block and "idProduct=0b5c" in block:
                d455_block = block
                break
        if not d455_block:
            results.append(("2.1", INFO, "d455_usb_power_policy", "D455 sysfs power policy not found", ""))
            return results

        delay_match = re.search(r"power_autosuspend_delay_ms=([^\n]+)", d455_block)
        delay = delay_match.group(1).strip() if delay_match else "unknown"
        control_match = re.search(r"power_control=([^\n]+)", d455_block)
        control = control_match.group(1).strip() if control_match else "unknown"
        if control == "on":
            results.append(("2.1", PASS, "d455_usb_autosuspend", "D455 USB autosuspend disabled (power/control=on)", ""))
        elif control == "auto" and delay == "-1":
            results.append(("2.1", PASS, "d455_usb_autosuspend", "D455 USB autosuspend disabled (power/control=auto, autosuspend_delay_ms=-1)", ""))
        elif control == "auto":
            results.append(
                (
                    "2.1",
                    WARN,
                    "d455_usb_autosuspend",
                    "D455 USB autosuspend is enabled (power/control=auto)",
                    "disable autosuspend for the D455 before long dataset runs",
                )
            )
        else:
            results.append(
                (
                    "2.1",
                    WARN,
                    "d455_usb_autosuspend",
                    f"could not determine D455 USB autosuspend state: {control}",
                    "inspect /sys/bus/usb/devices/*/power/control for the D455 and disable autosuspend if uncertain",
                )
            )
        if delay == "-1":
            results.append(("1.2", PASS, "d455_usb_autosuspend_delay", "D455 autosuspend_delay_ms=-1", ""))
        elif delay in {"", "unknown"}:
            status = FAIL if profile == "dataset" else WARN
            results.append(
                (
                    "1.2",
                    status,
                    "d455_usb_autosuspend_delay",
                    "D455 autosuspend_delay_ms could not be read",
                    "read /sys/bus/usb/devices/<D455>/power/autosuspend_delay_ms and persist -1 with the autosuspend fix",
                )
            )
        else:
            status = FAIL if profile == "dataset" else WARN
            results.append(
                (
                    "1.2",
                    status,
                    "d455_usb_autosuspend_delay",
                    f"D455 autosuspend_delay_ms={delay}, expected -1",
                    "run SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-autosuspend, then power-cycle and rerun",
                )
            )

        if "usbcore.quirks=8086:0b5c:kn" in text:
            results.append(("2.1", PASS, "d455_usb_boot_quirk", "D455 usbcore quirk is present in kernel cmdline", ""))
        else:
            results.append(("2.1", INFO, "d455_usb_boot_quirk", "D455 usbcore quirk not present in kernel cmdline", ""))
        return results

    @staticmethod
    def classify_d455_uvc_binding(text: str) -> List[Tuple[str, str, str, str, str]]:
        if "D455_DEVICE=" not in text:
            return [("2.1", INFO, "d455_uvc_binding", "D455 sysfs interfaces not found", "")]

        video_interfaces: List[Tuple[str, str]] = []
        for line in text.splitlines():
            if not line.startswith("interface="):
                continue
            parts: dict[str, str] = {}
            for token in line.split():
                if "=" in token:
                    key, value = token.split("=", 1)
                    parts[key] = value
            if parts.get("class", "").lower() == "0e":
                video_interfaces.append((parts.get("interface", "unknown"), parts.get("driver", "none")))

        if not video_interfaces:
            return [
                (
                    "2.1",
                    WARN,
                    "d455_uvc_binding",
                    "D455 has no sysfs video interfaces to inspect",
                    "replug the D455, rerun udev rules, and verify video interfaces bind to uvcvideo",
                )
            ]

        unbound = [f"{iface}={driver}" for iface, driver in video_interfaces if driver != "uvcvideo"]
        if unbound:
            return [
                (
                    "2.1",
                    FAIL,
                    "d455_uvc_binding",
                    "D455 video interfaces are not bound to uvcvideo: " + ", ".join(unbound),
                    "run SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-uvc-bind --fix d455-authorize-cycle, then rerun robot_doctor",
                )
            ]

        return [("2.1", PASS, "d455_uvc_binding", "D455 video interfaces are bound to uvcvideo", "")]

    @staticmethod
    def parse_d455_speed(usb_devices: str) -> Optional[int]:
        blocks = re.split(r"\n(?=T:)", usb_devices)
        for block in blocks:
            if (
                "Vendor=8086 ProdID=0b5c" in block
                or "RealSense" in block
                or "D455" in block
            ):
                match = re.search(r"Spd=(\d+)", block)
                if match:
                    return int(match.group(1))
        return None

    def check_dev_permissions(self) -> None:
        result = self.run(
            "dev_nodes_permissions",
            "id; groups; "
            "ls -l /dev/video* /dev/media* /dev/hidraw* /dev/ydlidar /dev/ttyS0 /dev/ttyAMA0 /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true",
            timeout=8,
        )
        text = self.command_output(result)
        groups = set()
        group_line = ""
        for line in text.splitlines():
            if line.startswith("ubuntu ") or line.startswith("groups=") or " dialout" in line or " video" in line:
                group_line += line + "\n"
        groups.update(re.findall(r"\b(video|dialout|plugdev|input)\b", group_line))
        missing = []
        if "/dev/video" in text and "video" not in groups:
            missing.append("video")
        if re.search(r"/dev/(?:ydlidar|tty(?:S|AMA|ACM|USB))", text) and "dialout" not in groups:
            missing.append("dialout")
        if "/dev/hidraw" in text and not ({"plugdev", "input"} & groups):
            missing.append("plugdev/input")
        if missing:
            self.add(
                "2.1",
                FAIL,
                "device_permissions",
                "user may lack required device groups: " + ", ".join(sorted(missing)),
                [result.log],
                "add ubuntu to required groups, install udev rules, then log out/reboot",
            )
        else:
            self.add("2.1", PASS, "device_permissions", "device permissions look usable", [result.log])

    def check_d455_imu_hid(self) -> None:
        if not self.args.expect_camera:
            self.add("1.1", INFO, "d455_imu_hid", "camera not expected; D455 IMU HID gate skipped")
            return
        result = self.run(
            "d455_imu_hid",
            "echo HIDRAW_DEVICES; "
            "for h in /sys/class/hidraw/hidraw*; do "
            "[ -e \"$h\" ] || continue; "
            "dev=/dev/$(basename \"$h\"); "
            "echo HIDRAW=$dev; "
            "ls -l \"$dev\" 2>/dev/null || true; "
            "udevadm info -q property -n \"$dev\" 2>/dev/null | "
            "grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_VENDOR=|ID_MODEL=|HID_ID|HID_NAME)=' || true; "
            "done; "
            "echo IIO_DEVICES; "
            "for i in /sys/bus/iio/devices/iio:device*; do "
            "[ -e \"$i\" ] || continue; "
            "echo IIO=$i PATH=$(readlink -f \"$i\"); "
            "[ -r \"$i/name\" ] && echo NAME=$(cat \"$i/name\") || true; "
            "ls -l \"/dev/$(basename \"$i\")\" 2>/dev/null || true; "
            "done",
            timeout=10,
        )
        text = self.command_output(result)
        code, status, check, summary, next_action = self.classify_d455_imu_hid(text, self.args.require_imu)
        self.add(code, status, check, summary, [result.log], next_action)

    @staticmethod
    def classify_d455_imu_hid(text: str, require_imu: bool) -> Tuple[str, str, str, str, str]:
        d455_hidraw = bool(
            re.search(
                r"(ID_VENDOR_ID=8086|HID_ID=.*00008086|RealSense|D455|Intel)",
                text,
                flags=re.IGNORECASE,
            )
        )
        if d455_hidraw:
            return ("1.1", PASS, "d455_imu_hid", "D455/Intel HID raw IMU path is visible", "")

        d455_iio = bool(
            re.search(
                r"(8086:0B5C|HID-SENSOR|iio:device\d+)",
                text,
                flags=re.IGNORECASE,
            )
        )
        if d455_iio:
            return ("1.1", PASS, "d455_imu_hid", "D455 IIO motion sensor path is visible", "")

        status = FAIL if require_imu else WARN
        return (
            "1.1",
            status,
            "d455_imu_hid",
            "D455 IMU HID/IIO path is not visible",
            "replug/reset D455, verify hidraw or IIO udev permissions, then run the standalone motion stream gate",
        )

    def check_realsense(self) -> None:
        if not self.args.expect_camera:
            self.add("2.2", INFO, "realsense_tools", "camera not expected; RealSense gates skipped")
            return

        pkg = self.run(
            "realsense_versions",
            "echo STANDALONE_RS_TOOLS; command -v rs-enumerate-devices || true; "
            "echo STANDALONE_PKG_CONFIG; pkg-config --modversion realsense2 2>/dev/null || true; "
            "echo DPKG; dpkg-query -W 'librealsense2*' 'ros-*-realsense2-camera' 2>/dev/null || true; "
            "echo APT_HOLDS; apt-mark showhold 2>/dev/null | grep -E '^(librealsense2|python3-pyrealsense2)' || true; "
            "echo REALSENSE_APT_SOURCES; "
            "for f in /etc/apt/sources.list.d/*librealsense* /etc/apt/sources.list.d/archive_uri-https_librealsense*; do "
            "[ -f \"$f\" ] || continue; echo FILE:$f; sed -n '1,20p' \"$f\"; done 2>/dev/null || true; "
            "echo PYREALSENSE2; "
            "python3 -c \"import importlib.metadata as m, pyrealsense2 as rs; "
            "print(getattr(rs, '__version__', '') or m.version('pyrealsense2'))\" "
            "2>&1 || echo IMPORT_ERROR:python3:pyrealsense2 import/version failed; "
            "echo ROS_REALSENSE_PACKAGE; "
            + self.ros_setup_prefix +
            "if command -v ros2 >/dev/null 2>&1; then "
            "prefix=$(ros2 pkg prefix realsense2_camera 2>/dev/null | tail -1 || true); "
            "echo ros2_prefix=${prefix}; "
            "[ -n \"${prefix}\" ] && sed -n '1,40p' \"${prefix}/share/realsense2_camera/package.xml\" 2>/dev/null || true; "
            "fi; "
            "if command -v rospack >/dev/null 2>&1; then "
            "pkg=$(rospack find realsense2_camera 2>/dev/null || true); "
            "echo rospack_path=${pkg}; "
            "[ -n \"${pkg}\" ] && sed -n '1,40p' \"${pkg}/package.xml\" 2>/dev/null || true; "
            "fi",
            timeout=10,
        )
        text = self.command_output(pkg)
        standalone_tool = self.parse_standalone_realsense_tool(text)
        if not standalone_tool:
            self.add(
                "2.2",
                FAIL,
                "realsense_tools",
                "standalone rs-enumerate-devices is missing from the non-ROS environment",
                [pkg.log],
                "install/pin librealsense2-utils before dataset collection",
            )
        else:
            self.add("2.2", PASS, "realsense_tools", f"standalone rs-enumerate-devices at {standalone_tool}", [pkg.log])

        expected_lrs = self.args.expected_librealsense or os.environ.get("EXPECTED_LIBREALSENSE", "")
        if expected_lrs:
            versions = self.parse_lrs_versions(text)
            if any(version.startswith(expected_lrs) for version in versions):
                self.add("2.2", PASS, "librealsense_version", f"found expected librealsense {expected_lrs}", [pkg.log])
            else:
                status = FAIL if self.args.strict_versions else WARN
                self.add(
                    "2.2",
                    status,
                    "librealsense_version",
                    f"expected standalone librealsense {expected_lrs}; found {', '.join(versions) or 'unknown'}",
                    [pkg.log],
                    "standardize librealsense across the fleet and rebuild/restart camera driver",
                )
        else:
            self.add("2.2", INFO, "librealsense_version", "no expected librealsense version configured", [pkg.log])

        self.check_realsense_setup_provenance(text, [pkg.log], expected_lrs)

        expected_ros_driver = self.args.expected_realsense_ros_driver or os.environ.get("EXPECTED_REALSENSE_ROS_DRIVER", "")
        ros_driver_version = self.parse_realsense_ros_driver_version(text)
        if expected_ros_driver and ros_driver_version == expected_ros_driver:
            self.add("2.2", PASS, "realsense_ros_driver_version", f"realsense2_camera {ros_driver_version}", [pkg.log])
        elif expected_ros_driver:
            status = FAIL if self.args.strict_versions else WARN
            self.add(
                "2.2",
                status,
                "realsense_ros_driver_version",
                f"expected realsense2_camera {expected_ros_driver}; found {ros_driver_version or 'unknown'}",
                [pkg.log],
                "standardize the RealSense ROS wrapper package/source checkout across the fleet",
            )
        elif ros_driver_version:
            self.add("2.2", INFO, "realsense_ros_driver_version", f"realsense2_camera {ros_driver_version}", [pkg.log])

        if not standalone_tool:
            self.add(
                "2.1",
                FAIL,
                "d455_rs_enumerate",
                "standalone librealsense visibility could not be tested because rs-enumerate-devices is missing",
                [pkg.log],
                "install standalone librealsense2-utils, then rerun robot_doctor",
            )
            self.add(
                "2.1",
                FAIL,
                "realsense_control_query",
                "standalone librealsense control query could not run because rs-enumerate-devices is missing",
                [pkg.log],
                "install standalone librealsense2-utils before classifying USB/control-path health",
            )
            return

        rs_tool_cmd = shell_quote(Path(standalone_tool))
        rs = self.run("rs_enumerate_summary", f"timeout 20 {rs_tool_cmd} -s 2>&1", timeout=25)
        rs_text = self.command_output(rs)
        if "Intel RealSense D455" in rs_text or "D455" in rs_text:
            self.add("2.1", PASS, "d455_rs_enumerate", "D455 visible to librealsense", [rs.log])
        elif self.args.expect_camera:
            self.add(
                "2.1",
                FAIL,
                "d455_rs_enumerate",
                "D455 not visible to librealsense",
                [rs.log],
                "run SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-usb-reset --fix d455-authorize-cycle once, then rerun robot_doctor; if it persists, do cable/port/camera A/B swap",
            )

        serial = self.parse_realsense_serial(rs_text)
        expected_serial = self.args.expected_d455_serial or os.environ.get("EXPECTED_D455_SERIAL", "")
        if expected_serial and serial and serial != expected_serial:
            status = FAIL if self.args.profile == "dataset" or self.args.strict_versions else WARN
            self.add(
                "1.1",
                status,
                "d455_serial_identity",
                f"D455 serial {serial}, expected {expected_serial}",
                [rs.log],
                "attach the assigned D455/cable pair for this robot or update the expected serial only after a deliberate hardware swap",
            )
        elif expected_serial and serial == expected_serial:
            self.add("1.1", PASS, "d455_serial_identity", f"D455 serial {serial}", [rs.log])
        elif expected_serial:
            self.add(
                "1.1",
                FAIL if self.args.expect_camera else WARN,
                "d455_serial_identity",
                f"D455 serial could not be parsed, expected {expected_serial}",
                [rs.log],
                "inspect rs-enumerate-devices output and rerun after the camera enumerates cleanly",
            )
        elif serial:
            self.add("1.1", INFO, "d455_serial_identity", f"D455 serial {serial}", [rs.log])

        firmware = self.parse_realsense_firmware(rs_text)
        expected_fw = self.args.expected_d455_firmware or os.environ.get("EXPECTED_D455_FIRMWARE", "")
        if expected_fw and firmware and firmware != expected_fw:
            status = FAIL if self.args.strict_versions else WARN
            self.add(
                "1.1",
                status,
                "d455_firmware",
                f"firmware {firmware}, expected {expected_fw}",
                [rs.log],
                "standardize D455 firmware before collecting a fleet dataset",
            )
        elif expected_fw and firmware == expected_fw:
            self.add("1.1", PASS, "d455_firmware", f"firmware {firmware}", [rs.log])
        elif firmware:
            self.add("1.1", INFO, "d455_firmware", f"firmware {firmware}", [rs.log])

        controls = self.run("rs_enumerate_controls", f"timeout 25 {rs_tool_cmd} -c 2>&1", timeout=30)
        controls_text = self.command_output(controls)
        if controls.rc == 0 and not self.has_realsense_error(controls_text):
            self.add("2.2", PASS, "realsense_control_query", "librealsense control query completed", [controls.log])
        else:
            self.add(
                "2.1",
                FAIL,
                "realsense_control_query",
                "librealsense control query failed or timed out",
                [controls.log],
                "run SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-usb-reset --fix d455-authorize-cycle once, then rerun; persistent failure means USB/kernel/cable/port/camera, not ROS",
            )

    def check_realsense_setup_provenance(
        self,
        text: str,
        evidence: Sequence[str],
        expected_lrs: str,
    ) -> None:
        source_status, source_summary, source_next = self.classify_realsense_apt_sources(text)
        self.add("2.2", source_status, "realsense_apt_source", source_summary, evidence, source_next)

        installed = self.parse_installed_realsense_packages(text)
        holds = self.parse_apt_holds(text)
        hold_targets = sorted(pkg for pkg in installed if pkg.startswith("librealsense2"))
        if not hold_targets:
            self.add("2.2", INFO, "realsense_package_holds", "no installed librealsense apt packages found", evidence)
        else:
            missing_holds = [pkg for pkg in hold_targets if pkg not in holds]
            if missing_holds:
                self.add(
                    "2.2",
                    WARN,
                    "realsense_package_holds",
                    "installed RealSense packages are not held: " + ", ".join(missing_holds),
                    evidence,
                    "run scripts/setup_robot_ros2.sh or apt-mark hold the standardized RealSense packages",
                )
            else:
                self.add(
                    "2.2",
                    PASS,
                    "realsense_package_holds",
                    "installed RealSense apt packages are held: " + ", ".join(hold_targets),
                    evidence,
                )

        py_version, py_error = self.parse_pyrealsense2_status(text)
        if py_error:
            self.add(
                "2.2",
                FAIL,
                "realsense_python_binding",
                f"pyrealsense2 import failed: {py_error}",
                evidence,
                "install pyrealsense2 so the standalone stream gate can run before dataset collection",
            )
        elif py_version:
            if expected_lrs and not py_version.startswith(expected_lrs):
                status = FAIL if self.args.strict_versions else WARN
                self.add(
                    "2.2",
                    status,
                    "realsense_python_binding",
                    f"pyrealsense2 {py_version}, expected prefix {expected_lrs}",
                    evidence,
                    "standardize pyrealsense2 with the standalone RealSense tools",
                )
            else:
                self.add("2.2", PASS, "realsense_python_binding", f"pyrealsense2 {py_version}", evidence)
        else:
            self.add(
                "2.2",
                WARN,
                "realsense_python_binding",
                "pyrealsense2 status could not be parsed",
                evidence,
                "rerun the pyrealsense2 import/version check and reinstall the binding if parsing still fails",
            )

    def check_realsense_ros_runtime_versions(self) -> None:
        expected = self.args.expected_realsense_ros_librealsense or os.environ.get(
            "EXPECTED_REALSENSE_ROS_LIBREALSENSE",
            "",
        )
        if not expected:
            return

        logs: List[str] = []
        if self.bringup_log and self.bringup_log.exists():
            logs.append(str(self.bringup_log))
        result = self.run(
            "realsense_ros_runtime_logs",
            "for d in \"$HOME/.ros/log/latest\" \"$HOME/.ros/log\"; do "
            "[ -d \"$d\" ] || continue; "
            "find \"$d\" -type f -mmin -120 -size -2M -print 2>/dev/null; "
            "done | head -80 | xargs grep -hE 'RealSense ROS v|Built with LibRealSense|Running with LibRealSense' 2>/dev/null | tail -120 || true",
            timeout=12,
        )
        logs.append(result.log)

        text = "\n".join(Path(path).read_text(errors="replace") for path in logs if Path(path).exists())
        built, running = self.parse_realsense_ros_librealsense_versions(text)
        if running == expected and (built in {"", expected}):
            detail = f"Running with LibRealSense {running}"
            if built:
                detail = f"Built/Running with LibRealSense {built}/{running}"
            self.add("2.2", PASS, "realsense_ros_librealsense", detail, logs)
            return

        status = FAIL if self.args.strict_versions else WARN
        found = []
        if built:
            found.append(f"built={built}")
        if running:
            found.append(f"running={running}")
        self.add(
            "2.2",
            status,
            "realsense_ros_librealsense",
            f"expected ROS node LibRealSense {expected}; found {', '.join(found) or 'unknown'}",
            logs,
            "restart bringup with logging enabled and standardize the RealSense ROS build/runtime SDK",
        )

    @staticmethod
    def marker_section(text: str, marker: str, stop_markers: Sequence[str]) -> List[str]:
        lines = text.splitlines()
        section: List[str] = []
        capture = False
        stops = set(stop_markers)
        for line in lines:
            stripped = line.strip()
            if stripped == marker:
                capture = True
                continue
            if capture and stripped in stops:
                break
            if capture:
                section.append(line)
        return section

    @staticmethod
    def parse_standalone_realsense_tool(text: str) -> str:
        lines = text.splitlines()
        for index, line in enumerate(lines):
            if line.strip() != "STANDALONE_RS_TOOLS":
                continue
            for candidate in lines[index + 1:]:
                candidate = candidate.strip()
                if candidate in {
                    "STANDALONE_PKG_CONFIG",
                    "DPKG",
                    "APT_HOLDS",
                    "REALSENSE_APT_SOURCES",
                    "PYREALSENSE2",
                    "ROS_REALSENSE_PACKAGE",
                }:
                    return ""
                if candidate.startswith("/"):
                    return candidate
            return ""
        return ""

    @staticmethod
    def parse_lrs_versions(text: str) -> List[str]:
        versions: List[str] = []
        lines = text.splitlines()
        capture_pkg_config = False
        for line in lines:
            stripped = line.strip()
            if stripped == "STANDALONE_PKG_CONFIG":
                capture_pkg_config = True
                continue
            if stripped in {
                "STANDALONE_RS_TOOLS",
                "DPKG",
                "APT_HOLDS",
                "REALSENSE_APT_SOURCES",
                "PYREALSENSE2",
                "ROS_REALSENSE_PACKAGE",
            }:
                capture_pkg_config = False
            if capture_pkg_config:
                match = re.search(r"\b(\d+\.\d+\.\d+)", stripped)
                if match:
                    versions.append(match.group(1))
            if stripped.startswith("librealsense2"):
                parts = stripped.split()
                if len(parts) >= 2:
                    match = re.search(r"\b(\d+\.\d+\.\d+)", parts[1])
                    if match:
                        versions.append(match.group(1))
        return list(dict.fromkeys(versions))

    @staticmethod
    def parse_installed_realsense_packages(text: str) -> List[str]:
        packages: List[str] = []
        for line in Doctor.marker_section(
            text,
            "DPKG",
            ["APT_HOLDS", "REALSENSE_APT_SOURCES", "PYREALSENSE2", "ROS_REALSENSE_PACKAGE"],
        ):
            stripped = line.strip()
            if not stripped.startswith("librealsense2"):
                continue
            pkg = stripped.split()[0].split(":", 1)[0]
            packages.append(pkg)
        return sorted(set(packages))

    @staticmethod
    def parse_apt_holds(text: str) -> List[str]:
        holds: List[str] = []
        for line in Doctor.marker_section(
            text,
            "APT_HOLDS",
            ["REALSENSE_APT_SOURCES", "PYREALSENSE2", "ROS_REALSENSE_PACKAGE"],
        ):
            stripped = line.strip()
            if stripped.startswith("librealsense2") or stripped == "python3-pyrealsense2":
                holds.append(stripped.split(":", 1)[0])
        return sorted(set(holds))

    @staticmethod
    def parse_pyrealsense2_status(text: str) -> Tuple[str, str]:
        for line in Doctor.marker_section(text, "PYREALSENSE2", ["ROS_REALSENSE_PACKAGE"]):
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith("IMPORT_ERROR:"):
                return "", stripped.removeprefix("IMPORT_ERROR:")
            if re.match(r"^\d+\.\d+\.\d+", stripped) or stripped == "unknown":
                return stripped, ""
        return "", ""

    @staticmethod
    def classify_realsense_apt_sources(text: str) -> Tuple[str, str, str]:
        enabled_sources: List[str] = []
        current_file = ""
        for line in Doctor.marker_section(text, "REALSENSE_APT_SOURCES", ["PYREALSENSE2", "ROS_REALSENSE_PACKAGE"]):
            stripped = line.strip()
            if stripped.startswith("FILE:"):
                current_file = stripped.removeprefix("FILE:")
                continue
            if not stripped or stripped.startswith("#"):
                continue
            if "librealsense.intel.com" not in stripped:
                continue
            if ".disabled-by-setup-" in current_file:
                continue
            if stripped not in enabled_sources:
                enabled_sources.append(stripped)

        if not enabled_sources:
            return (
                INFO,
                "no enabled Intel RealSense apt source captured; using installed packages as evidence",
                "",
            )
        if len(enabled_sources) > 1:
            return (
                WARN,
                "multiple enabled Intel RealSense apt sources: " + " | ".join(enabled_sources),
                "disable duplicate RealSense apt source files before provisioning more robots",
            )
        source = enabled_sources[0]
        if "trusted=yes" in source:
            return (
                INFO,
                "Intel RealSense apt source uses trusted=yes fallback; package versions remain checked",
                "",
            )
        if "signed-by=" in source:
            return (PASS, "Intel RealSense apt source uses signed-by keyring", "")
        return (
            WARN,
            "Intel RealSense apt source uses legacy trust mode without signed-by/trusted=yes marker",
            "rerun scripts/setup_robot_ros2.sh to standardize the apt source",
        )

    @staticmethod
    def parse_realsense_ros_driver_version(text: str) -> Optional[str]:
        dpkg = re.search(r"ros-[\w-]+-realsense2-camera\s+([0-9][^\s]*)", text)
        if dpkg:
            match = re.search(r"\d+\.\d+\.\d+", dpkg.group(1))
            return match.group(0) if match else dpkg.group(1)
        package = re.search(
            r"<name>\s*realsense2_camera\s*</name>.*?<version>\s*([^<\s]+)\s*</version>",
            text,
            flags=re.DOTALL,
        )
        if package:
            return package.group(1).strip()
        return None

    @staticmethod
    def parse_realsense_ros_librealsense_versions(text: str) -> Tuple[str, str]:
        built_matches = re.findall(r"Built with LibRealSense v?([0-9.]+)", text)
        running_matches = re.findall(r"Running with LibRealSense v?([0-9.]+)", text)
        built = built_matches[-1] if built_matches else ""
        running = running_matches[-1] if running_matches else ""
        return built, running

    @staticmethod
    def parse_realsense_firmware(text: str) -> Optional[str]:
        match = re.search(r"Firmware Version\s+([0-9.]+)", text)
        if match:
            return match.group(1)
        match = re.search(r"Firmware:\s*([0-9.]+)", text)
        if match:
            return match.group(1)
        for line in text.splitlines():
            if "RealSense" in line or "D455" in line:
                versions = re.findall(r"\b\d+\.\d+\.\d+\.\d+\b", line)
                if versions:
                    return versions[-1]
        return None

    @staticmethod
    def parse_realsense_serial(text: str) -> Optional[str]:
        for pattern in [
            r"Serial Number\s*:?\s*([0-9]{6,})",
            r"Intel RealSense D455\s+([0-9]{6,})\s+[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+",
            r"D455\s+([0-9]{6,})\s+[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+",
        ]:
            match = re.search(pattern, text)
            if match:
                return match.group(1)
        for line in text.splitlines():
            if "RealSense" not in line and "D455" not in line:
                continue
            numbers = re.findall(r"\b[0-9]{6,}\b", line)
            if numbers:
                return numbers[0]
        return None

    @staticmethod
    def has_realsense_error(text: str) -> bool:
        return bool(
            re.search(
                r"(UVCIOC_CTRL_QUERY|Failed to query|Frames didn't arrive|Connection timed out|No device connected|error -110)",
                text,
                flags=re.IGNORECASE,
            )
        )

    def check_serial_devices(self) -> None:
        result = self.run(
            "serial_devices",
            "ls -l /dev/ydlidar /dev/ttyS0 /dev/ttyAMA0 /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true; "
            "printf 'ydlidar_resolved=%s\\n' \"$(readlink -f /dev/ydlidar 2>/dev/null || true)\"; "
            "printf 'serial0_resolved=%s\\n' \"$(readlink -f /dev/serial0 2>/dev/null || true)\"; "
            "udevadm info -q property -n /dev/ydlidar 2>/dev/null | sed -n '1,80p' || true; "
            "udevadm info -q property -n /dev/ttyS0 2>/dev/null | sed -n '1,80p' || true",
            timeout=8,
        )
        text = self.command_output(result)
        if "/dev/tty" not in text and "/dev/ydlidar" not in text:
            status = FAIL if self.args.profile == "dataset" else WARN
            self.add(
                "1.1",
                status,
                "serial_devices",
                "no LiDAR/base serial device candidate found",
                [result.log],
                "check LiDAR/base wiring, udev rules, and configured serial port",
            )
        else:
            self.add("1.1", PASS, "serial_devices", "serial device candidates present", [result.log])
        ydlidar_match = re.search(r"^ydlidar_resolved=(.+)$", text, flags=re.MULTILINE)
        serial0_match = re.search(r"^serial0_resolved=(.+)$", text, flags=re.MULTILINE)
        ydlidar_resolved = ydlidar_match.group(1).strip() if ydlidar_match else ""
        serial0_resolved = serial0_match.group(1).strip() if serial0_match else ""
        if ydlidar_resolved and serial0_resolved:
            if ydlidar_resolved == serial0_resolved:
                self.add(
                    "2.1",
                    PASS,
                    "ydlidar_uart_alias",
                    f"/dev/ydlidar resolves to GPIO UART {ydlidar_resolved}",
                    [result.log],
                )
            else:
                status = FAIL if self.args.profile == "dataset" else WARN
                self.add(
                    "2.1",
                    status,
                    "ydlidar_uart_alias",
                    f"/dev/ydlidar resolves to {ydlidar_resolved}, but GPIO UART /dev/serial0 resolves to {serial0_resolved}",
                    [result.log],
                    "run setup_robot_ros2.sh or update /etc/udev/rules.d/99-ydlidar-uart.rules so /dev/ydlidar points at /dev/serial0",
                )

    def check_kernel_logs(self) -> None:
        since_epoch = max(0, self.start_epoch - 60)
        result = self.run(
            "kernel_usb_logs",
            f"journalctl -k --no-pager --since '@{since_epoch}' 2>/dev/null || "
            "sudo -n dmesg -T 2>/dev/null || "
            "dmesg -T 2>&1 || true",
            timeout=12,
        )
        text = self.command_output(result)
        if "Operation not permitted" in text or "read kernel buffer failed" in text:
            self.add(
                "2.1",
                WARN,
                "kernel_log_access",
                "dmesg is restricted, USB/kernel evidence is incomplete",
                [result.log],
                "rerun with sudo or allow dmesg access for a complete diagnosis",
            )
            return

        uvc = self.match_patterns(text, UVC_PATTERNS)
        xhci = self.match_patterns(text, XHCI_PATTERNS)
        reset_events = self.match_patterns(text, USB_RESET_EVENT_PATTERNS)
        disconnect = self.match_patterns(text, USB_DISCONNECT_PATTERNS)
        autosuspend = self.match_patterns(text, USB_AUTOSUSPEND_PATTERNS)
        usb2_fallback = self.match_patterns(text, USB2_FALLBACK_PATTERNS)
        overcurrent = self.match_patterns(text, USB_OVERCURRENT_PATTERNS)
        if uvc:
            self.add(
                "2.1",
                FAIL,
                "kernel_uvc_errors",
                f"UVC/control-path errors found: {uvc[0][:160]}",
                [result.log],
                "treat as kernel/USB/control-path issue; compare with RealSense standalone query and USB swap evidence",
            )
        else:
            self.add("2.1", PASS, "kernel_uvc_errors", "no UVC timeout patterns found", [result.log])

        if xhci:
            self.add(
                "2.1",
                FAIL,
                "kernel_xhci_errors",
                f"xHCI/USB reset errors found: {xhci[0][:160]}",
                [result.log],
                "check port/cable/power, boot USB quirks, and USB controller stability",
            )
        else:
            self.add("2.1", PASS, "kernel_xhci_errors", "no xHCI/reset patterns found", [result.log])

        if reset_events:
            self.add(
                "2.1",
                INFO,
                "kernel_usb_reset_events",
                f"USB reset event observed: {reset_events[0][:160]}",
                [result.log],
            )
        else:
            self.add("2.1", PASS, "kernel_usb_reset_events", "no USB reset events found", [result.log])

        if disconnect:
            self.add(
                "1.2",
                FAIL,
                "kernel_usb_disconnect",
                f"USB disconnect/reset evidence found: {disconnect[0][:160]}",
                [result.log],
                "suspect physical connection, cable, port, or power until swap tests disprove it",
            )
        else:
            self.add("1.2", PASS, "kernel_usb_disconnect", "no USB disconnect patterns found", [result.log])

        if autosuspend:
            self.add(
                "1.2",
                FAIL,
                "kernel_usb_autosuspend_elpg",
                f"autosuspend/ELPG evidence found: {autosuspend[0][:160]}",
                [result.log],
                "disable D455 autosuspend, power-cycle, and rerun the standalone stream gate",
            )
        else:
            self.add("1.2", PASS, "kernel_usb_autosuspend_elpg", "no autosuspend/ELPG kernel patterns found", [result.log])

        if usb2_fallback:
            self.add(
                "1.2",
                WARN,
                "kernel_usb2_fallback",
                f"high-speed USB enumeration observed: {usb2_fallback[0][:160]}",
                [result.log],
                "if this line belongs to the D455, swap cable/port until lsusb reports 5000 Mb/s",
            )
        else:
            self.add("1.2", PASS, "kernel_usb2_fallback", "no high-speed USB fallback patterns found", [result.log])

        if overcurrent:
            self.add(
                "1.2",
                FAIL,
                "kernel_usb_overcurrent",
                f"USB overcurrent/throttle evidence found: {overcurrent[0][:160]}",
                [result.log],
                "reduce USB load, improve power delivery, or use a powered hub before dataset collection",
            )
        else:
            self.add("1.2", PASS, "kernel_usb_overcurrent", "no USB overcurrent patterns found", [result.log])

    @staticmethod
    def match_patterns(text: str, patterns: Sequence[str]) -> List[str]:
        hits = []
        for line in text.splitlines():
            if any(re.search(pattern, line, flags=re.IGNORECASE) for pattern in patterns):
                hits.append(line.strip())
        return hits

    def write_mechanical_checklist(self) -> None:
        checklist = self.out_dir / "operator_mechanical_checklist.md"
        checklist.write_text(
            "\n".join(
                [
                    "# Operator Mechanical Checklist",
                    "",
                    "Software cannot prove these. Tick them before marking the robot dataset-ready.",
                    "",
                    "- [ ] Sensor mounts are rigid and unchanged since calibration",
                    "- [ ] D455 is firmly seated and has strain relief",
                    "- [ ] LiDAR is level and unobstructed",
                    "- [ ] MoCap marker mount is rigid and rigid-body name matches run manifest",
                    "- [ ] Wheels are clean, tight, and not slipping on the floor",
                    "- [ ] Chassis does not wobble under acceleration",
                    "- [ ] Robot was placed in a clear area before motion scripts",
                    "",
                ]
            )
        )
        if self.args.confirm_mechanical:
            self.add("1.3", PASS, "mechanical_operator_check", "operator confirmed mechanical checks", [str(checklist)])
        else:
            status = FAIL if self.args.strict_ops else (WARN if self.args.profile == "dataset" else INFO)
            self.add(
                "1.3",
                status,
                "mechanical_operator_check",
                "mechanical checks require human confirmation" if self.args.profile == "dataset" else "mechanical confirmation not required for this gate",
                [str(checklist)],
                "complete the checklist or rerun with --confirm-mechanical after inspection",
            )

    def check_odom_mocap_sanity(self) -> None:
        evidence_path = getattr(self.args, "odom_mocap_sanity_json", "") or ""
        if not evidence_path:
            if self.args.require_odom_mocap_sanity:
                self.add(
                    "1.3",
                    FAIL if self.args.profile == "dataset" else WARN,
                    "odom_mocap_sanity",
                    "mandatory odom-vs-MoCap sanity evidence is missing",
                    next_action="run the 1 m straight-line sanity check and pass its JSON via --odom-mocap-sanity-json before publishable collection",
                )
            else:
                self.add("1.3", INFO, "odom_mocap_sanity", "odom-vs-MoCap sanity evidence not required by this gate")
            return

        path = Path(evidence_path).expanduser()
        if not path.exists():
            self.add(
                "1.3",
                FAIL,
                "odom_mocap_sanity",
                f"odom-vs-MoCap sanity file does not exist: {path}",
                next_action="rerun the sanity check or pass the correct JSON evidence path",
            )
            return
        try:
            data = json.loads(path.read_text())
        except Exception as exc:
            self.add(
                "1.3",
                FAIL,
                "odom_mocap_sanity",
                f"odom-vs-MoCap sanity JSON is unreadable: {exc}",
                [str(path)],
                "fix the sanity output file so robot_doctor can parse odom_distance_m and mocap_distance_m",
            )
            return
        code, status, check, summary, next_action = self.classify_odom_mocap_sanity(
            data,
            float(self.args.odom_mocap_max_error_ratio),
            self.args.profile,
        )
        self.add(code, status, check, summary, [str(path)], next_action)

    @staticmethod
    def float_from_keys(data: Dict[str, object], keys: Sequence[str]) -> Optional[float]:
        for key in keys:
            if key not in data:
                continue
            try:
                return float(data[key])
            except Exception:
                return None
        return None

    @staticmethod
    def classify_odom_mocap_sanity(
        data: Dict[str, object],
        max_error_ratio: float,
        profile: str,
    ) -> Tuple[str, str, str, str, str]:
        odom = Doctor.float_from_keys(data, ["odom_distance_m", "odom_displacement_m", "odom_m"])
        mocap = Doctor.float_from_keys(data, ["mocap_distance_m", "gt_distance_m", "mocap_m"])
        if odom is None or mocap is None:
            return (
                "1.3",
                FAIL if profile == "dataset" else WARN,
                "odom_mocap_sanity",
                "sanity JSON must include numeric odom_distance_m and mocap_distance_m",
                "rerun the 1 m check with numeric odom and MoCap displacement fields",
            )
        if abs(mocap) < 1e-6:
            return (
                "1.3",
                FAIL,
                "odom_mocap_sanity",
                "MoCap displacement is zero, so odom sanity cannot be evaluated",
                "fix MoCap feedback or rerun the sanity motion before collecting data",
            )
        error_ratio = abs(odom - mocap) / abs(mocap)
        summary = f"odom={odom:.3f}m mocap={mocap:.3f}m error={error_ratio * 100:.1f}% threshold={max_error_ratio * 100:.1f}%"
        if error_ratio <= max_error_ratio:
            return ("1.3", PASS, "odom_mocap_sanity", summary, "")
        return (
            "1.3",
            FAIL if profile == "dataset" else WARN,
            "odom_mocap_sanity",
            summary,
            "do not trust wheel odometry for this robot until wheels/chassis/floor/slip are checked or use MoCap-only localisation for the session",
        )

    def write_d455_swap_checklist(self) -> Path:
        checklist = self.out_dir / "operator_d455_swap_checklist.md"
        checklist.write_text(
            "\n".join(
                [
                    "# D455 Physical A/B Swap Checklist",
                    "",
                    "Use this only when robot_doctor reports a D455 physical-path failure.",
                    "Keep the failing robot powered from the same source unless the step says otherwise.",
                    "",
                    "## Matrix",
                    "",
                    "- [ ] Baseline failing setup captured by robot_doctor report",
                    "- [ ] Known-good D455 camera tested on the suspect robot USB3 port with the suspect cable",
                    "- [ ] Suspect D455 camera tested on a known-good robot USB3 port with a known-good cable",
                    "- [ ] Known-good USB3 cable tested on the suspect robot with the suspect camera",
                    "- [ ] Suspect USB3 cable tested on a known-good robot with a known-good D455",
                    "- [ ] Suspect robot USB3 port tested with known-good camera and known-good cable",
                    "- [ ] Result recorded: fault follows camera, cable, robot/port, power, or is intermittent",
                    "",
                    "## Evidence To Keep",
                    "",
                    "- robot_doctor report before the swap",
                    "- robot_doctor report after each decisive swap",
                    "- D455 serial number and firmware from `rs-enumerate-devices -s`",
                    "- USB speed from `lsusb -t` or robot_doctor `d455_usb_speed`",
                    "- Any UVC/xHCI/disconnect lines from robot_doctor kernel logs",
                    "",
                    "When complete, rerun robot_doctor with:",
                    "",
                    "```bash",
                    "--confirm-d455-camera-swap --confirm-d455-cable-swap --confirm-d455-host-port-swap",
                    "```",
                    "",
                ]
            )
        )
        return checklist

    def d455_physical_failure_results(self) -> List[CheckResult]:
        return [
            item
            for item in self.results
            if item.status == FAIL and item.check in D455_PHYSICAL_FAILURE_CHECKS
        ]

    def check_d455_physical_swap_evidence(self) -> None:
        failures = self.d455_physical_failure_results()
        if not failures:
            return

        checklist = self.write_d455_swap_checklist()
        confirmations = {
            "camera": bool(self.args.confirm_d455_camera_swap),
            "cable": bool(self.args.confirm_d455_cable_swap),
            "host_port": bool(self.args.confirm_d455_host_port_swap),
        }
        missing = [name for name, confirmed in confirmations.items() if not confirmed]
        evidence = [str(checklist)] + [path for item in failures for path in item.evidence]
        notes = getattr(self.args, "d455_swap_notes", "") or ""
        notes_path = Path(notes).expanduser() if notes else None
        notes_present = bool(notes_path and notes_path.exists())
        if notes_path:
            evidence.append(str(notes_path))

        if not missing or notes_present:
            detail = "operator confirmed D455 camera/cable/host-port A/B swap evidence"
            if notes_present:
                detail += f"; notes={notes_path}"
            self.add("1.2", PASS, "d455_physical_swap_evidence", detail, evidence)
            return

        status = FAIL if self.args.strict_ops else WARN
        self.add(
            "1.2",
            status,
            "d455_physical_swap_evidence",
            "D455 physical-path failure needs A/B swap evidence; missing confirmations: " + ", ".join(missing),
            evidence,
            "complete the generated D455 swap checklist, then rerun with the D455 swap confirmation flags or --d455-swap-notes",
        )

    def run_realsense_stream_test(self) -> None:
        if not self.args.expect_camera:
            self.add("3.2", INFO, "realsense_stream_test", "camera not expected; standalone stream test skipped")
            return
        if self.args.stream_test_seconds <= 0:
            self.add("3.2", INFO, "realsense_stream_test", "standalone stream test skipped")
            self.run_d455_motion_stream_gate()
            return
        seconds = int(self.args.stream_test_seconds)
        color_width = int(os.environ.get("CAMERA_COLOR_WIDTH", self.args.camera_width))
        color_height = int(os.environ.get("CAMERA_COLOR_HEIGHT", self.args.camera_height))
        fps = int(os.environ.get("CAMERA_COLOR_FPS", self.args.camera_fps))
        probe = self.build_realsense_probe(
            mode="rgbd",
            seconds=seconds,
            width=color_width,
            height=color_height,
            fps=fps,
            enable_motion=False,
        )
        before = self.run("rs_enumerate_before_stream", "timeout 20 rs-enumerate-devices -s 2>&1", timeout=25)
        result = self.run("realsense_stream_probe", probe, timeout=seconds + 25)
        after = self.run("rs_enumerate_after_stream", "timeout 20 rs-enumerate-devices -s 2>&1", timeout=25)
        text = self.command_output(result)
        data = self.parse_last_json(text)
        evidence = [before.log, result.log, after.log]
        if not data:
            combined = "\n".join(
                [
                    self.command_output(before),
                    text,
                    self.command_output(after),
                ]
            )
            code, status, check, summary, next_action = self.classify_unparseable_realsense_stream(
                combined,
                result.timed_out,
            )
            self.add(code, status, check, summary, evidence, next_action)
            if status == FAIL:
                self.run_realsense_isolation_tests(min(5, seconds), color_width, color_height, fps)
            self.run_d455_motion_stream_gate()
            return
        code, status, check, summary, next_action = self.classify_realsense_stream_result(
            data,
            seconds,
            fps,
            False,
        )
        self.add(code, status, check, summary, evidence, next_action)
        if status == FAIL:
            self.run_realsense_isolation_tests(min(5, seconds), color_width, color_height, fps)
        self.run_d455_motion_stream_gate()

    def run_d455_motion_stream_gate(self) -> None:
        seconds = int(self.args.d455_motion_test_seconds)
        if seconds <= 0 or not (self.args.require_imu or self.args.stream_test_motion):
            self.add("1.1", INFO, "realsense_motion_stream_gate", "standalone D455 motion gate skipped")
            return
        probe = self.build_realsense_probe(
            mode="motion",
            seconds=seconds,
            width=int(os.environ.get("CAMERA_COLOR_WIDTH", self.args.camera_width)),
            height=int(os.environ.get("CAMERA_COLOR_HEIGHT", self.args.camera_height)),
            fps=int(os.environ.get("CAMERA_COLOR_FPS", self.args.camera_fps)),
        )
        result = self.run("realsense_motion_stream_gate", probe, timeout=seconds + 15)
        data = self.parse_last_json(self.command_output(result))
        if not data:
            self.add(
                "1.1",
                FAIL,
                "realsense_motion_stream_gate",
                "standalone D455 motion/IMU probe produced no parseable result",
                [result.log],
                "check D455 HID/hidraw path and rerun after USB reset; if repeatable, use camera/cable/port A/B evidence",
            )
            return
        code, status, check, summary, next_action = self.classify_realsense_single_stream_result(
            "motion",
            data,
            seconds,
            int(os.environ.get("CAMERA_COLOR_FPS", self.args.camera_fps)),
        )
        if status == PASS:
            code = "1.1"
            check = "realsense_motion_stream_gate"
        self.add(code, status, check, summary, [result.log], next_action)

    @staticmethod
    def build_realsense_probe(
        *,
        mode: str,
        seconds: int,
        width: int,
        height: int,
        fps: int,
        enable_motion: bool = False,
    ) -> str:
        enable_color = "True" if mode in {"rgbd", "color"} else "False"
        enable_depth = "True" if mode in {"rgbd", "depth"} else "False"
        enable_motion_literal = "True" if (enable_motion or mode == "motion") else "False"
        return f"""
python3 - <<'PY'
import json, sys, time
try:
    import pyrealsense2 as rs
except Exception as exc:
    print(json.dumps({{"error": "missing_pyrealsense2", "detail": str(exc)}}))
    sys.exit(3)

seconds = {seconds}
width = {width}
height = {height}
fps = {fps}
enable_color = {enable_color}
enable_depth = {enable_depth}
enable_motion = {enable_motion_literal}
pipe = rs.pipeline()
cfg = rs.config()
if enable_color:
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
if enable_depth:
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
if enable_motion:
    cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
counts = {{"color": 0, "depth": 0, "gyro": 0, "accel": 0, "timeouts": 0}}

def count_frameset(frameset):
    color = frameset.get_color_frame()
    depth = frameset.get_depth_frame()
    if color:
        counts["color"] += 1
    if depth:
        counts["depth"] += 1
    for i in range(frameset.size()):
        frame = frameset[i]
        stream_type = frame.get_profile().stream_type()
        if stream_type == rs.stream.gyro:
            counts["gyro"] += 1
        elif stream_type == rs.stream.accel:
            counts["accel"] += 1

try:
    pipe.start(cfg)
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            frameset = pipe.wait_for_frames(1000)
        except RuntimeError as exc:
            detail = str(exc)
            if "Frame didn't arrive" in detail or "frame didn't arrive" in detail.lower():
                counts["timeouts"] += 1
                continue
            raise
        count_frameset(frameset)
except Exception as exc:
    counts["error"] = "stream_exception"
    counts["detail"] = str(exc)
finally:
    try:
        pipe.stop()
    except Exception:
        pass
print(json.dumps(counts, sort_keys=True))
PY
"""

    def run_realsense_isolation_tests(self, seconds: int, width: int, height: int, fps: int) -> None:
        modes = ["color", "depth"]
        if self.args.stream_test_motion or self.args.require_imu:
            modes.append("motion")
        for mode in modes:
            probe = self.build_realsense_probe(mode=mode, seconds=seconds, width=width, height=height, fps=fps)
            result = self.run(f"realsense_{mode}_isolation_probe", probe, timeout=seconds + 15)
            text = self.command_output(result)
            data = self.parse_last_json(text)
            check = f"realsense_{mode}_stream" if mode != "motion" else "realsense_motion_stream_isolation"
            if not data:
                summary = f"standalone {mode}-only stream produced no JSON"
                if result.timed_out:
                    summary += " because it timed out"
                self.add(
                    "2.1",
                    FAIL,
                    check,
                    summary,
                    [result.log],
                    "this isolates the RealSense failure below ROS; rerun after D455 USB reset plus authorize-cycle, then use cable/port/camera A/B evidence if repeatable",
                )
                continue
            code, status, check, summary, next_action = self.classify_realsense_single_stream_result(
                mode,
                data,
                seconds,
                fps,
            )
            self.add(code, status, check, summary, [result.log], next_action)

    @staticmethod
    def classify_unparseable_realsense_stream(
        text: str,
        timed_out: bool,
    ) -> Tuple[str, str, str, str, str]:
        if timed_out or Doctor.has_realsense_error(text):
            reason = "timed out" if timed_out else "reported RealSense/UVC transport errors"
            return (
                "2.1",
                FAIL,
                "realsense_stream_transport",
                f"standalone stream probe produced no JSON because it {reason}",
                "run SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-usb-reset --fix d455-authorize-cycle once, then rerun; persistent failure needs cable/port/camera A/B evidence",
            )
        return (
            "3.2",
            FAIL,
            "realsense_stream_test",
            "stream probe produced no parseable result",
            "inspect stream log and driver configuration",
        )

    @staticmethod
    def classify_realsense_single_stream_result(
        mode: str,
        data: Dict[str, object],
        seconds: int,
        fps: int,
    ) -> Tuple[str, str, str, str, str]:
        check = f"realsense_{mode}_stream" if mode != "motion" else "realsense_motion_stream_isolation"
        if data.get("error") == "missing_pyrealsense2":
            return (
                "2.2",
                FAIL,
                "realsense_python_binding",
                "pyrealsense2 is missing, stream isolation could not run",
                "install the RealSense Python bindings before dataset collection",
            )
        if data.get("error") == "stream_exception":
            detail = str(data.get("detail", ""))
            code = "2.1" if Doctor.has_realsense_error(detail) else "3.2"
            return (
                code,
                FAIL,
                check,
                f"standalone {mode}-only stream failed: {detail[:180] or 'unknown exception'}",
                "rerun after D455 USB reset plus authorize-cycle; if repeatable, keep this as cable/port/camera A/B evidence",
            )

        timeouts = int(data.get("timeouts", 0) or 0)
        if mode == "motion":
            gyro = int(data.get("gyro", 0) or 0)
            accel = int(data.get("accel", 0) or 0)
            if timeouts > 0 or (gyro == 0 and accel == 0):
                return (
                    "2.1",
                    FAIL,
                    check,
                    f"standalone motion-only stream delivered no usable IMU frames: {data}",
                    "fix the RealSense motion/HID path before relying on camera IMU data",
                )
            min_gyro = max(1, int(seconds * 200 * 0.70))
            min_accel = max(1, int(seconds * 100 * 0.70))
            if gyro < min_gyro or accel < min_accel:
                return (
                    "3.2",
                    FAIL,
                    check,
                    f"low standalone motion-only frame count: {data}",
                    "fix IMU stream rate before dataset collection",
                )
            return ("3.2", PASS, check, f"standalone motion-only stream passed: {data}", "")

        count = int(data.get(mode, 0) or 0)
        if timeouts > 0 or count == 0:
            return (
                "2.1",
                FAIL,
                check,
                f"standalone {mode}-only stream delivered no usable frames: {data}",
                "this is below ROS; rerun after D455 USB reset plus authorize-cycle and use cable/port/camera A/B evidence if repeatable",
            )
        min_frames = max(1, int(seconds * fps * 0.80))
        if count < min_frames:
            return (
                "3.2",
                FAIL,
                check,
                f"low standalone {mode}-only frame count: {data}",
                "reduce load only after USB3, power, kernel logs, and driver versions are clean",
            )
        return ("3.2", PASS, check, f"standalone {mode}-only stream passed: {data}", "")

    @staticmethod
    def classify_realsense_stream_result(
        data: Dict[str, object],
        seconds: int,
        fps: int,
        motion: bool = False,
    ) -> Tuple[str, str, str, str, str]:
        if data.get("error") == "missing_pyrealsense2":
            return (
                "2.2",
                FAIL,
                "realsense_python_binding",
                "pyrealsense2 is missing, standalone stream gate could not run",
                "install the RealSense Python bindings or disable the stream gate only for non-dataset static checks",
            )
        if data.get("error") == "stream_exception":
            detail = str(data.get("detail", ""))
            if Doctor.has_realsense_error(detail):
                return (
                    "2.1",
                    FAIL,
                    "realsense_stream_exception",
                    f"standalone stream raised RealSense transport error: {detail[:180]}",
                    "run SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-usb-reset --fix d455-authorize-cycle once if not already tried; persistent failure after reset needs cable/port/camera A/B evidence, not ROS debugging",
                )
            return (
                "3.2",
                FAIL,
                "realsense_stream_test",
                f"standalone stream probe failed: {detail[:180] or 'unknown exception'}",
                "inspect stream log and driver configuration",
            )

        timeouts = int(data.get("timeouts", 0) or 0)
        if timeouts > 0:
            return (
                "2.1",
                FAIL,
                "realsense_stream_timeouts",
                f"standalone stream had {timeouts} wait timeouts",
                "run SUDO_PASSWORD=ubuntu bash scripts/diagnostics/apply_robot_doctor_fix.sh --apply --fix d455-usb-reset --fix d455-authorize-cycle once if not already tried; persistent failure after reset needs cable/port/camera A/B evidence, not ROS debugging",
            )

        min_frames = max(1, int(seconds * fps * 0.80))
        color_count = int(data.get("color", 0) or 0)
        depth_count = int(data.get("depth", 0) or 0)
        if color_count == 0 and depth_count == 0:
            return (
                "2.1",
                FAIL,
                "realsense_stream_no_frames",
                f"standalone stream started but delivered zero color/depth frames: {data}",
                "rerun once after D455 USB reset plus authorize-cycle; if repeatable, inspect rs-enumerate before/after, UVC/xHCI logs, CPU load, and cable/port/camera A/B evidence",
            )
        if color_count < min_frames or depth_count < min_frames:
            return (
                "3.2",
                FAIL,
                "realsense_stream_rate",
                f"low standalone frames: {data}",
                "reduce load only after proving USB3, power, kernel logs, and driver versions are clean",
            )
        if motion:
            gyro_count = int(data.get("gyro", 0) or 0)
            accel_count = int(data.get("accel", 0) or 0)
            min_gyro = max(1, int(seconds * 200 * 0.70))
            min_accel = max(1, int(seconds * 100 * 0.70))
            if gyro_count < min_gyro or accel_count < min_accel:
                return (
                    "3.2",
                    FAIL,
                    "realsense_motion_stream_rate",
                    f"low standalone motion frames: {data}",
                    "fix IMU/motion stream before relying on RealSense IMU data",
                )

        return (
            "3.2",
            PASS,
            "realsense_stream_test",
            f"standalone RGB-D stream passed: {data}",
            "",
        )

    @staticmethod
    def parse_last_json(text: str) -> Optional[Dict[str, object]]:
        for line in reversed(text.splitlines()):
            line = line.strip()
            if line.startswith("{") and line.endswith("}"):
                try:
                    return json.loads(line)
                except Exception:
                    return None
        return None

    def ros_live_checks(self) -> None:
        if self.args.no_ros or self.ros_mode == "none":
            if self.args.profile == "dataset":
                self.add(
                    "2.3",
                    WARN,
                    "ros_graph_skipped",
                    "ROS live checks were skipped in dataset profile; this is a partial gate, not full dataset readiness",
                    next_action="rerun without --no-ros, or provide --bringup-cmd so required ROS topics/rates are proven",
                )
            else:
                self.add("2.3", INFO, "ros_graph", "ROS live checks skipped")
            return
        self.start_bringup()
        try:
            topic_cmd = "(ros2 topic list -t --no-daemon --spin-time 5 2>&1 || ros2 topic list -t 2>&1)"
            topics = self.ros_cmd(topic_cmd, timeout=15, label="ros_topic_list")
            text = self.command_output(topics)
            self.topic_types = self.parse_topic_list(text)
            if not self.topic_types:
                status = FAIL if self.args.profile == "dataset" else WARN
                self.add(
                    "2.3",
                    status,
                    "ros_graph",
                    "no ROS topics discovered",
                    [topics.log],
                    "start bringup or fix ROS_DOMAIN_ID/ROS_MASTER_URI before recording",
                )
                return
            self.add("2.3", PASS, "ros_graph", f"{len(self.topic_types)} topics discovered", [topics.log])
            self.check_dataset_bringup_context()
            self.check_ydlidar_bringup_classification()
            self.check_realsense_ros_runtime_versions()
            self.check_required_live_topics()
            self.check_dds_discovery()
            self.check_realsense_ros2_failure_class()
            self.check_d455_infra_fps_cap()
            self.check_mocap_live()
            self.check_imu_live()
            self.check_stale_ros_processes()
        finally:
            self.stop_bringup()

    def parse_topic_list(self, text: str) -> Dict[str, str]:
        topics: Dict[str, str] = {}
        for line in text.splitlines():
            line = line.strip()
            if not line.startswith("/"):
                continue
            if "[" in line and line.endswith("]"):
                topic, msg_type = line.rsplit("[", 1)
                topics[topic.strip()] = msg_type.strip(" ]")
            else:
                topics[line] = ""
        return topics

    def expected_robot_namespaces(self) -> List[str]:
        namespaces: List[str] = []
        for value in getattr(self.args, "expected_robot_namespace", []) or []:
            for item in split_topics(str(value)):
                item = item.strip()
                if not item:
                    continue
                if not item.startswith("/"):
                    item = "/" + item
                namespaces.append(item.rstrip("/") or "/")
        return list(dict.fromkeys(namespaces))

    def check_dds_discovery(self) -> None:
        expected = self.expected_robot_namespaces()
        if self.ros_mode != "ros2":
            if expected:
                self.add(
                    "3.3",
                    FAIL if self.args.profile == "dataset" else WARN,
                    "dds_discovery",
                    "DDS discovery cannot be proven outside ROS2",
                    next_action="run the fleet discovery gate from a sourced ROS2 environment",
                )
            else:
                self.add("3.3", INFO, "dds_discovery", "DDS discovery gate skipped outside ROS2")
            return
        if not expected:
            self.add(
                "3.3",
                INFO,
                "dds_discovery",
                "no expected robot namespaces configured for fleet discovery gate",
                next_action="for fleet runs, pass --expected-robot-namespace for every robot before starting bags",
            )
            return
        result = self.ros_cmd(
            "(ros2 node list --no-daemon --spin-time 5 2>&1 || ros2 node list 2>&1)",
            timeout=15,
            label="ros2_node_list",
        )
        code, status, check, summary, next_action = self.classify_dds_discovery(
            self.command_output(result),
            expected,
            os.environ.get("ROS_DISCOVERY_SERVER", ""),
            self.args.profile,
        )
        self.add(code, status, check, summary, [result.log], next_action)
        if len(expected) > 4 and not os.environ.get("ROS_DISCOVERY_SERVER"):
            self.add(
                "3.3",
                WARN,
                "dds_discovery_server",
                f"{len(expected)} robot namespaces configured without ROS_DISCOVERY_SERVER",
                next_action="use a Fast-DDS discovery server for larger Wi-Fi fleet runs to avoid multicast discovery failures",
            )
        elif len(expected) > 4:
            self.add("3.3", PASS, "dds_discovery_server", "ROS_DISCOVERY_SERVER is configured for >4 robot fleet")

    @staticmethod
    def classify_dds_discovery(
        text: str,
        expected_namespaces: Sequence[str],
        discovery_server: str,
        profile: str,
    ) -> Tuple[str, str, str, str, str]:
        nodes = [line.strip() for line in text.splitlines() if line.strip().startswith("/")]
        if not nodes:
            return (
                "3.3",
                FAIL if profile == "dataset" else WARN,
                "dds_discovery",
                "ros2 node list returned no nodes",
                "check ROS_DOMAIN_ID, Wi-Fi multicast, and whether bringup is running on every robot",
            )
        missing = [
            namespace
            for namespace in expected_namespaces
            if not any(node == namespace or node.startswith(namespace + "/") for node in nodes)
        ]
        if missing:
            return (
                "3.3",
                FAIL if profile == "dataset" else WARN,
                "dds_discovery",
                "missing robot namespaces from ROS2 discovery: " + ", ".join(missing),
                "check ROS_DOMAIN_ID consistency, Wi-Fi multicast, robot bringup, and use ROS_DISCOVERY_SERVER for larger fleets",
            )
        server_note = " with discovery server" if discovery_server else ""
        return (
            "3.3",
            PASS,
            "dds_discovery",
            f"all {len(expected_namespaces)} expected robot namespaces are visible{server_note}",
            "",
        )

    def live_topic_specs(self) -> Dict[str, Dict[str, float]]:
        specs = dict(DEFAULT_TOPIC_SPECS)
        if not self.args.expect_camera:
            for topic in list(specs):
                if topic.startswith("/camera/"):
                    specs.pop(topic, None)
        cmd_topic = self.args.cmd_topic or os.environ.get("CMD_TOPIC")
        if cmd_topic:
            specs[cmd_topic] = {"min_hz": 0.0, "target_hz": 0.0}
        elif "/cmd_vel" in self.topic_types:
            specs["/cmd_vel"] = {"min_hz": 0.0, "target_hz": 0.0}
        else:
            for topic in self.topic_types:
                if topic.endswith("/cmd_vel"):
                    specs[topic] = {"min_hz": 0.0, "target_hz": 0.0}
                    break
        depth_topic = os.environ.get("DEPTH_TOPIC")
        if depth_topic:
            specs[depth_topic] = {"min_hz": 12.0, "target_hz": 15.0}
        for topic in self.args.required_topic:
            specs.setdefault(topic, {"min_hz": 0.0, "target_hz": 0.0})
        return specs

    def check_dataset_bringup_context(self) -> None:
        if self.args.profile != "dataset":
            return
        data_topics = [
            topic
            for topic, spec in self.live_topic_specs().items()
            if spec.get("min_hz", 0.0) > 0
        ]
        missing = [topic for topic in data_topics if topic not in self.topic_types]
        if self.args.bringup_cmd:
            if missing:
                self.add(
                    "2.2",
                    FAIL,
                    "dataset_bringup_context",
                    f"bringup command ran, but required data topics are still missing: {', '.join(missing)}",
                    [str(self.bringup_log)] if self.bringup_log else [],
                    "inspect the bringup log first; this is a launch/driver problem, not a bag problem",
                )
            else:
                self.add("2.2", PASS, "dataset_bringup_context", "bringup command produced required data topics")
            return
        if missing:
            self.add(
                "2.2",
                FAIL,
                "dataset_bringup_context",
                f"dataset gate ran against an existing ROS graph and required data topics are missing: {', '.join(missing)}",
                next_action="start sensor bringup first or rerun robot_doctor with --bringup-cmd so the report captures launch evidence",
            )
        else:
            self.add(
                "2.2",
                PASS,
                "dataset_bringup_context",
                "dataset gate used an existing ROS graph and required data topics were visible",
            )

    def check_ydlidar_bringup_classification(self) -> None:
        if not self.bringup_log or not self.bringup_log.exists():
            return
        text = self.bringup_log.read_text(errors="replace")
        if "YDLidar" not in text and "YDLIDAR" not in text:
            return
        evidence = [str(self.bringup_log)]
        if "/scan" in self.topic_types:
            self.add("2.2", PASS, "ydlidar_bringup", "YDLidar bringup produced /scan", evidence)
            return
        if re.search(r"cannot bind to the specified serial port", text, flags=re.IGNORECASE):
            self.add(
                "2.1",
                FAIL,
                "ydlidar_serial_bind",
                "YDLidar could not bind the configured serial port/baud",
                evidence,
                "check /dev/ydlidar, UART alias, hciuart, dialout permissions, and the configured baud/port",
            )
            return
        serial_opened = re.search(r"LiDAR successfully connected", text)
        scan_started = re.search(r"start scan mode", text, flags=re.IGNORECASE)
        scan_timeout = re.search(
            r"Failed to turn on the Lidar.*Operation timed out|Operation timed out",
            text,
            flags=re.IGNORECASE | re.DOTALL,
        )
        # The X2 single-channel SDK path can print a healthy status without
        # proving that scan frames are arriving, so keep the evidence literal.
        if serial_opened and scan_started and scan_timeout:
            self.add(
                "1.2",
                FAIL,
                "ydlidar_scan_frame_timeout",
                "YDLidar serial opens and scan command is issued, but no scan frames arrive after scan start",
                evidence,
                "confirm the LiDAR motor spins; check motor power/enable/wiring, then swap LiDAR/harness with a known-good robot if it still times out",
            )
            return
        if re.search(r"Fail to get device information|Failed to start scan mode", text, flags=re.IGNORECASE):
            self.add(
                "1.1",
                FAIL,
                "ydlidar_device_health",
                "YDLidar driver could not read device information or start scan mode",
                evidence,
                "check LiDAR identity, firmware/model assumptions, serial wiring, and try a known-good LiDAR",
            )
            return
        self.add(
            "2.2",
            WARN,
            "ydlidar_bringup",
            "YDLidar log is present but /scan is missing without a known signature",
            evidence,
            "inspect the YDLidar bringup log and add a classifier for the observed failure signature",
        )

    def check_required_live_topics(self) -> None:
        for topic, spec in self.live_topic_specs().items():
            if topic not in self.topic_types:
                status = FAIL if self.args.profile == "dataset" else WARN
                self.add(
                    "2.3",
                    status,
                    "topic_present",
                    f"{topic} missing",
                    next_action="fix launch/remap/driver before recording; the bag validator cannot recover missing topics",
                )
                continue
            self.add("2.3", PASS, "topic_present", f"{topic} type={self.topic_types.get(topic) or 'unknown'}")
            if spec["min_hz"] <= 0 or self.args.live_seconds <= 0:
                continue
            rate = self.measure_topic_rate(topic, int(self.args.live_seconds))
            if rate is None:
                self.add(
                    "2.3",
                    FAIL,
                    "topic_rate",
                    f"{topic} did not produce a measurable rate",
                    next_action="inspect publisher logs and device state; this is live data quality, not bag validation",
                )
            elif rate < spec["min_hz"]:
                self.add(
                    "2.3",
                    FAIL,
                    "topic_rate",
                    f"{topic} {rate:.1f} Hz below {spec['min_hz']:.1f} Hz",
                    next_action="check driver config, CPU/USB load, and kernel logs before collecting data",
                )
            else:
                self.add("2.3", PASS, "topic_rate", f"{topic} {rate:.1f} Hz")

    def check_realsense_ros2_failure_class(self) -> None:
        if not self.args.expect_camera:
            return
        if self.ros_mode != "ros2":
            self.add("2.1", INFO, "viewer_passes_ros2_fails", "ROS2-specific RealSense consistency check skipped outside ROS2")
            return

        standalone_ok = any(
            item.status == PASS
            and item.check in {
                "realsense_stream_test",
                "realsense_color_stream",
                "realsense_depth_stream",
            }
            for item in self.results
        )
        camera_topics = [
            "/camera/color/image_raw",
            "/camera/aligned_depth_to_color/image_raw",
            "/camera/depth/image_rect_raw",
        ]
        present = [topic for topic in camera_topics if topic in self.topic_types]
        if standalone_ok and not present:
            self.add(
                "2.1",
                FAIL,
                "viewer_passes_ros2_fails",
                "standalone RealSense stream passes, but ROS2 camera image topics are absent",
                next_action="treat as ROS2 wrapper/udev/version/launch issue: check realsense2_camera logs, udev rules, wrapper version, and LibRealSense runtime",
            )
        elif standalone_ok:
            self.add(
                "2.1",
                PASS,
                "viewer_passes_ros2_fails",
                "standalone RealSense path and ROS2 camera topics are both visible",
            )
        else:
            self.add(
                "2.1",
                INFO,
                "viewer_passes_ros2_fails",
                "standalone RealSense path did not pass, so ROS2-only failure class is not applicable",
            )

    def check_d455_infra_fps_cap(self) -> None:
        if self.ros_mode != "ros2" or not self.args.expect_camera:
            return
        try:
            expected_fps = float(self.args.camera_fps)
        except Exception:
            expected_fps = 0.0
        infra_topics = [
            topic
            for topic in self.topic_types
            if "infra" in topic and ("image" in topic or topic.endswith("/image_rect_raw"))
        ]
        if expected_fps <= 15.0:
            self.add(
                "2.2",
                PASS,
                "d455_infra_fps_cap",
                f"configured camera FPS is {expected_fps:.1f}; known 15 FPS infra cap is not limiting this gate",
            )
            return
        if not infra_topics:
            self.add(
                "2.2",
                INFO,
                "d455_infra_fps_cap",
                "no D455 infra image topics are active; RGB-D gate cannot observe the infra cap",
            )
            return
        topic = infra_topics[0]
        rate = self.measure_topic_rate(topic, min(10, int(self.args.live_seconds))) if self.args.live_seconds > 0 else None
        if rate is None:
            self.add(
                "2.2",
                WARN,
                "d455_infra_fps_cap",
                f"{topic} present but infra rate could not be measured",
                next_action="rerun with positive --live-seconds or inspect RealSense ROS diagnostics",
            )
        elif rate <= 16.0:
            self.add(
                "2.2",
                FAIL,
                "d455_infra_fps_cap",
                f"{topic} measured {rate:.1f} Hz while requested FPS is {expected_fps:.1f}",
                next_action="apply `ros2 param set /camera/camera depth_module.enable_auto_exposure true`, restart camera node, and rerun the gate",
            )
        else:
            self.add("2.2", PASS, "d455_infra_fps_cap", f"{topic} measured {rate:.1f} Hz")

    def measure_topic_rate(self, topic: str, seconds: int) -> Optional[float]:
        window = max(5, min(10, seconds * 2))
        cmd = f"timeout {seconds + 5} ros2 topic hz {topic} --window {window} 2>&1"
        result = self.ros_cmd(cmd, timeout=seconds + 8, label=f"hz_{topic.strip('/').replace('/', '_')}")
        return self.parse_topic_hz(self.command_output(result))

    @staticmethod
    def parse_topic_hz(text: str) -> Optional[float]:
        matches = re.findall(r"average rate:\s*([0-9.]+)", text)
        if not matches:
            return None
        try:
            return float(statistics.median(float(item) for item in matches))
        except Exception:
            return None

    def check_mocap_live(self) -> None:
        topic = self.args.mocap_topic or os.environ.get("MOCAP_TOPIC")
        if not topic:
            if self.args.require_gt:
                self.add(
                    "3.3",
                    FAIL,
                    "mocap_topic",
                    "ground truth required but no mocap topic configured",
                    next_action="set --mocap-topic or MOCAP_TOPIC to the robot's OptiTrack rigid-body topic",
                )
            else:
                self.add(
                    "3.3",
                    INFO,
                    "mocap_topic",
                    "no mocap topic configured; ground truth is not required for this gate",
                    next_action="set --mocap-topic before any run that needs ground truth",
                )
            return
        if topic not in self.topic_types:
            status = FAIL if self.args.require_gt else WARN
            self.add(
                "3.3",
                status,
                "mocap_topic",
                f"{topic} missing from ROS graph",
                next_action="fix NatNet/OptiTrack bridge or ROS_DOMAIN_ID before scenario run",
            )
            return
        rate = self.measure_topic_rate(topic, min(10, int(self.args.live_seconds))) if self.args.live_seconds > 0 else None
        if rate is not None and rate < 20.0:
            status = FAIL if self.args.require_gt else WARN
            self.add(
                "3.3",
                status,
                "mocap_rate",
                f"{topic} {rate:.1f} Hz below 20 Hz",
                next_action="repair OptiTrack/NatNet/ROS_DOMAIN_ID or reduce mocap load until ground truth is stable",
            )
        else:
            self.add("3.3", PASS, "mocap_topic", f"{topic} present" + (f" @ {rate:.1f} Hz" if rate else ""))

    def check_imu_live(self) -> None:
        candidates = split_topics(os.environ.get("IMU_TOPICS", "")) or [
            "/imu",
            "/camera/imu",
            "/camera/gyro/sample",
            "/camera/accel/sample",
        ]
        present = [topic for topic in candidates if topic in self.topic_types]
        if not present:
            status = FAIL if self.args.require_imu else WARN
            self.add(
                "2.3",
                status,
                "imu_topic",
                "no IMU topic present",
                next_action="start the base/camera IMU driver or configure IMU_TOPICS before recording",
            )
            return
        for topic in present:
            rate = self.measure_topic_rate(topic, min(10, int(self.args.live_seconds))) if self.args.live_seconds > 0 else None
            if rate is None:
                self.add(
                    "2.3",
                    WARN,
                    "imu_rate",
                    f"{topic} present but rate not measured",
                    next_action="rerun live checks with a positive --live-seconds value to prove IMU rate",
                )
            else:
                min_hz = 150.0 if "gyro" in topic or topic == "/camera/imu" else 10.0
                status = PASS if rate >= min_hz else (FAIL if self.args.require_imu else WARN)
                self.add(
                    "2.3",
                    status,
                    "imu_rate",
                    f"{topic} {rate:.1f} Hz",
                    next_action="fix the IMU publisher/rate before recording publishable data" if status != PASS else "",
                )

    def check_stale_ros_processes(self) -> None:
        result = self.run(
            "stale_ros_processes",
            "pgrep -fal 'rosbag|ros2 bag|roslaunch|realsense|ydlidar|myagv|robot_state_publisher|mocap' || true",
            timeout=8,
        )
        text = self.command_output(result)
        if "rosbag record" in text or "ros2 bag record" in text:
            self.add(
                "3.1",
                FAIL,
                "stale_recorder",
                "existing bag recorder process is running",
                [result.log],
                "stop stale recorders before starting a new dataset run",
            )
        else:
            self.add("3.1", PASS, "stale_recorder", "no stale recorder found", [result.log])

    def validate_bag(self) -> None:
        if not self.args.bag:
            if self.args.require_bag:
                self.add(
                    "3.2",
                    FAIL,
                    "bag_validation_missing",
                    "bag validation is required but no bag was supplied",
                    next_action="rerun robot_doctor with --bag pointing at the completed ROS bag",
                )
            elif self.args.profile == "dataset":
                self.add(
                    "3.2",
                    WARN,
                    "bag_validation_missing",
                    "dataset profile ran without a bag; post-run dataset quality is unproven",
                    next_action="after recording, rerun with --bag or use --require-bag for final publishable audits",
                )
            else:
                self.add("3.2", INFO, "bag_validation", "no bag supplied")
            return
        bag = Path(self.args.bag).expanduser()
        if not bag.exists():
            self.add(
                "3.2",
                FAIL,
                "bag_validation",
                f"bag path does not exist: {bag}",
                next_action="rerun with --bag pointing at an existing ROS2 bag file or rosbag2 directory",
            )
            return
        json_out = self.out_dir / "bag_validation.json"
        if bag.is_file() and bag.suffix == ".bag":
            self.add(
                "3.2",
                FAIL,
                "bag_validation",
                f"unsupported ROS1 bag artifact on ROS2 dataset branch: {bag}",
                next_action="record with scripts/logging/start_session.sh and pass the ROS2 bag directory or .mcap/.db3 artifact",
            )
            return
        validator = self.root / "scripts/logging/validate_ros2_bag.py"
        cmd = f"python3 {shell_quote(validator)} {shell_quote(bag)} --json-out {shell_quote(json_out)}"
        if self.args.required_topic:
            required_topics = " ".join(self.args.required_topic)
            cmd = f"REQUIRED_TOPICS={shell_quote(required_topics)} " + cmd
        if self.args.require_gt:
            cmd += " --require-gt"
        if self.args.require_imu:
            cmd += " --require-imu"
        if self.args.require_resilient_storage:
            cmd += " --require-resilient-storage"
        result = self.run("bag_validation", cmd, timeout=self.args.bag_validation_timeout)
        if result.rc == 0:
            self.add("3.2", PASS, "bag_validation", "bag validator passed", [result.log, str(json_out)])
        elif result.rc == 2:
            self.add(
                "3.2",
                WARN,
                "bag_validation",
                "bag validator returned warnings",
                [result.log, str(json_out)],
                "review warnings before labeling the run publishable",
            )
        else:
            self.add(
                "3.2",
                FAIL,
                "bag_validation",
                f"bag validator failed with rc={result.rc}",
                [result.log, str(json_out)],
                "use validator output to classify missing/low-rate topics before rerunning",
            )

    def experiment_ops_checks(self) -> None:
        if self.args.confirm_mocap:
            self.add("3.3", PASS, "mocap_operator_check", "operator confirmed MoCap rigid body and marker visibility")
        else:
            status = FAIL if self.args.strict_ops and self.args.require_gt else (WARN if self.args.require_gt else INFO)
            self.add(
                "3.3",
                status,
                "mocap_operator_check",
                "MoCap rigid body/marker visibility not operator-confirmed" if self.args.require_gt else "MoCap operator confirmation not required for this gate",
                next_action="confirm rigid body name, marker visibility, and mocap coverage before scenario run",
            )
        if self.args.confirm_anchors:
            self.add("3.3", PASS, "anchors_operator_check", "operator confirmed anchors/obstacles surveyed")
        else:
            status = FAIL if self.args.strict_ops else (WARN if self.args.require_gt else INFO)
            self.add(
                "3.3",
                status,
                "anchors_operator_check",
                "anchors/obstacles survey not operator-confirmed" if self.args.require_gt else "anchor/obstacle survey confirmation not required for this gate",
                next_action="survey fixed anchors/obstacles before publishable scenario collection",
            )

    def write_reports(self) -> Tuple[Path, Path, Path]:
        counts = {
            PASS: sum(1 for item in self.results if item.status == PASS),
            WARN: sum(1 for item in self.results if item.status == WARN),
            FAIL: sum(1 for item in self.results if item.status == FAIL),
            INFO: sum(1 for item in self.results if item.status == INFO),
        }
        by_code: Dict[str, Dict[str, int]] = {}
        for code in FAILURE_TREE:
            by_code[code] = {PASS: 0, WARN: 0, FAIL: 0, INFO: 0}
        for item in self.results:
            by_code.setdefault(item.code, {PASS: 0, WARN: 0, FAIL: 0, INFO: 0})
            by_code[item.code][item.status] += 1

        hard_failures = [item for item in self.results if item.status == FAIL]
        warnings = [item for item in self.results if item.status == WARN]
        decision = summarize_decision(self.results, self.args.profile)
        config_path = getattr(self.args, "config", "") or ""
        loaded_config = getattr(self.args, "loaded_config", {}) or {}
        report = {
            "schema_version": REPORT_SCHEMA_VERSION,
            "tool": "robot_doctor",
            "tool_version": ROBOT_DOCTOR_VERSION,
            "robot_id": self.args.robot_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "profile": self.args.profile,
            "config_path": config_path,
            "config_sha256": sha256_file(Path(config_path).expanduser()) if config_path and Path(config_path).expanduser().exists() else "",
            "loaded_config": loaded_config,
            "effective_gate": effective_gate_config(self.args),
            "repo_state": git_info(self.root),
            "can_run_tests": decision["can_run_tests"],
            "dataset_ready": decision["dataset_ready"],
            "verdict": decision["verdict"],
            "decision": decision,
            "output_dir": str(self.out_dir),
            "failure_tree": FAILURE_TREE,
            "counts": counts,
            "counts_by_code": by_code,
            "checks": [asdict(item) for item in self.results],
            "commands": [asdict(item) for item in self.commands],
        }

        json_path = self.out_dir / "summary.json"
        json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

        decision_text = format_operator_decision(report)
        decision_path = self.out_dir / "decision.txt"
        decision_path.write_text(decision_text + "\n")

        md_path = self.out_dir / "summary.md"
        lines = [
            f"# Robot Doctor Summary: {self.args.robot_id}",
            "",
            f"- verdict: **{report['verdict']}**",
            f"- state: `{decision['state']}`",
            f"- can_run_tests: `{str(report['can_run_tests']).lower()}`",
            f"- dataset_ready: `{str(report['dataset_ready']).lower()}`",
            f"- recommendation: {decision['recommendation']}",
            f"- schema_version: `{REPORT_SCHEMA_VERSION}`",
            f"- tool_version: `{ROBOT_DOCTOR_VERSION}`",
            f"- config: `{report['config_path'] or 'none'}`",
            f"- config_sha256: `{report['config_sha256'] or 'none'}`",
            f"- output_dir: `{self.out_dir}`",
            f"- counts: PASS={counts[PASS]} WARN={counts[WARN]} FAIL={counts[FAIL]} INFO={counts[INFO]}",
            "",
            "## Decision",
            "",
            "```text",
            decision_text,
            "```",
            "",
        ]
        primary_blocker = decision.get("primary_blocker")
        if primary_blocker:
            lines.extend(
                [
                    f"- primary_code: `{primary_blocker.get('code')}`",
                    f"- primary_check: `{primary_blocker.get('check')}`",
                    f"- primary_status: `{primary_blocker.get('status')}`",
                    f"- primary_summary: {primary_blocker.get('summary')}",
                    "",
                ]
            )
        else:
            lines.extend(["- no blockers", ""])

        lines.extend(
            [
            "## Failure Tree Counts",
            "",
            "| Code | Area | PASS | WARN | FAIL | INFO |",
            "|---|---|---:|---:|---:|---:|",
            ]
        )
        for code, meta in FAILURE_TREE.items():
            c = by_code.get(code, {})
            lines.append(
                f"| {code} | {meta['name']} | {c.get(PASS, 0)} | {c.get(WARN, 0)} | {c.get(FAIL, 0)} | {c.get(INFO, 0)} |"
            )
        if hard_failures:
            lines.extend(["", "## Required Fixes", ""])
            for item in hard_failures:
                lines.append(f"- `{item.code}` **{item.check}**: {item.summary}")
                if item.next_action:
                    lines.append(f"  - next: {item.next_action}")
                if item.evidence:
                    lines.append(f"  - evidence: {', '.join(item.evidence)}")
        if warnings:
            lines.extend(["", "## Warnings", ""])
            for item in warnings[:30]:
                lines.append(f"- `{item.code}` **{item.check}**: {item.summary}")
        md_path.write_text("\n".join(lines) + "\n")
        return json_path, md_path, decision_path

    def run_all(self) -> int:
        lock_acquired = self.acquire_lock()
        if not lock_acquired:
            json_path, md_path, decision_path = self.write_reports()
            print("")
            print("=" * 72)
            print("Robot Doctor Complete")
            print("=" * 72)
            print(f"summary: {md_path}")
            print(f"json:    {json_path}")
            print(f"decision:{decision_path}")
            print(f"logs:    {self.log_dir}")
            print("")
            print(decision_path.read_text().rstrip())
            print("verdict: FAIL")
            return 1
        try:
            self.detect_ros()
            self.inventory()
            self.platform_checks()
            self.run_realsense_stream_test()
            self.ros_live_checks()
            self.validate_bag()
            self.check_d455_physical_swap_evidence()
            self.experiment_ops_checks()
        finally:
            self.stop_bringup()
            self.release_lock()
        json_path, md_path, decision_path = self.write_reports()
        hard_failures = [item for item in self.results if item.status == FAIL]
        print("")
        print("=" * 72)
        print("Robot Doctor Complete")
        print("=" * 72)
        print(f"summary: {md_path}")
        print(f"json:    {json_path}")
        print(f"decision:{decision_path}")
        print(f"logs:    {self.log_dir}")
        print("")
        print(decision_path.read_text().rstrip())
        print(f"verdict: {'FAIL' if hard_failures else 'PASS/WARN'}")
        return 1 if hard_failures else 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the AGV robot diagnostic pipeline and classify failures."
    )
    parser.add_argument("robot_id", help="robot ID to stamp into reports, e.g. agv102")
    parser.add_argument("--config", help="JSON gate config; CLI flags override config values")
    parser.add_argument(
        "--profile",
        choices=["static", "preflight", "dataset"],
        default="preflight",
        help="static skips live readiness strictness; dataset makes gates stricter",
    )
    parser.add_argument(
        "--output-root",
        default=str(Path.home() / "agv_data/diagnostics"),
        help="directory where evidence folders are written",
    )
    parser.add_argument("--ros", choices=["auto", "ros2", "none"], default="auto")
    parser.add_argument("--no-ros", action="store_true", help="skip ROS graph checks")
    parser.add_argument("--bringup-cmd", help="optional command to launch bringup for bounded live checks")
    parser.add_argument("--bringup-wait", type=int, default=35)
    parser.add_argument("--live-seconds", type=int, default=12, help="seconds per live topic hz probe")
    parser.add_argument("--bag", help="ROS2 bag directory, .mcap, or .db3 to validate")
    parser.add_argument("--require-bag", action="store_true", default=env_bool("REQUIRE_BAG"))
    parser.add_argument("--bag-validation-timeout", type=int, default=180)
    parser.add_argument("--require-gt", action="store_true", default=env_bool("REQUIRE_GT"))
    parser.add_argument("--require-imu", action="store_true", default=env_bool("REQUIRE_IMU"))
    parser.add_argument("--mocap-topic", default=os.environ.get("MOCAP_TOPIC", ""))
    parser.add_argument("--cmd-topic", default=os.environ.get("CMD_TOPIC", ""))
    parser.add_argument("--required-topic", action="append", default=[])
    parser.add_argument(
        "--expect-native-ros2",
        action="store_true",
        default=env_bool("EXPECT_NATIVE_ROS2"),
        help="fail the dataset gate if ROS1 bridge/process/environment evidence is present",
    )
    parser.add_argument(
        "--expected-robot-namespace",
        action="append",
        default=split_topics(os.environ.get("EXPECTED_ROBOT_NAMESPACES", os.environ.get("EXPECTED_ROBOT_NAMESPACE", ""))),
        help="ROS2 namespace that must appear in ros2 node list; repeat for fleet DDS discovery gates",
    )
    parser.add_argument(
        "--require-odom-mocap-sanity",
        action="store_true",
        default=env_bool("REQUIRE_ODOM_MOCAP_SANITY"),
        help="require a 1m odom-vs-MoCap sanity JSON before declaring dataset readiness",
    )
    parser.add_argument("--odom-mocap-sanity-json", default=os.environ.get("ODOM_MOCAP_SANITY_JSON", ""))
    parser.add_argument(
        "--odom-mocap-max-error-ratio",
        type=float,
        default=float(os.environ.get("ODOM_MOCAP_MAX_ERROR_RATIO", "0.10")),
        help="maximum |odom-mocap|/mocap error for the mecanum odometry sanity gate",
    )
    parser.add_argument(
        "--require-resilient-storage",
        action="store_true",
        default=env_bool("REQUIRE_RESILIENT_STORAGE"),
        help="require MCAP or explicit sqlite_resilient/WAL evidence when validating ROS2 bags",
    )
    parser.add_argument("--min-free-gb", type=float, default=float(os.environ.get("MIN_FREE_GB", "5")))
    parser.add_argument("--expect-camera", dest="expect_camera", action="store_true")
    parser.add_argument("--no-expect-camera", dest="expect_camera", action="store_false")
    parser.set_defaults(expect_camera=True)
    parser.add_argument("--expected-d455-serial", default=os.environ.get("EXPECTED_D455_SERIAL", ""))
    parser.add_argument("--expected-d455-firmware", default=os.environ.get("EXPECTED_D455_FIRMWARE", ""))
    parser.add_argument("--expected-librealsense", default=os.environ.get("EXPECTED_LIBREALSENSE", ""))
    parser.add_argument(
        "--expected-realsense-ros-driver",
        default=os.environ.get("EXPECTED_REALSENSE_ROS_DRIVER", ""),
        help="expected realsense2_camera package version, e.g. 4.57.7",
    )
    parser.add_argument(
        "--expected-realsense-ros-librealsense",
        default=os.environ.get("EXPECTED_REALSENSE_ROS_LIBREALSENSE", ""),
        help="expected LibRealSense version reported by the running RealSense ROS node",
    )
    parser.add_argument("--strict-versions", action="store_true", default=env_bool("STRICT_VERSIONS"))
    parser.add_argument("--stream-test-seconds", type=int, default=0)
    parser.add_argument("--stream-test-motion", action="store_true", help="also run the separate standalone D455 motion-only gate")
    parser.add_argument(
        "--d455-motion-test-seconds",
        type=int,
        default=int(os.environ.get("D455_MOTION_TEST_SECONDS", "10")),
        help="seconds for the standalone D455 motion-only IMU gate when IMU is required",
    )
    parser.add_argument("--camera-width", default=os.environ.get("CAMERA_COLOR_WIDTH", "640"))
    parser.add_argument("--camera-height", default=os.environ.get("CAMERA_COLOR_HEIGHT", "480"))
    parser.add_argument("--camera-fps", default=os.environ.get("CAMERA_COLOR_FPS", "15"))
    parser.add_argument(
        "--max-clock-offset-ms",
        type=float,
        default=float(os.environ.get("MAX_CLOCK_OFFSET_MS", "1.0")),
        help="maximum allowed absolute chrony offset for dataset clock sync",
    )
    parser.add_argument("--strict-ops", action="store_true", help="make manual ops confirmations hard gates")
    parser.add_argument("--confirm-mechanical", action="store_true")
    parser.add_argument("--confirm-mocap", action="store_true")
    parser.add_argument("--confirm-anchors", action="store_true")
    parser.add_argument("--confirm-d455-camera-swap", action="store_true")
    parser.add_argument("--confirm-d455-cable-swap", action="store_true")
    parser.add_argument("--confirm-d455-host-port-swap", action="store_true")
    parser.add_argument("--d455-swap-notes", default=os.environ.get("D455_SWAP_NOTES", ""))
    parser.add_argument(
        "--lock-timeout-seconds",
        type=float,
        default=float(os.environ.get("ROBOT_DOCTOR_LOCK_TIMEOUT", "5")),
        help="seconds to wait for another robot_doctor run on the same robot before failing with a diagnostic_lock report",
    )
    parser.add_argument("--no-lock", action="store_true", help="disable the per-robot diagnostic lock")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.loaded_config = apply_gate_config(args, sys.argv[1:])
    except Exception as exc:
        print(f"ERROR: could not load diagnostic config: {exc}", file=sys.stderr)
        return 2
    if args.profile == "static":
        args.no_ros = True
        args.stream_test_seconds = 0
    if args.profile == "dataset" and args.stream_test_seconds == 0:
        # Keep the default bounded but meaningful. Operators can override this to
        # 90 or 120 seconds for final pre-run gates.
        args.stream_test_seconds = 60
    doctor = Doctor(args)
    return doctor.run_all()


if __name__ == "__main__":
    sys.exit(main())
