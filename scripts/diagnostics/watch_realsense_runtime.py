#!/usr/bin/env python3
"""Stop a dataset run when the RealSense source reports a fatal runtime fault.

This guard tails the existing bringup log. It creates no ROS subscriptions, so
it does not add load to camera topics or alter recorder QoS. On the first fatal
motion/HID signature it preserves the evidence needed by the RealSense fault
classifier and writes a machine-readable status file for the scenario runner.
"""

from __future__ import annotations

import argparse
import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional


HID_TIMEOUT = re.compile(
    r"iio_hid_sensor:\s*Frames didn't arrived within the predefined interval",
    re.IGNORECASE,
)


def classify_line(line: str, require_imu: bool = True) -> Optional[str]:
    """Return the fatal fault class represented by one bringup-log line."""
    if require_imu and HID_TIMEOUT.search(line):
        return "REALSENSE_HID_IMU_TIMEOUT"
    return None


def write_status(path: Path, state: str, details: list[str] | None = None) -> None:
    lines = [state]
    if details:
        lines.extend(details)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    temporary.replace(path)


def capture_command(path: Path, command: str, timeout: float = 6.0) -> None:
    with path.open("w", encoding="utf-8") as stream:
        stream.write(f"$ {command}\n")
        stream.write(f"captured_epoch: {time.time():.6f}\n\n")
        try:
            completed = subprocess.run(
                ["bash", "-lc", command],
                stdout=stream,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
                check=False,
            )
            stream.write(f"\nexit_code: {completed.returncode}\n")
        except subprocess.TimeoutExpired:
            stream.write(f"\nTIMEOUT after {timeout:.1f}s\n")


def capture_evidence(
    evidence_dir: Path,
    bringup_log: Path,
    start_epoch: float,
    fault: str,
    trigger_line: str,
) -> None:
    evidence_dir.mkdir(parents=True, exist_ok=True)
    (evidence_dir / "trigger.txt").write_text(
        "\n".join(
            [
                f"fault: {fault}",
                f"detected_epoch: {time.time():.6f}",
                f"monitor_start_epoch: {start_epoch:.6f}",
                f"trigger: {trigger_line.rstrip()}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    try:
        lines = bringup_log.read_text(encoding="utf-8", errors="replace").splitlines()
        (evidence_dir / "bringup_tail.log").write_text(
            "\n".join(lines[-250:]) + "\n",
            encoding="utf-8",
        )
    except OSError as exc:
        (evidence_dir / "bringup_tail.log").write_text(f"read failed: {exc}\n", encoding="utf-8")

    capture_command(
        evidence_dir / "power_thermal.log",
        "vcgencmd get_throttled 2>/dev/null || true; "
        "vcgencmd measure_temp 2>/dev/null || true; "
        "cat /proc/loadavg; "
        "awk '/MemAvailable|MemTotal/ {print}' /proc/meminfo",
    )
    capture_command(evidence_dir / "usb_topology.log", "lsusb; echo; lsusb -t")
    capture_command(
        evidence_dir / "kernel_runtime.log",
        f"journalctl -k --no-pager --since '@{int(start_epoch)}' 2>/dev/null || "
        "sudo -n dmesg -T 2>/dev/null || dmesg -T 2>&1 || true",
        timeout=10.0,
    )
    capture_command(
        evidence_dir / "process_snapshot.log",
        "ps -eo pid,ppid,stat,pcpu,pmem,comm,args --sort=-pcpu | sed -n '1,160p'",
    )


def watch(
    bringup_log: Path,
    status_file: Path,
    evidence_dir: Path,
    require_imu: bool,
    poll_seconds: float,
) -> int:
    status_file.parent.mkdir(parents=True, exist_ok=True)
    start_epoch = time.time()
    write_status(status_file, "RUNNING")
    stopping = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    while not bringup_log.exists() and not stopping:
        time.sleep(poll_seconds)
    if stopping:
        write_status(status_file, "STOPPED_CLEANLY")
        return 0

    with bringup_log.open("r", encoding="utf-8", errors="replace") as stream:
        while not stopping:
            line = stream.readline()
            if not line:
                time.sleep(poll_seconds)
                continue
            fault = classify_line(line, require_imu=require_imu)
            if not fault:
                continue

            details = [
                "FAIL camera gyro stream: /camera/gyro/sample source HID timeout",
                "FAIL camera accel stream: /camera/accel/sample source HID timeout",
                f"trigger: {line.rstrip()}",
                f"evidence_dir: {evidence_dir}",
            ]
            write_status(status_file, f"FAIL_RUNTIME_GUARD {fault}", details)
            print(f"FAIL_RUNTIME_GUARD {fault}", flush=True)
            print(line.rstrip(), flush=True)
            capture_evidence(evidence_dir, bringup_log, start_epoch, fault, line)
            return 2

    write_status(status_file, "STOPPED_CLEANLY")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bringup-log", type=Path, required=True)
    parser.add_argument("--status-file", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--require-imu", action="store_true")
    parser.add_argument("--poll-seconds", type=float, default=0.2)
    args = parser.parse_args()

    if args.poll_seconds <= 0:
        parser.error("--poll-seconds must be positive")
    return watch(
        bringup_log=args.bringup_log,
        status_file=args.status_file,
        evidence_dir=args.evidence_dir,
        require_imu=args.require_imu,
        poll_seconds=args.poll_seconds,
    )


if __name__ == "__main__":
    raise SystemExit(main())
