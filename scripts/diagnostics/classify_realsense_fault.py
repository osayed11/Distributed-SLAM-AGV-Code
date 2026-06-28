#!/usr/bin/env python3
"""Classify RealSense readiness failures from collected robot logs.

The classifier is deliberately conservative: it identifies the failing
software/USB layer, and only claims physical root cause when the logs directly
support it. Camera-vs-cable-vs-host ownership normally requires an A/B swap.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


VIDEO_TOPICS = (
    "/camera/color/image_raw",
    "/camera/aligned_depth_to_color/image_raw",
    "/camera/depth/image_rect_raw",
)
REQUIRED_VIDEO_TOPICS = (
    "/camera/color/image_raw",
    "/camera/aligned_depth_to_color/image_raw",
)
IMU_TOPICS = (
    "/camera/imu",
    "/camera/accel/sample",
    "/camera/gyro/sample",
)
REQUIRED_IMU_TOPICS = (
    "/camera/imu",
)
CORE_TOPICS = (
    "/scan",
    "/odom",
    "/tf",
)


def read_file(path: str | None) -> str:
    if not path:
        return ""
    p = Path(path)
    try:
        return p.read_text(errors="replace")
    except OSError:
        return ""


def has_any(text: str, patterns: tuple[str, ...]) -> bool:
    return any(re.search(pattern, text, re.IGNORECASE) for pattern in patterns)


GATE_LABEL_TOPICS = {
    "color stream": "/camera/color/image_raw",
    "aligned depth stream": "/camera/aligned_depth_to_color/image_raw",
    "depth stream": "/camera/depth/image_rect_raw",
    "camera imu stream": "/camera/imu",
    "camera gyro stream": "/camera/gyro/sample",
    "camera accel stream": "/camera/accel/sample",
}


def topic_state(text: str, topics: tuple[str, ...]) -> tuple[list[str], list[str], list[str]]:
    passed: list[str] = []
    failed: list[str] = []
    warned: list[str] = []
    for line in text.splitlines():
        for label, topic in GATE_LABEL_TOPICS.items():
            if topic not in topics:
                continue
            if re.match(rf"^PASS {re.escape(label)}( (steady )?max gap)?:", line):
                passed.append(topic)
            if re.match(rf"^WARN {re.escape(label)}( (steady )?max gap)?:", line):
                warned.append(topic)
            if re.match(rf"^FAIL {re.escape(label)}( (steady )?max gap)?:", line):
                failed.append(topic)

    for topic in topics:
        for line in text.splitlines():
            if topic not in line:
                continue
            if line.startswith("PASS ") and ("average rate" in line or re.search(r"\b[0-9.]+\s+Hz\b", line)):
                passed.append(topic)
            if line.startswith("WARN "):
                warned.append(topic)
            if line.startswith("FAIL "):
                failed.append(topic)
    return sorted(set(passed)), sorted(set(failed)), sorted(set(warned))

def extract_rates(text: str) -> list[str]:
    rates: list[str] = []
    for line in text.splitlines():
        if not re.search(r"^(PASS|WARN|FAIL)\s+", line):
            continue
        if not re.search(r"/(camera|scan|odom|tf)", line):
            continue
        if "registered" not in line:
            rates.append(line.strip())
    return rates


def throttled_bad(text: str) -> bool:
    for match in re.finditer(r"throttled=0x([0-9a-fA-F]+)", text):
        try:
            if int(match.group(1), 16) != 0:
                return True
        except ValueError:
            pass
    return has_any(text, (r"under-voltage", r"undervoltage", r"voltage.*warn"))


def classify(text: str) -> tuple[str, list[str], list[str]]:
    evidence: list[str] = []
    limitations: list[str] = []

    video_pass, video_fail, video_warn = topic_state(text, VIDEO_TOPICS)
    imu_pass, imu_fail, imu_warn = topic_state(text, IMU_TOPICS)
    core_pass, core_fail, core_warn = topic_state(text, CORE_TOPICS)

    uvc_timeout = has_any(
        text,
        (
            r"UVCIOC_CTRL_QUERY",
            r"\bset_xu\b",
            r"\bget_xu\b",
            r"control_transfer.*failed",
            r"Connection timed out",
            r"Failed to query \(GET_CUR\) UVC control",
            r"Failed to query \(SET_CUR\) UVC control",
        ),
    )
    hid_timeout = has_any(
        text,
        (
            r"iio_hid_sensor: Frames didn't arrived",
            r"No report with id",
            r"hid-sensor",
        ),
    )
    disconnect = has_any(
        text,
        (
            r"USB disconnect",
            r"The device has been disconnected",
            r"device removed",
            r"No such device",
            r"Failed to create device",
        ),
    )
    usb2_or_slow = has_any(
        text,
        (
            r"Driver=uvcvideo,\s*480M",
            r"Usb Type Descriptor:\s*2",
            r"speed=480",
        ),
    )
    usb3_seen = has_any(
        text,
        (
            r"Driver=uvcvideo,\s*5000M",
            r"Usb Type Descriptor:\s*3",
            r"speed=5000",
        ),
    )

    if video_pass:
        evidence.append("RGB-D video topics produced rate data")
    if video_fail:
        evidence.append("one or more RGB-D video topics failed rate checks")
    if video_warn:
        evidence.append("one or more RGB-D video topics produced bounded warning gaps")
    if imu_pass:
        evidence.append("one or more D455 IMU/HID topics produced rate data")
    if imu_fail:
        evidence.append("one or more D455 IMU/HID topics failed rate checks")
    if imu_warn:
        evidence.append("one or more D455 IMU/HID topics produced warning gaps")
    if core_pass:
        evidence.append("non-camera core ROS topics produced rate data")
    if core_fail:
        evidence.append("one or more non-camera core ROS topics failed rate checks")
    if core_warn:
        evidence.append("one or more non-camera core ROS topics produced warning gaps")
    if usb3_seen:
        evidence.append("D455 was observed on USB3/SuperSpeed")
    if uvc_timeout:
        evidence.append("UVC extension-unit control timeout text was observed")
    if hid_timeout:
        evidence.append("HID/IIO motion frame timeout text was observed")
    if disconnect:
        evidence.append("USB disconnect/device-drop text was observed")
    required_video_pass = all(topic in video_pass for topic in REQUIRED_VIDEO_TOPICS)
    fused_imu_pass = all(topic in imu_pass for topic in REQUIRED_IMU_TOPICS)
    raw_imu_pass = "/camera/gyro/sample" in imu_pass and "/camera/accel/sample" in imu_pass
    required_imu_pass = fused_imu_pass or raw_imu_pass
    critical_imu_fail = bool(imu_fail) and not required_imu_pass
    all_stream_rates_pass = required_video_pass and required_imu_pass and not video_fail and not critical_imu_fail

    if all_stream_rates_pass:
        if video_warn or imu_warn:
            limitations.append(
                "Critical stream rates passed, but bounded stream continuity warnings were observed. Final bag validation decides publishability."
            )
            return "PASS_WITH_STREAM_WARNINGS", evidence, limitations
        if uvc_timeout or hid_timeout or disconnect:
            limitations.append(
                "Critical stream rates passed. Low-level timeout/reset text is evidence to monitor, not a readiness failure for this run."
            )
            return "PASS_WITH_LOW_LEVEL_WARNINGS", evidence, limitations
        return "PASS", evidence, limitations

    if throttled_bad(text):
        evidence.append("Pi throttling/undervoltage evidence was observed")
        return "HOST_POWER_OR_THERMAL", evidence, limitations
    if usb2_or_slow:
        evidence.append("D455 is on USB2/480M or reported a USB2 descriptor")
        return "USB_LINK_DEGRADED", evidence, limitations
    if video_fail and imu_pass and uvc_timeout:
        limitations.append(
            "Software proves failure below ROS on the UVC video/control path. Camera/cable vs Pi/port requires A/B swap."
        )
        return "REALSENSE_UVC_VIDEO_CONTROL_TIMEOUT", evidence, limitations
    if critical_imu_fail and video_pass and (hid_timeout or uvc_timeout):
        limitations.append(
            "Software proves failure below ROS on the D455 HID/IMU path. Camera/cable vs Pi/port requires A/B swap."
        )
        return "REALSENSE_HID_IMU_TIMEOUT", evidence, limitations
    if (video_fail or critical_imu_fail) and uvc_timeout:
        limitations.append(
            "Software proves D455 USB control-path failure, but physical ownership requires A/B swap."
        )
        return "REALSENSE_DEVICE_CONTROL_TIMEOUT", evidence, limitations
    if video_fail or critical_imu_fail:
        limitations.append(
            "The live gate proved topic continuity failure even if average rates passed. Physical ownership requires A/B swap."
        )
        return "REALSENSE_STREAM_GAP_FAILURE", evidence, limitations
    if disconnect:
        limitations.append(
            "Logs prove a USB device drop/reset, but not whether the owner is cable, camera, port, or power without A/B."
        )
        return "USB_DEVICE_DISCONNECT_OR_RESET", evidence, limitations
    if (video_fail or critical_imu_fail) and not uvc_timeout and not hid_timeout:
        limitations.append(
            "ROS topics failed without clear USB/UVC/HID log evidence; inspect launch logs and topic remaps."
        )
        return "ROS_GRAPH_OR_DRIVER_STARTUP_FAILURE", evidence, limitations
    return "INCONCLUSIVE", evidence, limitations


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Classify RealSense readiness failures from readiness, bringup, kernel, and hardware logs."
    )
    parser.add_argument("--readiness-log", help="Full readiness or RealSense gate output log")
    parser.add_argument("--bringup-log", help="ROS bringup log containing realsense2_camera output")
    parser.add_argument("--kernel-log", help="Kernel/dmesg log or snapshot")
    parser.add_argument("--hardware-log", action="append", default=[], help="Hardware snapshot log; may be repeated")
    parser.add_argument("--label", default="", help="Optional robot/run label")
    args = parser.parse_args()

    parts = [
        read_file(args.readiness_log),
        read_file(args.bringup_log),
        read_file(args.kernel_log),
    ]
    parts.extend(read_file(path) for path in args.hardware_log)
    text = "\n".join(part for part in parts if part)

    if not text.strip():
        print("classification: INCONCLUSIVE")
        print("reason: no readable log content")
        return 2

    classification, evidence, limitations = classify(text)
    if args.label:
        print(f"label: {args.label}")
    print(f"classification: {classification}")

    rates = extract_rates(text)
    if rates:
        print("rates:")
        for line in rates[-20:]:
            print(f"  {line}")

    if evidence:
        print("evidence:")
        for item in evidence:
            print(f"  - {item}")

    if limitations:
        print("limits:")
        for item in limitations:
            print(f"  - {item}")

    if classification.startswith("PASS"):
        return 0
    if classification == "INCONCLUSIVE":
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
