#!/usr/bin/env python3
"""Validate a ROS 2 rosbag2 dataset using metadata plus SQLite/MCAP timing.

This is intentionally dependency-light so it can run on the robot after a
session. It checks required topics, average rates, timestamp gaps, storage
timestamp monotonicity, IMU presence, ground truth, and bag readability. It
does not deserialize ROS messages; rosbag2 storage timestamps are enough to
detect missing streams, time source regressions, and frame drop patterns.
"""

from __future__ import annotations

import argparse
import json
import os
import sqlite3
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"


@dataclass
class TopicSpec:
    label: str
    candidates: Sequence[str]
    min_hz: float
    target_hz: float
    required: bool = True


@dataclass
class Result:
    level: str
    check: str
    message: str
    topic: Optional[str] = None


@dataclass
class TopicStats:
    topic: str
    msg_type: str
    count: int
    first_ns: Optional[int]
    last_ns: Optional[int]
    max_gap_sec: Optional[float]
    minor_gaps: int
    major_gaps: int
    non_monotonic_count: int
    gap_events: Sequence[Tuple[int, int, float]]

    @property
    def duration_sec(self) -> float:
        if self.first_ns is None or self.last_ns is None or self.last_ns <= self.first_ns:
            return 0.0
        return (self.last_ns - self.first_ns) / 1e9

    @property
    def hz(self) -> float:
        if self.count <= 1:
            return 0.0
        duration = self.duration_sec
        if duration <= 0:
            return 0.0
        return (self.count - 1) / duration


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def split_topics(value: str) -> List[str]:
    return [item.strip() for item in value.replace(",", " ").split() if item.strip()]


def required_specs() -> List[TopicSpec]:
    cmd_topic = os.environ.get("CMD_TOPIC", "/cmd_vel")
    require_cmd_vel = env_bool("REQUIRE_CMD_VEL", True)
    depth_topic = os.environ.get("DEPTH_TOPIC", "/camera/aligned_depth_to_color/image_raw")
    depth_info_topic = os.environ.get(
        "DEPTH_INFO_TOPIC", "/camera/aligned_depth_to_color/camera_info"
    )

    specs = [
        TopicSpec("scan", ["/scan"], 5.0, 18.0),
        TopicSpec("odom", ["/odom"], 12.0, 12.5),
        TopicSpec("cmd_vel", [cmd_topic, "/cmd_vel"], 0.0, 0.0, required=require_cmd_vel),
        TopicSpec("tf", ["/tf"], 10.0, 12.5),
        TopicSpec("tf_static", ["/tf_static"], 0.0, 0.0),
        TopicSpec("color_image", ["/camera/color/image_raw"], 12.0, 15.0),
        TopicSpec("color_info", ["/camera/color/camera_info"], 12.0, 15.0),
        TopicSpec(
            "depth_image",
            [depth_topic, "/camera/aligned_depth_to_color/image_raw", "/camera/depth/image_rect_raw"],
            12.0,
            15.0,
        ),
        TopicSpec(
            "depth_info",
            [depth_info_topic, "/camera/aligned_depth_to_color/camera_info", "/camera/depth/camera_info"],
            12.0,
            15.0,
        ),
    ]

    extra = split_topics(os.environ.get("REQUIRED_TOPICS", ""))
    for topic in extra:
        specs.append(TopicSpec(topic, [topic], 0.0, 0.0))
    return specs


def ground_truth_topics() -> List[str]:
    topics = [
        "/mocap",
        "/ground_truth",
        "/ground_truth/pose",
        "/optitrack/rigid_bodies",
    ]
    env_topic = os.environ.get("MOCAP_TOPIC")
    if env_topic:
        topics.insert(0, env_topic)
    for topic in split_topics(os.environ.get("GROUND_TRUTH_TOPICS", "")):
        if topic not in topics:
            topics.append(topic)
    return topics


def imu_topics() -> List[str]:
    topics = ["/imu", "/camera/imu", "/camera/accel/sample", "/camera/gyro/sample"]
    for topic in split_topics(os.environ.get("IMU_TOPICS", "")):
        if topic not in topics:
            topics.append(topic)
    return topics


def find_db3_files(path: Path) -> List[Path]:
    if path.is_file() and path.suffix == ".db3":
        return [path]
    if path.is_dir():
        return sorted(path.glob("*.db3"))
    return []


def find_mcap_files(path: Path) -> List[Path]:
    if path.is_file() and path.suffix == ".mcap":
        return [path]
    if path.is_dir():
        return sorted(path.glob("*.mcap"))
    return []


def target_hz_for_topic(topic: str) -> float:
    for spec in required_specs():
        if topic in spec.candidates:
            return spec.target_hz
    if topic == "/camera/imu" or topic.endswith("/gyro/sample"):
        return 200.0
    if topic.endswith("/accel/sample"):
        return 100.0
    if topic == "/imu":
        return 100.0
    return 0.0


def stats_from_timestamps(topic: str, msg_type: str, timestamps: Sequence[int]) -> TopicStats:
    sequence = [int(value) for value in timestamps]
    non_monotonic_count = sum(1 for index in range(1, len(sequence)) if sequence[index] < sequence[index - 1])
    if sequence:
        ordered = sorted(sequence)
        gap_events = [
            (ordered[i], ordered[i + 1], (ordered[i + 1] - ordered[i]) / 1e9)
            for i in range(len(ordered) - 1)
        ]
        gaps = [item[2] for item in gap_events]
        max_gap = max(gaps) if gaps else None
    else:
        ordered = []
        gap_events = []
        max_gap = None

    target_hz = target_hz_for_topic(topic)
    if target_hz > 0:
        minor_limit, major_limit = gap_limits_for_topic(topic, target_hz)
        minor_gaps = sum(1 for _, _, gap in gap_events if minor_limit < gap <= major_limit)
        major_gaps = sum(1 for _, _, gap in gap_events if gap > major_limit)
    else:
        minor_gaps = 0
        major_gaps = 0

    return TopicStats(
        topic=topic,
        msg_type=msg_type,
        count=len(ordered),
        first_ns=ordered[0] if ordered else None,
        last_ns=ordered[-1] if ordered else None,
        max_gap_sec=max_gap,
        minor_gaps=minor_gaps,
        major_gaps=major_gaps,
        non_monotonic_count=non_monotonic_count,
        gap_events=gap_events,
    )


def inspect_sqlite(db_path: Path) -> Dict[str, TopicStats]:
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    try:
        topics = {
            int(row["id"]): (row["name"], row["type"])
            for row in conn.execute("SELECT id, name, type FROM topics")
        }
        stats: Dict[str, TopicStats] = {}
        for topic_id, (name, msg_type) in topics.items():
            rows = conn.execute(
                "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY id",
                (topic_id,),
            ).fetchall()
            timestamps = [int(row["timestamp"]) for row in rows]
            stats[name] = stats_from_timestamps(name, msg_type, timestamps)
        return stats
    finally:
        conn.close()


def inspect_ros2_bag(db_paths: Sequence[Path]) -> Dict[str, TopicStats]:
    """Aggregate timestamps across every rosbag2 SQLite chunk.

    rosbag2 can split a recording into multiple `.db3` files. A validator that
    only checks each file separately can miss the exact gap at a split boundary,
    so this function merges timestamps by topic first and then computes rates and
    gaps once.
    """

    timestamps_by_topic: Dict[str, List[int]] = {}
    type_by_topic: Dict[str, str] = {}
    for db_path in db_paths:
        conn = sqlite3.connect(str(db_path))
        conn.row_factory = sqlite3.Row
        try:
            topics = {
                int(row["id"]): (row["name"], row["type"])
                for row in conn.execute("SELECT id, name, type FROM topics")
            }
            for topic_id, (name, msg_type) in topics.items():
                type_by_topic.setdefault(name, msg_type)
                rows = conn.execute(
                    "SELECT timestamp FROM messages WHERE topic_id=?",
                    (topic_id,),
                ).fetchall()
                timestamps_by_topic.setdefault(name, []).extend(int(row["timestamp"]) for row in rows)
        finally:
            conn.close()

    return {
        topic: stats_from_timestamps(topic, type_by_topic.get(topic, ""), timestamps)
        for topic, timestamps in timestamps_by_topic.items()
    }


def inspect_mcap_bag(mcap_paths: Sequence[Path]) -> Dict[str, TopicStats]:
    try:
        from mcap.reader import make_reader  # type: ignore
    except Exception as exc:
        raise RuntimeError(f"MCAP parser unavailable: {exc}") from exc

    timestamps_by_topic: Dict[str, List[int]] = {}
    type_by_topic: Dict[str, str] = {}
    for mcap_path in mcap_paths:
        with mcap_path.open("rb") as stream:
            reader = make_reader(stream)
            for schema, channel, message in reader.iter_messages():
                topic = channel.topic
                msg_type = getattr(schema, "name", "") if schema else ""
                type_by_topic.setdefault(topic, msg_type)
                timestamps_by_topic.setdefault(topic, []).append(int(message.log_time))
    return {
        topic: stats_from_timestamps(topic, type_by_topic.get(topic, ""), timestamps)
        for topic, timestamps in timestamps_by_topic.items()
    }


def validate_ros2_metadata(path: Path, storage_files: Sequence[Path], results: List[Result]) -> None:
    if not path.is_dir():
        record(results, WARN, "metadata_yaml", "bag path is a storage file; metadata.yaml was not checked")
        return
    metadata = path / "metadata.yaml"
    if not metadata.exists():
        record(results, WARN, "metadata_yaml", "metadata.yaml missing beside ROS2 bag storage files")
        return
    try:
        text = metadata.read_text(errors="replace")
    except Exception as exc:
        record(results, FAIL, "metadata_yaml", f"metadata.yaml unreadable: {exc}")
        return
    required_tokens = ["duration", "message_count", "topics_with_message_count"]
    missing = [token for token in required_tokens if token not in text]
    if missing:
        record(results, WARN, "metadata_yaml", "metadata.yaml missing expected fields: " + ", ".join(missing))
    else:
        record(results, PASS, "metadata_yaml", f"metadata.yaml present for {len(storage_files)} storage file(s)")


def storage_resilience_evidence(path: Path) -> List[str]:
    evidence: List[str] = []
    candidates: List[Path] = []
    if path.is_dir():
        candidates.extend(path.glob("*manifest*.yaml"))
        candidates.extend(path.glob("*manifest*.yml"))
        parent_manifest = path.parent / f"{path.name}_manifest.yaml"
        if parent_manifest.exists():
            candidates.append(parent_manifest)
        metadata = path / "metadata.yaml"
        if metadata.exists():
            candidates.append(metadata)
        candidates.extend(path.glob("*.db3-wal"))
    elif path.is_file():
        candidates.extend(path.parent.glob(f"{path.stem}*manifest*.yaml"))
        candidates.extend(path.parent.glob(f"{path.stem}*manifest*.yml"))
        wal = path.with_name(path.name + "-wal")
        if wal.exists():
            candidates.append(wal)

    for candidate in dict.fromkeys(candidates):
        if candidate.suffix == ".db3-wal" or candidate.name.endswith(".db3-wal"):
            evidence.append(f"active WAL sidecar: {candidate.name}")
            continue
        try:
            text = candidate.read_text(errors="replace").lower()
        except Exception:
            continue
        if (
            "sqlite_resilient" in text
            or "journal_mode=wal" in text
            or "journal_mode = wal" in text
            or "journal_mode: wal" in text
            or "storage_preset_profile: resilient" in text
            or "storage-preset-profile resilient" in text
            or "storage-config-file" in text and "wal" in text
            or "storage_config_uri" in text and "wal" in text
        ):
            evidence.append(f"{candidate.name} records sqlite_resilient/WAL")
    return evidence


def validate_storage_resilience(
    path: Path,
    db3_files: Sequence[Path],
    mcap_files: Sequence[Path],
    require_resilient_storage: bool,
    results: List[Result],
) -> None:
    print("\n--- Storage resilience ---")
    if mcap_files and not db3_files:
        record(results, PASS, "storage_resilience", "MCAP storage used")
        return
    if not db3_files:
        record(results, WARN, "storage_resilience", "storage resilience could not be evaluated")
        return
    evidence = storage_resilience_evidence(path)
    if evidence:
        record(results, PASS, "storage_resilience", "; ".join(evidence))
        return
    level = FAIL if require_resilient_storage else WARN
    record(
        results,
        level,
        "storage_resilience",
        "SQLite .db3 storage used, but no sqlite_resilient/WAL evidence was found",
    )


def choose_topic(stats: Dict[str, TopicStats], candidates: Sequence[str]) -> Optional[TopicStats]:
    for topic in candidates:
        if topic in stats:
            return stats[topic]
    suffix_candidates = [topic for topic in candidates if topic.endswith("/cmd_vel")]
    if suffix_candidates:
        for topic, item in stats.items():
            if topic.endswith("/cmd_vel"):
                return item
    return None


def record(results: List[Result], level: str, check: str, message: str, topic: str = None) -> None:
    results.append(Result(level, check, message, topic))
    symbol = {PASS: "OK", WARN: "WARN", FAIL: "FAIL"}[level]
    print(f"  [{symbol}] {check}: {message}")


def classify_gaps(
    item: TopicStats,
    target_hz: float,
    bag_start_ns: Optional[int],
    bag_end_ns: Optional[int],
) -> Tuple[int, int, Optional[float], int]:
    if target_hz <= 0 or item.count <= 2 or item.first_ns is None:
        return 0, 0, item.max_gap_sec, 0

    minor_limit, major_limit = gap_limits_for_topic(item.topic, target_hz)
    edge_ignore_sec = env_float("EDGE_GAP_IGNORE_SEC", 3.0)
    edge_ignore_ns = int(max(0.0, edge_ignore_sec) * 1e9)
    edge_start_ns = (bag_start_ns + edge_ignore_ns) if bag_start_ns is not None else None
    edge_end_ns = (bag_end_ns - edge_ignore_ns) if bag_end_ns is not None else None

    minor = 0
    major = 0
    ignored_edge = 0
    internal_gaps: List[float] = []
    for prev_ns, next_ns, gap in item.gap_events:
        if gap <= minor_limit:
            internal_gaps.append(gap)
            continue
        is_edge_gap = False
        if edge_start_ns is not None and next_ns <= edge_start_ns:
            is_edge_gap = True
        if edge_end_ns is not None and prev_ns >= edge_end_ns:
            is_edge_gap = True
        if is_edge_gap:
            ignored_edge += 1
            continue
        internal_gaps.append(gap)
        if gap > major_limit:
            major += 1
        else:
            minor += 1
    max_gap = max(internal_gaps) if internal_gaps else None
    return minor, major, max_gap, ignored_edge


def gap_limits_for_topic(topic: str, target_hz: float) -> Tuple[float, float]:
    if topic.startswith("/camera/") and topic.endswith("/camera_info"):
        return (
            env_float("CAMERA_INFO_MINOR_GAP_SEC", 0.75),
            env_float("CAMERA_INFO_MAJOR_GAP_SEC", 2.0),
        )
    if topic.startswith("/camera/") and topic.endswith("/image_raw"):
        return (
            env_float("RGBD_MINOR_GAP_SEC", 0.25),
            env_float("RGBD_MAJOR_GAP_SEC", 0.75),
        )
    expected = 1.0 / target_hz
    return 1.5 * expected, 3.0 * expected


def coverage_tolerance_sec(target_hz: float) -> float:
    if target_hz <= 0:
        return 0.0
    return max(env_float("COVERAGE_TOLERANCE_SEC", 3.0), 3.0 / target_hz)


def validate_topic_coverage(
    results: List[Result],
    item: TopicStats,
    target_hz: float,
    bag_start_ns: Optional[int],
    bag_end_ns: Optional[int],
    check_name: str,
) -> None:
    if target_hz <= 0 or item.first_ns is None or item.last_ns is None:
        return
    if bag_start_ns is None or bag_end_ns is None or bag_end_ns <= bag_start_ns:
        return
    tolerance = coverage_tolerance_sec(target_hz)
    late_start_sec = max(0.0, (item.first_ns - bag_start_ns) / 1e9)
    early_stop_sec = max(0.0, (bag_end_ns - item.last_ns) / 1e9)
    if late_start_sec > tolerance or early_stop_sec > tolerance:
        record(
            results,
            FAIL,
            check_name,
            (
                f"{item.topic} does not cover full bag: starts {late_start_sec:.2f}s after bag start, "
                f"stops {early_stop_sec:.2f}s before bag end"
            ),
            item.topic,
        )
    else:
        record(
            results,
            PASS,
            check_name,
            f"{item.topic} covers bag within {tolerance:.2f}s tolerance",
            item.topic,
        )


def validate_storage_timestamp_monotonicity(stats: Dict[str, TopicStats], results: List[Result]) -> None:
    print("\n--- Storage timestamp monotonicity ---")
    for topic, item in sorted(stats.items()):
        if item.count <= 1:
            continue
        if item.non_monotonic_count:
            record(
                results,
                FAIL,
                "timestamp_monotonic",
                f"{topic}: {item.non_monotonic_count} backwards storage timestamp jump(s)",
                topic,
            )
        else:
            record(results, PASS, "timestamp_monotonic", f"{topic}: storage timestamps monotonic", topic)


def validate_topics(
    stats: Dict[str, TopicStats],
    results: List[Result],
    duration_sec: float,
    bag_start_ns: Optional[int],
    bag_end_ns: Optional[int],
) -> None:
    print("\n--- Required topics ---")
    for spec in required_specs():
        item = choose_topic(stats, spec.candidates)
        if item is None:
            record(
                results,
                FAIL if spec.required else WARN,
                spec.label,
                "missing; checked {}".format(", ".join(dict.fromkeys(spec.candidates))),
            )
            continue
        if item.count <= 0:
            record(results, FAIL if spec.required else WARN, spec.label, f"{item.topic} present but empty", item.topic)
            continue
        if spec.min_hz > 0 and item.hz < spec.min_hz:
            record(
                results,
                FAIL,
                spec.label,
                f"{item.topic}: {item.count} msgs @ {item.hz:.1f} Hz below {spec.min_hz:.1f} Hz",
                item.topic,
            )
        else:
            record(
                results,
                PASS,
                spec.label,
                f"{item.topic}: {item.count} msgs @ {item.hz:.1f} Hz",
                item.topic,
            )

        validate_topic_coverage(
            results,
            item,
            spec.target_hz,
            bag_start_ns,
            bag_end_ns,
            spec.label + "_coverage",
        )

        minor, major, max_gap, ignored_edge = classify_gaps(
            item,
            spec.target_hz,
            bag_start_ns,
            bag_end_ns,
        )
        if spec.target_hz > 0 and max_gap is not None:
            edge_note = f"; ignored {ignored_edge} start/stop edge gap(s)" if ignored_edge else ""
            if major:
                _, major_limit = gap_limits_for_topic(item.topic, spec.target_hz)
                record(
                    results,
                    FAIL,
                    spec.label + "_gaps",
                    f"{major} major gap(s), {minor} minor gap(s); max internal gap {max_gap:.3f}s exceeds major threshold {major_limit:.3f}s{edge_note}",
                    item.topic,
                )
            elif minor:
                minor_limit, _ = gap_limits_for_topic(item.topic, spec.target_hz)
                record(
                    results,
                    WARN,
                    spec.label + "_gaps",
                    f"{minor} minor gap(s); max internal gap {max_gap:.3f}s exceeds warning threshold {minor_limit:.3f}s{edge_note}",
                    item.topic,
                )
            else:
                record(
                    results,
                    PASS,
                    spec.label + "_gaps",
                    f"max internal gap {max_gap:.3f}s{edge_note}",
                    item.topic,
                )

    print("\n--- Optional streams ---")
    for topic in ["/diagnostics", "/tag_detections", "/aruco/target_pose"]:
        item = stats.get(topic)
        if item:
            record(results, PASS, topic, f"{item.count} msgs present", topic)
        else:
            record(results, WARN, topic, "not recorded")


def validate_ground_truth(
    stats: Dict[str, TopicStats],
    results: List[Result],
    require_gt: bool,
    bag_start_ns: Optional[int],
    bag_end_ns: Optional[int],
) -> None:
    print("\n--- Ground truth ---")
    present = [stats[topic] for topic in ground_truth_topics() if topic in stats]
    if not present:
        record(
            results,
            FAIL if require_gt else WARN,
            "ground_truth",
            "missing; checked {}".format(", ".join(ground_truth_topics())),
        )
        return
    for item in present:
        level = PASS if item.hz >= 30.0 else WARN
        record(results, level, item.topic, f"{item.count} msgs @ {item.hz:.1f} Hz", item.topic)
        if require_gt:
            validate_topic_coverage(
                results,
                item,
                30.0,
                bag_start_ns,
                bag_end_ns,
                "ground_truth_coverage",
            )


def validate_imu(
    stats: Dict[str, TopicStats],
    results: List[Result],
    require_imu: bool,
    bag_start_ns: Optional[int],
    bag_end_ns: Optional[int],
) -> None:
    print("\n--- IMU ---")
    present = [stats[topic] for topic in imu_topics() if topic in stats]
    if not present:
        record(
            results,
            FAIL if require_imu else WARN,
            "imu",
            "missing; checked {}".format(", ".join(imu_topics())),
        )
        return
    live = [item for item in present if item.count > 0]
    for item in present:
        if item.count <= 0:
            record(results, WARN, item.topic, "present but empty", item.topic)
            continue
        min_hz = 150.0 if "gyro" in item.topic or item.topic == "/camera/imu" else 60.0
        level = PASS if item.hz >= min_hz else WARN
        if require_imu and item.hz < min_hz:
            level = FAIL
        record(results, level, item.topic, f"{item.count} msgs @ {item.hz:.1f} Hz", item.topic)
        if require_imu:
            validate_topic_coverage(
                results,
                item,
                min_hz,
                bag_start_ns,
                bag_end_ns,
                item.topic.strip("/").replace("/", "_") + "_coverage",
            )
    if require_imu and not live:
        record(results, FAIL, "imu", "IMU required but all IMU topics are empty")


def bag_bounds(stats: Dict[str, TopicStats]) -> Tuple[Optional[int], Optional[int], float]:
    starts = [item.first_ns for item in stats.values() if item.first_ns is not None]
    ends = [item.last_ns for item in stats.values() if item.last_ns is not None]
    if not starts or not ends:
        return None, None, 0.0
    start = min(starts)
    end = max(ends)
    return start, end, (end - start) / 1e9


def bag_duration(stats: Dict[str, TopicStats]) -> float:
    _, _, duration = bag_bounds(stats)
    return duration


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a ROS 2 rosbag2 directory or .db3 file.")
    parser.add_argument("bag", help="rosbag2 directory or .db3 file")
    parser.add_argument("--strict", action="store_true", help="return WARN as non-zero")
    parser.add_argument("--require-gt", action="store_true", default=env_bool("REQUIRE_GT"))
    parser.add_argument("--require-imu", action="store_true", default=env_bool("REQUIRE_IMU"))
    parser.add_argument(
        "--require-resilient-storage",
        action="store_true",
        default=env_bool("REQUIRE_RESILIENT_STORAGE"),
        help="fail SQLite .db3 bags unless sqlite_resilient/WAL evidence is recorded, or use MCAP",
    )
    parser.add_argument("--min-duration", type=float, default=float(os.environ.get("MIN_DURATION_SEC", "30")))
    parser.add_argument("--json-out", help="write machine-readable validation report")
    args = parser.parse_args()

    path = Path(args.bag).expanduser()
    db3_files = find_db3_files(path)
    mcap_files = find_mcap_files(path)
    results: List[Result] = []

    print("=" * 68)
    print("AGV ROS 2 Dataset Bag Validator")
    print(f"Bag: {path}")
    print("=" * 68)

    if not db3_files and not mcap_files:
        record(results, FAIL, "bag_integrity", "no .db3 or .mcap files found")
        duration_sec = 0.0
        stats: Dict[str, TopicStats] = {}
    elif db3_files:
        for db3 in db3_files:
            try:
                inspect_sqlite(db3)
                record(results, PASS, "bag_integrity", f"readable SQLite: {db3.name}")
            except Exception as exc:
                record(results, FAIL, "bag_integrity", f"{db3.name}: {exc}")
        try:
            stats = inspect_ros2_bag(db3_files)
        except Exception as exc:
            record(results, FAIL, "bag_integrity", f"SQLite aggregate read failed: {exc}")
            stats = {}
        validate_ros2_metadata(path, db3_files, results)
        validate_storage_resilience(path, db3_files, mcap_files, args.require_resilient_storage, results)
        bag_start_ns, bag_end_ns, duration_sec = bag_bounds(stats)
    else:
        try:
            stats = inspect_mcap_bag(mcap_files)
            for mcap in mcap_files:
                record(results, PASS, "bag_integrity", f"readable MCAP: {mcap.name}")
        except Exception as exc:
            record(results, FAIL, "bag_integrity", str(exc))
            stats = {}
        validate_ros2_metadata(path, mcap_files, results)
        validate_storage_resilience(path, db3_files, mcap_files, args.require_resilient_storage, results)
        bag_start_ns, bag_end_ns, duration_sec = bag_bounds(stats)

    print("\n--- Duration ---")
    if duration_sec >= args.min_duration:
        record(results, PASS, "duration", f"{duration_sec:.1f}s")
    else:
        record(
            results,
            FAIL,
            "duration",
            f"{duration_sec:.1f}s below minimum {args.min_duration:.1f}s",
        )

    if stats:
        validate_topics(stats, results, duration_sec, bag_start_ns, bag_end_ns)
        validate_storage_timestamp_monotonicity(stats, results)
        validate_ground_truth(stats, results, args.require_gt, bag_start_ns, bag_end_ns)
        validate_imu(stats, results, args.require_imu, bag_start_ns, bag_end_ns)

    n_pass = sum(1 for item in results if item.level == PASS)
    n_warn = sum(1 for item in results if item.level == WARN)
    n_fail = sum(1 for item in results if item.level == FAIL)

    print("\n" + "=" * 68)
    print("VALIDATION SUMMARY")
    print("=" * 68)
    print(f"PASS: {n_pass}  WARN: {n_warn}  FAIL: {n_fail}")
    if n_fail:
        verdict = "FAIL"
        exit_code = 1
    elif n_warn:
        verdict = "WARN"
        exit_code = 2 if args.strict else 0
    else:
        verdict = "PASS"
        exit_code = 0
    print(f"Verdict: {verdict}")

    report = {
        "bag": str(path),
        "duration_sec": duration_sec,
        "verdict": verdict,
        "counts": {"pass": n_pass, "warn": n_warn, "fail": n_fail},
        "require_resilient_storage": args.require_resilient_storage,
        "topics": {name: asdict(item) for name, item in sorted(stats.items())},
        "results": [asdict(item) for item in results],
    }
    if args.json_out:
        out_path = Path(args.json_out).expanduser()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
