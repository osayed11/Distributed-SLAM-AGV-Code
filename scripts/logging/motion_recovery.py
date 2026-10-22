"""Small, ROS-independent stop/recovery state machine and evidence helpers."""

import json
import math
import os
from pathlib import Path


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def read_control(path, run_id, now, timeout):
    """Heartbeat timestamps are stamped on the robot, never on the laptop."""
    if not path:
        return {'mode': 'run', 'resume_generation': 0}, ''
    try:
        data = json.loads(Path(path).read_text())
        age = now - float(data['received_monotonic'])
        if data['run_id'] != run_id or not math.isfinite(age) or not 0 <= age < timeout:
            raise ValueError('expired or wrong session')
        if data['mode'] not in ('hold', 'run', 'abort'):
            raise ValueError('unknown control mode')
        int(data['resume_generation'])
        return data, ''
    except (OSError, ValueError, TypeError, KeyError):
        return {'mode': 'hold', 'resume_generation': 0}, 'coordinator_lease_expired'


class MotionRecovery:
    def __init__(self, timeout=15.0, stable_seconds=1.0, max_pauses=3, max_pause_seconds=30.0):
        self.timeout = timeout
        self.stable_seconds = stable_seconds
        self.max_pauses = max_pauses
        self.max_pause_seconds = max_pause_seconds
        self.state = 'RUNNING'
        self.reason = ''
        self.pause_count = 0
        self.paused_seconds = 0.0
        self.pause_started = None
        self.good_since = None
        self.resume_after = 0

    def tick(self, now, fault='', permit=True, generation=0):
        if self.state == 'ABORTED':
            return self.state
        if self.state == 'RUNNING' and (fault or not permit):
            self.state = 'PAUSED'
            self.reason = fault or 'fleet_pause'
            self.pause_count += 1
            self.pause_started = now
            self.good_since = None
            self.resume_after = generation
        if self.state == 'PAUSED':
            elapsed = now - self.pause_started
            if (elapsed >= self.timeout or self.pause_count > self.max_pauses or
                    self.paused_seconds + elapsed >= self.max_pause_seconds):
                self.state = 'ABORTED'
                self.reason = 'recovery_budget_exhausted: ' + self.reason
                return self.state
            if fault:
                self.reason = fault
                self.good_since = None
                # A resume sent before a fault cleared must not become a delayed auto-resume.
                self.resume_after = generation
            elif self.good_since is None:
                self.good_since = now
                self.resume_after = generation
            elif self.ready(now) and permit and generation > self.resume_after:
                self.paused_seconds += elapsed
                self.pause_started = None
                self.state = 'RUNNING'
                self.reason = ''
        return self.state

    def ready(self, now):
        return (self.state == 'PAUSED' and self.good_since is not None and
                now - self.good_since >= self.stable_seconds)

    def total_paused(self, now):
        return self.paused_seconds + (0.0 if self.pause_started is None else now - self.pause_started)
