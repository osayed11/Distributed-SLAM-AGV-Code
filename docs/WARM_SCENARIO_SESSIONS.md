# Warm Scenario Sessions

Status: implemented for S1 circles and exercised in four-robot runs on
2026-10-22; qualification remains specific to each robot and configuration.
This is not a safety-certified system. An accessible physical stop and an
operator remain required. Completed motion does not certify sensor continuity.

## Latest Lab Evidence: 2026-10-22

AGV100, AGV107, AGV110 and AGV24 completed a 180-second fleet circle run
(`502842619aba451f9ab52604cd41b88c`) without controller pauses. MCAP bags were
readable and recorded colour/depth averages exceeded 10 Hz. The audits still
flagged IMU coverage: AGV100 lost most of its run's IMU, AGV110 lost approximately
the final 20 seconds, and AGV24 had a roughly two-second gap. AGV107 had no hard
bag-validation failures, with image-gap and optional-topic warnings remaining.

After one AGV110 camera recovery, a 300-second stationary MCAP test
(`agv110_stationary_imu_20261022_142229`) recorded colour/depth at 13.5/14.1 Hz
and gyro/accel at 198.4/99.9 Hz. The missing IMU tail did not recur, but a 2.05-second
IMU receipt gap and roughly two-second sample-timestamp gap remained. Other
streams, including GT in the same auxiliary recorder, continued during that gap.
Separate GT delivery gaps occurred elsewhere. This does not establish the failing
physical component or prove a permanent fix. The stationary diagnostic deliberately
published no velocity commands, so its missing `cmd_vel` audit finding is expected.

After AGV100 was power-cycled, Doctor preflight passed 61 checks with no warnings
or failures (`agv100_20261022_135953`). Its initial preflight exposed a diagnostic
startup race: the topic snapshot preceded camera initialization. Doctor now waits
for all expected data topics, not only the base/LiDAR topics, before probing rates.
This is a diagnostic fix, not an established fix for the earlier IMU dropout.
The subsequent 180-second stationary S1 dry-run recording
(`agv100_stationary_doctor_retest_20261022_140435`) retained gyro/accel throughout
at 199.7/100.6 Hz, colour/depth at 12.5/13.1 Hz and GT at 74.9 Hz. Maximum IMU
receipt gaps were 0.159/0.160 seconds; there was no missing tail or HID timeout.
The strict audit was still not clean: LiDAR had one 0.355-second gap, odometry
and TF each had one coincident 0.325-second gap, and the stationary evaluation
window intentionally contained no velocity commands. Preserve these findings;
one recovered recording does not prove a permanent fix or motion readiness.

Keep raw bags and per-sensor findings. A failed continuity criterion is not an
automatic judgement that every modality or interval is unusable. Select and
document usable benchmark intervals consistently, preserve dropout annotations,
and never present filled-in IMU values as measurements. Do not claim all four
robots are gap-free or that an eight-robot session has already been validated.
Before the next run, confirm identity, clear paths, current sensor/GT availability,
clock synchronization and recorder readiness on the complete intended fleet.

## 1. Prepare Once

Keep the existing laptop NatNet source and official Zenoh router running for the
whole session in dedicated terminals. Do not run duplicate sources or routers.
Their commands remain in the README's Zenoh section. A terminal/tool session
exiting is not a reason to restart every robot; restore the failed GT component.

Deploy the changed files and rebuild the base driver on each robot, stationary:

```bash
cd ~/slam_project
sudo loginctl enable-linger ubuntu
source /opt/ros/humble/setup.bash
cd agv2_ws
colcon build --packages-select myagv_odometry --symlink-install
colcon test --packages-select myagv_odometry
colcon test-result --verbose
```

User lingering keeps the user's IPC objects after SSH logout. Without it,
logind can unlink Fast DDS shared-memory files while the system sensor service
continues running: topics remain discoverable but new subscribers get no data.
Preparation checks this prerequisite. This does not start a scenario at boot.
The laptop router command explicitly sets localhost-only DDS because the bridge
CLI default can override the value in the JSON configuration.

The base now expires commands using a monotonic clock (default 0.3 s), protects
shared command state, and bounds serial reads/writes. This covers a stopped or
crashed command publisher while the base process is healthy. It does NOT prove
the MCU stops if the Pi/base process dies; test the MCU timeout with wheels raised
before enabling fleet motion. A software zero command cannot prove physical stop.

Create a run configuration from `configs/s1_fleet.example.json`. Its centre and
yaw offset are examples, not surveyed values. Set each robot's actual hostname,
pose topic, measured forward-yaw offset, centre, radius and direction. Confirm
clear swept footprints including braking distance, not just centreline circles.

The top-level `linear` is the default speed command. An optional `linear` on
each robot overrides it (positive, at most 0.2 m/s), including its minimum-speed
setting. Use measured motion to calibrate commands; the base encodes linear
commands in 0.01 m/s steps. These overrides are saved in run manifests and do
not provide closed-loop speed or inter-robot spacing control. Recheck spacing
during motion; equal commands do not guarantee equal physical speeds.

Instead of measuring positions and typing centre/radii by hand, plan from where
the robots stand. With the MoCap router running on this laptop (it serves the
poses the robots use), a base config needs only each robot's name, host, pose
topic and calibrated `forward_yaw_offset_deg` plus the run defaults:

```bash
python3 scripts/scenarios/fleet_plan.py --config configs/my_fleet_base.json \
    --output configs/my_s1.json --direction cw --rings 1.0,1.7
```

It reads one pose per robot from the router's REST endpoint (no ROS on the
laptop), fits the shared centre to the requested rings (or uses `--center x,y`,
or the midpoint of the outermost pair when robots are lined up on a diameter),
sets every radius from the measured distance, and checks that each robot faces
its tangent for the chosen direction, that rings are ≥ 0.6 m apart and that
robots sharing a ring are ≥ 1 m apart. It refuses with the robot's name and the
fix ("turn it left", "0.18 m off its 1.0 m ring") instead of letting the
precheck fail a minute later; `--force` writes anyway. The measured placement
is kept under `plan` in the written config.

From the laptop:

```bash
python3 scripts/scenarios/fleet_session.py --config configs/my_s1.json --action prepare --password
```

`prepare` starts a non-moving systemd sensor service per robot. It refuses to kill
unmanaged drivers/recorders or reuse a changed boot/configuration. Remove
`--password` when SSH keys are configured. Credentials are prompted, never stored
in the configuration or manifests. Robots need passwordless sudo for service
management; failure is explicit, not an interactive hang.

Existing Robot Doctor and full under-recording qualification remain mandatory
for a new robot, changed hardware/software, or a diagnosed fault. Warm checks
do not turn a previous readiness result into a permanent certification.

## 2. Run And Recover

```bash
python3 scripts/scenarios/fleet_session.py --config configs/my_s1.json --action run --password --yes
```

The coordinator performs no unrecorded movement. All robot wrappers check live
streams concurrently, verify the base watchdog parameter and free space, start
stock MCAP recorders, then arm the controller. Every controller must be ARMED
before a common future start epoch is released. Late starts are rejected.
The explicit `--yes` means the operator has checked identity and clear paths.
Initial steering/axis qualification must already have passed separately.

| Event | Motion | Recording and services |
| --- | --- | --- |
| MoCap stale, pose implausible, heading/radius outside corridor | Stop; fleet holds | Continue while waiting for bounded recovery |
| Stable feedback returns | Remain stopped | Continue recording |
| Operator types `r` then Enter, all robots recovery-ready | Resume same fixed paths | Record the interruption; no centre reset |
| Recovery deadline/budget exhausted | Abort attempt | Finalise bags; retain healthy sensor services |
| Sensor rate below quality target or IMU timeout log message | Continue if motion feedback is safe | Keep recording; preserve warning and evidence for post-run review |
| Recorder dies | Abort attempt | Preserve evidence and finalise remaining bags |
| `q` then Enter, Ctrl+C, or coordinator/SSH failure | Abort/expire robot-local lease | Bounded cleanup; independent base command timeout |

Recovery requires one continuous second of fresh, plausible poses by default.
Repeated/future/stale source timestamps and frame changes are rejected. Motion
timers and leases use monotonic time; source clock checks require synchronized
robot/laptop clocks. The default pose timeout is NOT relaxed for recovery.
The recovery allowance is time spent stopped, not permission to drive blind.
Current budgets: at most three pauses, at most 30 s cumulative stopped time;
the example permits 30 s for one recovery. Do not tune limits mid-run.

Fleet pause detection is bounded by status/heartbeat periods, not an atomic
physical all-stop. Local controllers stop on their own fault without waiting for
SSH. Validate detection and braking times in the real environment before use.

Colour/depth quality minima are 10/10 Hz, with acquisition still configured for
15 Hz. The S1 live check reports low rates as warnings, not launch failures.
Raw gyro/accel quality targets remain 150/75 Hz, LiDAR 10 Hz, odom 8 Hz; GT defaults
to 30-120 Hz. A required stream with fewer than two samples or no fresh sample
blocks launch. GT freshness, valid source time, duplicate-source checks and
exclusive command ownership still block unsafe motion. During motion, existing
GT recovery and physical safety limits are unchanged. An IMU log timeout alone
does not prove the data is unusable: S1 preserves the evidence and continues
recording, even if that modality later proves incomplete. Post-run validation
reports the actual gaps and missing data; continuation is not a dataset PASS.
These collection rules apply to S1; standalone qualification/Doctor and strict
post-run audits can still fail quality checks without stopping an active run.
Slow subscription establishment is logged, not fatal once all required recorder
subscriptions exist; no motion starts before that readiness barrier.

After a normal attempt, drivers stay running. Reposition with motion stopped,
check clearances, and run the same command for a NEW run ID and new bags.
No files are automatically deleted. Target warm startup is under one minute;
this is a target to measure live, not a measured guarantee.

### Recovery budget and automatic resume

A pose fault (stale, implausible jump, lost coordinator lease) stops the robot
and holds the whole fleet; nothing restarts while any robot still reports a
fault. How long the fleet may stay stopped, and who releases it, come from the
run configuration:

| key | default | meaning |
|---|---|---|
| `recovery_timeout` | 30 | longest single stop (s) before that robot aborts |
| `max_pauses` | 3 | stops allowed per robot per run (0 = any fault aborts) |
| `max_pause_seconds` | 30 | cumulative stopped time per robot per run |
| `recovery_stable_sec` | 1.0 | fault-free time before a robot counts as recovery-ready |
| `auto_resume_sec` | 0 | > 0: the coordinator releases by itself once *every* robot has been recovery-ready for this long; a new fault on any robot resets the timer. 0: only `r + Enter` releases |
| `link_abort_sec` | 15 | coordinator↔robot link silence tolerated before abort; shorter stalls hold the fleet (the robot already stops when its 2 s lease expires) |

The defaults reproduce the original conservative behaviour. The example
configuration uses the lab values (45 / 20 / 120 / 1.5 / 3 / 15), which let
the 0.5–3 s ground-truth gaps seen on this Wi-Fi pause-and-continue instead of
ending the session. Every pause is still recorded in `motion.json`, the events
log and the circle summary (`pause_count`, `paused_sec`, `interrupted`), and the
bag validator's windowed checks will show the gaps; auto-resume changes who
presses the button, not what is recorded.

## 3. Validate And Finish

Each robot now audits its own run as soon as its recorders have stopped: the
worker runs the bag validator (index-based, about a second) and the circle
summary check, writes `validation.json` / `validation.log` next to the bags,
and reports the verdict in its FINISHED status, so the coordinator prints a
per-robot verdict table when the session ends. The manual action below remains
for re-auditing a run with different limits or after copying bags.


The laptop prints a run ID and keeps a manifest/event timeline under
`data/fleet_sessions/<run_id>/`. Robots keep configuration, artifact paths,
logs and results under `~/agv_data/fleet_<run_id>/`; MCAP shards remain in
`~/agv_data/`. Control/heartbeat files use `/run/user/<uid>` (tmpfs), not the SD card.

Run the existing strict bag validator and motion validator after recording:

```bash
python3 scripts/scenarios/fleet_session.py --config configs/my_s1.json --action validate --run-id <run_id> --password
```

Validation is deliberately separate from the run so heavy scans do not compete
with another recording on a Pi. Do not run this while the next bag is recording.
Successful collection is `PENDING`, NOT dataset-ready. A pause is stored in the
motion summary and events and fails uninterrupted-scenario acceptance even if
the requested active motion duration was eventually reached. Retain partial
data with explicit labels; do not interpolate away GT gaps or silently stitch
segments. Use the original run configuration for validation, not a different
fleet configuration. Calibration/image content remain separate checks.

End the session only after motion and recorders have exited:

```bash
python3 scripts/scenarios/fleet_session.py --config configs/my_s1.json --action stop --password
```

Then stop the laptop source/router terminals. Do not stop sensor services while
a scenario is active. The legacy single-robot wrapper also supports warm mode:
`python3 scripts/scenarios/sensor_session.py prepare <robot>` first. Explicit
`S1_BRINGUP_MODE=cold` owns and closes its own bringup, but never kills an unknown
existing stack. Straight/square scripts have not acquired the S1 recovery state
machine and must not be described as doing so.

## Qualification Before Collection

1. Build/test the driver; verify command loss stops wheels with wheels raised,
   including base/Pi failure to establish the actual MCU watchdog behaviour.
2. One robot: stationary MCAP check, steering check, then short controlled motion.
   Inject a brief GT interruption and prove stop, hold, deliberate resume, correct
   active/wall times and an interrupted (not clean-pass) validation verdict.
3. Fleet: verify all-ready release, one-fault fleet hold, expired coordinator
   lease, sustained dropout, recorder failure and two consecutive warm retries.
   Measure stop/start timings. Then record and strictly validate a scenario.

Offline regression checks:

```bash
python3 scripts/logging/test_drive_mocap_circle_ros2.py
python3 scripts/logging/test_circle_recovery_integration.py
python3 scripts/scenarios/test_session_recovery.py
bash -n scripts/scenarios/run_s1_mocap_pilot_robot.sh
```
