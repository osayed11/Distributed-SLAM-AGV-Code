# IMU Investigation Handoff: 28 October 2026

## Scope And Current State

Investigate intermittent RealSense raw IMU stream stalls using the existing
Robot Doctor pipeline. Do not treat a service restart as a proven permanent fix.
The stationary comparison was paused at the operator's request, and its test
processes were stopped. No motion is authorized by this handoff.

Latest operator roster update: **AGV105 replaces AGV106** in Circle 1's outer
position. This is an operator-reported substitution, not a qualification result
for AGV105. Update the next run configuration and verify its identity, live pose,
forward axis and steering before motion; do not reuse AGV106's calibration.

Intended roster after that substitution:

| Group | Direction | Inner robots | Outer robots |
| --- | --- | --- | --- |
| Circle 1 | Counter-clockwise | AGV101, AGV102 | AGV100, AGV105 |
| Circle 2 | Clockwise | AGV104, AGV107 | AGV108, AGV109 |

Last configured radii were 0.8 m inner, 1.5 m Circle 1 outer and approximately
1.8 m Circle 2 outer. Repositioning can invalidate the saved centres and clearance
assessment. The operator must authorize the next movement.

## Observed Failure, Not Yet Its Cause

Both `/camera/gyro/sample` and `/camera/accel/sample` stop while RGB-D, LiDAR,
odometry and ground truth continue. Camera logs contain:

```text
backend-hid.cpp:641 iio_hid_sensor: Frames didn't arrived within the predefined interval
```

Restarting the sensor service restored streams, but subsequent runs failed on
other robots. This implicates the camera/HID acquisition path without proving
whether the initiating cause is camera, cable, power, kernel, SDK or host load.
No USB voltage/current measurement was collected. Do not infer power health from
charging state, or a hardware fault solely from this timeout.

| Run ID | Observed IMU outcome |
| --- | --- |
| `7a6cd3f4e2324bb88cabc4b1d621955e` | AGV102 missing approximately final 26 s of bag, including approximately 16 s of movement; other seven continued |
| `ba9da8aaf4f243cc982f96d5147c0561` | AGV102 continued after restart; AGV108 missing approximately final 24 s of bag, including approximately 10 s of movement |
| `8f1a0685f2be4ababe9c04ece0c7138e` | AGV102/108 continued; AGV109 missing approximately final 58 s of bag; AGV106 had an approximately 0.50 s interior gap |

The last run completed 120 active motion seconds with two fleet GT pauses and
recoveries. Average colour rates were approximately 11.56-12.19 Hz and depth
12.55-13.27 Hz. These averages do not certify complete IMU coverage or all dataset
requirements. Preserve and report the actual missing intervals.

Version differences observed in bringup logs must be checked, not assumed away:

| Robot | ROS wrapper / runtime SDK | D455 serial | Firmware |
| --- | --- | --- | --- |
| AGV109 | 4.58.3 / 2.58.3 | 333422300768 | 5.17.0.10 |
| AGV108 | 4.58.4 / 2.58.4 | 333422300124 | 5.17.0.10 |

Record loaded library paths/hashes, backend and local patches as well as version
strings. IMU calibration warnings/default calibration are a separate quality
concern; they do not by themselves explain a stopped stream.

## Evidence Locations

- Laptop evidence root: `/tmp/orkar_lab_20261028/`.
- Per-robot timing summaries: `agvN_<runid>_bag.json` under that root.
- Fleet event timelines/manifests: `fleet_sessions/<runid>/` under that root.
- Robot artifacts and MCAP path references: `~/agv_data/fleet_<runid>/artifacts.json`.
- Live sensor log: `~/.local/state/orkar/agvN/bringup.log`; use the copied per-run
  logs when investigating an earlier failure because service restarts change it.
- Partial AGV109 A/B evidence:
  `~/agv_data/diagnostics/imu_ab_20261028_185557` on AGV109, copied to
  `/tmp/orkar_lab_20261028/imu_ab_20261028_185557` on the laptop.

The temporary test harness is `/tmp/orkar_imu_ab.py` on the laptop and AGV109.
Phase A was interrupted; phase B did not run. **There is no completed A/B result.**
Copy temporary evidence to durable project storage before machine cleanup.

## Next Discriminating Test

1. Capture Doctor state, exact versions/backend/launch settings, serial, USB link,
   kernel log and existing failure log before resetting anything. AGV109 was on
   **battery**; leave camera, cable and power arrangement unchanged for the pair.
2. Compare stationary streaming-only against full local MCAP recording with the
   same topic observer, profiles and startup procedure. Track last raw IMU samples
   against HID errors, CPU/I/O/memory pressure, temperature/throttling and kernel
   USB events. Preserve recorder and driver logs. Repeat/reverse the order and
   control time since startup if an apparent difference emerges.
3. If needed, compare ROS with standalone acquisition using the **same SDK and
   backend**, then perform a one-variable hardware swap. Do not compare different
   SDKs and call the result a ROS-only effect. Report measured facts, remaining
   hypotheses and the next discriminating test separately.

Do not suppress warnings, silently relax quality criteria, upgrade firmware
during collection, or reset mid-test without recording the intervention. A fix
needs repeat full-recording evidence, including a restart, before fleet rollout.

## Integrated Software Review

Retained upstream model-definition refresh, configurable recovery/link budgets,
live-pose planning and fast post-run timing checks. Retained lab-tested shared
phase timing, separation checks, atomic motion status and GT-only recovery.

Review corrections prevent stale NatNet body-ID reuse, reject multi-centre input
to the single-circle planner, recalculate phase speeds after radius changes, and
distinguish index-only `TIMING_PASS` from full payload-read validation. MCAP
fallback reads sequentially when message indexes are absent. Full validation
checks available CRCs but does not certify image semantics or calibration.

The integrated snapshot has automated regression coverage, **not a new live fleet
qualification**. Pull the same commit deliberately, preserve robot-local changes,
and perform a short stationary check before the next authorized scenario.
