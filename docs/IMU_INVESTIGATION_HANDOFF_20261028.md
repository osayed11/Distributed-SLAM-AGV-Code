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

## Findings Added 28 October 2026 (evening), Read-Only

Measured on AGV108 and AGV109 from their own logs and bags; nothing was reset,
re-plugged or re-flashed, and no motion was commanded.

**1. The stall leaves no kernel trace.** Every `uvcvideo ... Non-zero status
(-71)` / `hid-sensor-hub: No report with id 0xffffffff` pair in both kernels
sits 40 s after a *service restart* (AGV109 18:05:52 → 18:06:32, 18:56:02 →
18:56:43; AGV108 18:41:02 → 18:41:42): they belong to the camera start-up
sequence, not to the failures. At the actual stall times — AGV109 ≈18:45:09
(run `8f1a0685`, gyro/accel last sample 58.4 s before bag end) and AGV108
≈18:38:29 (run `ba9da8aa`, 23.9 s before end) — the kernel logged nothing on
`usb 2-2`, xHCI, HID or IIO. The UVC interfaces kept streaming (colour/depth
last samples 0.0–0.1 s before bag end in every run).

**2. Recording is not required.** AGV109's first stall today began at
18:04:04 with no bag open (previous bag closed 17:57:34, next activity 18:04:52)
and no motion; the node had been streaming since 17:50:07. The only host
activity in the preceding minutes was `apt-daily` (18:02:40–18:03:15) —
AGV109's apt timers are still enabled (AGV108's are masked). Host pressure
during today's runs is low (PSI cpu some avg10 ≈ 9, io ≈ 1–2, mem 0;
`get_throttled` 0x0 since boot on both, so no under-voltage event on either
Pi during the failures).

**3. SDK versions are not the discriminator.** Runtime libraries actually
mapped by the running `realsense2_camera_node` (from `/proc/<pid>/maps`):
AGV109 `/opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so.2.58.3`,
AGV108 `.../librealsense2.so.2.58.4`; both stall. Hazard found while checking:
both robots also carry Intel's standalone `librealsense2 2.58.1`
(`/usr/lib/aarch64-linux-gnu/librealsense2.so.2.58.1`, md5 `5650ea0b…`) with
the **same soname** `librealsense2.so.2.58`; plain `ldd` resolves to 2.58.1
and only the ROS environment's `LD_LIBRARY_PATH` makes the node load the ROS
build. Any standalone probe (Doctor stream tests, `rs-enumerate-devices`) is
therefore exercising a *different* SDK build from the node.

**4. Same backend everywhere.** Kernel HID/IIO path on both: `hid_sensor_hub`
→ `hid_sensor_gyro_3d`/`accel_3d` → `/dev/iio:device0/1` (buffer length 128
samples ≈ 0.64 s of gyro), librealsense `backend-hid.cpp` (`iio_hid_sensor`).
Kernels differ (AGV108 5.15.0-1061, AGV109 5.15.0-1105) and both stall.

**5. Time-since-start is not deterministic.** Stalls occurred 14 min (AGV109,
idle), 13 min (AGV108, in-run) and 39 min (AGV109, in-run) after the node
started; at 20:20 both nodes had streamed 1 h 25–40 min without a stall.

**Instrument now armed (read-only) for the next stall.** The healthy camera
delivers exactly **300 HID input reports/s** on
`/sys/kernel/debug/hid/0003:8086:0B5C.0001/events` (200 gyro + 100 accel).
`scripts/diagnostics/imu_hid_probe.sh` samples that stream, IIO buffer state,
PSI, throttling and the kernel log; `/tmp/imu_stall_watch.sh` on AGV108/109
runs it automatically when `bringup.log` gains a HID-timeout line, and again
20 s later. A 1 Hz CSV (`~/agv_data/diagnostics/psi_agvN_*.csv`) logs PSI,
temperature, throttle bits and the HID report rate. **If the stall shows 0
reports/s, the camera/USB stopped sending (device side); if it shows 300, the
kernel IIO/librealsense path stopped consuming (host side).** That single
number decides the next test (firmware/power/cable vs backend/kernel), and a
cheaper recovery than restarting the whole service — toggling the motion
module (`enable_gyro`/`enable_accel`) — should be tried once the side is known.

Riyaadh's phase A evidence (`imu_ab_20261028_185557/A_streaming`) covers only
19 s before interruption and contains no stall; it neither supports nor
refutes a recording effect.

## Controlled Stationary Comparison, 28 October 2026 19:34–… UTC (AGV106, AGV103)

Operator provided AGV106 (ROS SDK 2.58.4, kernel 5.15.0-1061) and AGV103
(2.58.3, 5.15.0-1105), both freshly booted, stationary, no motion commanded.
Harness: `~/agv_data/diagnostics/imu_soak_20261028_193429_agv10{6,3}/` — same
cold bringup as the pipeline, a per-second topic-count observer, a 1 Hz
PSI/temperature/throttle/**HID-report-rate** log, and the pipeline's two
recorders (same topics, MCAP profile, QoS overrides, cache sizes) in the
recording leg. Order reversed between robots (106: streaming→recording,
103: recording→streaming).

| robot | leg | time to stall | host at stall | HID reports/s at stall | kernel log |
|---|---|---|---|---|---|
| AGV106 | **streaming only** | **≈15 s** after `RealSense Node Is Up` (19:35:54) | PSI cpu 3, io 0.03, 42 °C, throttle 0x0 | 301 → 233 → **0** | nothing |
| AGV103 | **full recording** | **134 s** (19:36:47), bag at 879 MB and still writing | PSI cpu 17, io 0.5, 49 °C, throttle 0x0 | 300 → 35 → 72 → **0** | nothing |

Both legs stall; recording is neither necessary nor protective. In the failed
state the camera still answers HID control requests (feature GET of the IIO
sampling frequency returns in 46 ms), UVC keeps delivering colour/depth at
rate, and the HID interrupt endpoint (`0x86`, 250 µs interval) simply carries
no reports. So: **the D455 stops producing IMU input reports while the USB
link, the UVC pipes and its control path stay alive.** Not host load, not
recording I/O, not SDK version, not kernel version, not throttling.

**Recovery without a service restart (tested on both, logged as an
intervention in `soak.log` / `intervention.json`):** `ros2 param set /camera
enable_gyro false`, `enable_accel false`, 2 s, then both `true`. HID reports
were back at ~300/s within **3 s** on both robots and `/camera/gyro|accel/
sample` resumed at 199–200 / 100–102 Hz with no new timeout lines in the next
40 s. The motion module is re-armed by HID power/report-state writes alone,
which again points at the camera's motion-sensor report pipeline stopping
rather than anything on the host. This is a seconds-long gap instead of a
60–90 s service restart; it does not explain why the pipeline stops.

### Repeated recordings with the in-place re-arm (28 October 2026, 19:55–20:12 UTC)

Same two robots, same stationary set-up, the soak harness stopped and the
pipeline's two recorders run four times back-to-back (180 s each) while the
runtime guard watched the camera log with `--rearm-node /camera` (the opt-in
added as `S1_RUNTIME_IMU_REARM`, commit `6300ba0`). Each bag was then audited
with `validate_ros2_bag.py --require-imu`.

| run | AGV103 | AGV106 |
|---|---|---|
| 1 | PASS 43 / FAIL 0 — gyro 199.8 Hz, max gap 0.020 s | PASS 45 / FAIL 0 — 200.2 Hz, 0.031 s |
| 2 | PASS 44 / FAIL 0 — 199.7 Hz, 0.025 s | PASS 45 / FAIL 0 — 200.2 Hz, 0.019 s |
| 3 | PASS 45 / FAIL 0 — 199.7 Hz, 0.020 s | PASS 45 / FAIL 0 — 200.3 Hz, 0.021 s |
| 4 | PASS 45 / FAIL 0 — 199.7 Hz, 0.054 s | PASS 45 / FAIL 0 — 200.2 Hz, 0.027 s |

WARNs are the stationary set's expected ones (no `cmd_vel`, no GT, optional
topics absent, one 0.27–0.33 s image gap on AGV103 in run 1). During the
sequence AGV106's camera stalled once (20:02:43, between runs 2 and 3); the
guard re-armed it automatically (`accepted: true`) and run 3 recorded with
full IMU coverage. AGV103 did not stall again after its 19:54 re-arm. Evidence:
`~/agv_data/diagnostics/imu_rearm_recordings_20261028_1955*_agv10{3,6}/`.

**Status of the handoff's questions.** Root cause: device-side, i.e. the D455
stops producing IMU HID reports while its control path, UVC and the USB link
stay alive; recording, host load, SDK and kernel versions are excluded.
Which camera-internal factor (firmware, IMU chip, 5 V supply) cannot be
separated without the hardware tests the operator has declined for now
(USB voltage/current, one-variable camera/Pi/cable swaps). Mitigation: the
in-place re-arm restores the stream in ~3 s; detection latency is ~5 s
(librealsense's own timeout) plus the parameter writes, so a fired re-arm
leaves a gap of roughly 10–20 s in gyro/accel that the validator will show.
Eight consecutive full recordings, one of them following an automatic re-arm,
are the repeat evidence the handoff asked for; enabling the re-arm by default
in the fleet configuration remains a team decision. Not changed: firmware,
cables, power, profiles, quality criteria.

### Continuous 8-minute recordings (28 October 2026, 20:17–20:38 UTC)

Operator question: can a robot do an 8-minute run without the IMU dying?
Two 480 s stationary recordings per robot with the guard re-arm active:

| | AGV103 | AGV106 |
|---|---|---|
| run A (20:17) | **PASS 44 / FAIL 0**; gyro 199.7 Hz, max gap 0.031 s | stall at 20:22, re-armed (CLI path, ~15 s); bag shows a **21 s gyro / 24 s accel gap** → validator FAIL |
| run B (20:30) | stall at 20:33, re-armed (service call, 7 s); **16 s gap** → FAIL on gap threshold, coverage otherwise complete | IMU stopped at 20:37:47 and **ten consecutive re-arms (20:37:57–20:39:45, all "accepted", 1.5–2.9 s each) did not revive it**; bag coverage ends 16.9 s before the recorder stopped |

Reading: with the re-arm, a stall no longer ends a run, but it leaves a gap of
~5 s (librealsense's timeout) + re-arm time; tonight's gaps were 16–24 s. The
guard now (a) detects gyro/accel silence directly through a lightweight
subscription (1 s) instead of waiting for the log line and (b) re-arms through
one in-process service call (1.5–3 s), so the expected gap is ~3–5 s. AGV106's
camera additionally enters a state the toggle cannot recover (also seen in the
19:41–19:45 re-arm loop: 2 of 10 re-arms read 0 reports/s); that unit should be
treated as a camera to replace or exclude from IMU-critical runs, not as a
software problem. AGV103 ran 8 minutes clean once and lost 16 s once.
