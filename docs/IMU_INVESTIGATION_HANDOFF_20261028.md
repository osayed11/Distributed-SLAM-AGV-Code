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

### New unit, same stall — AGV112 commissioning bag, 29 October 2026

AGV112 is a freshly provisioned robot (installer 7e17063, pins unchanged) with a
new D455 (serial 337543062350, firmware 5.17.0.10, USB 3 link). Stationary, no
motion, no GT. Sequence, all UTC:

| time | event |
|---|---|
| ~16:58 | `start_session.sh` pre-run live gate: gyro 199.5 Hz, accel steady, max gap 0.024 s → PASS |
| 16:59:43 | camera log: first `backend-hid.cpp:641 iio_hid_sensor: Frames didn't arrived` |
| 17:00:56 | `ros2 bag record` starts (subscribed to `/camera/gyro/sample` and `/camera/accel/sample`) |
| 17:00:56–17:19:47 | bag: **0 gyro, 0 accel messages**; colour 15 828 / depth 15 933 frames recorded normally; 484 HID timeout lines until stop |

So the IMU died about 60 s after a clean gate and ~70 s before the recorder
opened, and never came back for 19 minutes while colour, depth, LiDAR and odom
continued. Validator: `imu: missing` → FAIL. This is the device-side stall
already characterised on AGV103/106/108/109, now reproduced on a new camera,
new Pi, new card and new cable; it is not specific to the older units. The
in-run re-arm guard (`S1_RUNTIME_IMU_REARM`) was not active in this
commissioning session. AGV110's bag from the same hour (808 s) had a complete
IMU; AGV111's (1158 s) had a complete IMU with ~1 s system-wide stalls.
Evidence on agv112: `~/agv_data/agv112_commission_stationary_20261029_165754*`,
`~/agv112-bag-validation-2.log`.

### AGV102 as a test rig — 2 November 2026 (stationary, camera only unless stated)

Rule for this section: every line is a measurement on agv102 today, or is marked
as a hypothesis with its test. agv102: Pi 4 2 GB, kernel 5.15.0-1110 (first boot
on it today), ROS librealsense 2.58.4 / camera 4.58.4, D455 serial 342343061372
on USB 3, fw 5.17.0.10, USB autosuspend off (live sysfs). Its own July logs show
HID timeouts 2.5–3.5 min after node start in 6 of 55 runs (kernel 1103, SDK
2.58.2), so the fault predates every autumn change.

Instrument: per-second CSV (gyro/accel/colour/depth topic rates, HID input-report
count from `/sys/kernel/debug/hid/0003:8086:0B5C.*/events`, USB power state,
CPU and camera temperature, PSI, dmesg delta); "hard stall" = HID reports < 50/s
for 30 s with no recovery in the following 120 s. Scripts: `~/imu_observer.py`,
`~/soak_node.sh`, `~/soak_with_base.sh`, `~/analyze_runs.py` on the robot;
evidence under `~/agv_data/diagnostics/imu102_*`.

| Run (UTC) | Configuration | Time to hard stall | Notes |
|---|---|---|---|
| 10:53 baseline | fleet bringup (launch file: base + lidar + colour/depth 640x480x15 + IMU) | **12 s** | no recovery in 5 min |
| 10:57 re-arm | param toggle on the stalled node | revived 300 reports/s for ~15 s, stalled again | toggle took 15.5 s on this checkout |
| 11:32 A | node only, colour+depth+IMU (+infra 848x480x30, my mistake) | **476 s** | 10 self-recovering 1–3 s micro-stalls before it |
| 11:44 B | as A + base driver running, 0 cmd_vel | **94 s** | |
| 11:50 A2 | node only, fleet-equivalent colour+depth+IMU, infra off | **151 s** | |
| 11:57 C2 | node only, IMU only (no video) | **56 s** | |

Also: Intel's standalone `rs-data-collect` (gyro only, 200 Hz, 60 s, no ROS)
recorded 11 941 samples with no gap; a librealsense hardware reset at node start
did not change the picture; all stalls occurred with camera temperature 30–33 °C,
CPU 54–60 °C, `throttled=0x0`, PSI low, kernel log silent, IIO buffers still
enabled and the camera's control channel still answering.

What this rules out **on this unit today**: video load (C2), the base driver
(A, A2, C2), the launch file, host thermal/power-throttle flags, USB autosuspend,
and anything that a librealsense hardware reset fixes. What it does not rule
out: camera firmware, the kernel HID-sensor path, USB-side electrical issues,
and the camera unit itself.

Measurement gotchas found today (they invalidated some of my first numbers):
- `ros2 run realsense2_camera_node` without `-r __ns:=/` publishes under
  `/camera/camera/...`; count HID reports, not topics, as the primary signal.
- `ros2 topic list` returns stale daemon entries right after a restart.
- The driver's "Frames didn't arrived within 5 seconds" lines come from the
  video backend every 5 s even while streaming; not a stall metric.
- Running the node directly opens infra1/infra2 at 848x480x30 unless disabled.
- The pip `pyrealsense2` 2.58.1 aarch64 wheel is a kernel-backend build (v4l2 +
  iio_hid symbols), so it cannot serve as a userspace-backend comparison.

Hypotheses queued with their tests: (1) firmware — Intel's v2.58.1 table lists
the D455 family at "5.17.3.10 or later" while the fleet runs 5.17.0.10; test =
flash agv102 to 5.17.3.10 and repeat A2/C2 (authorised 12:05, running);
(2) kernel HID path — test = same runs on kernel 5.15.0-1103, still installed on
agv102; (3) the unit — test = swap cameras with a robot whose camera has a clean
long run; (4) base-driver effect on time-to-stall (B vs A, n = 1) — needs repeats.

**Firmware trial, 2 Nov 12:02–12:28 UTC (agv102 D455 flashed 5.17.0.10 → 5.17.3.10, reboot, same runs):**

| Run | Configuration | fw 5.17.0.10 | fw 5.17.3.10 |
|---|---|---|---|
| full, fleet-equivalent, node only | colour+depth 640x480x15 + IMU | hard stall at 151 s (A2) | hard stall at 453 s (A3) |
| IMU only, node only | gyro+accel | hard stall at 56 s (C2) | hard stall at 149 s (C3) |

Both configurations still stall hard with no recovery on 5.17.3.10, with the
same signature (HID reports 300/s → 0 in one second, control path alive, kernel
silent) and the same 1–3 s micro-stalls beforehand. With n = 1 per cell and a
time-to-stall that spanned 12–476 s on the old firmware the same morning, the
longer times are not evidence of improvement. **Hypothesis 1 (firmware) is
not a fix.** agv102 stays on 5.17.3.10 for now (Intel's stated minimum for the
D455 row); note it when comparing it with fleet robots on 5.17.0.10.

**Base controller IMU as a fallback — measured 2 Nov 12:29–12:39 UTC on agv102
(myagv_odometry_node alone, `publish_imu:=true`, no cmd_vel, stationary):**
`/imu` 7 604 msgs in 600 s = 12.67 Hz, 12–13 per second, max gap 0.084 s, no
gaps > 0.5 s. Accelerometer plausible: z 9.879 m/s² (σ 0.051), x −0.034
(σ 0.019), y 0.030 (σ 0.060). **Gyro: exactly 0.0 on all three axes for the whole
run (σ 0).** Covariances published as zeros, frame `imu_link`, orientation zero.
So the base provides a 13 Hz accelerometer and no angular rate: not a
substitute for the camera IMU. An external IMU would have to be a real board.

**Kernel A/B, 2 Nov 12:40–12:57 UTC (agv102 booted on the previously installed
5.15.0-1103, same runs, fw 5.17.3.10; then reverted to 1110):**

| Run | Configuration | kernel 1110 (A3/C3) | kernel 1103 (A4/C4) |
|---|---|---|---|
| full, fleet-equivalent, node only | colour+depth+IMU | hard stall 453 s | hard stall 298 s |
| IMU only, node only | gyro+accel | hard stall 149 s | hard stall 85 s |

Same signature on both kernels. **Kernel package version (1103 vs 1110) is not
the fix.** Combined with the July logs (1103, SDK 2.58.2) and 112/103/106/108/109
(1061/1105), the stall is now measured across four kernel builds, two firmware
versions and three SDK versions. The remaining software-side candidate is the
kernel HID-sensor *path itself* (hid-sensor-hub → IIO), testable only by
bypassing it (userspace USB backend build, next).

**Re-arm guard measured on agv102, 2 Nov 12:59–13:10 UTC (fleet-equivalent full
streams, guard `--silence-seconds 1 --rearm-cooldown 3 --max-rearms 60`):**
60 re-arms in 10 minutes, every one accepted by the driver, each 1.1–4.5 s. The
IMU came back after every micro-stall for the first ~5.5 minutes; from ~340 s
the camera stopped answering the re-arm (HID 0 reports/s for the remaining
260 s while the driver kept accepting the parameter toggles). Total seconds
with IMU silent: 257 of 600. The re-arm therefore covers the micro-stalls on
this unit but not the hard stall; a camera-node restart is the next rung (it
revived 102 after every hard stall today, ~30 s), a USB replug was never
needed. On this camera the mitigation alone does not yield usable 8-minute IMU
coverage.

**Second camera, agv111 (D455 224223132684, fw 5.17.0.10, kernel 1110), same
instrument, 2 Nov 13:06–13:18 UTC:** full fleet-equivalent streams → hard stall
after **19 s**; IMU only → hard stall after **254 s**; neither recovered in 120 s.
This is the camera that recorded a complete-IMU 19-minute bag on 29 Oct. So the
behaviour is not peculiar to agv102, and a clean long bag on one day says
nothing about the next power cycle. Across 102 and 111 today: 11 hard stalls in
11 driver runs.

**Tool vs node on the same camera, agv111, 2 Nov 13:19–13:58 UTC (kernel HID
path in every case):**

| Run | Software | Result over 600 s |
|---|---|---|
| collector, IMU only | Intel `rs-data-collect`, standalone librealsense 2.58.1 | 119 370 gyro @ 199 Hz, 60 230 accel @ 100 Hz; two 2.05 s gaps; **no hard stall** |
| collector, colour+depth+IMU | same | 118 282 gyro, 59 635 accel, colour/depth 14.9 Hz; three 2.05 s gaps; **no hard stall** |
| ROS node, IMU only (repeat) | realsense2_camera 4.58.4 + ROS librealsense 2.58.4 | **hard stall at 165 s** (first run: 254 s) |
| collector, IMU only (repeat) | standalone 2.58.1 | one 2.06 s gap; **no hard stall** |

The 2.05 s micro-gaps appear on both paths and are therefore below the
library (device or kernel chain); the hard stall appeared only under the ROS
node: 3 of 3 collector runs clean, 3 of 3 node runs hard-stalled on this camera
(5 of 5 node runs on agv102). Tests now running: the ROS node with the
standalone 2.58.1 library preloaded in place of 2.58.4 (agv111); collector on
agv102 (worst camera); collector and node on agv110 (third camera). Userspace
backend builds finished on 110 and 102 (next: collector and node on that
backend).

**Third camera, agv110 (D455 342343060269, fw 5.17.0.10, kernel 1110), 2 Nov
13:59–14:15 UTC:** Intel collector, kernel path, IMU only, 600 s → **120 285 gyro
@ 200.5 Hz, 60 266 accel @ 100.4 Hz, zero gaps over 0.5 s**. Our ROS node, IMU
only, same camera, 5 minutes later → **hard stall at 134 s**, no recovery.
Tally across three cameras today: collector 5 of 5 ten-minute runs with no
hard stall (micro-gaps of 2.05 s on 111 only); ROS node 10 of 10 runs ended in
a hard stall. The hard stall is produced by our ROS camera-node stack, not by
the kernel HID path and not by any one camera.

**Library version vs wrapper, agv111, 2 Nov 14:07–14:17 UTC:** our ROS node run
with the *standalone* librealsense 2.58.1 preloaded in place of the ROS 2.58.4
build (mapping verified in `/proc/<pid>/maps`) still hard-stalled, at 247 s.
So the difference between Intel's collector (clean 5/5) and our node (hard
stall 11/11) is **not the library version**; it is in the wrapper
(realsense2_camera 4.58.4) or how it drives the motion module. The node also
started and streamed the IMU with the userspace-backend library preloaded
(gyro 200 Hz on the topic); that run was cut short by my own instrument, whose
stall rule counted kernel HID reports that a userspace backend bypasses — rerun
with a topic-based rule in progress.

**agv102 (worst camera), collector on both backends, 2 Nov 14:00–14:21 UTC, IMU only, 600 s each:**

| Backend | Gyro | Accel | Gaps > 0.5 s |
|---|---|---|---|
| kernel HID path (stock 2.58.1) | 118 520 @ 197.5 Hz, full span | 59 714 @ 99.5 Hz | three of 2.05 s (278, 469, 479 s); **no hard stall** |
| userspace USB backend (our FORCE_RSUSB build of 2.58.1, mapping verified) | **36 975 @ 63 Hz** | 18 638 @ 32 Hz | **179 gaps of ~2.3 s, one every 2.5 s, from the first second** |

The userspace backend delivers the IMU in bursts with a 2.3 s hole between
them on this platform: unusable as built. **Hypothesis "bypass the kernel HID
driver" is dead as a fix** (it may be configurable, but it is not a drop-in
improvement). Meanwhile the stock kernel path with Intel's collector has now
run 6 of 6 ten-minute windows on three cameras without a hard stall, against
11 of 11 hard stalls for our ROS node on the same cameras and path. A 60 minute
collector stress run with colour+depth+IMU on agv102 started 14:33.

**agv110, node with the standalone 2.58.1 library preloaded (mapping verified),
2 Nov 14:16 UTC:** hard stall at 89 s. Second camera confirming that the
library version is not what separates the node from the collector. Tally for
the ROS node: 13 of 13 runs ended in a hard stall (102, 110, 111; libraries
2.58.4 and 2.58.1; kernels 1103 and 1110; firmware 5.17.0.10 and 5.17.3.10).
Start/stop cycle stress of the collector (5 × 480 s, full streams, camera
reopened each cycle) running on 102, 110 and 111 from 14:48 UTC.

**Start/stop stress of Intel's collector on the worst camera (agv102), 2 Nov
14:24–15:06 UTC, five fresh 480 s cycles, colour 640x480x15 + depth 640x480x15 +
gyro 200 + accel 100, kernel HID path, camera closed and reopened each cycle:**

| cycle | gyro | accel | IMU gaps > 0.5 s | colour |
|---|---|---|---|---|
| 1 | 95 203 @ 199.8 Hz | 48 009 @ 100.7 Hz | 0 | 14.8 Hz |
| 2 | 95 046 @ 199.5 Hz | 47 865 @ 100.5 Hz | 0 | 14.8 Hz |
| 3 | 94 645 @ 199.5 Hz | 47 649 @ 100.4 Hz | 0 | 14.8 Hz |
| 4 | 95 044 @ 199.5 Hz | 47 834 @ 100.4 Hz | 0 | 14.8 Hz |
| 5 | 94 886 @ 199.5 Hz | 47 744 @ 100.4 Hz | 0 | 14.8 Hz |

40 minutes of IMU on the camera that hard-stalled our ROS node five times out
of five this morning, with not a single gap over 0.5 s (the only colour "gap"
is the 4.9 s first-frame start-up). Same cycles running on 110 and 114.

**Same cycles on the robot now named agv114 (camera 224223132684, ex-agv111),
14:25–15:08 UTC:** cycle 1 void (camera unplugged by hand for ~2.5 min); cycles
2–5: gyro 95 001–95 364 @ 200.3 Hz, accel 47 863–48 088 @ 100.9 Hz, **0 gaps
> 0.5 s in each**; a replacement sixth cycle is running. agv110's set pending.
All four robots' clocks checked at 15:08 UTC: chrony-synced within 1.5 ms.

**Same cycles on agv110 (camera 342343060269), 14:27–15:13 UTC:** cycles 1–2
void — the collector printed "Connect Realsense Camera to proceed" for 10
minutes each: the preceding userspace-backend node experiment had detached the
kernel drivers from the camera and the device was not visible to the kernel
path again until ~14:48 (lesson: after any libusb-backend run, check the
interface drivers or replug before kernel-path tests). Cycles 3–5: gyro
95 385–95 709 @ 200.5–200.8 Hz, accel 47 780–48 033 @ 100.4–100.8 Hz, **0 gaps
> 0.5 s**. Two replacement cycles running. Running tally for the collector
path, fresh 8-minute cycles with full streams: **12 valid cycles, 0 IMU gaps**,
across the three cameras that gave our ROS node 13 hard stalls in 13 runs.

### Candidate fix: take the IMU out of the ROS wrapper (2 Nov, afternoon)

Design: `realsense2_camera` keeps colour and depth (`enable_gyro`/`enable_accel`
false) and a small standalone node, `agv_bringup/scripts/rs_imu_publisher.py`,
opens the motion module through librealsense the way Intel's collector does and
publishes the same topics (`/camera/gyro/sample`, `/camera/accel/sample`,
sensor-data QoS, frames `camera_gyro_optical_frame` / `camera_accel_optical_frame`,
global-time timestamps), the same static TF chain and the two extrinsics
topics. Selected by a new launch argument `agv_imu_source` (`wrapper`, the
current default, or `standalone`); `start_session.sh` reads `IMU_SOURCE`, the
S1 runner reads `S1_IMU_SOURCE`. Draft only, uncommitted at the time of writing.

**First measurement, agv102 (worst camera), 15:38–15:50 UTC, wrapper video-only
+ standalone publisher sharing the camera, stationary, 10 min:** gyro 119 255 msgs
@ 198.8 Hz, accel 60 086 @ 100.1 Hz, max gap 0.33 s, no gap > 0.5 s, colour
14.9 Hz mean (min 13.2), publisher never below 199 Hz in any 10 s window, no
errors. Same camera, same path, our wrapper alone hard-stalled 5/5 in the morning.

Measurement gotcha that invalidated the parallel runs on 110/111/114: those
robots share one DDS domain on the lab network, so a subscriber on one robot
receives the identically named `/camera/gyro/sample` from every robot that is
streaming (three publishers, 600 Hz bursts, phantom gaps). Per-robot
measurements need `ROS_LOCALHOST_ONLY=1` and a publisher-count check. Reruns in
progress; an 8-minute MCAP recording of the split configuration on 102 through
`validate_ros2_bag.py` is the next gate.

**Split configuration, isolated measurements (ROS_LOCALHOST_ONLY=1), 2 Nov
15:57–16:08 UTC, 600 s each, stationary, wrapper video-only + standalone
publisher (v3):**

| Robot / camera | gyro | accel | max gap | gaps > 0.5 s | colour |
|---|---|---|---|---|---|
| agv110 / 342343060269 | 119 974 @ 199.96 Hz | 60 137 @ 100.2 Hz | 0.053 s | 0 | 14.9 Hz |
| agv111 / 224223133587 | 119 898 @ 199.83 Hz | 60 449 @ 100.8 Hz | 0.043 s | 0 | 14.9 Hz |
| agv114 / 224223132684 | 119 831 @ 199.72 Hz | 60 430 @ 100.7 Hz | 0.336 s | 0 | 14.9 Hz |
| agv102 / 342343061372 (15:38) | 119 255 @ 198.8 Hz | 60 086 @ 100.1 Hz | 0.33 s | 0 | 14.9 Hz |

Four cameras, four clean 10-minute windows, publisher never below 199 Hz in any
10 s window, versus 13 of 13 hard stalls for the wrapper's own IMU path on the
same cameras today.

**Under recording load** the picture is different and has to be separated
from the IMU question: the first split recording on agv102 (480 s, 6.1 GB
MCAP) kept 199 Hz gyro and full coverage but showed ten gaps of 0.25–1.44 s
that coincide with I/O pressure from the bag writer (io PSI ~31 %) and with
colour gaps, i.e. the SD write-stall class already documented in
`project_gt_staleness_recording_io`; the publisher itself dipped once. A second
102 recording with the publisher behind a 2048-frame queue at raised priority
and the IMU topics recorded with reliable QoS is in progress, and 10-minute
start_session.sh recordings with `IMU_SOURCE=standalone` are running on 110,
111 and 114 (draft deployed robot-locally with backups; not committed).

### End-of-day state, 2 November 2026 (~18:00 UTC)

**Established by measurement today**
- The D455 IMU hard stall is produced by our ROS camera-node stack: 13 of 13
  wrapper runs on three cameras ended in a hard stall; the same cameras,
  same kernel HID path, driven by Intel's collector or by a thin standalone
  publisher, ran 15 fresh 8-minute cycles and four 10-minute split windows
  with zero gaps over 0.5 s. Firmware 5.17.3.10, kernel 5.15.0-1103, the
  userspace USB backend, the base controller, the launch file and the
  library version were each tested and are not the cause or the fix.
- The split configuration (wrapper video-only + `rs_imu_publisher.py`,
  `agv_imu_source:=standalone`) streams cleanly in isolation on four cameras.

**Not yet achieved: the split configuration inside the recording pipeline.**
- agv110, start_session.sh, 600 s: IMU covered the bag (max gap 0.5 s) but
  the host stalled under the bag writer: colour 9.5 Hz, tf gaps up to 7.9 s,
  odom gaps up to 4.1 s. Without the recorder the same launch gives colour
  15 Hz, tf 12.7 Hz; the publisher costs ~35 % of one core (Python).
  Thursday's wrapper-IMU bag on the same robot had max gap 0.255 s.
- agv111 and agv114 sessions failed at start-up: once because wrapper and
  publisher opened the camera simultaneously (wrapper segfault; publisher
  `set_xu` failure), then colour did not come up even with the publisher
  delayed 25 s. Both still need a clean, graceful-stop retry.
- The session bags report gyro at 211–219 Hz and accel at 110 Hz, above the
  configured 200/100: suspect duplicate delivery from the publisher's
  `frame_queue(keep_frames=True)` path. Must be checked before any rollout.

**Lessons that cost time today (all recorded in memory):** pattern kills
hitting my own shell; SIGKILL leaving the camera's depth stream unstartable
until a librealsense hardware reset; robots sharing one DDS domain merging
same-named topics (use `ROS_LOCALHOST_ONLY=1` for measurements); a reliable
recorder QoS silently recording nothing from a best-effort publisher; a
script overwritten while running.

**Next session, in order:** (1) make the publisher cheap and verify the
218 Hz anomaly (C++ node or a leaner Python loop; confirm one message per
sample); (2) one graceful 600 s start_session recording on one robot with
the recorder, validator and doctor; (3) the moving 8-minute run with the
operator present; (4) rollout decision. Repo state: draft uncommitted
(launch arg, publisher script, CMake install, IMU_SOURCE / S1_IMU_SOURCE
pass-through); robot-local copies deployed on 110/111/114 with backups in
`~/draft_backup_*`; none of today's doc commits pushed (user rule).

## 2026-11-03 — agv110 only (102/111/114 powered off)

**The 218 Hz gyro anomaly is closed: it was not the publisher.** The 2 Nov
session manifest records `ros_localhost_only: "0"`. The stock ROS hook
(`/opt/ros/humble/share/ros_environment/environment/1.ros_localhost_only.*`)
exports `ROS_LOCALHOST_ONLY=0` whenever the variable is unset, so any
wrapper that sources `setup.bash` *before* `start_session.sh` (my
`deploy_and_record.sh` did) defeats start_session's set-if-unset auto rule.
The recorder therefore also took 111's and 114's `/camera/gyro/sample`
while their launches were up (per-100 s counts 27947, 20046, 20046, 20037,
21247, 21898: exactly 200 Hz whenever the others were down), and 110's
camera node was streaming colour to their recorders over Wi-Fi. The
publisher's own log is 200.4–200.6 Hz in every 10 s window; the bag has no
duplicate samples. Consequence: last night's colour 9.5 Hz / tf gaps are
confounded and have to be re-measured. Baseline for the recorder alone
(wrapper source, 29 Oct session on this robot, localhost-only 1): colour
13.9 Hz, depth 12.7 Hz.

**Publisher timestamps (measured at arrival with a subscriber, 90 s
windows, `ROS_LOCALHOST_ONLY=1`, no recorder):**

| configuration | gyro arrival−stamp | colour camera_info arrival−stamp |
|---|---|---|
| wrapper source (reference) | +0.050 s (p5 +0.047, p95 +0.054) | +0.067 s |
| standalone, publisher on librealsense global time (v6) | −0.52 s drifting to −0.91 s within 90 s | +0.29 s* |
| standalone, publisher v7 device clock + min-latency offset | +0.002 s (p95 +0.003), stable | +0.39 s, next launch +0.60 s |

\*image_raw probe, probe itself CPU-bound; later rows use camera_info.
librealsense "global time" inside the standalone process is unusable here
(stamps ahead of wall time and drifting ~4000 ppm). v7 (`--stamp-mode
hw-minlat`, now the default) tracks the running minimum of
arrival − device_time, so HID bursts only raise latency and are ignored;
it also exits cleanly on launch SIGINT (caught `ExternalShutdownException`).
Open problem exposed by the same probe: with the wrapper running
**video-only**, its own colour/depth stamps sit 0.39–0.60 s before arrival
and the bias changes per launch, versus +0.067 s when the wrapper also runs
the IMU. The wrapper's `rgb_camera/depth_module/motion_module.global_time_enabled`
read True in both modes and every log has exactly one "frame's time domain
is HARDWARE_CLOCK" line. A five-case test (wrapper alone ×2, +publisher,
+publisher with video global time off, alone with it off) is running to
separate wrapper-video-only behaviour from the publisher's presence.

Tools (robot `~`): `stamp_offset_probe.py <s> topic:type…`,
`stamp_test.sh <s> "<sources>"`, `gt_param_probe.sh`, `video_stamp_test.sh`,
`bag_imu_audit.py`, `bag_offset.py`; `~/deploy/run_clean_session.sh <robot>
<s>` forces localhost-only, samples host metrics, stops gracefully and
runs validator + both audits. Evidence: `~/agv_data/diagnostics/stampcmp_*`,
`gtparam_*`, `videostamp_*`.

**Five-case video-stamp test result (agv110, 30 s probes on camera_info,
`ROS_LOCALHOST_ONLY=1`, wrapper started with the bringup parameters via
`ros2 run`, publisher v7 started 15 s after the wrapper where present):**

| case | wrapper | publisher | colour arrival−stamp | depth | gyro |
|---|---|---|---|---|---|
| A | video-only, global time on | no | +0.110 s | +0.102 s | — |
| B | same, second launch | no | +0.072 s | +0.064 s | — |
| C | video-only, global time on | yes | **−1.90 s, drifting −1.0 → −2.8 s in 30 s** | −1.66 s | +0.002 s |
| D | video-only, `rgb_camera/depth_module.global_time_enabled=false` | yes | **+0.004 s** | **−0.004 s** | **+0.002 s** |
| E | same as D | no | +0.001 s | −0.006 s | — |

So the wrapper's librealsense global-time estimate for colour/depth is
destroyed by the standalone publisher's presence on the same camera (C),
while with the option off the wrapper's constant-base device-clock mapping
puts colour, depth and the publisher's gyro on one timeline within 8 ms
(D). The launch file now sets both `global_time_enabled` parameters false
whenever `agv_imu_source:=standalone` (draft, deployed on 110). Note for
wrapper mode: the 29 Oct wrapper-source bag has gyro, colour and depth all
≈0.2 s ahead of recorder receive time while odom is at 0.0 s, i.e. the
wrapper's global time was ≈0.2 s off wall time that day even without the
publisher; whether to switch wrapper mode to the constant-base mapping too
is a separate decision (not changed).

Clean 600 s `start_session.sh` recording with this configuration started
11:45:41 UTC on agv110 (`~/run_clean_session_agv110.out`).

**Pipeline sessions on agv110, 3 Nov (all `ROS_LOCALHOST_ONLY=1`, graceful stops):**

| run | publisher | start (UTC) | result |
|---|---|---|---|
| 1 | v7, keeper off (motion-module global time 0), no watchdog | 11:45 | recorder 600 s, every node exited cleanly; **IMU stopped 11:50:24** (publisher log 200.5 Hz → 0.0 Hz, queue 0), 4 min after start, ~95 s into recording; colour 14.3 Hz, odom/tf max gap 0.43 s; validator FAIL on IMU coverage (gyro covers 92 s of 604) |
| 2 | v8, keeper on, 1 s silence watchdog | 12:13 | start-up micro-gap of 1.5 s tripped the watchdog → restart storm (92 restarts, IMU never settled). Stopped by me. Threshold raised to 5 s with ≥30 s between restarts (v9) |
| 3 | v9, keeper on, wrapper video global time off (via `config_file`) | 12:21 | **IMU stopped 12:22:14**, 28 s after the motion module started, during the session's pre-run gate; kernel HID report counter 301/s → 26 → **0 for the rest of the run**; in-process `stop/close/open/start` of the motion module gave one short burst (51.8 Hz window) then nothing (12 restarts); session aborted at the IMU wait |

So the device-side stall (zero HID reports reaching the kernel) also occurs
with the standalone publisher under the recording pipeline: tally for the
standalone path with a recorder is now 2 clean (102 and 110, 2 Nov) vs 2
stalled (110, today); isolated 10-min runs 4/4 clean; Intel collector
15/15 clean. Both of today's stalls happened while the session's gate (four
`ros2 topic hz` processes at 40–50 % CPU each) or the recorder start-up
loaded the host, so a CPU-load trigger is the hypothesis under test now
(isolated split config + 3 CPU hogs, 600 s, HID counter logged).

Also measured: in run 1 the wrapper's video global time (the
`global_time_enabled` launch arguments were silently dropped by
`rs_launch.py`, which only forwards its own argument list) drifted from
−0.19 s to **+1.19 s** relative to recorder receive time over 10 min while
the publisher's gyro stayed at +0.001 s and odom at 0.000 s. The parameter
file `agv2_ws/src/agv_bringup/config/realsense_standalone_imu.yaml` passed
through rs_launch's `config_file` hook is the working route (verified: no
"not supported" warning, run 3).

**CPU load is not the trigger.** Isolated split configuration (video-only
wrapper with the standalone parameter file + publisher v9) under three
`yes` hogs for 600 s on agv110: load 5.2, SoC 80 °C, publisher 200.4–200.5 Hz
in all 60 windows (one 157 Hz window = start-up micro-gap), kernel HID
report rate never below 229/s, zero restarts
(`~/agv_data/diagnostics/stress_cpu_20261103_123033`). Next discriminator
running: the full bringup launch (odometry + lidar serial nodes + camera +
publisher) without session scaffolding, 600 s.

**Full bringup launch, no session scaffolding, 600 s (agv110 12:42–12:52):
stalled.** Odometry + lidar nodes + video-only wrapper + publisher v9. The
HID report rate was ragged from the first minute (10 s bins 118–301/s,
publisher windows 100–200 Hz) and went to **0 at 12:48:23**; nine
in-process restarts recovered nothing (`launchsoak_20261103_124231`). In
the five camera-only runs (four on 2 Nov, one under CPU hogs today) the
HID rate never fell below 229/s. USB topology on this Pi: the D455 is the
only device on the SuperSpeed root (bus 2); the only other USB device is
the base's CDC-ACM link (QinHeng 1a86:55d4, 12 Mbit/s full speed) behind
the VIA hub on bus 1 of the same VL805 controller; the lidar is not on USB.
Running now: camera-only split configuration + the odometry node alone,
600 s, to test the shared-controller hypothesis.

**Camera-only split configuration + odometry node alone, 600 s (12:54–13:05):
no hard stall, but the IMU is degraded the whole time** — publisher windows
80–200 Hz (typical 120/160 Hz = 2–4 s of every 10 s missing), kernel HID
bins down to 119/s, versus ≥229/s and 200 Hz windows in every camera-only
run (`splitplus_odom_20261103_125430`). The odometry node's serial link
to the base is the only other device on the camera's USB controller.

**Camera-only + lidar node alone, 600 s (13:06–13:17): no stall, same
degradation** — publisher windows 117–180 Hz for most of the run, HID bins
down to 102/s (`splitplus_lidar_20261103_130645`). The lidar is on the Pi
UART, not USB, so the shared-USB-controller explanation is dead. What the
odometry and lidar nodes share is continuous serial I/O, i.e. interrupt
load on the Pi; three user-space CPU hogs produced none of this. Next test:
interrupt affinity (camera's xhci IRQ on its own core).

**Interrupt topology on agv110 (same image fleet-wide):** the camera's
xhci controller is a PCIe MSI (IRQ 49) that lands on **CPU0 and cannot be
moved** (`echo 8 > smp_affinity` → I/O error, BRCM STB PCIe MSI). The lidar
is on the **mini-UART** (`/dev/ydlidar → ttyS0`, driver bcm2835-aux-uart,
8-byte FIFO, no DMA; `enable_uart=1`, PL011 left to Bluetooth), whose
shared "aux" interrupt (IRQ 20) had fired 36 M times, all on CPU0. The SD
card (mmc, IRQ 33) is also on CPU0 (the recorder's writes). `irqbalance`
is active but leaves these on CPU0. `config.txt` already carries
`dtoverlay=dwc2,dr_mode=host`, i.e. the USB-C port is a second, idle host
controller (IRQ 21) — a cabling option for the base link. Test running:
mini-UART IRQ moved to CPU1 and mmc to CPU2 (xhci keeps CPU0 to itself),
camera-only + odometry + lidar nodes, 600 s.

**Baseline for the A/B (13:20–13:31, odometry + lidar + camera-only split,
irqbalance had reverted the affinities within seconds):** no hard stall in
600 s, HID bins down to 121/s, publisher min window 80.8 Hz; interrupt
rates on CPU0 the whole time: mini-UART (lidar) **≈9,250/s**, xhci
(camera) ≈950/s, mmc ≈20/s (`splitplus_both_irqmoved_20261103_132039`).

**A/B result — interrupt affinity is not the lever.** With irqbalance
stopped, mini-UART on CPU1 and mmc on CPU2 (held for the whole run, xhci
alone on CPU0): 29 of 61 publisher windows below 195 Hz, min 96 Hz, HID
floor 120/s, versus 32 of 61, min 81 Hz, floor 121/s in the baseline
(`splitplus_both_irqB_20261103_133217`). For reference the camera-only run
under CPU hogs had 2 of 60 windows below 195 Hz. Next variable: CPU
frequency scaling / idle states (the serial nodes leave the cores idle
between ~10 k wake-ups per second; the hogs kept them at full clock).

**CPU frequency is not the lever either.** Governor `performance`
(1.8 GHz pinned; this kernel exposes no cpuidle states), affinities as in
the A/B run, odometry + lidar + camera-only split: 55 of 61 windows below
195 Hz and a **hard stall at 13:51** (HID 0 for the last 3.5 min)
(`splitplus_both_perf_20261103_134416`). Tally for "camera-only + both
serial nodes": 3 runs, 1 hard stall, all three heavily degraded.

**The trigger is open, streaming serial ports — not ROS, not IRQ placement,
not CPU frequency.** Camera-only split configuration + plain `cat` readers
on `/dev/ttyS0` (lidar, 9.2 KB/s) and `/dev/ttyACM0` (base, 1.9 KB/s
unsolicited), no odometry/lidar nodes, 600 s: **34 of 61 windows below
195 Hz**, same degradation as with the nodes
(`splitplus_raw_20261103_135653`). Both ports stream on their own as soon
as they are opened. Pi under-voltage flags since boot: `throttled=0x0`.

**Where this leaves the standalone-IMU route (3 Nov, agv110, end of day):**

| condition (600 s each) | runs | degraded windows (<195 Hz of 61) | hard stalls |
|---|---|---|---|
| camera-only split, idle or 3 CPU hogs | 5 | 0–2 | 0 |
| + odometry node only | 1 | 38 | 0 |
| + lidar node only | 1 | 30 | 0 |
| + both nodes (default / IRQs moved / performance governor) | 3 | 32 / 29 / 55 | 1 |
| + raw `cat` on both serial ports, no nodes | 1 | 34 | 0 |
| full bringup launch, no session | 1 | many | 1 |
| full start_session recording | 3 (+2 on 2 Nov) | — | 2 of 3 today (0 of 2 on 2 Nov) |

The D455's HID/IIO IMU path on this Pi loses 2–4 s of every 10 s and
sometimes stops for good whenever another serial stream is open on the
host, independent of which core takes the interrupts or how fast the CPU
runs. The in-process motion-module restart does not recover a hard stall;
a full device close/reopen does. Mechanism below the tty/USB layer not
identified today.

**Candidate mitigations, all needing the operator (not tested):**
1. Lidar off the mini-UART onto the PL011 (`dtoverlay=disable-bt`,
   `/dev/ydlidar → ttyAMA0`): cuts the 9.2 k interrupts/s to a DMA/FIFO
   path. config.txt change + reboot on a test robot first.
2. Base link onto the idle second host controller (USB-C, `dwc2,dr_mode=host`
   is already in config.txt; needs a USB-C OTG adapter), so the camera's
   VL805 carries nothing else.
3. Pi EEPROM/VL805 firmware update (bootloader is 2022-01-25).
4. If none of these restore the clean camera-only behaviour under real
   load, the external IMU is the remaining route; the standalone publisher
   and the split launch remain useful (correct stamps within 8 ms, clean
   shutdown) but cannot be called reliable in the integrated system.

Robot state at hand-back: agv110 governor `ondemand`, irqbalance running,
no test processes left, draft files installed (`~/draft_backup_*` holds
the originals), diagnostics in `~/agv_data/diagnostics` (56 MB), two
session bags from today in `~/agv_data` (6.1 GB + aborted ones).

**CAVEAT found at 14:08 — the serial-port table above is confounded.** My
HID report sampler (`cat /sys/kernel/debug/hid/…/events` as root) was never
killed between runs (the reader is exec'd as plain `cat`, my kill pattern
looked for `stdbuf`), so the runs accumulated 1 → 10 concurrent debugfs
readers; every HID report is string-formatted once per open reader inside
the report path. Reader counts per run: session 1 (stall) 0; session 3
(stall) 1; CPU-hog run 3; full-launch soak (stall) 4; odom-only 5;
lidar-only 6; both-nodes 7/8/9 (stall at 9); raw-serial 10. The 2 Nov
clean runs had none. The session stalls without readers are real; the
"degraded windows" numbers for the serial-port runs are not trustworthy
until repeated with zero readers. Re-run started 14:10: camera-only +
raw serial readers, no debugfs reader at all.

**RETRACTION (14:21).** Camera-only split configuration + raw `cat`
readers on both serial ports, **zero debugfs readers**: 0 of 61 windows
below 195 Hz, min 197.3 Hz (`splitplus_raw_noHIDreaders_20261103_141005`).
The serial-port degradation, the odometry-only/lidar-only numbers, the
IRQ-affinity and governor comparisons and the mitigation list that
followed from them are all artefacts of my leaked HID debugfs readers and
are withdrawn. What stands from today: the standalone publisher's stamps
are correct (hw-minlat, within 8 ms of colour/depth with the wrapper's
video global time off); the recording-session stall at 11:50 happened with
no probe attached (standalone + pipeline tally, probe-free: 2 clean on
2 Nov, 1 stall on 3 Nov); the Intel collector has never been run under the
real pipeline load — that run is now in progress (`collector_under_load.sh`).

**Intel collector under the real pipeline load — clean (14:23–14:34,
agv110).** Full bringup launch (`agv_imu_source:=none`: odometry, lidar,
wrapper video-only with global time off) + `ros2 bag record` with the
pipeline's mcap/QoS/cache settings writing 6.2 GB of colour/depth/odom/tf/
scan to the SD card + `rs-data-collect` (gyro 200, accel 100) for 600 s:
gyro 119 599 samples at 199.8 Hz, accel 59 842 at 100.0 Hz, span 598.7 s,
**one 2.05 s gap at 375.6 s (the known micro-gap class), no stall**
(`~/agv_data/diagnostics/collector_load_20261103_142237`, no probes
attached). First run of its kind; n = 1.

Probe-free tally for the D455 IMU under the real pipeline load:
Intel collector 1/1 clean; standalone Python publisher 2 clean / 1 stall;
ROS wrapper 0 clean in every recording that ran long enough.

**C++ publisher on the collector's API path (`rs_imu_publisher_node`,
rs2::pipeline + frame callback, device-clock stamps, 5 s watchdog) —
built 14:43, sanity clean (200.5/100.4 Hz, stamps +0.001 s, clean exit).
Pipeline run 1 (14:46): stalled.** Node 200.5 Hz for 60 s, motion frames
stopped 14:47:55, the watchdog's `pipe.stop()` never returned; the wrapper
had already logged UVC control timeouts (`get_xu … UVCIOC_CTRL_QUERY …
Connection timed out`) from 14:47:09, 54 in total, and the gate aborted.
No earlier run has those timeouts: collector-under-load 0, sanity 0,
sessions 1 and 3 0, 29 Oct 0. After the run the camera's control path was
dead: `hardware_reset()` failed with the same timeout, the sysfs
`authorized` cycle left the device enumerated without a configuration,
librealsense sees no device. Run 2 (decoupled publish) could not start on
the wedged camera and was stopped. Probe-free pipeline tally for ROS
publishers: Python 2 clean / 1 stall, C++ 0 clean / 1 stall; collector
(no ROS publishing) 1/1 clean under the same load. A `publish:=false`
mode is in the C++ node for the direct A/B once the camera is back.

**Camera recovery after run 1:** `hardware_reset()` and the sysfs
`authorized` cycle both failed (control path dead); unbinding/rebinding
the VL805 (`0000:01:00.0` on `xhci_hcd`) twice brought back the camera, the
internal hub and the base's serial link (the first rebind restored only
the SuperSpeed side). Documented as the software-only recovery for a
wedged D455 control path; a replug does the same.

**Pipeline run 2 (15:07, C++ node, publish decoupled from the frame
callback, 5 s watchdog, flat parameter file so the wrapper's video global
time is really off):** no UVC control timeouts; motion frames stopped at
15:11:27, i.e. at the moment the recorder started and subscribed; the
watchdog's full `pipe.stop()/start()` **recovered the IMU** (0 Hz window,
89.6 Hz, then 200.5 Hz; ~17 s hole). Run 1's stall (14:47:55) coincided
with the gate's `ros2 topic hz` subscribers starting; session 3's
(12:22:14) likewise. Node CPU 13 % of a core (Python: 35 %).

**Run 2 result (bag `agv110_commission_standalone_imu_20261103_150757`,
5.7 GB, 604 s):** gyro 118 656 msgs @ 200.5 Hz, accel 59 367 @ 100.3 Hz,
**no IMU gap > 0.25 s after the start-up recovery**, IMU covers the bag
from 12 s after bag start to the end (validator FAIL only on that
coverage and on the expected missing cmd_vel); colour 13.4 Hz (6 warn
gaps ≤ 0.35 s), depth 1 warn gap, odom/tf max gap 0.2 s. Stamps
(recorder receive − header): gyro/accel/odom 0.000–0.001 s; colour/depth
camera_info −0.01 s drifting to −0.05 s over 10 min (the wrapper's
constant-base device-clock mapping drifts ≈80 ppm; acceptable for now,
noted). Host: recorder 49 %, C++ node 13 %, wrapper 11 % of a core.
First full-pipeline recording with usable IMU end to end. Run 3 (same
configuration) started 15:35 for reproducibility.

**Run 3 (15:35, identical to run 2):** recorder 15:38:09–15:48:21,
**zero watchdog restarts, zero UVC control timeouts** — no stall at all.
Validator/audits pending at the time of writing.

**Run 3 validated (bag `agv110_commission_standalone_imu_20261103_153502`,
5.5 GB, 601.5 s):** gyro 120 574 @ 200.5 Hz, accel 60 330 @ 100.3 Hz,
**IMU covers the whole bag, max internal gap 0.147 s**, zero restarts;
colour 13.4 Hz (7 warn gaps ≤ 0.34 s), depth 1 warn gap, odom/tf max
0.17 s; validator FAIL only on the expected missing cmd_vel (stationary).
Stamps: gyro/accel/odom +0.001 s, colour/depth −0.02/−0.03 s median.
Host: recorder 45 %, C++ node 14 %, wrapper 12 %.

### Where the standalone-IMU route stands, end of 3 Nov

| full start_session recording, 600 s, agv110, probe-free | runs | IMU result |
|---|---|---|
| ROS wrapper IMU (all fleet cameras, 28 Oct–2 Nov) | many | hard stall every time, unrecoverable |
| Python publisher (sensor API + frame_queue) | 3 | 2 clean, 1 unrecoverable stall |
| C++ node, publish inside the frame callback | 1 | stall at gate start + camera control path wedged |
| **C++ node, decoupled publish + 5 s watchdog (current)** | **2** | **run 2: one stall at recorder start, recovered in 17 s; run 3: clean** |
| Intel collector (no ROS publishing) under the same load | 1 | clean |

Current configuration = `agv_imu_source:=standalone` → video-only wrapper
with `config/realsense_standalone_imu.yaml` (flat dict; video global time
off) + `rs_imu_publisher_node` (C++, `agv_bringup/src`, started 25 s after
the wrapper, device-clock stamps, decoupled publish, watchdog restart).
`IMU_SOURCE=standalone` in `start_session.sh`, `S1_IMU_SOURCE` in the S1
pilot script. The Python `scripts/rs_imu_publisher.py` is superseded and
kept only for reference. Draft is uncommitted; deployed and built on
agv110 only (111/114 still carry the Python draft from 2 Nov).

Open items: (1) more runs for confidence (two clean-or-recovered in a
row is not many); (2) the moving run with the operator; (3) roll the
C++ node to 111/114 and build there; (4) the 17 s recovery hole at
recorder start, if it recurs, could be hidden by starting the recorder
before the gate's IMU subscribers detach or by lowering the 5 s
threshold; (5) wrapper video stamps drift ≈80 ppm against the host over
10 min (constant-base mapping) — small, noted.

### MoCap-driven circle attempts on agv110 (16:08–17:03, operator present)

- GT chain brought up session-scoped (no persistent install): second NatNet
  source on the Mac for `orkar_agv110` only (the long-running one serves
  104/108), existing router on 192.168.50.98:7447, Zenoh client binary +
  config in `~/zenoh` on agv110 (`start_client.sh`/`stop_client.sh`),
  `ip link set lo multicast on` (transient). Pose arrives at 72 Hz idle.
- S1 pilot needed `S1_TOPIC_WAIT_SEC=90` (the C++ IMU node starts 25 s
  after the wrapper) and `S1_RUNTIME_IMU_GUARD=false`/`REARM=false` (the
  guard would re-arm the wrapper IMU against the standalone node).
- Precheck failed 4× with "Timed out waiting for MoCap pose": while the
  pilot's `check_live_topics.py --hold-after-pass` was subscribed, the pose
  stopped reaching every other subscriber (measured: hz empty for ~45 s,
  back when it exited). With `reliable_routes_blocking: false` in the
  client config the checker no longer stalled the feed.
- Precheck then failed on geometry: radius error 0.15 m in 4 s. Quick
  MoCap calibration (13 cm nudge, 50° turn): rigid-body forward offset
  ≈ **+148°**, base angular response ≈ **0.45×** commanded at 0.3 rad/s
  (single short measurement each; the longer repeat was stopped by the
  operator). With `S1_FORWARD_YAW_OFFSET_DEG=148` the precheck passed
  (max radius error 0.066 m).
- Recorded run 16:52:48: MoCap-closed circle for **81.5 s** (radius drifted
  to 1.05 m, controller at min speed, base under-turning), then PAUSED on
  `pose_source_timestamp_out_of_bounds`, aborted after the 15 s recovery
  budget; the pose topic then stayed absent from the bag for its last
  88 s and came back only when the session's processes exited. Repeat with
  a 60 s budget (16:56): the pose feed degraded as soon as bringup +
  recorders started (per-second logger: ages 0.3–0.85 s, arrival gaps up
  to 0.7 s; idle it is median 0.05 s, max 0.33 s) and the precheck itself
  paused/aborted. The Mac source was steady (~145 poses per 2 s) through
  both stalls, so the loss is router→Wi-Fi→robot client→DDS under load —
  the known GT-staleness class, not Motive. IMU: zero restarts in every
  moving attempt.
- Bags: `agv110_s1_circle_0p8m_20261103_165130_{sensors,aux}` (81 s of
  valid circle; validator FAIL on GT/IMU coverage after the abort).

Left running: Mac second NatNet source (`scratchpad/gt/natnet_agv110.log`),
Mac router (pre-existing), agv110 `~/zenoh` client. Nothing installed.

### Pose-feed A/B on agv110 (17:09–17:26, no motion): the Zenoh client is CPU-starved during bringup

Per-second pose logger + gateway ping + TCP-connect timing to the router,
with the full bringup + 4-minute recording as load (`feedtest_A/B_*`):

| phase | A: client nice 0 | B: client nice −15 |
|---|---|---|
| idle | 3 bad of 18 s | 0 of 14 s |
| bringup + gate | **50 bad of 187 s**, worst pose age 0.80 s | **0 of 220 s**, worst 0.40 s |
| steady recording | 0 of 248 s | 1 of 248 s |

("bad" = fewer than 40 poses in the second or an arrival gap > 0.4 s.)
Wi-Fi was clean in both: 0.16 % gateway ping loss, 7 RTT spikes > 100 ms,
TCP connect to the router max 0.32 s, 12 tx failures total. The Mac source
was steady. Conclusion: the pose dropouts that abort the S1 precheck (which
runs right after bringup) come from the robot's Zenoh client losing CPU to
bringup, not from Wi-Fi or Motive. The fleet unit
`configs/systemd/orkar-zenoh-gt.service` runs the client at default
priority; a `Nice=-15` (or a realtime class) is the candidate fleet fix.

**Straight MoCap run attempt 1 (17:29):** the C++ IMU node stalled 15 s
after start (bringup phase again), its watchdog restart blocked in
`pipe.stop()` with the process in uninterruptible sleep (`D` state, inside
the camera driver) until the launch exited; session never reached motion.
Every IMU stall today in a session happened during bringup/gate/recorder
start, the same window in which the Zenoh client was shown to be CPU
starved; next run renices the IMU node to −15 as soon as it appears.

**Straight MoCap run (17:56–18:01, agv110, camera replugged + hardware
reset first; IMU node reniced −15 at start):** recorded session, MoCap
straight driver (`--line-yaw-offset-deg 148`) drove **0.86 m** of the
1.5 m before stopping on lateral error 0.251 m (limit 0.25), max heading
error 8°; pose feed held throughout (no pose faults). The IMU node
**stalled and restarted 5 times** in this session even at nice −15, so
priority does not prevent the stall; the pipeline restart recovered it
each time here (it hung in D state in the 17:29 attempt). Bag
`agv110_straight_mocap_20261103_175608` (868 MB).

**Moving A/B, run A (18:10, after a reboot; Zenoh client −15, IMU node
default):** precheck passed (radius error 0.065 m); recorded circle
stopped at 14 s with "no circle progress": MoCap shows the robot
physically held at x ≈ 4.79 m, y ≈ −0.88 m from t = 6 s (mm jitter, yaw
creeping −158° → −173°) while the wheels kept turning (odom 0.02–0.17 m/s)
— it drove into something at the far side of the arena; the inferred
centre from its parked spot put the ring there. MoCap tracking was fine
(stamp−recv −0.03 s). Lesson: place the robot on its ring before every
S1 run; never start from where the last run left it.

**Straight MoCap runs after the reboot (IMU node at default priority,
Zenoh client −15):** run A (18:17) aborted at 5 cm on the straight
driver's default 0.25 s stale-pose limit (feed jitter reaches 0.3 s; use
`--pose-timeout 0.6`); run A2 (18:22) drove **0.71 m**, stopped on
lateral error 0.25 m. Both: **IMU 0 restarts, 25/25 and 24/24 windows at
full rate** — a freshly booted camera at default priority is clean in
moving runs. The lateral drift came from the yaw offset: from the
recorded start/end poses the true forward offset is **≈ +170°** (travel
−92.1° vs body yaw +98.4° over 0.76 m), not the 148° from the short
nudge. Run B (18:28) uses 170° and the IMU node at −15.

**Run B (18:28, offset 170°, IMU node −15): the straight line completed —
1.473 m of 1.5 m reached, max lateral 0.16 m, max heading 10.4°, pose feed
held; bag `agv110_straight_mocap_B_20261103_182857` (899 MB).** IMU:
2 restarts, 9 of 25 windows below 150 Hz. Priority A/B on the IMU node,
fresh boot, moving: default priority 0 restarts / 0 slow windows (two
runs); −15: 2 restarts / 9 slow windows (plus 5 restarts in the 17:56 run
on the post-wedge camera). **Do not renice the IMU node**; keep the Zenoh
client at −15 (that one is the pose-feed fix).

End-of-day state for agv110: `forward_yaw_offset_deg ≈ 170`, straight
driver `--pose-timeout 0.6`, Zenoh client session-scoped at −15, IMU
node default priority, camera healthy after reboot. First complete
MoCap-driven recorded run of the day is run B.

**Reverse straight run (18:42, offset 170°, IMU node default priority):
completed — 1.48 m, max lateral 0.09 m, max heading 7.9°, pose feed held;
bag `agv110_straight_mocap_rev_20261103_184227` (858 MB).** IMU: 2
restarts, 9 of 25 slow windows — at default priority. That retracts the
priority conclusion above: after the reboot the first two camera sessions
(A, A2) were clean and the third and fourth (B at −15, reverse at default)
both degraded. Priority is not the variable; the degradation tracks the
number of camera start/stop cycles since boot (or elapsed time), which
also fits the morning's pattern (clean early, worse later) and the
collector/isolated runs (one long session each). Hypothesis for tomorrow,
not tested: a single long camera session per boot is clean; repeated
sessions degrade the D455 until a power cycle.

Motion side, end of day: forward and reverse 1.5 m MoCap-driven straight
runs complete and recorded with `forward_yaw_offset_deg=170`,
`--pose-timeout 0.6`, Zenoh client −15.

### Mitigation under test (18:50): split capture from publishing

`rs_imu_capture` (C++, librealsense only, no ROS: `rs2::pipeline` +
callback exactly like `rs-data-collect`; one UDP datagram per sample to
127.0.0.1:27500; extrinsics meta datagrams at start; 5 s watchdog with
full pipeline restart) + `rs_imu_udp_publisher_node` (rclcpp only: UDP →
`/camera/gyro/sample`, `/camera/accel/sample`, static TF, latched
extrinsics; device-clock stamps with the running-minimum offset). The
process that holds the camera never sees DDS, so subscriber joins (gate
probes, recorder) cannot touch it. Launch `agv_imu_source:=standalone`
now starts both 25 s after the wrapper. Sanity (18:52): both up, TF and
extrinsics out, clean exit. First recorded moving test started 18:56
(`straight_split1`).

**Split run 1 (18:56, forward 1.5 m, default priorities, 5th camera
session since boot): drive complete (1.48 m, lateral 0.13 m), gate 200.5 /
100.4 Hz, capture process 0 restarts (no hard stall), but 4 of 24 capture
windows and 6 of 25 publisher windows below 150 Hz (120–186 Hz windows =
2–4 s of micro-gaps per 10 s, spread through the run).** Previous two
runs in the same state had 2 hard stalls each. Bag
`agv110_straight_split1_20261103_185644`. Run 2 (reverse) started 19:01.

**Split run 2 (19:01, reverse, 6th camera session since boot): capture got
75 Hz in its first window then nothing; its in-process `pipe.stop()/start()`
returned but delivered no frames; session stuck at the IMU wait and was
stopped. Immediately afterwards a brand-new pipeline in a new process
streamed gyro at 194 Hz.** Change: `rs_imu_capture` now exits (code 3)
on silence and the launch respawns it (`respawn=True, respawn_delay=2`),
so recovery is a fresh process, which is the only thing that has ever
recovered this camera short of a replug.

**Split run 3 (19:10, forward, respawning capture, 7th camera session
since boot): drive complete, 1.47 m, lateral 0.04 m (best line of the
day). Capture exited on silence twice and was respawned each time — the
stream came back on its own both times; 10 of 26 capture windows below
150 Hz (micro-gap degradation, heavy at this session count). Session
shutdown took 70 s (respawn during teardown to be checked). Bag
`agv110_straight_split3_20261103_191038`.**

Standing at 19:17: the split capture + respawn is the configuration to
keep — it is the only one that has never left the IMU dead for the rest
of a run. What it does not fix is the camera's degradation with the
number of sessions since power-up (sessions 1–2 clean, 5–7 full of
micro-gaps and stalls). Procedure until that is understood: power-cycle
the robot (or replug the camera) before each dataset session; test
tomorrow whether a sysfs `authorized` cycle on a *healthy* camera is a
sufficient software replug between sessions.

**After a second reboot (19:19), split configuration, straight MoCap runs
(direction chosen by the operator from here on):**

| camera session since boot | run | drive | capture respawns | capture windows < 150 Hz |
|---|---|---|---|---|
| 1 | split4 (19:20, reverse) | 1.19 m then driver abort on its direction check | 1 | 1 of 29 (min 99.7 Hz) |
| 2 | split5 (19:27, forward) | **complete, 1.48 m, lateral 0.19 m** | **0** | **0 of 28 (min 200.5 Hz)** |

Together with runs A/A2 after the first reboot (sessions 1–2 clean) and
sessions 5–7 (stalls, 10 of 26 slow windows), the pattern holds: the D455
IMU is clean for the first sessions after power-up and degrades with
session count. Split capture + respawn keeps the stream alive through the
degraded state; a reboot restores the clean state. Session teardown still
takes 70 s (capture respawned once during shutdown despite the 1.5 s
delay) — cosmetic, to fix.

**Split run 6 (19:36, forward, 3rd camera session after the boot):
complete, 1.47 m, lateral 0.12 m; capture 0 respawns, 0 of 24 windows
below 150 Hz, min 200.5 Hz; no respawn during teardown with the
SIGTERM-only exit. Session teardown still ~80 s (to be traced).**
Three consecutive clean-IMU moving runs after the 19:19 reboot
(sessions 1–3: one early respawn, then none).

**19:46–20:02:** first lateral-correction attempt never started its IMU:
the supervisor script resolved its binary through the symlink-install
link into the source tree (exit 127) — fixed. The retry found the camera
wedged again (wrapper no video, capture "No device connected", process in
D state) on the 4th–5th camera session after the 19:19 reboot; recovery
by USB-controller rebind as before. `--enable-lateral` added to
`drive_mocap_straight_ros2.py` (strafe correction existed but could not be
switched on from the command line).

**Drift fix verified (20:07, forward 1.5 m, 1st camera session after the
reset):** with `--enable-lateral --lateral-kp 0.8 --max-lateral 0.05` and
offset 173°, lateral error peaked at **0.054 m** and finished at
**−0.006 m** (previous forward runs: 0.12–0.19 m uncorrected); heading
max 6.9°; the strafe command settled at ≈0.03–0.04 m/s against 0.08 m/s
forward, i.e. the base crabs noticeably and the correction is doing real
work. IMU: 0 restarts, 27 of 27 windows at full rate. **Session teardown
back to 10 s** with the supervisor (no launch respawn). Bag
`agv110_straight_lat3_20261103_200748` (930 MB, pose topic recorded).

Final straight-run recipe for agv110: `drive_mocap_straight_ros2.py
--line-yaw-offset-deg 173 --enable-lateral --lateral-kp 0.8 --max-lateral
0.05 --pose-timeout 0.6`, Zenoh client at −15, IMU node default priority,
split capture + supervisor, camera power-cycled/reset before the first
session. The residual offset can still be trimmed a few degrees (the
strafe term is compensating part of a constant angle); measure from the
recorded pose track next time.

**20:15 forward, same recipe, 2nd session after the reset:** complete,
1.48 m, max lateral 0.084 m, final 0.014 m; the strafe command sat at its
0.05 m/s clamp for ~5 s mid-run (that is where the 8 cm came from) and
heading error reached 7.6° before the slow heading loop pulled it back.
IMU 0 restarts, 24 of 24 windows; teardown 10 s. Next tuning: `--max-lateral
0.08`, `--heading-kp 1.0` (base turns at ≈0.45× commanded), offset 175°.

**20:22 forward (offset 175, heading-kp 1.0, max-lateral 0.08), 3rd
session after the reset:** complete, 1.47 m, max lateral 0.076 m, final
−0.024 m, heading max 7.0°; strafe command peaked at 0.056 m/s (not
clamped) — the lateral loop itself is too soft for this base's crab;
next: `--lateral-kp 1.5`. IMU 0 restarts, 24 of 24 windows; teardown 15 s.
Three consecutive clean-IMU sessions after the 20:04 reset.

**20:28 reverse with lateral-kp 1.5: aborted at 0.39 m, lateral −0.25 m in
7 s — a driver bug: `--reverse` rotates the line frame by 180° but the
strafe term kept its forward sign, so correction became positive
feedback. Fixed (`linear.y *= motion_sign`). IMU 0 restarts, 25 of 25;
teardown 10 s.**

**20:34 reverse, sign fix + lateral-kp 1.5 + heading-kp 1.0 + max-lateral
0.08 + offset 175: complete, 1.46 m, max lateral 0.027 m, final 0.025 m,
heading max 6.6°. IMU 0 restarts, 24 of 24; teardown 10 s. Best line of
the day; 5th consecutive clean-IMU session after the 20:04 reset.**

**Final straight-run recipe (agv110):** `drive_mocap_straight_ros2.py
--line-yaw-offset-deg 175 --enable-lateral --lateral-kp 1.5 --max-lateral
0.08 --heading-kp 1.0 --pose-timeout 0.6 [--reverse]`, with the split IMU
capture + supervisor, Zenoh client at −15, IMU node default priority,
camera reset before the first session of a power-up.
