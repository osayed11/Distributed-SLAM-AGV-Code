# AGV Fleet Status

Rolling record of each robot's verified state. Update the "Last checked" row
whenever a robot is gated, driven, or reconfigured.

> IPs are DHCP and change with the network (lab `192.168.50.x`, home
> `192.168.1.x`, campus `10.23.x.x`). Treat them as last-known, not fixed.

| Robot | Last checked | Last known IP | Sensor gate | Dataset gate | Fit to run S1 |
|-------|--------------|---------------|-------------|--------------|---------------|
| agv100 | **2026-10-14** | DHCP (offline at final motion check) | PASS | **PASS/WARN - 0 FAIL** | Pending guarded out-and-back only; MoCap live |
| agv102 | **2026-10-14** | 192.168.50.196 | PASS | **PASS/WARN - 0 FAIL** | **Yes** - out/back passed; yaw offset **0** |
| agv105 | **2026-10-22** | 192.168.50.226 | PASS (61 OK, 0 FAIL, 0 WARN) at 4.58.4 / fw 5.17.0.10 | 2026-10-14: PASS/WARN - 0 FAIL; **2026-10-22: FAIL only `odom_mocap_sanity` — no sanity JSON on the robot** (GT 72–78 Hz, Zenoh OK) | out/back passed 10-14; yaw offset **0**; needs the 1 m traced run to restore sanity evidence. 10-22: pulled to `335f2c7`, all ROS RealSense pkgs held, apt timers masked, Zenoh re-pointed `.125 -> .98` |
| agv107 | **2026-10-14** | DHCP | PASS | **PASS/WARN - 0 FAIL** | **Yes** - out/back passed; yaw offset **+90** |
| agv24  | **2026-10-12** | 192.168.50.158 | PASS (0 FAIL, 0 WARN) | **PASS/WARN — 0 FAIL** | **Yes** — needs operator sign-off + a real bag |
| agv104 | **2026-10-28** | 192.168.50.192 | PASS (61 OK, 0 FAIL, 0 WARN) at **4.58.4** | 2026-10-22 PASS/WARN — 0 FAIL | **Base MCU firmware replaced 2026-10-28 with agv106's image → proportional steering restored.** 0.5 m cw MoCap circle: 2.03 laps/45 s, max radius err 5.9 cm, heading sd 3.8°, **0 command sign flips** (was 97). Needs: odom angular check, re-gate, full S1 ring run |
| agv103 | **2026-10-22** | 192.168.50.176 | 2026-10-14 PASS; **still 4.58.3 (not upgraded)** | 2026-10-14 PASS/WARN — 0 FAIL | Calibrated: yaw **180**, `linear_scale` **1.027**. 10-22: pulled to `f350b51`, WiFi power-save off + unit, apt timers masked; standing in for agv104 on the 1.7 m ring |
| agv106 | **2026-10-22** | 192.168.50.208 | PASS (61 OK, 0 FAIL, 0 WARN) at **4.58.4** | **PASS/WARN — 0 FAIL** (65 OK; sanity 5.1%; GT 77 Hz) | Calibrated: yaw **−7**, `linear_scale` **1.054** (verified 1.003); **odom yaw drifts to 19°/m**; WiFi GT gaps; needs sign-off |
| agv107 | August 2026 | 192.168.1.19 | PASS | not recorded | Yes — full S1 circle completed |
| agv110 | **2026-10-14** | 192.168.50.132 | PASS (0 FAIL, 0 WARN) | **PASS/WARN — 0 FAIL** | Calibrated: yaw **0**, `linear_scale` **1.026** (verified 0.998); **odom yaw drifts to 23°/m**; WiFi assoc-rejects + GT gaps; needs sign-off + 4.58.4 upgrade |
| agv101 | **2026-10-22** | 192.168.50.218 | PASS (62 OK, 0 FAIL, 0 WARN) | **FAIL — only `odom_mocap_sanity` missing** (needs the 1 m drive) + 4 operator WARNs | **Not yet** — stationary phase complete at 4.58.4 / fw 5.17.0.10; motion phase (yaw offset, `linear_scale`) next |

---

## Fleet run 2026-10-22 — S1 concentric circles, four robots, one start epoch

**agv105 + agv106 on r = 1.0 m, agv101 + agv104 on r = 1.7 m**, all clockwise
about a centre derived from the robots themselves (they were lined up on one
diameter; centre = midpoint of the outer pair = MoCap `(-0.655, 0.002)`), 70 s,
0.10 m/s (agv104 0.16 m/s to clear its speed floor). Released via
`S1_START_SIGNAL_FILE` with a shared epoch; robot clocks agreed within 1 s.

| Robot | precheck | laps | max radius err | validator | sensors bag |
|---|---|---|---|---|---|
| agv105 | PASS (GT age 0.033 s) | 1.11 | 0.060 m | **PASS 52 / WARN 3 / FAIL 0** | 1.1 GB |
| agv106 | PASS (GT age 0.03 s) | 1.07 | 0.097 m | **PASS 51 / WARN 4 / FAIL 0** | 1.1 GB |
| agv104 | PASS with `S1_PRECHECK_MAX_RADIUS_ERROR=0.20` (0.151 m) | 1.06 | 0.137 m | **PASS 51 / WARN 4 / FAIL 0** | 1.1 GB |
| agv101 | PASS (GT age 0.033 s), **yaw offset 0 confirmed** | 0.67 | 0.133 m | **PASS 51 / WARN 4 / FAIL 0** | 1.2 GB |

Recorded rates on all four: colour 13.7–14.1 Hz, GT 76.3 Hz, gyro 199–200 Hz,
`/odom` 12.7 Hz, `/scan` 17.3–17.9 Hz; GT pose age ≤ 0.04 s for the whole run.
WARNs are the usual unrecorded `/diagnostics`, `/tag_detections`,
`/aruco/target_pose`, plus one or two image gaps of 0.27–0.33 s on agv101/106.
Bags: `~/agv_data/agv<N>_s1_concentric_20261022_*_{sensors,aux}` with
`_circle_summary.json` and `_validate.json`. agv104 at 0.16 m/s gained ~140° on
agv101 over the run (same ring, started 180° apart) — fine for 70 s, not for
longer runs; give agv104 its own ring or match speeds.

**Second run, same session, agv103 in agv104's place (101 and 105 swapped
rings by the operator): inner 1.0 m = agv106 + agv101, outer 1.7 m = agv103 +
agv105, cw, 0.10 m/s, 70 s, shared epoch (skew ≤ 2 s).** All four prechecks
passed first time (GT max age ≤ 0.04 s); 106 1.07 laps / 101 1.31 / 103 0.68 /
105 0.65; max radius error 0.04–0.14 m; **validator 0 FAIL on all four**
(agv101 PASS 52 / WARN 3; 103/105/106 PASS 51 / WARN 4 — the unrecorded
`/diagnostics`, `/tag_detections`, `/aruco/target_pose` plus depth gaps of
0.27–0.49 s). GT 76.2 Hz, gyro 199–200 Hz, colour 13.5–14.2 Hz. agv103 ran
with yaw offset **180** as calibrated. Bags
`~/agv_data/agv<N>_s1_concentric_20261022_*_{sensors,aux}` (second set).

**agv104 "wiggled" through its circle while the other three were smooth (same
controller, same gains — seven other robots run it cleanly).** Bag analysis:
commanded ω changed sign 97 times in 70 s (105: 0), actual yaw-rate sd 0.49 rad/s
(105: 0.04), 1.4 s limit cycle, heading error ±25°. An in-place ω sweep against
MoCap afterwards gave the mechanism: **the base rotates at a constant
0.74–0.79 rad/s for any commanded angular from 0.05 to 0.55 rad/s** (odom twist
agrees, 0.69–0.71) — magnitude ignored, sign honoured, pure bang-bang. The
driver sends the rot byte unscaled (`writeSpeed`), so no gain or command scale
above the MCU can fix it; it is the base firmware (same family as its 0.145 m/s
linear floor). **agv104 removed from the lineup; agv103 takes its ring.** Needs
an MCU firmware fix/reflash before any S1 use. Ruled out on the Pi side the
same day: the driver source is identical to HEAD (last change 2026-06-28) and a
**clean rebuild** of `myagv_odometry` reproduced the identical sweep
(0.69–0.71 rad/s for every command). With the base IMU topic enabled
(`publish_imu: true`, reverted afterwards) the **MCU's own gyro reports
+0.76 / −0.80 / +0.80 rad/s for commands of +0.20 / −0.20 / +0.05** — the MCU
measures the turn correctly and still drives a fixed rate, so this is not a
dead-sensor saturation; the firmware maps any non-zero rotation byte to one
speed. The other bases (agv106 measured: +0.20 → 0.25–0.27 rad/s) scale
proportionally. Decisive next step: swap agv104's Pi onto a good base (or a good
Pi onto agv104's base) — if the behaviour follows the base, reflash its MCU with
the firmware the good bases carry (the protocol as used here carries no version
field, so the version can only be established from the board/tooling).

**What it took to get there (all fleet-relevant):**
1. **WiFi power-save was ON on every robot** and caused 0.4–0.6 s GT stalls that
   failed prechecks (agv101/105 while agv104 was clean at the same instant).
   `iw dev wlan0 set power_save off` → next prechecks 0.033 s. Persisted as
   `orkar-wifi-powersave-off.service` on 101/104/105/106; **still needed on
   agv24, agv103, agv110, agv100, agv102, agv107.**
2. The robots must face their **cw tangent**; a first launch with the ccw
   default spun them ~120° and dragged them off-radius.
3. agv106 and agv101 each once came up **without IMU** (`/camera/gyro/sample`
   never published, HID timeout) — a reboot fixed both. agv106 once failed the
   GT rate check right after a reboot (clock still 2024 at bridge start).
4. A killed runner leaves the **motion lock held** by orphaned
   `static_transform_publisher` children (fd 9 inherited) — fixed in the runner
   after this session's run.
5. The NatNet publisher **died when Motive was restarted** (NatNet version
   handshake returned `None`); relaunch publisher, then router.

## Scenario quartet - last checked 2026-10-14

All four robots have strict, GT-inclusive MCAP evidence with **zero validator
FAILs**. Recorded RGB-D rates were 13.5-14.1 Hz, gyro rates 199.7-200.3 Hz,
accel rates 100.7-101.5 Hz, and GT rates 75.3-77.0 Hz.

Guarded 0.4 m forward/reverse tests returned agv105 within 1.4 cm and agv107
within 3.2 mm of their starts. A guarded 0.15 m calibration returned agv102
within 1 cm. Use these measured marker-to-forward offsets:

| Robot | `S1_FORWARD_YAW_OFFSET_DEG` | Motion status |
|-------|-----------------------------:|---------------|
| agv100 | 0 (previously measured; recheck after marker changes) | Pending final out/back because SSH was unavailable |
| agv102 | 0 | PASS |
| agv105 | 0 | PASS |
| agv107 | 90 | PASS - recreated rigid body is rotated 90 degrees |

AGV100 remained live at the NatNet source, but did not expose SSH anywhere on
the lab subnet during the final check. Restore its Wi-Fi connection and run the
same guarded out/back before including it in a scenario. Re-run the five-second
dry check whenever a rigid body is recreated; marker yaw is a physical
calibration, not a robot-ID default.

---

## agv24 — last checked 2026-10-12

**Hardware:** Pi 4 Model B, 2 GB (1844 MB), kernel `5.15.0-1103-raspi`, 48 G free.
D455 firmware `5.17.0.10` at USB 5000 Mb/s, `/dev/ydlidar`, `/dev/ttyACM0`.
Passwordless sudo.

**Software:** repo `ros2-migration @ 9a014aa`, clean tree, workspace built.

**Checks run 2026-10-12:**
- Sensor-logging gate — **PASS**, no FAIL, zero WARN. Colour/depth 15.0 Hz,
  gyro 198.7–201.5 Hz, accel 101.5–101.7 Hz, `/scan` 17.5 Hz, `/odom` 12.7 Hz,
  standalone RGB-D stream 0 timeouts.
- Dataset gate — **FAIL**, single item: `odom_mocap_sanity` (needs a 1 m drive to
  produce its JSON). Plus 4 operator-confirmation WARNs (mechanical, MoCap marker
  visibility, anchors survey, no-bag-recorded).
- `robot_doctor_selftest.py` — **OK**, exit 0.
- `diagnostic_pipeline_audit.py` — **28 PASS / 0 FAIL / 0 WARN**.
- Clock sync 0.25 ms (limit 1.0 ms). GT `orkar_agv24` 74.8–81 Hz, max gap 0.115 s.

**Changed 2026-10-12:** pulled `f415f4d` → `9a014aa`; installed
`/etc/sysctl.d/99-orkar-recording.conf` (the agv107-validated writeback fix —
`dirty_bytes=16 MB`, `dirty_background_bytes=8 MB`, writeback 100cs, expire 200cs).

**Drive test 2026-10-12 (1 m straight, 0.08 m/s):**
- **Yaw offset measured: `-90`** (two runs: -88.1 deg strafing, -89.4 deg forward).
  agv24's marker frame is rotated ~90 deg from physical forward, so
  `S1_FORWARD_YAW_OFFSET_DEG=-90` is required. With offset 0 the robot strafes
  sideways and the drive aborts on lateral error.
- **Base is healthy** — with the offset applied it tracked 0.980 m of a 1.000 m
  target with 1 cm lateral error and 1.2 deg max heading. No sign of agv104's
  speed floor.
- **Odometry under-reads ~11%**: odom 0.8721 m vs MoCap 0.9799 m (ratio 0.890).
  Gate limit is 10%, so this marginally FAILs. Single sample, and the pose
  snapshots were taken ~2 s post-stop, so treat as provisional until repeated.
  Evidence written to `~/agv_data/agv24_odom_mocap_sanity.json`.

**Odometry calibration — FIXED 2026-10-12.** Four baseline runs showed a
systematic under-read: mean odom/mocap ratio **0.9045** (sd 0.0089), per-run error
8.56-11.00%, with 1 of 4 breaching the 10% gate limit. Set `linear_scale: 1.106`
in `agv2_ws/src/agv_bringup/config/base_ros2.yaml` (derived as 1/0.9045). Two
verification runs after: ratios 0.9830 / 1.0138, **per-run error 1.38-1.70%**,
mean ratio 0.9984 — systematic bias eliminated, errors now straddle 1.0 as noise.

> `linear_scale` is a **per-robot** calibration. Do NOT copy 1.106 to other
> robots; measure each one. `lateral_scale` and `angular_scale` remain 1.0 on
> agv24 and are **unmeasured** — an accidental strafe run suggested lateral
> odometry is much worse (~25%), which matters only if the pipeline uses lateral
> odometry.

**Dataset gate re-run with sanity evidence: PASS/WARN, 0 FAIL.**
`odom_mocap_sanity` OK at 1.4% error. `READY: false` only because the three
operator checklists are unconfirmed (`--confirm-mechanical --confirm-mocap
--confirm-anchors`) and no bag has been recorded yet.

**Open:**
- Operator sign-off on mechanical / MoCap visibility / anchors checklists.
- One five-minute full MCAP run passing the post-run validator (the README's
  commissioning bar for a new robot).
- **Uncommitted change on the robot:** `base_ros2.yaml` holds `linear_scale:
  1.106`; original backed up at `/tmp/base_ros2.yaml.orig`. Needs a commit with
  user sign-off per the branch rules.
- Yaw offset for all future runs: **`S1_FORWARD_YAW_OFFSET_DEG=-90`**.
- The drive script's `--pose-timeout` default (0.25 s) is tighter than S1's
  0.50 s and tripped twice on GT transients; use `--pose-timeout 0.5` for manual
  drives.

---

## agv104 — last checked 2026-10-12

**Everything except the base is clean.** Sensor-logging gate **PASS** (0 FAIL,
0 WARN): colour/depth 15.0 Hz, gyro 200.7 Hz, accel 101.3 Hz, 0 stream timeouts,
D455 fw 5.17.0.10 @ USB 5000 Mb/s. Dev checks: selftest **OK**, audit
**28 PASS / 0 FAIL**. Clock sync 0.37 ms. GT `orkar_agv104` 72-86 Hz via Zenoh.
`99-orkar-recording.conf` already installed. Repo pulled to `9a014aa`.
Marker frame needs **no** yaw offset (rigid_yaw ~0 deg, unlike agv24's -90).

**BLOCKER — the MCU ignores angular command magnitude (measured 2026-10-12).**
A direct step-response sweep, bypassing the feedback controller:

| cmd rad/s | mean achieved | peak achieved |
|-----------|---------------|---------------|
| 0.05 | 0.672 | 0.910 |
| 0.10 | 0.460 | 1.060 |
| 0.20 | 0.481 | 1.030 |
| 0.40 | 0.503 | 1.200 |
| 0.80 | 0.522 | 1.170 |

**Confirmed independently against MoCap** (MCU out of the measurement chain),
because the first sweep relied on the MCU's own odometry self-report:

| cmd rad/s | GT rate (MoCap) | odom rate |
|-----------|-----------------|-----------|
| 0.05 | 0.739 | 0.665 |
| 0.10 | 0.751 | 0.448 |
| 0.20 | 0.730 | 0.460 |
| 0.40 | 0.757 | 0.452 |
| 0.80 | 0.875 | 0.495 |

Ground truth is **flat at ~0.75 rad/s for every command from 0.05 to 0.80**.
Commanding 0.05 yields 0.739 -- about 15x the request.

**CORRECTED CHARACTERISATION (2026-10-12): this is a minimum-speed FLOOR on both
channels, not a base that discards command magnitude.** A linear sweep against GT
settled it:

| cmd m/s | GT speed | odom speed |
|---------|----------|------------|
| 0.04 | 0.143 | 0.121 |
| 0.08 | 0.146 | 0.074 |
| 0.12 | 0.146 | 0.073 |
| 0.17 | 0.162 | 0.085 |
| 0.25 | 0.241 | 0.122 |

Below ~0.145 m/s everything runs at the floor; **above it the base tracks the
command well** (0.25 -> 0.241, 4% error). So command handling works -- there is a
floor beneath which nothing is controllable. Shaped like a PWM deadband /
minimum-duty setting pushed too high, which is a far more tractable fix than
broken command handling. Driver-side command scaling could never have helped,
since scaling cannot lift a command above a floor imposed below the driver.

Floors measured: **~0.145 m/s linear, ~0.75 rad/s angular**. S1 asks for
0.10 m/s and 0.125 rad/s -- both well under both floors, which is exactly why S1
is unusable on this robot.

**Untested:** whether angular tracks proportionally above 0.75 rad/s. An attempt
at 1.0/1.5/2.0 rad/s used 1.6 s holds, too short for accel/decel to settle, and
produced non-monotonic junk (2.00 -> 0.724). Redo with >=4 s holds if it matters.

**Unreconciled:** instantaneous odom `linear.x` reads ~50% of GT speed here
(0.074 vs 0.146), yet distance-integrated ratios over 1 m drives were 0.83-0.92.
Those disagree; possibly mecanum odom distance integrates vy as well. Treat the
instantaneous figure with caution.

**OPEN-LOOP RUN (2026-10-12) -- two findings.** Driving 1 m at 0.20 m/s with
heading correction fully disabled (`--heading-kp 0 --max-angular 0`):

| | lateral error | max heading |
|---|---|---|
| closed-loop (4 runs) | 0.081-0.101 m | 9.5-11.0 deg |
| **open-loop** | **0.038 m** | 10.5 deg |

1. **Lateral error more than halves with the steering controller OFF.** The
   script's `--max-angular` default is 0.25 rad/s, *below* the 0.75 rad/s angular
   floor, so every correction it issues is either ignored or answered with a
   0.75 rad/s lurch. The controller was making tracking worse. Practical
   consequence: until the floor is fixed, this robot tracks straighter open-loop.
2. ~~10.5 deg of yaw excursion implies mechanical asymmetry~~ -- **RETRACTED.**
   That read a single worst-case `max_heading` sample as if it were continuous.
   A time-resolved GT yaw trace (797 samples over 9.9 s) shows it is a **single
   start-up kick**, not wobble:

   | t (s) | yaw err |
   |-------|---------|
   | 0.0-2.0 | +-0.03 deg (stationary) |
   | 2.77 | **+7.32 deg (peak)** |
   | 3.75 | +0.31 deg (recovered) |
   | 4.1-9.7 | +1.44 to +1.47 deg, flat to +-0.03 deg |

   The robot yaws ~7 deg as motion begins, recovers in ~1.4 s, then runs dead
   straight for the remaining 6 s. The operator confirmed it looked perfectly
   straight. **There is no evidence of continuous mechanical asymmetry.**

**SINGLE ROOT CAUSE.** The floor explains all three symptoms: at command onset
both sides get a command at/below the floor and do not break traction together,
so one side leads and the robot yaws (the start-up kick); once both are above the
floor they track together; and any steering correction the controller issues is
below the floor so it is ignored or answered with a lurch (the closed-loop
oscillation). One fault, three symptoms -- not a floor plus a mechanical problem.

**ODOMETRY CALIBRATED 2026-10-12.** Measured open-loop (steering disabled) so the
path is straight and endpoint distance is valid. Four baseline samples: ratios
0.8291 / 0.8398 / 0.8456 / 0.8435, **mean 0.8395, sd 0.0063, error 16.05%** --
tighter than agv24's baseline spread. The earlier 8.3-9.2% figures were flattered
by the wiggle (odom integrated the longer curved path). Applied
**`linear_scale: 1.191`** (= 1/0.8395). Two verification runs: ratios 1.0014 /
0.9872, **per-run error 0.14-1.28%, mean 0.57%**.

> agv104 needs **1.191** while agv24 needs **1.106** -- these are genuinely
> per-robot values, not a shared fleet constant. Do not copy either.
> `lateral_scale` and `angular_scale` remain 1.0 and unmeasured on both robots;
> agv104's angular odometry is known to under-read ~40% (see above), so
> `angular_scale` ~1.6 is indicated but unverified.

**Dataset gate re-run with sanity evidence: PASS/WARN, 0 FAIL.**
`odom_mocap_sanity` OK at 1.3% error (odom 1.000 m vs mocap 1.013 m). All sensor,
GT, clock and transport checks green. `READY: false` only because the three
operator checklists are unconfirmed and no bag has been recorded.

**Uncommitted on the robot:** `base_ros2.yaml` holds `linear_scale: 1.191`
(backups `/tmp/base_ros2_104.prescale`, `/tmp/base_ros2_104.orig`), plus the
August `start_session.sh` MCAP-compression fix. Laptop repo stays clean.

**WHERE the fault lies is NOT established.** It was initially called "firmware",
but that does not survive scrutiny: the fleet is supposed to run identical base
firmware, and agv24 tracks a 1 m line to 1-2 mm lateral with ~1 deg max heading,
which requires proportional angular control. Identical firmware cannot produce
both behaviours. So one of these must be true and none has been tested:
  1. the firmware is not actually identical between robots (check versions),
  2. the difference is **hardware** -- motor driver, encoders, wiring, or battery
     voltage under load,
  3. the agv104 sweep is confounded in a way the agv24 sweep would reveal.

**Required next step: run the identical angular sweep on agv24** (same script,
same commands, compare odom-to-odom so the units match). If agv24 responds
proportionally, the difference is robot-specific and the hunt moves to hardware
and firmware versions. If agv24 is also flat, the sweep method or the whole
fleet is implicated and agv24's good straight-line tracking needs re-explaining.
Attempted 2026-10-12 but agv24 had dropped off the network.

> Note for whoever looks at this next: the robot *looks* fine rotating -- smooth
> and consistent, nothing visibly wrong. That is the trap. It rotates at the same
> ~0.75 rad/s whatever it is asked for, which is only visible by comparing
> commanded against achieved.

**Secondary finding: angular odometry under-reads ~40%.** odom reported 0.45-0.50
rad/s while MoCap measured 0.73-0.88 for the same motion; `angular_scale` would
need ~1.6 to match truth. Independent of the controllability fault, and fixing the
reporting would not make the base steerable.

Consequences observed: the robot cannot hold a straight line. Four 1 m runs all
drifted the same way (lateral -0.081 to -0.101 m, max heading 9.5-11.0 deg) --
the operator described it as visible side-to-side wiggling with the two sides out
of sync. The controller asks for a gentle trim, the base lurches, overshoots, and
oscillates. It also stalls entirely below ~0.17 m/s linear: a 1 m run at 0.08 m/s
aborted with `no mocap progress for 5.0s` after 0.085 m. It does complete 1 m when
driven at 0.20 m/s with `--min-linear 0.18`.

**Odometry scale is NOT measurable on this robot** by the endpoint method: the
wiggle means the real path is longer than straight-line displacement, so the
apparent 8.3% under-read is optimistic and would bake the wiggle into any derived
`linear_scale`. A valid figure needs MoCap path length integrated sample-by-sample
during the drive -- pointless until the wheels track together.

Also noted (minor, fleet-wide): `myAGV.cpp:113` encodes commands with
`static_cast<int>(movex * 100)`, which **truncates** rather than rounds, giving a
dead zone below 0.01 -- a ~1 deg steering deadband. `std::lround` would halve it.
Real but irrelevant next to the MCU issue.

---

## agv103 — last checked 2026-10-14

Stationary commissioning run by a delegated agent (~18.5 min, no motion, no reboot).

**Hardware:** Pi 4 2 GB, kernel `5.15.0-1105-raspi`, **SD 90% full — 5.7 GB free**
(the dataset gate's `min_free_gb` is 5, so this is on the edge), 2 GB swapfile
present (0 used). D455 fw `5.17.0.10` @ USB 5000 Mb/s, serial 234322303415,
`/dev/ydlidar -> ttyS0`, `/dev/ttyACM0`. Clock offset 0.15-0.22 ms. Quirk active.

**Software:** repo `d78dc2d -> cee3e6c` (ff-only, no rebuild needed). Two
untracked files left in place: `scripts/scenarios/.run06_wrapper.sh` /
`.run07_wrapper.sh` (`.run07` is byte-identical to `run_s1_mocap_pilot_robot.sh`;
`.run06` adds an 8-line `flock` motion-lock block) -- decide delete vs commit.

**Changed 2026-10-14:** created `99-orkar-recording.conf` (was stock); masked
`apt-daily` timers; **Zenoh `zenoh.env` was pointed at a stale router
`tcp/192.168.50.153:7447`** -- re-pointed to `tcp/192.168.50.98:7447` via
`configure_zenoh.sh`, service restarted. Nothing else.

**RealSense pin -- RESOLVED without a fleet decision.** Robot had
`4.58.2 / 2.58.2`; fleet pin `4.58.3 / 2.58.3`; `packages.ros.org` now offers only
`4.58.4 / 2.58.4` (it keeps just the latest). Instead of bumping the pin (which
would have failed the three robots already at 4.58.3), the exact-pin `.deb`s were
pulled from **agv110's `/var/cache/apt/archives`** and installed:
`realsense2-camera 4.58.3-1jammy.20260804.190440`,
`ros-humble-librealsense2 2.58.3-1jammy.20260720.173911`, `-camera-msgs 4.58.3`,
`-description 4.58.3`. Copies kept for future robots.

**The install wedged the camera; a reboot cleared it.** Immediately after the
package install the sensor gate failed completely -- `D455 not visible to
librealsense`, six kernel `usb 2-2: Failed to query (SET_CUR) UVC control 1 on
unit 3: -110`, every standalone probe timing out, no camera topics -- with the
USB quirk active and the camera still enumerated at 5000 Mb/s. `systemctl reboot`
cleared it (0 UVC errors, 6 interfaces bound). Same signature and same fix as
agv104 in August. **Reboot before gating after any RealSense package change.**

**Checks (post-install, post-reboot):**
- Sensor-logging gate -- **PASS, 0 FAIL, 0 WARN** at 4.58.3/2.58.3: colour/depth
  15.0 Hz, gyro 200.3 Hz, accel 100.4 Hz, standalone RGB-D 890/890 0 timeouts,
  no UVC/xHCI kernel errors. (Pre-install run: all hardware checks OK too, with
  `/scan` 16.9 Hz, `/odom` 12.7 Hz, motion probe 2956/2956.)
- Dev checks -- selftest **OK** (exit 0); audit **28 / 0 / 0**.
- GT -- `orkar_agv103` 70.7-72.8 Hz via Zenoh from `.98`, route created, max gap
  0.043 s.
- Dataset gate -- **FAIL, single item** `odom_mocap_sanity` (needs the 1 m drive)
  + the 4 operator WARNs. `mocap_topic @ 70.1 Hz`, `zenoh_gt_transport` OK,
  `clock_sync` OK, stream 888/888 0 timeouts. **Disk now 5.5 GB free** -- each
  gate run consumes space; offload before it crosses the 5 GB gate minimum.

**Disk cleared 2026-10-14** by the operator: `~/agv_data` emptied (41 GB of
July runs), now **47 GB free**.

**Motion phase 2026-10-14 (traced open-loop 1 m straights, 0.08 m/s):**
- **Marker frame is 180 deg from drive direction**: with offset 0 every run drove
  *backwards* 0.10 m and tripped the script's reverse-abort (measured travel-rigid
  offset +177.8..+179.6). With `--line-yaw-offset-deg 180` three 1 m runs
  completed. **`S1_FORWARD_YAW_OFFSET_DEG=180`** (measured mean +178.6, sd 0.3).
- **No speed floor**: 0.077 m/s against 0.08 commanded; lateral 1.8-3.1 cm.
- **Linear odometry nearly right**: odom path / GT straight-line **0.9741**
  (sd 0.0070, n=3) -> suggested `linear_scale 1.027` (**not applied**). Endpoint
  method would have said 0.926 -> 1.08, wrong (see fleet gotcha).
- Odom yaw drift +2.2 / -0.6 / +2.7 deg per metre (mean +1.4) -- small.
- GT via Zenoh 70-72 Hz, max gap 0.17 s.

**Dataset gate with trace-derived sanity evidence: PASS/WARN, 0 FAIL.**
`odom_mocap_sanity` OK at 2.6% (odom 0.953 m vs mocap 0.978 m). 46.4 GB free.

**`linear_scale 1.027` APPLIED** (uncommitted, `base_ros2.yaml`; backup
`/tmp/base_ros2.prescale`). Verification, 3 traced 1 m runs: path ratio **1.0119**
(sd 0.0095) -- within noise of 1.0 (endpoint 0.965, the yaw-drift effect). Odom
yaw drift on these runs -6.1 / -13.2 / +4.0 deg -- more erratic than the first
batch; the angular channel is not addressed by this scale.

**Open:** operator sign-offs; 4.58.4 upgrade + holds (see fleet note); commissioning
bag; the two untracked `.run0[67]_wrapper.sh` files.

---

## agv106 — last checked 2026-10-14

Stationary commissioning run by a delegated agent (~23 min, no motion), then a
pin install, reboot and re-gate by the operator's session.

**Hardware:** Pi 4 2 GB, kernel `5.15.0-1061-raspi`, 44 G free, 2 GB swapfile
(0 used). D455 fw `5.17.0.10` @ USB 5000 Mb/s, `/dev/ydlidar -> ttyS0`,
`/dev/ttyACM0`. Clock offset 56 us. USB quirk active. Passwordless sudo.

**Software:** repo `8adbd70 -> cee3e6c` (ff-only, clean, no rebuild needed).
Fresh-ish install: no Zenoh, no writeback conf, apt timers enabled.

**Changed 2026-10-14:** created `99-orkar-recording.conf`; masked `apt-daily`
timers; **Zenoh bridge installed from scratch** (`configure_zenoh.sh robot
192.168.50.98 7447`, service active + enabled); RealSense ROS packages
`4.58.2 -> 4.58.3 / 2.58.2 -> 2.58.3` from agv110's apt cache -- this robot had
the three ROS packages `apt-mark hold`-ed, so the first attempt was refused
(`Held packages were changed ... without --allow-change-held-packages`) and did
nothing; the retry with that flag succeeded and **dropped those holds** (now only
standalone `librealsense2*` is held, matching the fleet). Then rebooted
pre-emptively before re-gating (because agv103's camera wedged after the same
install).

**It did not come back from that reboot on its own** -- no ping/mDNS for 15+ min
(agv103 returned in 45 s from the same procedure). The operator power-cycled it
and it came up normally on `.208` with everything intact (4.58.3/2.58.3, quirk
active, 6 D455 interfaces, 0 UVC errors, Zenoh active, tuning applied). Cause of
the failed first reboot **not established**; watch for it on the next reboot.
MAC `e4:5f:01:c1:75:d0`.

**Checks (post-install, post-power-cycle, at 4.58.3):**
- Sensor-logging gate -- **PASS, 0 FAIL, 0 WARN.** Colour/depth 15.0 Hz, gyro
  199.2 Hz, accel 101.0 Hz, standalone RGB-D 895/895 0 timeouts, no UVC errors.
- Dataset gate -- **FAIL, single item** `odom_mocap_sanity` + the 4 operator
  WARNs. `mocap_topic @ 69.8 Hz`, `zenoh_gt_transport` OK, `clock_sync` OK,
  43.2 GB free.

**Checks (agent run, pre-install, at 4.58.2 -- so both gates failed on the pin):**
- Sensor-logging gate -- FAIL on the two pin checks **only**; 0 WARN; hardware all
  OK: colour/depth 15.0 Hz, gyro 199.7 Hz, accel 101.1 Hz, `/scan` 17.1 Hz,
  `/odom` 12.7 Hz, standalone RGB-D 895/895 0 timeouts, no UVC `-110` (two `-71`
  teardown lines only).
- Dev checks -- selftest **OK** (exit 0); audit **28 / 0 / 0**.
- GT -- `orkar_agv106` 71.4-73.8 Hz via Zenoh, route created.
- Dataset gate -- FAIL: the two pin checks + expected `odom_mocap_sanity`; the 4
  operator WARNs; everything else OK.

**Motion phase 2026-10-14 (traced open-loop 1 m straights, 0.08 m/s):**
- **Yaw offset -6.5 deg** (sd 0.6, probe + 3 runs) -> **`S1_FORWARD_YAW_OFFSET_DEG=-7`**.
- **No speed floor**: 0.078 m/s against 0.08; lateral 0.5-2.0 cm.
- **Linear odometry**: odom path / GT **0.9492** (sd 0.0017 -- tightest in the
  fleet) -> suggested `linear_scale 1.054` (**not applied**). Endpoint method
  0.907 -> 1.10, wrong.
- **Odom yaw drift is the worst measured: -1.1 / -6.3 / -16.3 deg per metre** on
  runs MoCap held to <2 deg. Linear scale is fine; angular odometry is not
  (`angular_scale` / gyro bias). Do not trust this robot's odom heading.
- GT via Zenoh 69-74 Hz, max gap 0.12 s.

**Dataset gate with trace-derived sanity evidence: PASS/WARN, 0 FAIL.**
`odom_mocap_sanity` OK at 5.1% (odom 0.927 m vs mocap 0.977 m). 43.2 GB free.

**`linear_scale 1.054` APPLIED** (uncommitted; backup `/tmp/base_ros2.prescale`).
Verification, 2 of 3 traced runs completed: path ratio **1.0028** (sd 0.0088).
Third run aborted on a 0.61 s GT gap before moving -- the same marginal-WiFi
signature as agv110. Odom yaw drift -18.7 / -11.7 deg per metre persists.

**Open:** operator sign-offs; 4.58.4 upgrade + holds; the angular odometry drift
(worst in fleet with agv110); WiFi GT gaps; commissioning bag; the unexplained
failed first reboot.

---

## agv107 — last checked August 2026

**Reference robot for the GT-staleness fix.** `99-orkar-recording.conf` was
validated here: a full S1 circle completes with GT max age ~0.13 s (previously
GT stalled 0.8–1.2 s under recording load and aborted the drive).

Note it was last seen on the home network and may still carry the superseded
`99-agv-writeback.conf`; worth checking next time it is on the lab network.

---

## agv110 — last checked 2026-10-14

Stationary commissioning run by a delegated agent (~16 min, no motion, no reboot).

**Hardware:** Pi 4 2 GB (1844 MB), kernel `5.15.0-1061-raspi`, 48 G free, **no swap
configured**. D455 fw `5.17.0.10` @ USB 5000 Mb/s, `/dev/ydlidar -> ttyS0`,
`/dev/ttyACM0`. Clock offset 0.11 ms. USB quirk active in `/proc/cmdline`.

**Software:** repo `f415f4d -> cee3e6c` (ff-only, clean tree, no rebuild needed).
RealSense already at pins (4.58.3 / 2.58.3 / standalone 2.58.1). Zenoh bridge was
already configured (`tcp/192.168.50.98:7447`).

**Changed 2026-10-14:** created `/etc/sysctl.d/99-orkar-recording.conf` (was stock
`dirty_ratio 20/10`); masked `apt-daily` timers. Nothing else.

**Checks:**
- Sensor-logging gate — **PASS, 0 FAIL, 0 WARN.** Colour/depth 15.0 Hz, gyro
  199.6 Hz, accel 100.6 Hz, `/scan` 17.3 Hz, `/odom` 12.7 Hz, standalone RGB-D
  895/895 frames 0 timeouts, no UVC/xHCI kernel errors.
- Dev checks — selftest **OK** (exit 0); audit **28 PASS / 0 FAIL / 0 WARN**.
- GT — `orkar_agv110` 74-78 Hz via Zenoh, route created, max gap 0.073 s.
- Dataset gate — **FAIL, single item** `odom_mocap_sanity` (needs the 1 m drive)
  + the 4 operator WARNs. `mocap_topic @ 75.3 Hz`, `zenoh_gt_transport` OK.

**Motion phase started 2026-10-14 (open-loop 1 m straights at 0.08 m/s):**
- **Yaw offset ~0** (-0.6 deg, +0.5 deg across runs): marker frame is aligned.
  `S1_FORWARD_YAW_OFFSET_DEG=0`.
- **No speed floor**: mean |vx| 0.079 m/s against 0.08 commanded; 1 m runs reach
  target with 0.5-1.4 cm lateral error and <2.3 deg heading. agv24-class base.
- **Odometry scale UNRESOLVED -- do not derive it from endpoints on this robot.**
  Endpoint ratios scattered 0.82 / 0.89 / 0.90 / 1.09 (vs agv24's sd 0.009). A
  continuous trace explained why: on a MoCap-confirmed straight run odom's
  integrated *path* was 0.965 m but its *endpoint* only 0.882 m -- odom's yaw
  integrates a bias and curves its own track. Path-length ratio was
  0.965/0.977 = **0.988**, i.e. the linear scale is nearly right and the fault is
  **angular drift** (cf. agv104's ~40% angular under-read). Deriving
  `linear_scale` from endpoints would have set ~1.10 and over-corrected the
  linear axis to hide a yaw problem. Needs 3+ traced runs (path-length for
  linear, odom-vs-GT yaw change per metre for angular) before any scale is set.
- One GT stall (0.503 s) aborted a run with nothing recording -- WiFi on this
  robot may be marginal.
- D455 logged `Hardware Notification: Right MIPI error ... Hardware Error` twice
  at bringup -- not seen on other robots; gates still passed with 0 timeouts;
  impact unknown.
- **Then it dropped off the network entirely** during the traced-run batch: no
  ping/mDNS/ARP, MAC absent from the subnet, Zenoh link gone, while Motive kept
  tracking its (stationary) markers. **Root cause found after the operator's
  restart**, in the previous-boot journal: `wlan0: CTRL-EVENT-ASSOC-REJECT
  bssid=00:00:00:00:00:00 status_code=16` on `DELTA_FLIGHT_ARENA`, then
  `CTRL-EVENT-SSID-TEMP-DISABLED ... auth_fail`, re-enabled ~90 s later. The Pi
  stayed up; WiFi re-association was being rejected. This robot's WiFi is
  **marginal** -- it also had a 0.5 s and a 2.6 s GT gap abort drives today.

**Traced runs after restart (offset 0, 0.08 m/s), 2 of 3 completed** (the third
aborted on the 2.6 s GT gap):
- **Linear odometry**: odom path / GT **0.9746** (sd 0.0027) -> suggested
  `linear_scale 1.026` (**not applied**). Endpoint 0.908 -> 1.10, wrong.
- **Odom yaw drift -19.2 / -16.7 deg per metre** on MoCap-straight runs (GT yaw
  change +0.5 / +5.4). Angular odometry is badly off; linear is fine.
- Yaw offset confirmed ~0 (+0.1, +3.7).

**Dataset gate with trace-derived sanity evidence: PASS/WARN, 0 FAIL.**
`odom_mocap_sanity` OK at 2.5% (odom 0.954 m vs mocap 0.978 m). 47.2 GB free.

**`linear_scale 1.026` APPLIED** (uncommitted; backup `/tmp/base_ros2.prescale`).
Verification, 2 of 3 traced runs completed: path ratio **0.9979** (sd 0.0048).
GT gaps 0.63 s (aborted run 1) and 0.74 s (run 3) -- WiFi remains marginal. Odom
yaw drift -23.0 / -2.6 deg per metre.

**Open:** WiFi reliability (assoc rejects, 3 GT-gap aborts today); operator
sign-offs; 4.58.4 upgrade + holds; the angular odometry drift; D455 MIPI warning;
commissioning bag.

**History:** in August 2026 it produced a validator-clean S1 bag
(`PASS 42 / WARN 5 / FAIL 1` benign) on stock writeback settings.

---

## agv101 — last checked 2026-10-21

First contact. Was invisible on the lab network until 2026-10-21 (Motive tracked
`orkar_agv101`, id 23, the whole time); appeared at `192.168.50.218` after the
operator got it onto `DELTA_FLIGHT_ARENA` and added the SSH key.

**Hardware:** Pi 4 2 GB (1844 MB), kernel `5.15.0-1103-raspi`, 18 G free
(`~/agv_data` holds 31 G of July `fixed_center_test` logs — not touched),
`/dev/ydlidar`, `/dev/ttyACM0`. USB quirk active in `/proc/cmdline`. D455
fw `5.13.0.55` enumerates — **but on port `1-1.2` at 480 Mb/s (USB 2)**; every
other robot runs it at 5000 Mb/s on a `2-x` port. It is in a black USB 2 socket
or on a USB 2 cable. **Physical: move it to a blue USB 3 port before gating.**
Note the camera was not enumerated at all on the first look (3 min after boot)
and appeared a few minutes later — watch for that.

**Software, changed 2026-10-21:**
- Repo `ffe0588 -> d2b1318` (ff-only). Two tracked files carried an older hand
  copy of the `d78dc2d` fleet-start feature; stashed as `stash@{0}` (strictly a
  superseded draft: MOCAP_MIN_HZ 20 vs 30, no motion lock) — not deleted. One
  untracked `scripts/scenarios/.run06_wrapper.sh` left in place (same as agv103).
- RealSense **4.58.2 -> 4.58.4 / 2.58.4** from packages.ros.org (exact builds
  `4.58.4-1jammy.20260916.045339`, `2.58.4-1jammy.20260831.021400`, msgs
  `20260907.223031`, description `20260908.040315` — first install of the
  description package). **All four ROS packages now `apt-mark hold`**, as the
  `8847489` gate requires. Install completed cleanly (dpkg audit clean); the
  SSH session that drove it timed out at 10 min, the install did not.
- `/etc/sysctl.d/99-orkar-recording.conf` created (was stock); `apt-daily`
  timers masked.
- Zenoh bridge re-pointed `tcp/192.168.50.153:7447 -> tcp/192.168.50.98:7447`;
  service active; **GT `orkar_agv101` 75 Hz on the robot**, single publisher.

**Checks 2026-10-21:** selftest **OK** (152 tests, exit 0); pipeline audit
**30 PASS / 0 FAIL**.

**2026-10-22 (operator moved the D455 to USB 3 and restarted):**
- D455 now `2-2` @ **5000 Mb/s**. First sensor gate: **1 FAIL** `d455_firmware`
  (`5.13.0.55`, fleet `5.17.0.10`) + a `realsense_package_holds` WARN that was
  a doctor bug (filter only matched bare `librealsense2`; fixed `d4f665b`).
- **Firmware flashed to `5.17.0.10`** with Intel's signed image
  `D4XX_FW_Image-5.17.0.10.bin` (from `librealsense.intel.com/Releases/RS4xx/FW/`,
  sha256 `a7e10cf7…`, kept in `~/d455_fw/`). `rs-fw-update -b` (backup) hung
  for 7 min and was abandoned; the plain `-f` flash took ~2.5 min. Rebooted.
- Sensor gate: **PASS, 62 OK / 0 FAIL / 0 WARN** — 15.0 Hz colour/depth,
  890/890 standalone frames, 0 timeouts, 0 UVC errors.
- Dataset gate: **65 OK, 1 FAIL (`odom_mocap_sanity` evidence missing — needs
  the 1 m drive), 4 operator WARNs.** `mocap_topic` 70 Hz, `zenoh_gt_transport`
  OK, `recording_writeback` OK. (A first attempt failed `mocap_rate` at 122 Hz —
  the Mac router was delivering every GT pose twice after a publisher restart;
  router restarted, now 1.00× — see Fleet-wide gotchas.)

**Next:** motion phase — three open-loop 1 m traced straights for yaw offset
and `linear_scale` (currently 1.0), write
`~/agv_data/agv101_odom_mocap_sanity.json`, re-run dataset gate; operator
sign-offs; commissioning bag.

---

## agv104 — base firmware replaced 2026-10-28

**Root cause of the 10-22 "wiggle" and the August speed floors: agv104's base
ESP32 (ESP32-PICO-D4, behind the CH340 on `/dev/ttyACM0`) was running a
different application image from the rest of the fleet.** Read-only `esptool`
dumps (4 MB each; esptool 4.8.1 via `pip --user`, the apt 2.8 lacks the stub
loader) of agv104 and agv106 showed identical bootloader, partition table,
NVS and EEPROM, but different `app0`: 284,624 B (md5 `59f038d3…`) on 104 vs
222,956 B (md5 `06dd363c…`) on 106 — 205k of 223k bytes differ, i.e. a
different sketch (same Arduino-ESP32 core, IDF v3.3.5). 106's image prints a
`current speed …` text line between its 0xfe-framed packets; 104's did not.

**Fix (operator-approved):** wrote agv106's raw `app0` partition (0x10000,
0x140000 B, `--flash_mode/freq/size keep`) to agv104; verified; hard reset.
A first attempt with a trimmed app image boot-looped (the image carries an
appended SHA-256 — 3 bytes short → bootloader rejects it, ROM banner every
~0.1 s); writing the whole partition fixed it. **Rollback:** `~/mcu_fw/
agv104_esp32_flash_20261028.bin` (full) and `agv104_app0_orig.bin` on the
robot, copies in the session scratchpad.

**Verification against MoCap (same day):**
- In-place ω sweep: 0.05 → 0.24, 0.55 → 0.47, −0.10 → −0.40, −0.20 → −0.50
  rad/s (was a constant 0.74–0.79 for every command). Residual in-place floor
  ≈0.25 rad/s is the fleet firmware's behaviour (agv106 showed the same).
- Axis signs: forward/backward/rotate all correct, yaw offset 0 confirmed;
  v=0.10 with ω=∓0.20 curves right/left as commanded, ω=0 runs straight.
- **0.5 m cw circle, 45 s:** 2.03 laps, radius error sd 1.7 cm / max 5.9 cm,
  heading error −13° ± 3.8°, **zero ω sign flips** (10-22 at r=1.7: 97 flips,
  heading ± 25°, 1.4 s limit cycle). Linear floor gone (0.05 → 0.065 m/s).
- Open: the base's **odometry** under motion looks poor (odom yaw −27° on a
  GT −13° arc; −8.8° on a straight) — `linear_scale 1.191` was fitted to the
  old firmware and must be re-measured; angular needs a look before S1 bags.

**Motive side-quest:** `orkar_agv104` went `tracking_valid=False` for most of
the session — one of its four balls (Motive "Marker #1", the one 5 cm from the
centre ball) produced no dot in 1,149 frames while the other three matched the
asset to 2 mm; after the operator handled the robot it came back 4/4. Treat
that ball as suspect. The asset id changed 42 → 24 today (publisher restart
needed to pick up a re-created asset).

**Also noted:** Riyaadh's 10-28 `prepare` left agv104's GT bridge pointed at
`tcp/192.168.50.125:7447`; it is on `.98` for these tests.

## NOTE — fleet pin moved to 4.58.4 / 2.58.4 (commit `8847489`, 2026-10-14)

Every gate verdict recorded on this page for 2026-10-12..14 was run against the
**4.58.3 / 2.58.3** pin. Commit `8847489` (Riyaadh, "Harden ROS2 dataset recording
and validation") raises `expected_realsense_ros_driver` / `_librealsense` to
**4.58.4 / 2.58.4** in both gate JSONs and `setup_robot_ros2.sh`, raises
`mocap_min_hz` 20 -> 30 in the dataset gate, and adds a `recording_writeback`
check. Consequences:
- All five gated robots (24, 104, 103, 106, 110) are at 4.58.3 and will FAIL the
  version checks as soon as they pull `8847489` or later. They need
  `ros-humble-realsense2-camera 4.58.4` + `ros-humble-librealsense2 2.58.4` (+
  `-camera-msgs`, `-description`) from `packages.ros.org`, **then a reboot**
  before re-gating (the install wedges UVC -- see gotchas).
- The cached 4.58.3 `.deb`s (agv110 apt cache, session scratchpad) are now the
  *old* standard; keep them only as a rollback.
- `mocap_min_hz 30`: all robots measured 56-81 Hz, so no impact.
- The `start_session.sh` per-backend writer-config fix survived: `8847489`
  keeps the same `case` and adds a readable-file check and `_APPLIED` manifest
  fields.
- `realsense_package_holds` now requires **every installed RealSense package to be
  held** (`hold_targets = installed`), not just standalone `librealsense2*`. No
  robot currently holds the ROS packages (agv106's were dropped by the cache
  install; the others never had them), so that check fails fleet-wide too until
  `apt-mark hold ros-humble-realsense2-camera ros-humble-librealsense2
  ros-humble-realsense2-camera-msgs ros-humble-realsense2-description` is set.
Robots were deliberately **not** pulled past `d96758e` pending this upgrade; the
operator chose to **defer it to the next session** (2026-10-14). Procedure per
robot: install 4.58.4/2.58.4 (with `--allow-change-held-packages` where held),
hold all four, **reboot**, pull, re-gate with the existing sanity JSON.

## Fleet-wide finding: odometry *heading* drifts on straight runs

Measured 2026-10-14 with continuous traces (odom yaw change minus MoCap yaw change
over a 1 m open-loop straight, `linear_scale` 1.0 on all):

| Robot | odom yaw drift per metre | linear path ratio |
|-------|--------------------------|-------------------|
| agv103 | +1.4 deg (small) | 0.974 |
| agv106 | -1.1 / -6.3 / **-16.3** deg | 0.949 |
| agv110 | **-19.2 / -16.7** deg | 0.975 |
| agv104 | angular rate under-read ~40% vs GT (sweep) | 0.840 (open-loop) |

Linear scales are within 3-6% across 103/106/110 and 10-16% on 24/104; the
*angular* channel is the bigger problem on at least three robots. Any
`linear_scale` must be derived from odom **path length** (not endpoints) or it
silently absorbs this drift. `angular_scale` is still 1.0 everywhere and
unmeasured as a correction; a yaw-rate sweep against GT (as done on agv104) per
robot is the next calibration step.

## Fleet-wide gotchas

- **GT duplication from the Mac Zenoh router (found 2026-10-22).** With
  zenoh-bridge-ros2dds 1.9.0 as the router, every restart of the NatNet
  publisher under a running router adds one more copy of every GT pose
  delivered to the robots (3 `Route Publisher ... created` lines in the router
  log = each `header.stamp` arriving 3×; robot-side `topic hz` showed 230 Hz).
  The Mac DDS side stays 1×. Restart the **router after the publisher's final
  start**, and check messages vs distinct stamps on a robot (ratio must be
  1.00). The dataset gate's `mocap_rate` (>120 Hz FAIL) catches it; the sensor
  gate does not. Bags recorded under this condition carry inflated GT counts.

These bite repeatedly; full detail lives in the session memory notes.

- **Reboot after `setup_robot_ros2.sh`** — it writes the D455 `usbcore.quirks` to
  `cmdline.txt` but a boot is required for it to apply. Without it the camera can
  fail entirely, and a USB reset makes it worse.
- **MoCap router** must be launched with `ROS_DISTRO=humble`, or it discards all
  DDS discovery and silently routes nothing. Its REST port `8000` also collides
  with the long-running `uvicorn` app on the laptop.
- **A rigid body disabled in Motive** is absent from streamed model definitions;
  re-enabling it requires restarting the NatNet publisher to pick it up.
- **`start_session.sh` used to skip the MCAP storage config** (gated on sqlite3
  only), so home-test bags recorded uncompressed at ~2x the I/O and stalled. Fixed
  in `91c7575` (per-backend default writer config; validated on agv110: 199 zstd
  chunks at 1.82x). agv104 still carries the old hand-patched one-liner locally and
  needs `git checkout -- scripts/logging/start_session.sh` before it can pull.
- **Mask `apt-daily` timers** before a session; unattended-upgrades blocks dpkg
  and starves SD I/O mid-run.
- **`packages.ros.org` keeps only the latest RealSense build**, so the fleet pin
  drifts out of reach (4.58.3 pinned, only 4.58.4 offered by Oct 2026). Before
  changing the pin, pull the exact `.deb`s from a sibling robot's
  `/var/cache/apt/archives` and install those. If the robot has the ROS packages
  held, add `--allow-change-held-packages` (it drops the holds).
- **Reboot before gating after any RealSense package install** -- on agv103 the
  install alone wedged UVC (`SET_CUR ... -110`, camera invisible to librealsense)
  with the quirk active; a reboot cleared it. The USB-reset remedies make it worse.
- **Odom-vs-MoCap scale: compare odom *path length* to GT straight-line, from a
  continuous trace, not endpoint snapshots.** If a robot's odom yaw drifts, its
  endpoint under-measures its own travelled distance and an endpoint-derived
  `linear_scale` over-corrects. Endpoints were fine on agv24 (no yaw drift) and
  misleading on agv110. Quantify yaw drift (odom yaw change minus GT yaw change
  per metre) as a separate `angular_scale` question.
