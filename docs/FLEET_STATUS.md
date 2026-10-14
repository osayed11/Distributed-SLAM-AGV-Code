# AGV Fleet Status

Rolling record of each robot's verified state. Update the "Last checked" row
whenever a robot is gated, driven, or reconfigured.

> IPs are DHCP and change with the network (lab `192.168.50.x`, home
> `192.168.1.x`, campus `10.23.x.x`). Treat them as last-known, not fixed.

| Robot | Last checked | Last known IP | Sensor gate | Dataset gate | Fit to run S1 |
|-------|--------------|---------------|-------------|--------------|---------------|
| agv24  | **2026-10-12** | 192.168.50.158 | PASS (0 FAIL, 0 WARN) | **PASS/WARN — 0 FAIL** | **Yes** — needs operator sign-off + a real bag |
| agv104 | **2026-10-12** | 192.168.50.192 | PASS (0 FAIL, 0 WARN) | **PASS/WARN — 0 FAIL** | **No** — min-speed floors block S1 |
| agv103 | **2026-10-14** | 192.168.50.176 | PASS (0 FAIL, 0 WARN) | FAIL — `odom_mocap_sanity` only | Pending 1 m drive (yaw offset + odom scale) |
| agv106 | **2026-10-14** | 192.168.50.208 | PASS (0 FAIL, 0 WARN) | FAIL — `odom_mocap_sanity` only | Pending 1 m drive (yaw offset + odom scale) |
| agv107 | August 2026 | 192.168.1.19 | PASS | not recorded | Yes — full S1 circle completed |
| agv110 | **2026-10-14** | 192.168.50.132 | PASS (0 FAIL, 0 WARN) | FAIL — `odom_mocap_sanity` only | Pending 1 m drive (yaw offset + odom scale) |

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

**Open:** 1 m open-loop straight for yaw offset + `linear_scale` (unmeasured --
do not copy other robots' values); operator sign-offs; decide on the two
untracked `.run0[67]_wrapper.sh` files.

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

**Open:** 1 m open-loop straight for yaw offset + `linear_scale` (unmeasured --
do not copy other robots' values); operator sign-offs; the unexplained failed
first reboot.

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

**Open:** 1 m open-loop straight for yaw offset + `linear_scale` (not yet measured
on this robot -- do NOT copy 1.106/1.191 from the others); operator sign-offs;
no bag recorded under the new writeback tuning yet.

**History:** in August 2026 it produced a validator-clean S1 bag
(`PASS 42 / WARN 5 / FAIL 1` benign) on stock writeback settings.

---

## Fleet-wide gotchas

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
