---
name: agv-robot-ops
description: >-
  Operate the Distributed SLAM AGV fleet from this repo — set up, update, test,
  or debug a robot (Raspberry Pi 4 + ROS 2 Humble + Intel D455 + YDLidar +
  wheel odometry). Use this whenever provisioning a new or reflashed robot,
  pulling code onto a robot, running readiness/doctor gates, recording and
  validating bags, configuring MoCap/Zenoh ground-truth transport, or
  diagnosing sensor/USB/SD-card problems. It encodes the branch model, the
  commit/push safety rules, and the mandatory validation steps so fleet work
  stays cautious, consistent, and reproducible.
---

# AGV Robot Operations

You are operating physical data-collection robots. Mistakes cost lab time,
corrupt datasets, or brick hardware. Work slowly, verify everything with real
command output, and never assume — confirm.

## GOLDEN RULES (read first, always apply)

1. **Never `git commit`, `git push`, or merge without explicit user
   confirmation.** Show the diff / plan first and wait for a clear yes. This
   applies even when the change seems obviously correct.
2. **Never add `Co-Authored-By` (or any co-author) lines to commits.** Standing
   user instruction.
3. **Branch model — respect it strictly:**
   - `main` = the **last known-good, fleet-validated** version. Stable baseline.
   - `ros2-migration` = **staging** for changes not yet proven on every robot.
   - Do **all** work on `ros2-migration`. New robots are set up from it too.
   - Merge `ros2-migration → main` **only after** a change is validated across
     the fleet, and **only with explicit user sign-off**. It is normal and
     desired for `ros2-migration` to sit ahead of `main`. Do not "fix" that.
   - **Never force-push** a shared branch (`main` or `ros2-migration`).
4. **Always run the doctor and validation scripts** — never declare a robot or
   bag "ready" from eyeballing logs. The **post-run bag validator is the
   authority** on publishability. `READY: false` / `dataset_ready=false` on the
   preflight/sensor-logging gate is **normal** (it isn't the dataset gate).
5. **Report outcomes honestly.** If a gate FAILs, say so with the evidence. Never
   soften or skip a failure. Distinguish benign failures (e.g. `cmd_vel` missing
   in a stationary test) from real ones.
6. **Confirm before anything hard to reverse:** reboot, reflash, `apt`
   install/upgrade, killing processes, deleting bags, pushing, merging. State
   what you're about to do and why, then wait.
7. **Physical faults are not software-fixable.** USB `-71`/HID-timeout, SD-card
   read-only, cable/port issues → diagnose, present the evidence, and hand off
   to the human. Do not loop on software "fixes" for hardware problems.
8. **Networking:** use **NetworkManager (`nmcli`) only**. Never run manual
   `wpa_supplicant`/`dhclient` — duplicate WiFi/DHCP processes break SSH and ROS
   discovery.

## Repo orientation

- Production branch: `ros2-migration`. Robots clone/track it.
- Workspace: `~/slam_project` on each robot; ROS 2 workspace at
  `agv2_ws/` (symlink-installed — launch/config file changes take effect
  **without** a colcon rebuild; only C++/message changes need a rebuild).
- Session artifacts go to `~/agv_data/` (bags, manifests, logs) and
  `~/agv_data/diagnostics/` (doctor reports) — **outside** the repo, gitignored.
- Key scripts:
  - `scripts/setup_robot_ros2.sh` — one-time provisioning.
  - `scripts/diagnostics/robot_doctor.sh` — readiness gates (via `--config`).
  - `scripts/diagnostics/dataset_ready_gate.sh` — read-only pre-run dataset gate.
  - `scripts/logging/start_session.sh` — managed bringup + record + manifest.
  - `scripts/logging/validate_ros2_bag.py` — post-run bag validator (authority).
  - `scripts/scenarios/run_s1_mocap_pilot_robot.sh` — gated S1 MoCap circle run.
  - `scripts/network/configure_zenoh.sh` — Zenoh ground-truth transport.
  - Dev checks: `scripts/diagnostics/robot_doctor_selftest.py`,
    `scripts/diagnostics/diagnostic_pipeline_audit.py`.
- Two readiness gates:
  - `configs/robot_doctor_sensor_logging_gate.json` — proves the robot-local
    sensor stack, **no ground truth required** (`require_gt: false`).
  - `configs/robot_doctor_dataset_gate.json` — full publishable gate (needs
    MoCap GT + odom-vs-MoCap sanity).

## Fleet hardware standard (must match before collection)

D455 firmware `5.17.0.10`; USB **5000 Mb/s** (USB3); RGB-D `640x480x15`; raw
gyro `/camera/gyro/sample` ~200 Hz, raw accel `/camera/accel/sample` ~100 Hz;
MCAP storage. RealSense ROS versions are **fleet-wide config**, pinned in the
gate JSONs, `setup_robot_ros2.sh`, docs, and the selftest — see the version
workflow below. Standalone librealsense tools stay at `2.58.1`; do not bump them
with the ROS packages.

---

## Workflow A — Set up a new or reflashed robot

1. Console (or SSH): `sudo hostnamectl set-hostname agv<N> && sudo reboot`.
2. WiFi via `nmcli` (see Golden Rule 8):
   `sudo nmcli device wifi connect "<SSID>" password "<PW>"`, then verify with
   `ping -c3 8.8.8.8`.
3. Clone + provision (needs internet + sudo password):
   ```
   git clone -b ros2-migration https://github.com/osayed11/Distributed-SLAM-AGV-Code.git ~/slam_project
   SUDO_PASSWORD=<pw> bash ~/slam_project/scripts/setup_robot_ros2.sh agv<N>
   ```
4. Re-add the operator's SSH public key to `~/.ssh/authorized_keys` if remote
   access is needed (ask the user for the key; do not invent one).
5. Prefer a **high-endurance SD card or USB-SSD boot** — recording write loads
   wear out cheap cards (see SD gotcha below).

## Workflow B — Update code on an existing robot

1. SSH in. `cd ~/slam_project`.
2. `git status --porcelain` — if there are local changes, **diff them first**
   (`git diff <file>`). Discard a local hand-patch **only after confirming** it's
   superseded upstream (e.g. an old IMU `sleep` patch now in the mainline file).
   Ask if unsure.
3. `git fetch origin`; check `git merge-base --is-ancestor HEAD origin/ros2-migration`.
   - Clean tree + ancestor → `git pull --ff-only origin ros2-migration`.
   - Diverged (e.g. sitting on a `main` merge commit) → `git reset --hard
     origin/ros2-migration` **only** if the tree is clean and the user agrees.
4. Launch/config-only changes need no rebuild (symlink install). Verify with the
   installed vs source file if in doubt.
5. Check RealSense versions against the gate config (see Workflow D + version
   note). Bump if needed.
6. Run the dev checks before trusting diagnostic/logging changes:
   `python3 scripts/diagnostics/robot_doctor_selftest.py` (expect `OK`) and
   `python3 scripts/diagnostics/diagnostic_pipeline_audit.py` (all rows PASS).

## Workflow C — Test / validate a robot (no MoCap needed)

1. **Sensor-logging gate** (~3–5 min, needs D455 connected):
   ```
   ROS_LOCALHOST_ONLY=1 bash scripts/diagnostics/robot_doctor.sh agv<N> \
     --config configs/robot_doctor_sensor_logging_gate.json \
     --bringup-cmd "ros2 launch agv_bringup bringup.launch.py agv_color_profile:=640x480x15 agv_depth_profile:=640x480x15 initial_reset:=false agv_cmd_vel_topic:=/agv<N>/cmd_vel" \
     --bringup-wait 180
   ```
   Pass = **no FAIL checks**. `READY: false` here is expected.
2. **Home-test record → validate** (proves the full loop, incl. IMU-in-bag).
   Backgrounded sessions need a clean process-group SIGINT to finalize MCAP —
   launch under `setsid`, wait for the "starting bag recording" log line, record
   ~60s, then `kill -INT -<PGID>` and wait for "Recording stopped" + flush.
   ```
   REQUIRE_IMU=true REQUIRE_GT=false CMD_TOPIC=/agv<N>/cmd_vel \
     bash scripts/logging/start_session.sh agv<N> home_test
   # after it finalizes:
   CMD_TOPIC=/agv<N>/cmd_vel REQUIRE_IMU=true REQUIRE_GT=false \
     python3 scripts/logging/validate_ros2_bag.py <bag_dir> --require-resilient-storage
   ```
   Then **confirm IMU actually landed**: `ros2 bag info <bag_dir>` should show
   non-zero counts on `/camera/gyro/sample` and `/camera/accel/sample`. A
   `cmd_vel: missing` FAIL is **benign** when the robot was stationary (nothing
   published cmd_vel); it clears once the robot is driven during a run.

## Workflow D — RealSense version standardization

The ROS apt repo keeps **only the latest** version, so pins drift out of the
repo over time. If the gate FAILs on `realsense_ros_driver_version` /
`realsense_ros_librealsense`:

1. `echo <pw> | sudo -S apt-get update` first (the index is often stale).
2. `apt-cache madison ros-humble-realsense2-camera` to see the actual candidate.
3. If it matches the pin → install it. If the repo has **moved past** the pin
   (only a newer version available), this is a **fleet decision** — surface it
   to the user. To advance the standard, update **every** reference together:
   both gate JSONs, `setup_robot_ros2.sh` `EXPECTED_*` defaults,
   `diagnostic_pipeline_audit.py`, `robot_doctor_selftest.py` assertions,
   `README.md`, and `docs/ROBOT_DIAGNOSTIC_PIPELINE.md`. Leave standalone
   librealsense (`2.58.1`) and D455 firmware (`5.17.0.10`) untouched. Then run
   the dev checks (Workflow B step 6) before proposing a commit.
4. Install (mirrors `install_realsense_ros_stack`): the 3 pinned ROS packages
   (`ros-humble-librealsense2`, `-realsense2-camera-msgs`, `-realsense2-camera`)
   plus `-diagnostic-updater` and `-realsense2-description`.

## Workflow E — MoCap / fleet ground-truth (lab only)

Ground truth crosses the network via **Zenoh** (`zenoh-bridge-ros2dds`), not
DDS multicast. Robots stay `ROS_LOCALHOST_ONLY=1` on `rmw_fastrtps_cpp`; only
`/optitrack/rigid_bodies/orkar_agv*` is bridged in. **Never combine with
`ROS_DISCOVERY_SERVER`.**

1. Lab laptop: `configure_zenoh.sh router-run` + start the NatNet pose source
   (Motive streaming, rigid body `orkar_agv<N>`).
2. Robot: `bash scripts/network/configure_zenoh.sh robot <LAPTOP_IP> 7447` then
   `source scripts/network/load_ros_transport_env.sh`. (Pre-staging the pinned
   bridge binary at `/usr/local/bin/zenoh-bridge-ros2dds` while on reliable
   internet lets this step skip the download.)
3. **Prove GT samples actually arrive** (a discovered topic ≠ a publishing one):
   `ros2 topic echo /optitrack/rigid_bodies/orkar_agv<N> --once` and
   `ros2 topic hz ...` (must be inside the configured 20–120 Hz band).
4. Run the gated S1 circle (needs a measured per-robot
   `S1_FORWARD_YAW_OFFSET_DEG` — never inferred from robot ID):
   ```
   MOCAP_TOPIC=/optitrack/rigid_bodies/orkar_agv<N> CMD_TOPIC=/agv<N>/cmd_vel \
     bash scripts/scenarios/run_s1_mocap_pilot_robot.sh agv<N> s1_circle_1m
   ```
   It self-gates: GT freshness/rate, a 5 s unrecorded precheck (radius error,
   direction progress, pose age), and recorder subscription readiness before it
   records or moves. Trust those gates.

---

## Hard-won gotchas

- **IMU missing from a bag** (`/camera/gyro|accel/sample` count 0) despite live
  rates before recording → usually a **marginal USB3 cable**. The kernel shows
  `uvcvideo ... Non-zero status (-71)` (EPROTO) and
  `hid-sensor-hub ... No report`; librealsense logs `Frames didn't arrive`.
  Video (UVC) survives, IMU (HID) dies under combined load — the gates test
  video-only and motion-only separately, so they can pass while a real record
  fails. Fix is physical: reseat/swap the Intel cable, direct USB3 (blue) port,
  no hub. A software USB reset will not fix it.
- **Root filesystem read-only / `EXT4-fs ... Remounting filesystem read-only`**
  → SD-card wear-out (write-protect latch). Try a **cold power cycle** (full
  power off ~30 s, not a warm reboot) then `fsck -y /dev/mmcblk0p2`. If fsck
  reports "unable to set superblock flags" or the OS can't even
  unmount/probe the card, it's dead — **reflash a fresh card**; don't fight it.
- **Pi 4 USB buses:** bus 2 = USB3 (blue, 5000 Mb/s), bus 1 = USB2 (480). A
  D455 that enumerates on bus 1 is in the wrong port and will be degraded.
- **`D455_RESET_MODE` default is `none`** — a healthy camera must not be reset
  as routine setup (reset can itself wedge UVC). Reset only for explicit
  recovery.
- **Depth is raw** (`/camera/depth/image_rect_raw`), not aligned; IMU is raw
  split streams (`unite_imu_method:=0`), fused `/camera/imu` is compat-only.
- Don't run the SD card near full; clean-shutdown before cutting battery power.

## Verification discipline (how to close out any task)

- Re-read actual command output; never report success you didn't observe.
- After a gate/validation, quote the PASS/WARN/FAIL counts and the verdict.
- Before proposing a commit: run the dev checks, show `git diff`/`--stat`, list
  every file touched, and confirm no `4.58.x`-style drift or stray edits.
- Leave the robot in a known state (branch, clean tree, services as expected)
  and summarize what changed, what's validated, and what's still pending.
