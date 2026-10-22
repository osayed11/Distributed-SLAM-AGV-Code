# Next Lab Square Session

This is the short operational path for the next square pilot. It deliberately
separates physical geometry, transport readiness, recording, motion, and bag
acceptance so a failure cannot be mistaken for a successful dataset run.

## Before motion

1. Power the selected robots and confirm each rigid body is continuously visible
   in Motive.
2. Survey the conservative rectangular area where rigid bodies remain reliably
   tracked. Record it as `MIN_X MAX_X MIN_Y MAX_Y` in arena coordinates. Do not
   use the room dimensions unless they match the tracked area.
3. Start the laptop Zenoh router and NatNet pose source using the laptop's current
   lab IP. Verify each robot's GT topic for at least five seconds.
4. Delete only the failed `s2a_square_1m_pilot_r3` artifacts from the previous
   attempt. Do not delete accepted bags.
5. Place robots at their marked starts. Robots on the same side must take the
   direction that keeps their second edge inside the tracked area. Do not assign
   direction from robot ID alone.

## Route preflight

Run this on each robot before starting the recording gate:

```bash
cd ~/slam_project
source /opt/ros/humble/setup.bash
source ~/slam_project/agv2_ws/install/setup.bash

python3 scripts/logging/drive_mocap_square_ros2.py \
  --pose-topic /optitrack/rigid_bodies/orkar_ROBOT \
  --cmd-topic /ROBOT/cmd_vel \
  --side-length 1.0 \
  --line-yaw-offset-deg OFFSET \
  --turn-sign SIGN \
  --bounds MIN_X MAX_X MIN_Y MAX_Y \
  --boundary-margin 0.20 \
  --preview-only
```

`SIGN=1` means left/counter-clockwise turns. `SIGN=-1` means
right/clockwise turns. Accept the plan only when every printed corner lies
inside the taped route and the script reports `MoCap boundary: PASS`.

Marker yaw offsets are physical calibrations, not permanent robot-ID settings.
Recheck the offset after any rigid body is recreated or its marker mount moves.

## Recorded pilot

The managed wrapper now repeats the route check and watches stationary GT for
five seconds before starting the sensor gate:

```bash
cd ~/slam_project
ROS_DOMAIN_ID=0 \
REALSENSE_CAMERA_GATE_SECONDS=15 \
MOCAP_TOPIC=/optitrack/rigid_bodies/orkar_ROBOT \
CMD_TOPIC=/ROBOT/cmd_vel \
LINE_YAW_OFFSET_DEG=OFFSET \
TURN_DIRECTION=clockwise \
MOCAP_BOUNDS="MIN_X MAX_X MIN_Y MAX_Y" \
MOCAP_BOUNDARY_MARGIN=0.20 \
bash scripts/logging/record_mocap_square_session_ros2.sh ROBOT s2a_square_1m_pilot
```

Use `TURN_DIRECTION=counter-clockwise` for the opposite route. The 15-second
camera gate is for geometry pilots only. Use the normal 90-second gate for an
accepted publishable run.

Start the wrappers in parallel, wait until every robot reports recording ready,
then release motion together. If one robot fails preflight, do not move any of
them.

## Acceptance

Stop motion before recorders. Preserve a short stationary tail, then allow every
wrapper to finish validation. A run is accepted only when all robot validators
exit successfully and the central GT recording covers the complete interval.
Copy bags, manifests, logs, and validation reports before resetting the layout.

The previous GT loss is classified as a trajectory/coverage failure with high
confidence: GT was healthy before motion, the wrong turn carried the robot out
of the reliable Motive area, and the controller stopped on stale pose. The next
pilot confirms this classification by keeping the complete planned route inside
the surveyed bounds.
