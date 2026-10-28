# myAGV base controller (ESP32) firmware — fleet reference image

`myagv_base_app0_from_agv106_20261028.bin` is the `app0` application image read
back from **agv106's** base controller on 2026-10-28 (ESP32-PICO-D4 rev 1,
Arduino-ESP32 core on IDF v3.3.5; 222,992 bytes including the appended
SHA-256; md5 `ae9f4712bb19134c17ee2886a0d7573c`). It is the firmware the
well-behaved bases run. Bootloader, partition table (`nvs 0x9000`,
`otadata 0xe000`, `app0 0x10000`, `app1 0x150000`, `eeprom 0x290000`,
`spiffs 0x291000`) and NVS were identical on every base inspected, so only
`app0` needs writing.

Why it exists: agv104's base ran a different application (284,624 B) whose
rotation output was bang-bang — a constant ~0.75 rad/s for any commanded
angular rate — and had a ~0.145 m/s linear floor. Writing this image fixed
both (see `docs/FLEET_STATUS.md`, agv104, 2026-10-28). The serial protocol
carries no version field, so a mismatched base can only be detected by dumping
it or by behaviour (in-place ω sweep against MoCap/odom).

This directory is reference material only; nothing in the ROS workspace,
scripts or gates reads it.

## Procedure (on the robot's Pi, base driver stopped)

```bash
# apt's esptool (2.8) lacks the stub loader -> install a current one
python3 -m pip install --user --upgrade pip && python3 -m pip install --user "esptool==4.8.1"
E=~/.local/bin/esptool.py
pgrep -f myagv_odometry_node && echo "stop bringup first" && exit 1

# 1. identify + full read-only backup (keep this; it is the rollback)
$E --port /dev/ttyACM0 --chip esp32 --baud 115200 flash_id
$E --port /dev/ttyACM0 --chip esp32 --baud 921600 read_flash 0 0x400000 agv<N>_esp32_flash_$(date +%Y%m%d).bin

# 2. write the reference app0 — pad to the whole partition so the appended
#    SHA-256 stays intact (a truncated image makes the bootloader reject it and
#    the ESP32 reboots every ~0.1 s printing only the ROM banner)
python3 - <<'PY'
b=open("myagv_base_app0_from_agv106_20261028.bin","rb").read()
open("app0_padded.bin","wb").write(b+b"\xff"*(0x140000-len(b)))
PY
$E --port /dev/ttyACM0 --chip esp32 --baud 921600 --before default_reset --after hard_reset \
   write_flash --flash_mode keep --flash_freq keep --flash_size keep 0x10000 app0_padded.bin

# 3. check it streams again: 0xfe 0xfe frames at ~11 Hz (this image also prints
#    "current speed ..." text lines between frames; the driver ignores them)
```

Rollback: `write_flash ... 0x10000 <your-backup-app0>` where the app0 region is
bytes `0x10000..0x150000` of the full backup.

After flashing, re-measure `linear_scale` (and check angular) for that robot —
scales fitted to the old firmware do not carry over.
