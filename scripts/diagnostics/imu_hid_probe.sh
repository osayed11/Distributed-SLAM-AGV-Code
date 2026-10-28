#!/usr/bin/env bash
# Snapshot the D455 IMU (HID/IIO) path the moment the raw gyro/accel topics stall.
#
# Discriminates device-side from host-side stalls without touching the camera:
#   * HID debugfs event stream: are input reports still arriving from the USB
#     HID interface?  (device/USB alive)  -- needs sudo
#   * IIO buffer state and ROS topic rates: is the kernel/IIO path delivering?
#   * librealsense timeout lines, kernel USB/HID messages, PSI, throttling.
# Read-only. Usage: bash scripts/diagnostics/imu_hid_probe.sh [seconds=3] [outdir]
set -u
SECONDS_TO_SAMPLE="${1:-3}"
OUT="${2:-$HOME/agv_data/diagnostics/imu_hid_probe_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
log() { echo "$*" | tee -a "$OUT/summary.txt"; }
HID=$(ls /sys/bus/hid/devices/ 2>/dev/null | grep -i 8086 | head -1)
log "probe $(date -Is) host=$(hostname) hid=${HID:-none} out=$OUT"

log "--- librealsense HID timeouts (last 60 s of bringup log) ---"
BL="$HOME/.local/state/orkar/$(hostname)/bringup.log"
[ -f "$BL" ] && grep -E "didn.t arrive|Hardware Notification|error" "$BL" | tail -200 | awk -v cut="$(date -d '-60 sec' +%H:%M:%S)" '{for(i=1;i<=NF;i++) if ($i ~ /^[0-9][0-9]:[0-9][0-9]:[0-9][0-9]/) {t=substr($i,1,8); if (t>=cut) c++}} END{print "HID-timeout lines in last 60 s:", c+0}' | tee -a "$OUT/summary.txt"

log "--- IIO devices ---"
for d in /sys/bus/iio/devices/iio:device*; do
    log "$(basename "$d") name=$(cat "$d/name") buffer_enable=$(cat "$d/buffer/enable" 2>/dev/null) length=$(cat "$d/buffer/length" 2>/dev/null) sampling=$(cat "$d"/*sampling_frequency 2>/dev/null | head -1) watermark=$(cat "$d/buffer/watermark" 2>/dev/null)"
done

log "--- HID input reports from the camera over ${SECONDS_TO_SAMPLE}s (debugfs; needs sudo) ---"
if [ -n "$HID" ] && sudo -n true 2>/dev/null; then
    sudo -n timeout "$SECONDS_TO_SAMPLE" cat "/sys/kernel/debug/hid/$HID/events" > "$OUT/hid_events.txt" 2>/dev/null
    log "report lines captured: $(grep -c 'report' "$OUT/hid_events.txt") (0 while gyro/accel topics are silent => device/USB stopped sending; >0 => reports arrive, stall is host-side/IIO/librealsense)"
    log "report ids seen: $(grep -oE 'report \(size [0-9]+\) \(numbered\) = [0-9a-f]{2}' "$OUT/hid_events.txt" | awk '{print $NF}' | sort | uniq -c | tr '\n' ' ')"
elif [ -n "$HID" ]; then
    printf '%s\n' "${SUDO_PASSWORD:-}" | sudo -S -p '' timeout "$SECONDS_TO_SAMPLE" cat "/sys/kernel/debug/hid/$HID/events" > "$OUT/hid_events.txt" 2>/dev/null
    log "report lines captured: $(grep -c 'report' "$OUT/hid_events.txt")"
else
    log "no Intel HID device bound"
fi

log "--- ROS topic rates (5 s windows) ---"
if command -v ros2 >/dev/null 2>&1; then
    for t in /camera/gyro/sample /camera/accel/sample /camera/color/image_raw; do
        log "$t: $(timeout 8 ros2 topic hz "$t" --window 200 2>&1 | grep -oE 'average rate: [0-9.]+' | head -1 || echo NONE)"
    done
fi

log "--- kernel USB/HID (last 5 min) ---"
{ sudo -n journalctl -k --since '-5min' --no-pager 2>/dev/null || printf '%s\n' "${SUDO_PASSWORD:-}" | sudo -S -p '' journalctl -k --since '-5min' --no-pager 2>/dev/null; } | grep -iE 'hid|iio|uvcvideo|usb 2-|xhci|oom' | tail -20 | tee -a "$OUT/kernel_recent.txt" | sed 's/^/  /' | tee -a "$OUT/summary.txt" >/dev/null

log "--- host pressure ---"
log "psi cpu: $(head -1 /proc/pressure/cpu)"; log "psi io:  $(head -1 /proc/pressure/io)"; log "psi mem: $(head -1 /proc/pressure/memory)"
log "load: $(cut -d' ' -f1-3 /proc/loadavg) throttled: $(vcgencmd get_throttled 2>/dev/null) temp: $(vcgencmd measure_temp 2>/dev/null)"
for d in /sys/bus/usb/devices/*/idProduct; do [ "$(cat "$d" 2>/dev/null)" = "0b5c" ] && log "camera $(dirname "$d" | xargs basename): speed=$(cat "$(dirname "$d")/speed") power/control=$(cat "$(dirname "$d")/power/control") runtime_status=$(cat "$(dirname "$d")/power/runtime_status" 2>/dev/null)"; done
cp "$BL" "$OUT/bringup.log" 2>/dev/null
log "evidence: $OUT"
