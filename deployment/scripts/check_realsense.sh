#!/usr/bin/env bash
# Read-only RealSense enumeration and USB link-speed check.
set -u
set +e

set +u
source /opt/ros/humble/setup.bash
set -u

echo '--- rs-enumerate-devices -s ---'
rs-enumerate-devices -s 2>&1
echo
echo '--- Intel USB devices ---'
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  [ "$(cat "$d/idVendor" 2>/dev/null)" = "8086" ] || continue
  echo "path=$(basename "$d") product=$(cat "$d/product" 2>/dev/null)"
  echo "  id=$(cat "$d/idVendor"):$(cat "$d/idProduct") serial=$(cat "$d/serial" 2>/dev/null)"
  echo "  speed=$(cat "$d/speed" 2>/dev/null) Mbps usb_version=$(cat "$d/version" 2>/dev/null)"
  find "$d" -maxdepth 4 -type d -path '*/video4linux/video*' -printf '  %f\n' 2>/dev/null
done
echo
echo '--- video by-id ---'
find /dev/v4l/by-id -maxdepth 1 -type l -printf '%f -> %l\n' 2>/dev/null | sort
