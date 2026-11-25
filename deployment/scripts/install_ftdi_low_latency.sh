#!/usr/bin/env bash
# Set the FTDI USB-serial latency timer to 1 ms.
#
# The default 16 ms timer makes every reply wait one 16 ms USB frame, which
# caps the OP3 force read loop at ~62.5 Hz.  Run once with sudo:
#
#   sudo ./deployment/scripts/install_ftdi_low_latency.sh
set -euo pipefail

RULE=/etc/udev/rules.d/99-ftdi-low-latency.rules

if [[ "${EUID}" -ne 0 ]]; then
  echo "error: run with sudo" >&2
  exit 1
fi

cat > "${RULE}" <<'EOF'
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
EOF

udevadm control --reload-rules
udevadm trigger --subsystem-match=usb-serial
sleep 1

for dev in /sys/bus/usb-serial/devices/ttyUSB*; do
  [[ -e "${dev}/latency_timer" ]] || continue
  echo 1 > "${dev}/latency_timer"
  echo "$(basename "${dev}"): latency_timer=$(cat "${dev}/latency_timer") ms"
done
