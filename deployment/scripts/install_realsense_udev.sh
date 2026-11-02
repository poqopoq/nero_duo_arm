#!/usr/bin/env bash
# Install Intel RealSense udev rules on a native Ubuntu host.
set -euo pipefail

URL="https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules"
TMP_FILE="$(mktemp)"
trap 'rm -f "${TMP_FILE}"' EXIT

curl -fsSL "${URL}" -o "${TMP_FILE}"
sudo install -m 0644 "${TMP_FILE}" /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
echo "Installed /etc/udev/rules.d/99-realsense-libusb.rules"
echo "Reconnect the RealSense cameras before testing."
