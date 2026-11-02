#!/usr/bin/env bash
# Enable one CAN interface at NERO's 1 Mbit/s bitrate. Requires root.
# Usage: sudo ./scripts/enable_can.sh can_left
set -euo pipefail

IFACE="${1:-}"
BITRATE="${2:-1000000}"

if [ "${EUID}" -ne 0 ]; then
  echo "请以 root 执行: $0 <can_interface> [bitrate]" >&2
  exit 1
fi
if [ -z "${IFACE}" ]; then
  echo "用法: $0 <can_interface> [bitrate]" >&2
  exit 1
fi
if ! ip -details link show "${IFACE}" >/dev/null 2>&1; then
  echo "CAN 接口不存在: ${IFACE}" >&2
  exit 1
fi

ip link set "${IFACE}" down
ip link set "${IFACE}" type can bitrate "${BITRATE}"
ip link set "${IFACE}" up

echo "${IFACE} 已启用为 ${BITRATE} bit/s。"
ip -details -statistics link show "${IFACE}"
