#!/usr/bin/env bash
# Recover a power-off pose that drifted past joint4/joint7 or another limit.
# The script clears latched faults, enables the requested sides, ramps back
# inside the deployment caps with small joint steps, verifies brake release,
# and leaves the motors enabled with the control gate closed.
# Usage: ./recover_joint_limits.sh [--dry-run] [--side left|right] [--disable]
#        ./recover_joint_limits.sh --watch
set -u
set +e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WS="${NERO_DUO_WS:-${ROOT_DIR}/ros2_ws}"

for iface in can_left can_right; do
  if ! ip -details link show "${iface}" 2>/dev/null | grep -q 'state UP'; then
    echo "${iface} 尚未启用。先运行 deployment/scripts/enable_can.sh。" >&2
    exit 2
  fi
done

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

exec /usr/bin/python3 -u "${SCRIPT_DIR}/recover_joint_limits.py" "$@"
