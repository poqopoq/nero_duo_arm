#!/usr/bin/env bash
# Guarded MoveIt reset to the stored standard support pose.
# Usage: ./reset_to_standard_support.sh [--plan-only | --disable]
set -u
set +e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WS="${NERO_DUO_WS:-${ROOT_DIR}/ros2_ws}"
DEPLOY="${NERO_DUO_DEPLOY:-${ROOT_DIR}/deployment}"
TCP_OFFSET='[0.2443, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'
RUN_DIR="/tmp/nero_reset_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${RUN_DIR}"
LEFT_LOG="${RUN_DIR}/left_driver.log"
RIGHT_LOG="${RUN_DIR}/right_driver.log"
MOVEIT_LOG="${RUN_DIR}/moveit.log"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

for iface in can_left can_right; do
  if ! ip -details link show "${iface}" 2>/dev/null | grep -q 'state UP'; then
    echo "${iface} 尚未启用。" >&2
    exit 1
  fi
done

start_driver() {
  local side="$1" iface="$2" log="$3"
  setsid ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args \
    -r "__ns:=/${side}" -r __node:=agx_arm_ctrl_single_node \
    -p "can_port:=${iface}" \
    -p arm_type:=nero \
    -p auto_enable:=false \
    -p speed_percent:=5 \
    -p control_enabled:=false \
    -p effector_type:=none \
    -p "tcp_offset:=${TCP_OFFSET}" \
    >"${log}" 2>&1 &
  echo $!
}

LEFT_PID=$(start_driver left can_left "${LEFT_LOG}")
RIGHT_PID=$(start_driver right can_right "${RIGHT_LOG}")
MOVEIT_PID=''

cleanup() {
  local pids=("${LEFT_PID}" "${RIGHT_PID}")
  if [ -n "${MOVEIT_PID}" ]; then
    pids+=("${MOVEIT_PID}")
  fi
  for pid in "${pids[@]}"; do
    kill -INT -- "-${pid}" 2>/dev/null
  done
  sleep 1
  for pid in "${pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null
  done
  sleep 1
  for pid in "${pids[@]}"; do
    kill -KILL -- "-${pid}" 2>/dev/null
    wait "${pid}" 2>/dev/null
  done
}
trap cleanup EXIT INT TERM

sleep 2
setsid ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false rviz:=false >"${MOVEIT_LOG}" 2>&1 &
MOVEIT_PID=$!

python3 "${DEPLOY}/scripts/reset_to_standard_support.py" "$@"
RESET_RC=$?

echo "RESET_RC ${RESET_RC}"
echo "RUN_DIR ${RUN_DIR}"
echo '--- left driver tail ---'
tail -n 60 "${LEFT_LOG}"
echo '--- right driver tail ---'
tail -n 60 "${RIGHT_LOG}"
echo '--- moveit tail ---'
tail -n 80 "${MOVEIT_LOG}"

cleanup
trap - EXIT INT TERM
exit "${RESET_RC}"
