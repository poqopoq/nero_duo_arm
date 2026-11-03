#!/usr/bin/env bash
# Real dual-arm trajectory-cancel safety test with an automatic park reset.
set -u
set +e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WS="${NERO_DUO_WS:-${ROOT_DIR}/ros2_ws}"
DEPLOY="${NERO_DUO_DEPLOY:-${ROOT_DIR}/deployment}"
RUN_DIR="/tmp/nero_cancel_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${RUN_DIR}"
LEFT_LOG="${RUN_DIR}/left_driver.log"
RIGHT_LOG="${RUN_DIR}/right_driver.log"
MOVEIT_LOG="${RUN_DIR}/moveit.log"
TEST_LOG="${RUN_DIR}/cancel_test.log"
RESET_LOG="${RUN_DIR}/reset.log"

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

sleep 3
python3 "${DEPLOY}/scripts/test_trajectory_cancel_real.py" 2>&1 | tee "${TEST_LOG}"
CANCEL_RC=${PIPESTATUS[0]}

cleanup
trap - EXIT INT TERM

LEFT_ESTOP_COUNT=$(grep -c 'Emergency stop command sent to nero' "${LEFT_LOG}")
RIGHT_ESTOP_COUNT=$(grep -c 'Emergency stop command sent to nero' "${RIGHT_LOG}")
LEFT_GATE_CLOSED_COUNT=$(grep -c 'External control gate closed' "${LEFT_LOG}")
RIGHT_GATE_CLOSED_COUNT=$(grep -c 'External control gate closed' "${RIGHT_LOG}")

echo "CANCEL_TEST_RC ${CANCEL_RC}"
echo "LEFT_ESTOP_COUNT ${LEFT_ESTOP_COUNT}"
echo "RIGHT_ESTOP_COUNT ${RIGHT_ESTOP_COUNT}"
echo "LEFT_GATE_CLOSED_COUNT ${LEFT_GATE_CLOSED_COUNT}"
echo "RIGHT_GATE_CLOSED_COUNT ${RIGHT_GATE_CLOSED_COUNT}"

"${DEPLOY}/scripts/reset_to_standard_support.sh" --disable 2>&1 | tee "${RESET_LOG}"
RESET_RC=${PIPESTATUS[0]}
echo "RESET_RC ${RESET_RC}"
echo "RUN_DIR ${RUN_DIR}"

if [ "${CANCEL_RC}" -eq 0 ] && [ "${RESET_RC}" -eq 0 ] && \
   [ "${LEFT_ESTOP_COUNT}" -ge 1 ] && [ "${RIGHT_ESTOP_COUNT}" -ge 1 ] && \
   [ "${LEFT_GATE_CLOSED_COUNT}" -ge 1 ] && [ "${RIGHT_GATE_CLOSED_COUNT}" -ge 1 ]; then
  exit 0
fi
exit 1
