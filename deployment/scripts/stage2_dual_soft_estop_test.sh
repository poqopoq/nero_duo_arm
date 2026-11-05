#!/usr/bin/env bash
# Low-speed dual-arm motion followed by linked software emergency stop.
set -u
set +e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WS="${NERO_DUO_WS:-${ROOT_DIR}/ros2_ws}"
DEPLOY="${NERO_DUO_DEPLOY:-${ROOT_DIR}/deployment}"
ROS_PYTHON="${ROS_PYTHON:-/usr/bin/python3}"
LEFT_LOG=/tmp/nero_left_dual_estop.log
RIGHT_LOG=/tmp/nero_right_dual_estop.log
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
    -p auto_enable:=true \
    -p speed_percent:=5 \
    -p control_enabled:=false \
    -p effector_type:=none \
    >"${log}" 2>&1 &
  echo $!
}

LEFT_PID=$(start_driver left can_left "${LEFT_LOG}")
RIGHT_PID=$(start_driver right can_right "${RIGHT_LOG}")
cleanup() {
  for pid in "${LEFT_PID}" "${RIGHT_PID}"; do
    kill -TERM -- "-${pid}" 2>/dev/null
  done
  sleep 1
  for pid in "${LEFT_PID}" "${RIGHT_PID}"; do
    kill -KILL -- "-${pid}" 2>/dev/null
    wait "${pid}" 2>/dev/null
  done
}
trap cleanup EXIT INT TERM

sleep 4
NODES=$(ros2 node list 2>/dev/null)
if ! grep -q '^/left/agx_arm_ctrl_single_node$' <<<"${NODES}" || \
   ! grep -q '^/right/agx_arm_ctrl_single_node$' <<<"${NODES}"; then
  echo '至少一侧驱动未启动，取消测试。' >&2
  sed -n '1,180p' "${LEFT_LOG}"
  sed -n '1,180p' "${RIGHT_LOG}"
  exit 2
fi

"${ROS_PYTHON}" "${DEPLOY}/scripts/soft_estop_motion_test.py" --both
TEST_RC=$?
echo '--- left driver log ---'
sed -n '1,220p' "${LEFT_LOG}"
echo '--- right driver log ---'
sed -n '1,220p' "${RIGHT_LOG}"
exit "${TEST_RC}"
