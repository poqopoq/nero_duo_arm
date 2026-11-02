#!/usr/bin/env bash
# Low-speed single-joint motion test followed by software emergency stop.
set -u
set +e

SIDE="${1:-}"
case "${SIDE}" in
  left) IFACE=can_left ;;
  right) IFACE=can_right ;;
  *) echo "用法: $0 <left|right>" >&2; exit 1 ;;
esac

WS=/home/a/workspace/dual_nero_ws
DEPLOY=/home/a/workspace/dual_nero_deployment
LOG="/tmp/nero_${SIDE}_soft_estop_motion_driver.log"
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

if ! ip -details link show "${IFACE}" 2>/dev/null | grep -q 'state UP'; then
  echo "${IFACE} 尚未启用。" >&2
  exit 1
fi

setsid ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args \
  -r "__ns:=/${SIDE}" -r __node:=agx_arm_ctrl_single_node \
  -p "can_port:=${IFACE}" \
  -p arm_type:=nero \
  -p auto_enable:=true \
  -p speed_percent:=5 \
  -p control_enabled:=false \
  -p effector_type:=none \
  >"${LOG}" 2>&1 &
DRIVER_PID=$!
cleanup() {
  kill -TERM -- "-${DRIVER_PID}" 2>/dev/null
  for _ in $(seq 1 10); do
    kill -0 "${DRIVER_PID}" 2>/dev/null || break
    sleep 0.2
  done
  kill -KILL -- "-${DRIVER_PID}" 2>/dev/null
  wait "${DRIVER_PID}" 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep 4
if ! ros2 node list 2>/dev/null | grep -q "^/${SIDE}/agx_arm_ctrl_single_node$"; then
  echo '驱动未启动，取消测试。' >&2
  sed -n '1,180p' "${LOG}"
  exit 2
fi

python3 "${DEPLOY}/scripts/soft_estop_motion_test.py" --namespace "${SIDE}"
TEST_RC=$?
echo "--- 驱动日志 ---"
sed -n '1,260p' "${LOG}"
exit "${TEST_RC}"
