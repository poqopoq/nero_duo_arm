#!/usr/bin/env bash
# Read-only NERO feedback check for a specified CAN interface.
# Usage: ./scripts/check_nero_feedback.sh can1
set -u
set +e

IFACE="${1:-}"
if [ -z "${IFACE}" ]; then
  echo "用法: $0 <can_interface>" >&2
  exit 1
fi

WS=/home/a/workspace/dual_nero_ws
LOG="/tmp/nero_${IFACE}_feedback.log"
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

if ! ip -details link show "${IFACE}" 2>/dev/null | grep -q 'state UP'; then
  echo "${IFACE} 尚未启用。" >&2
  exit 1
fi

ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args \
  -p can_port:="${IFACE}" \
  -p arm_type:=nero \
  -p auto_enable:=false \
  -p effector_type:=none \
  >"${LOG}" 2>&1 &
DRIVER_PID=$!
cleanup() {
  kill "${DRIVER_PID}" 2>/dev/null
  wait "${DRIVER_PID}" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "等待 ${IFACE} 的 NERO 反馈..."
found=0
for _ in $(seq 1 15); do
  if timeout 2 ros2 topic echo --once /feedback/joint_states sensor_msgs/msg/JointState 2>/dev/null; then
    found=1
    break
  fi
  sleep 1
done

echo
echo "--- 驱动日志 ---"
sed -n '1,240p' "${LOG}"
echo "--- CAN 状态 ---"
ip -details -statistics link show "${IFACE}"

if [ "${found}" -eq 1 ]; then
  echo "[PASS] ${IFACE} 已收到 /feedback/joint_states；auto_enable=false，未发送运动命令。"
  exit 0
fi
echo "[FAIL] ${IFACE} 未收到 /feedback/joint_states。" >&2
exit 2
