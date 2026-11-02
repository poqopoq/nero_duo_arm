#!/usr/bin/env bash
# Stage 2 read-only check: launch the left driver in /left namespace with
# auto_enable=false, then wait for /left/feedback/joint_states.
set -u
set +e

WS=/home/a/workspace/dual_nero_ws
LOG=/tmp/nero_left_namespace.log
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

if ! ip -details link show can_left 2>/dev/null | grep -q 'state UP'; then
  echo "can_left 尚未启用。" >&2
  exit 1
fi

setsid ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  namespace:=left \
  can_port:=can_left \
  arm_type:=nero \
  auto_enable:=false \
  effector_type:=none \
  >"${LOG}" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null
  for _ in $(seq 1 10); do
    kill -0 "${LAUNCH_PID}" 2>/dev/null || break
    sleep 0.2
  done
  kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null
  wait "${LAUNCH_PID}" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo '等待 /left/feedback/joint_states ...'
found=0
for _ in $(seq 1 15); do
  if timeout 2 ros2 topic echo --once /left/feedback/joint_states \
      sensor_msgs/msg/JointState 2>/dev/null; then
    found=1
    break
  fi
  sleep 1
done

echo
echo '--- 节点 ---'
ros2 node list 2>/dev/null
echo '--- 话题 ---'
ros2 topic list 2>/dev/null | grep -E '^/(left|right)/' | sort
echo '--- 驱动日志 ---'
sed -n '1,240p' "${LOG}"
echo '--- CAN ---'
ip -details -statistics link show can_left

if [ "${found}" -eq 1 ]; then
  echo '[PASS] /left namespace 反馈正常；auto_enable=false，未发送运动命令。'
  exit 0
fi
echo '[FAIL] /left namespace 未收到反馈。' >&2
exit 2
