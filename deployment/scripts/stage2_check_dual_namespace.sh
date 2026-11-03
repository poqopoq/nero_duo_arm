#!/usr/bin/env bash
# Read-only dual-arm namespace check with both arms disabled.
set -u
set +e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WS="${NERO_DUO_WS:-${ROOT_DIR}/ros2_ws}"
LEFT_LOG=/tmp/nero_left_namespace.log
RIGHT_LOG=/tmp/nero_right_namespace.log
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

for IFACE in can_left can_right; do
  if ! ip -details link show "${IFACE}" 2>/dev/null | grep -q 'state UP'; then
    echo "${IFACE} 尚未启用。" >&2
    exit 1
  fi
done

start_driver() {
  local ns="$1" iface="$2" log="$3"
  setsid ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args \
    -r "__ns:=/${ns}" -r __node:=agx_arm_ctrl_single_node \
    -p "can_port:=${iface}" \
    -p arm_type:=nero \
    -p auto_enable:=false \
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

sleep 3
echo '--- nodes ---'
ros2 node list 2>/dev/null | grep -E '^/(left|right)/' | sort
echo '--- topics ---'
ros2 topic list 2>/dev/null | grep -E '^/(left|right)/(control|feedback)/' | sort
echo '--- left feedback ---'
timeout 3 ros2 topic echo --once /left/feedback/joint_states sensor_msgs/msg/JointState 2>/dev/null | sed -n '/^name:/,/^velocity:/p'
LEFT_RC=${PIPESTATUS[0]}
echo '--- right feedback ---'
timeout 3 ros2 topic echo --once /right/feedback/joint_states sensor_msgs/msg/JointState 2>/dev/null | sed -n '/^name:/,/^velocity:/p'
RIGHT_RC=${PIPESTATUS[0]}
echo '--- driver logs ---'
sed -n '1,100p' "${LEFT_LOG}"
sed -n '1,100p' "${RIGHT_LOG}"

if [ "${LEFT_RC}" -eq 0 ] && [ "${RIGHT_RC}" -eq 0 ]; then
  echo '[PASS] 左右 namespace 和反馈均正常；auto_enable=false，未发送运动命令。'
  exit 0
fi
echo '[FAIL] 至少一侧 namespace 反馈失败。' >&2
exit 2
