#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_ws="${repo_root}/ros2_ws"
lerobot_python="/home/work/miniforge3/envs/lerobot/bin/python"
realsense_lib="/home/work/.local/opt/realsense-2.59/opt/ros/humble/lib/x86_64-linux-gnu"

fail() {
  echo "CHECK FAILED: $*" >&2
  exit 1
}

[[ -x "${lerobot_python}" ]] || fail "LeRobot Python 不存在: ${lerobot_python}"
[[ -f "${ros_ws}/install/setup.bash" ]] || fail "ROS 工作区尚未编译: ${ros_ws}/install/setup.bash"
[[ -x "${ros_ws}/install/dual_nero_webapp/lib/dual_nero_webapp/webapp_node" ]] \
  || fail "WebApp 尚未编译，先执行 colcon build --packages-select dual_nero_webapp"
[[ -x "${repo_root}/deployment/scripts/joint16_run_rollout.sh" ]] \
  || fail "rollout 脚本不可执行: ${repo_root}/deployment/scripts/joint16_run_rollout.sh"
[[ -S /tmp/nero_bimanual_vla_bridge.sock ]] || echo "INFO: bridge socket 尚未启动"

command -v tmux >/dev/null || fail "缺少 tmux"
command -v ros2 >/dev/null || fail "缺少 ros2"
[[ -e "${realsense_lib}/librealsense2.so.2.59" ]] || fail "缺少 librealsense2 2.59: ${realsense_lib}"

for iface in can_left can_right; do
  if ! ip link show "${iface}" >/dev/null 2>&1; then
    fail "CAN 接口不存在: ${iface}"
  fi
  if ip -details link show "${iface}" | grep -q 'state UP'; then
    echo "OK: ${iface} UP"
  else
    echo "INFO: ${iface} 尚未启用，启动本地时会自动执行 enable_can.sh"
  fi
done

"${repo_root}/deployment/scripts/patch_lerobot_position_hold_compat.py" \
  --lerobot-root /home/work/lerobot --check

env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    "${lerobot_python}" - <<'PY'
from lerobot_robot_nero import NeroBimanual, NeroBimanualConfig

robot = NeroBimanual(NeroBimanualConfig(action_schema='joint16'))
assert len(robot.action_features) == 16
assert tuple(robot.position_hold_action_keys) == tuple(robot.action_features)
print('OK: joint16 plugin and position-hold contract')
PY

if tmux has-session -t joint16 2>/dev/null; then
  echo "INFO: tmux 会话 joint16 已存在，请先检查: tmux attach -t joint16"
fi

if pgrep -f '[l]erobot.scripts.lerobot_rollout|[b]imanual_vla_bridge.launch.py' >/dev/null; then
  echo "INFO: 已存在 rollout 或 bridge 进程，请勿重复启动"
fi

"${repo_root}/deployment/scripts/test_bimanual_vla_pipeline.sh" >/tmp/joint16_pipeline_check.log 2>&1 \
  || fail "离线 pipeline 测试失败，日志: /tmp/joint16_pipeline_check.log"

echo "CHECK OK"
echo "下一步: ./deployment/scripts/joint16_start_local.sh"
