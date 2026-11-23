#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
session="${JOINT16_TMUX_SESSION:-joint16}"
component="${repo_root}/deployment/scripts/joint16_component.sh"
gripper_motion="${JOINT16_GRIPPER_MOTION:-false}"
socket="/tmp/nero_bimanual_vla_bridge.sock"
active_pattern='[d]ual_nero_drivers.launch.py|[a]ll_cameras.launch.py|[d]ual_grippers.launch.py|[m]oveit_execution.launch.py|[b]imanual_vla_bridge.launch.py|[a]gx_arm_ctrl_single_node|[l]eft_gripper_node|[r]ight_gripper_node|[m]ove_group|[d]ual_nero_trajectory_bridge|[b]imanual_vla_bridge|[l]erobot.scripts.lerobot_rollout'

command -v tmux >/dev/null || {
  echo "缺少 tmux" >&2
  exit 1
}

set +u
source /opt/ros/humble/setup.bash
source "${repo_root}/ros2_ws/install/setup.bash"
set -u

if tmux has-session -t "${session}" 2>/dev/null; then
  if tmux list-panes -s -t "${session}" -F '#{pane_dead}' | grep -q '^0$'; then
    echo "tmux 会话已存在且仍有活动进程: ${session}" >&2
    echo "tmux attach -t ${session}" >&2
    exit 1
  fi
  echo "tmux 会话 ${session} 只剩已退出窗口，清理后重新启动"
  tmux kill-session -t "${session}"
fi

if pgrep -f "${active_pattern}" >/dev/null; then
  echo "已有本地节点在运行，禁止重复启动。当前进程:" >&2
  pgrep -af "${active_pattern}" >&2
  exit 1
fi

for iface in can_left can_right; do
  if ! ip link show "${iface}" >/dev/null 2>&1; then
    echo "CAN 接口不存在: ${iface}" >&2
    exit 1
  fi
  if ! ip -details link show "${iface}" | grep -q 'state UP'; then
    echo "启用 ${iface} ..."
    sudo "${repo_root}/deployment/scripts/enable_can.sh" "${iface}" 1000000
  fi
done

tmux new-session -d -s "${session}" -n drivers "${component} drivers"
tmux set-option -t "${session}" remain-on-exit on
tmux new-window -d -t "${session}" -n cameras "${component} cameras"
tmux new-window -d -t "${session}" -n grippers "${component} grippers"
tmux new-window -d -t "${session}" -n moveit "${component} moveit"
tmux new-window -d -t "${session}" -n bridge \
  "env JOINT16_GRIPPER_MOTION=${gripper_motion} ${component} bridge"

socket_ready=0
moveit_ready=0
for _ in $(seq 1 60); do
  [[ -S "${socket}" ]] && socket_ready=1
  if ros2 action list 2>/dev/null | grep -qx '/move_action'; then
    moveit_ready=1
  fi
  [[ "${socket_ready}" -eq 1 && "${moveit_ready}" -eq 1 ]] && break
  if tmux list-panes -s -t "${session}" -F '#{pane_dead}' | grep -q '^1$'; then
    break
  fi
  sleep 1
done

if tmux list-panes -s -t "${session}" -F '#{pane_dead}' | grep -q '^1$'; then
  echo "tmux 有窗口异常退出，未继续等待。当前窗口:" >&2
  tmux list-windows -t "${session}" -F '#{window_index} #{window_name} dead=#{pane_dead}' >&2
  echo "tmux attach -t ${session}" >&2
  exit 1
fi

if [[ "${socket_ready}" -ne 1 ]]; then
  echo "bridge socket 未创建，请检查:" >&2
  echo "tmux attach -t ${session}" >&2
  exit 1
fi

if [[ "${moveit_ready}" -ne 1 ]]; then
  echo "MoveIt /move_action 在 60 秒内未就绪，检查窗口 3=moveit:" >&2
  echo "tmux attach -t ${session}" >&2
  exit 1
fi

echo "本地节点已启动"
echo "查看日志: tmux attach -t ${session}"
echo "窗口: 0=drivers 1=cameras 2=grippers 3=moveit 4=bridge"
echo "下一步: ./deployment/scripts/joint16_move_home.sh"
