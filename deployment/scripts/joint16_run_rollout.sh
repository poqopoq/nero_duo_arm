#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
lerobot_python="/home/work/miniforge3/envs/lerobot/bin/python"
socket="/tmp/nero_bimanual_vla_bridge.sock"
task="${JOINT16_TASK:-Use the right arm to put apple into white box.}"
home_position_file="${repo_root}/deployment/config/joint16_home.yaml"
inference_mode="${JOINT16_INFERENCE_MODE:-rtc_guided}"
refill_seconds="${JOINT16_REFILL_SEC:-0.8}"

if ! awk -v value="${refill_seconds}" 'BEGIN { exit !(value + 0 > 0) }'; then
  echo "JOINT16_REFILL_SEC 必须是正数，当前: ${refill_seconds}" >&2
  exit 2
fi

case "${inference_mode}" in
  rtc_guided)
    chunk_merge="append"
    ;;
  chunk)
    chunk_merge="aligned"
    ;;
  *)
    echo "JOINT16_INFERENCE_MODE 必须是 rtc_guided 或 chunk，当前: ${inference_mode}" >&2
    exit 2
    ;;
esac

if [[ ! -t 0 ]]; then
  echo "rollout 必须在可见的交互终端运行；非交互启动会在退出时向 bridge 发送 stop" >&2
  exit 1
fi

[[ -S "${socket}" ]] || {
  echo "bridge socket 不存在: ${socket}" >&2
  exit 1
}

if ! ss -ltn | grep -q '127.0.0.1:7447'; then
  echo "本地 7447 未监听，先执行第 3 步启动服务器和隧道" >&2
  exit 1
fi

cd "${repo_root}"
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
  "${lerobot_python}" -m lerobot.scripts.lerobot_rollout \
  --inference.type=remote \
  --inference.endpoint=tcp/127.0.0.1:7447 \
  --inference.deployment=nero-bimanual-joint16 \
  --inference.mode="${inference_mode}" \
  --inference.chunk_merge="${chunk_merge}" \
  --inference.refill_seconds="${refill_seconds}" \
  --inference.semantics=nero-bimanual-joint16-rad-metres-v1 \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket="${socket}" \
  --robot.action_schema=joint16 \
  --robot.policy_robot_type=nero_dual_joint16 \
  --robot.home_position_file="${home_position_file}" \
  --robot.scene_camera_key=base_0_rgb \
  --robot.left_wrist_camera_key=left_wrist_0_rgb \
  --robot.right_wrist_camera_key=right_wrist_0_rgb \
  --robot.scene_image_width=224 --robot.scene_image_height=224 \
  --robot.left_wrist_image_width=224 --robot.left_wrist_image_height=224 \
  --robot.right_wrist_image_width=224 --robot.right_wrist_image_height=224 \
  --fps=30 \
  --return_to_initial_position=false \
  --task="${task}" \
  --interactive=true
