#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
lerobot_root="${LEROBOT_ROOT:-/home/zhiyuan/lerobot}"
lerobot_python="${LEROBOT_PYTHON:-/home/zhiyuan/miniforge3/envs/lerobot/bin/python}"
default_config="deployment/config/server_nero_joint16_rtc.yaml"
server_config="${JOINT16_SERVER_CONFIG:-${default_config}}"

cd "${repo_root}"
[[ -x "${lerobot_python}" ]] || {
  echo "LeRobot Python 不存在: ${lerobot_python}" >&2
  exit 1
}
[[ -f "${server_config}" ]] || {
  echo "policy server 配置不存在: ${server_config}" >&2
  exit 1
}

python3 deployment/scripts/patch_lerobot_pi05_checkpoint_compat.py \
  --lerobot-root "${lerobot_root}"
python3 deployment/scripts/patch_lerobot_pi05_checkpoint_compat.py \
  --lerobot-root "${lerobot_root}" --check

if ss -ltn | grep -q ':7447 '; then
  echo "端口 7447 已占用，停止旧 policy server ..."
  pkill -f lerobot_policy_server || true
  sleep 2
fi

exec env CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-1}" \
  "${lerobot_python}" -m lerobot.scripts.lerobot_policy_server \
  --config_path="${server_config}"
