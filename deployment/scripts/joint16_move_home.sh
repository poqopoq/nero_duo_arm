#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${repo_root}/ros2_ws/install/setup.bash"
set -u

set +e
/usr/bin/python3 "${repo_root}/deployment/scripts/joint16_move_home.py" "$@"
status=$?
set -e

if [[ "${status}" -eq 3 ]]; then
  echo "MoveIt /move_action 不可用：先确认 tmux 会话 joint16 的 3=moveit 窗口已启动。" >&2
  echo "tmux attach -t joint16" >&2
fi
exit "${status}"
