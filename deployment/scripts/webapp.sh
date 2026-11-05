#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

inner_command="set -e; source /opt/ros/humble/setup.bash; source ${repo_root}/ros2_ws/install/setup.bash; exec ros2 launch dual_nero_webapp webapp.launch.py \"\$@\""

exec "${repo_root}/deployment/scripts/run_in_dialout.sh" \
  bash -lc "${inner_command}" _ "$@"
