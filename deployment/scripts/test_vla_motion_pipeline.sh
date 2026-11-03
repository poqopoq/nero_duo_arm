#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_ws="${repo_root}/ros2_ws"

set +u
source /opt/ros/humble/setup.bash
source "${ros_ws}/install/setup.bash"
set -u

cd "${ros_ws}"
python3 -m unittest -v \
  "${ros_ws}/src/dual_nero_vla_bridge/test/test_motion_pipeline.py"
