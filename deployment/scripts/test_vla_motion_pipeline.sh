#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_ws="${repo_root}/ros2_ws"
ros_python="${ROS_PYTHON:-/usr/bin/python3}"

set +u
source /opt/ros/humble/setup.bash
source "${ros_ws}/install/setup.bash"
set -u

export ROS_DOMAIN_ID=81
export ROS_LOCALHOST_ONLY=1

cd "${ros_ws}"
"${ros_python}" -m unittest -v \
  "${ros_ws}/src/dual_nero_vla_bridge/test/test_motion_pipeline.py"
