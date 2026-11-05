#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_ws="${repo_root}/ros2_ws"
ros_python="${ROS_PYTHON:-/usr/bin/python3}"

set +u
source /opt/ros/humble/setup.bash
source "${ros_ws}/install/setup.bash"
set -u

cd "${ros_ws}/src/dual_nero_control"
"${ros_python}" -m unittest discover -s test -v
