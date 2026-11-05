#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_ws="${repo_root}/ros2_ws"
lerobot_python="/home/work/miniforge3/envs/lerobot/bin/python"
ros_python="${ROS_PYTHON:-/usr/bin/python3}"

set +u
source /opt/ros/humble/setup.bash
source "${ros_ws}/install/setup.bash"
set -u

export ROS_DOMAIN_ID=82
export ROS_LOCALHOST_ONLY=1

cd "${ros_ws}"
"${ros_python}" -m unittest -v \
  "${ros_ws}/src/dual_nero_vla_bridge/test/test_bimanual_motion_pipeline.py"

cd "${repo_root}"
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    PYTHONPATH="${repo_root}/lerobot_robot_nero" \
    "${lerobot_python}" \
    "${repo_root}/lerobot_robot_nero/tests/test_bimanual_plugin.py" -v
