#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_ws="${repo_root}/ros2_ws"
lerobot_python="/home/work/miniforge3/envs/lerobot/bin/python"

set +u
source /opt/ros/humble/setup.bash
source "${ros_ws}/install/setup.bash"
set -u

cd "${ros_ws}"
python3 -m unittest -v \
  "${ros_ws}/src/dual_nero_vla_bridge/test/test_bimanual_motion_pipeline.py"

cd "${repo_root}"
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    PYTHONPATH="${repo_root}/lerobot_robot_nero" \
    "${lerobot_python}" \
    "${repo_root}/lerobot_robot_nero/tests/test_bimanual_plugin.py" -v
