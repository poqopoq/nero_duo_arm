#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
package_dir="${repo_root}/ros2_ws/src/dual_nero_gripper_ft_sensor"
ros_python="${ROS_PYTHON:-/usr/bin/python3}"

cd "${package_dir}"
PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  "${ros_python}" -m pytest -q -p no:cacheprovider test
