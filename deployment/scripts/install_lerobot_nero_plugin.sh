#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LEROBOT_PYTHON="${LEROBOT_PYTHON:-/home/work/miniforge3/envs/lerobot/bin/python}"
PLUGIN_DIR="${REPO_ROOT}/lerobot_robot_nero"

if [[ ! -x "${LEROBOT_PYTHON}" ]]; then
  echo "LeRobot Python not found: ${LEROBOT_PYTHON}" >&2
  echo "Set LEROBOT_PYTHON to the Python executable inside the LeRobot environment." >&2
  exit 1
fi

run_clean() {
  env -u PYTHONPATH \
      -u AMENT_PREFIX_PATH \
      -u CMAKE_PREFIX_PATH \
      -u LD_LIBRARY_PATH \
      -u ROS_DISTRO \
      -u ROS_VERSION \
      -u ROS_PYTHON_VERSION \
      "$@"
}

run_clean "${LEROBOT_PYTHON}" -m pip install -e "${PLUGIN_DIR}" \
  --no-deps --config-settings editable_mode=compat

# The repository root contains a same-named project directory.  The default
# strict editable finder is appended to sys.meta_path and can lose to that
# directory's implicit namespace package when tools start from the repo root.
# Compat mode puts the source root on sys.path, so LeRobot's plugin discovery
# continues to work from the repository root as documented.
cd "${REPO_ROOT}"
run_clean "${LEROBOT_PYTHON}" - <<'PY'
from lerobot.robots import RobotConfig
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_robot_nero import NeroConfig, NeroBimanualConfig

register_third_party_plugins()
if RobotConfig.get_choice_class('nero_ros') is not NeroConfig:
    raise SystemExit('nero_ros plugin registration failed')
if RobotConfig.get_choice_class('nero_bimanual_ros') is not NeroBimanualConfig:
    raise SystemExit('nero_bimanual_ros plugin registration failed')
print('lerobot_robot_nero installed and registered (nero_ros, nero_bimanual_ros)')
PY

