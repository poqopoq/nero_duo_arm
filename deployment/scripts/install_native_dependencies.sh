#!/usr/bin/env bash
# Install ROS 2 Humble dependencies for the dual NERO project on native Ubuntu 22.04.
set -euo pipefail

if [ "${EUID}" -eq 0 ]; then
  echo "请以普通用户运行；脚本会在需要时调用 sudo。" >&2
  echo "Do not run this installer with sudo: pip --user would target /root." >&2
  exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

sudo apt-get update
sudo apt-get install -y \
  can-utils ethtool v4l-utils \
  python3-pip python3-scipy python3-opencv python3-yaml \
  python3-fastapi python3-uvicorn python3-wsproto \
  python3-colcon-common-extensions \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-topic-tools \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-moveit \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description \
  ros-humble-realsense2-camera-msgs

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${NERO_DUO_ROOT:-$(cd -- "${SCRIPT_DIR}/../.." && pwd)}"
WS_DIR="${NERO_DUO_WS:-${ROOT_DIR}/ros2_ws}"
DEPLOY_DIR="${NERO_DUO_DEPLOY:-${ROOT_DIR}/deployment}"

python3 -m pip install --user python-can

PYAGXARM_DIR="${NERO_DUO_PYAGXARM_DIR:-${ROOT_DIR}/dependencies/pyAgxArm}"
if [ -d "${PYAGXARM_DIR}" ]; then
  python3 -m pip install --user "${PYAGXARM_DIR}"
else
  echo "pyAgxArm source not found at ${PYAGXARM_DIR}" >&2
  exit 1
fi

"${DEPLOY_DIR}/scripts/apply_submodule_patches.sh"

echo "Native dependencies installed."
echo "Install udev rules with:"
echo "  sudo ${DEPLOY_DIR}/scripts/install_can_stable_names.sh"
echo "  ${DEPLOY_DIR}/scripts/install_realsense_udev.sh"
echo "Workspace: ${WS_DIR}"
echo "pyAgxArm:  ${PYAGXARM_DIR}"
