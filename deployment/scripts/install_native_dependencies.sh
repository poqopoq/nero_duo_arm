#!/usr/bin/env bash
# Install ROS 2 Humble dependencies for the dual NERO project on native Ubuntu 22.04.
set -euo pipefail

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

sudo apt-get update
sudo apt-get install -y \
  can-utils ethtool v4l-utils \
  python3-pip python3-scipy python3-opencv python3-yaml \
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

python3 -m pip install --user python-can

PYAGXARM_DIR="/home/a/workspace/dual_nero_ws/dependencies/pyAgxArm"
if [ -d "${PYAGXARM_DIR}" ]; then
  python3 -m pip install --user "${PYAGXARM_DIR}"
else
  echo "pyAgxArm source not found at ${PYAGXARM_DIR}" >&2
  exit 1
fi

echo "Native dependencies installed."
echo "Install udev rules with:"
echo "  sudo /home/a/workspace/dual_nero_deployment/scripts/install_can_stable_names.sh"
echo "  /home/a/workspace/dual_nero_deployment/scripts/install_realsense_udev.sh"
