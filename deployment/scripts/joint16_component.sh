#!/usr/bin/env bash
set -euo pipefail

component="${1:-}"
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_ws="${repo_root}/ros2_ws"

if [[ -z "${component}" ]]; then
  echo "usage: $0 {drivers|cameras|grippers|moveit|bridge}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${ros_ws}/install/setup.bash"
set -u

cd "${repo_root}"

case "${component}" in
  drivers)
    exec ros2 launch dual_nero_control dual_nero_drivers.launch.py \
      can_left:=can_left can_right:=can_right \
      speed_percent:=5 auto_enable:=true \
      tcp_offset:='[0.2035, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'
    ;;
  cameras)
    export LD_LIBRARY_PATH="/home/work/.local/opt/realsense-2.59/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
    exec ros2 launch dual_nero_cameras all_cameras.launch.py
    ;;
  grippers)
    exec ros2 launch dual_nero_control dual_grippers.launch.py \
      allow_motion:=false enable_motor:=false
    ;;
  moveit)
    exec ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
      dry_run:=false rviz:=false
    ;;
  bridge)
    gripper_motion="${JOINT16_GRIPPER_MOTION:-false}"
    case "${gripper_motion}" in
      true|false) ;;
      *)
        echo "JOINT16_GRIPPER_MOTION must be true or false, got: ${gripper_motion}" >&2
        exit 2
        ;;
    esac
    exec ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
      action_schema:=joint16 \
      execution_mode:=stream \
      allow_real_motion:=true \
      gripper_enabled:="${gripper_motion}" \
      allow_gripper_motion:="${gripper_motion}" \
      camera_scene_key:=base_0_rgb \
      camera_left_wrist_key:=left_wrist_0_rgb \
      camera_right_wrist_key:=right_wrist_0_rgb \
      max_joint_step_rad:=0.02 \
      max_gripper_step_m:=0.02 \
      stream_command_rate_hz:=50.0 \
      stream_feedback_timeout_sec:=0.75
    ;;
  *)
    echo "unknown component: ${component}" >&2
    exit 2
    ;;
esac
