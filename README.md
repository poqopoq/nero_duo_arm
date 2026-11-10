# Dual NERO Deployment

ROS 2 Humble deployment workspace for two AgileX NERO 7-DoF arms with table mounting, dual-arm MoveIt planning, RealSense integration, software emergency-stop procedures, and camera calibration tooling.

## Repository layout

```text
ros2_ws/src/
  agx_arm_ros/                 # official AgileX driver submodule
  dual_nero_description/       # table-mounted dual-arm model
  dual_nero_cameras/           # RealSense launch configuration
  dual_nero_control/           # MoveIt FollowJointTrajectory bridge
  dual_nero_moveit_config/     # dual-arm MoveIt configuration
  dual_nero_calibration/       # checkerboard camera calibration
  dual_nero_gripper_ft_sensor/ # gripper six-axis force/torque UART driver
  dual_nero_webapp/            # browser joint and camera dashboard

dependencies/pyAgxArm/         # official AgileX Python SDK submodule
references/cc_agx_arm_ros/     # CloudChef dual-arm reference submodule
deployment/                    # handoff docs, scripts, support pose
```

## Clone

```bash
git clone --recurse-submodules <repo-url> dual_nero_project
cd dual_nero_project
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
./deployment/scripts/apply_submodule_patches.sh
```

The patch script adds the guarded `clear_joint_error` and `reset_motion`
services used by the Web dashboard to the pinned AgileX driver submodule. It is
idempotent, and the native dependency installer invokes it automatically.

## Install on native Ubuntu 22.04

```bash
./deployment/scripts/install_native_dependencies.sh
sudo ./deployment/scripts/install_can_stable_names.sh
./deployment/scripts/install_realsense_udev.sh
```

Scripts resolve the repository and ROS workspace paths from their own
location. Set `NERO_DUO_ROOT`, `NERO_DUO_WS`, or `NERO_DUO_DEPLOY` only when
using a nonstandard layout.

Then build:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Documentation

Start with:

```text
deployment/PROJECT_HANDOFF.md
deployment/docs/00_hardware_baseline.md
deployment/docs/01_driver_can_safety.md
deployment/docs/02_sensors_and_calibration.md
deployment/docs/03_moveit_and_park.md
deployment/docs/04_umi_lerobot.md
deployment/docs/05_vla_pipeline.md
deployment/docs/11_stage8_webapp.md
deployment/docs/12_usage_and_commands.md
deployment/docs/17_gripper_can_diagnostics.md
deployment/docs/18_gripper_ft_sensor.md
```

## Web dashboard

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true
```

Open `http://<robot-host>:8080`. The driver and webapp control gates both start
closed; open the intended side explicitly in the UI before sending a command.
Motion requests are planned and executed through MoveIt `/move_action`. MoveIt
checks the dual-arm robot and table geometry; camera point clouds are not yet
part of the planning scene. `moveit_dry_run` defaults to `true`, so set
`moveit_dry_run:=false` only for a validated real-hardware run. See
`deployment/docs/11_stage8_webapp.md` for the safety sequence and camera topics.

## Safety

This setup currently uses software emergency stop and a gravity-stable park pose instead of a physical E-stop, as selected by the project owner. Return both arms with `deployment/scripts/reset_to_standard_support.sh`; the script keeps motors enabled by default and only supports power-off with `--disable` after hardware-limit and post-disable stability checks. See `deployment/docs/01_driver_can_safety.md`.
