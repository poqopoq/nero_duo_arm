<<<<<<< HEAD
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
```

## Install on native Ubuntu 22.04

```bash
sudo ./deployment/scripts/install_native_dependencies.sh
sudo ./deployment/scripts/install_can_stable_names.sh
./deployment/scripts/install_realsense_udev.sh
```

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
deployment/docs/00_stage0_system_hardware_baseline.md
deployment/docs/03_stage2_dual_can_namespace.md
deployment/docs/04_software_estop_validation.md
deployment/docs/05_stage3_model_parameters.md
deployment/docs/06_stage4_realsense_setup.md
deployment/docs/07_stage5_moveit_plan.md
deployment/docs/08_stage6_camera_calibration.md
```

## Safety

This setup currently uses software emergency stop and a standard support pose instead of a physical E-stop, as selected by the project owner. Always return both arms to the verified support pose before disabling motors. See `deployment/docs/04_software_estop_validation.md`.
=======
# nero_duo_arm
>>>>>>> origin/main
