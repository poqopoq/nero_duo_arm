# dual_nero_description

This package contains the dual NERO table model and the AGILINK OmniPicker 3
gripper description.

## OmniPicker 3 files

- `urdf/omnipicker3_gripper_macro.xacro`: prefixed dual-arm macro.
- `urdf/omnipicker3.urdf.xacro`: standalone display model.
- `meshes/omnipicker3/`: official vendor meshes reproduced under
  `meshes/omnipicker3/SOURCE.md`.

The current AGILINK `OP3_P0_URDF_EC` package represents the single driven
mechanism with prismatic `joint1`; the remaining twelve revolute joints model
closed-loop linkage members. The vendor file does not provide mimic equations,
so this package exposes those joints as passive and does not invent coupling
ratios. For the ROS control and MoveIt contract, `<side>_op3_joint1` is the
master joint, with a corrected SI range of `0..0.02 m`.

The initial pinching-point estimate is `0.128 m` from the OP3 mounting face. The deployed
25 mm force sensor and 17.5 mm connector give a default `tool0 -> tcp` offset
of `0.1705 m`. Treat this as an initial model value and replace it with pivot
calibration.

Validate both models after changing transforms:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
xacro ros2_ws/src/dual_nero_description/urdf/omnipicker3.urdf.xacro > /tmp/op3.urdf
check_urdf /tmp/op3.urdf
xacro ros2_ws/src/dual_nero_description/urdf/two_nero_tabletop.urdf.xacro > /tmp/dual.urdf
check_urdf /tmp/dual.urdf
```
