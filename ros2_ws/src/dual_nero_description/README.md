# dual_nero_description

This package contains the dual NERO table model and the AGILINK OmniPicker 3
gripper description.

## OmniPicker 3 files

- `urdf/omnipicker3_gripper_macro.xacro`: prefixed dual-arm macro.
- `urdf/omnipicker3.urdf.xacro`: standalone display model.
- `meshes/omnipicker3/`: official vendor meshes reproduced under
  `meshes/omnipicker3/SOURCE.md`.

The vendor URDF has independent revolute joints for the finger loops. The
deployment macro couples the seven non-master joints to
`<side>_op3_wide1_joint` with a multiplier of 1. The resulting master range is
`0..0.39 rad`, with `0` matching the documented closed dimensions and `0.39`
matching the open dimensions.

The fingerprint estimate is `0.130 m` from the OP3 mounting face. The deployed
25 mm force sensor and 17.5 mm connector give a default `tool0 -> tcp` offset
of `0.1725 m`. Treat this as an initial model value and replace it with pivot
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
