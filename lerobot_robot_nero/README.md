# lerobot_robot_nero

LeRobot 0.6.x robot plugin for the NERO arm.

It exposes a 7-D absolute end-effector state/action contract through a local
Unix socket served by the ROS 2 Humble package `dual_nero_vla_bridge`:

```text
[x, y, z, rx, ry, rz, gripper]
```

The plugin does not import `rclpy`. It can run in the Python 3.12 LeRobot
environment while ROS 2 remains in its own Python 3.10 process.

## Install

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python -m pip install -e \
    /home/work/nero_duo_arm/lerobot_robot_nero --no-deps \
    --config-settings editable_mode=compat
```

`editable_mode=compat` is required because the repository root contains a
directory with the same name as the plugin. Re-run the install after changing
the plugin package or its `pyproject.toml`.

## Use

Start `dual_nero_vla_bridge` in the ROS environment first, then pass
`--robot.type=nero_ros` to LeRobot tools. The bridge socket defaults to
`/tmp/nero_vla_bridge.sock`.

## Bimanual rollout

For the 20-D Xense UMI contract and `lerobot-rollout`, use the separate plugin:

```bash
--robot.type=nero_bimanual_ros \
--robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
--robot.action_schema=rot6d20 \
--robot.scene_camera_key=side \
--robot.left_wrist_camera_key=left_wrist \
--robot.right_wrist_camera_key=right_wrist
```

The matching ROS node is launched with
`ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py`. See
`deployment/docs/15_stage11_vla_bimanual_eval.md` for the full gate sequence.

