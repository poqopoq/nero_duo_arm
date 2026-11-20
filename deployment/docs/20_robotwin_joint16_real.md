# RoboTwin joint16 checkpoint 真机部署

本文对应 `pi05_place_apple_box_staged_joint16`：

```text
/Data/zhiyuan/checkpoints/pi05_place_apple_box_staged_joint16/checkpoints/030000/pretrained_model
```

它不是 `server_nero_pi05.yaml` 的 20-D TCP/rot6d 契约，不能直接把
`server_nero_pi05.yaml` 指到该目录。正确契约是：

```text
state/action = right_7_joints + left_7_joints + right_gripper_width_m + left_gripper_width_m
```

关节单位 rad，夹爪宽度单位 m。checkpoint 使用绝对关节动作，
`use_relative_actions=false`，训练帧率 30 Hz，预测 50 步。

## 0. 机器人控制机：同步代码并重装插件

本次新增了 `joint16` schema；只替换 checkpoint 而不更新控制机代码，会在
启动 bridge 或 rollout 时报 `action_schema` 不支持。先把当前工作区（包括未提交
改动）同步到控制机，然后执行：

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash

env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python -m pip install -e \
    /home/work/nero_duo_arm/lerobot_robot_nero --no-deps \
    --config-settings editable_mode=compat

cd /home/work/nero_duo_arm/ros2_ws
colcon build --symlink-install --packages-select dual_nero_vla_bridge
source install/setup.bash
ros2 pkg executables dual_nero_vla_bridge

cd /home/work/nero_duo_arm
./deployment/scripts/test_bimanual_vla_pipeline.sh
```

脚本最后应显示 `OK`。该测试不会驱动真机。

## 1. GPU 主机：启动 policy server

先停掉 7447 上的旧仿真 server：

```bash
if [ -f /home/zhiyuan/vla_server/server.pid ]; then
  kill "$(cat /home/zhiyuan/vla_server/server.pid)" || true
fi
ss -ltnp | grep ':7447' || true
```

启动 16-D server。`server_nero_joint16.yaml` 已把 `n_action_steps` 设为 25，
即每 25 步重规划一次：

```bash
cd /home/zhiyuan/nero_duo_arm
/home/zhiyuan/miniforge3/envs/lerobot/bin/python \
  -m lerobot.scripts.lerobot_policy_server \
  --config_path=deployment/config/server_nero_joint16.yaml
```

看到 ready 后保持该终端运行。机器人端通过 `tcp/<GPU_IP>:7447` 连接；如果网络
策略阻断 7447，可先做 SSH 转发：

```bash
ssh -N -L 7447:127.0.0.1:7447 <USER>@<GPU_IP>
```

## 2. 机器人控制机：启动驱动、相机和夹爪反馈

以下命令在 NERO 控制机执行，路径按现场工作区调整。

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

sudo ./deployment/scripts/enable_can.sh can_left 1000000
sudo ./deployment/scripts/enable_can.sh can_right 1000000
./deployment/scripts/check_nero_feedback.sh can_left
./deployment/scripts/check_nero_feedback.sh can_right
./deployment/scripts/check_realsense.sh
```

驱动和相机：

```bash
ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=true \
  tcp_offset:='[0.2443, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'

ros2 launch dual_nero_cameras all_cameras.launch.py
```

joint16 的 state 包含夹爪宽度，因此必须先启动夹爪**只读反馈**：

```bash
ros2 launch dual_nero_control dual_grippers.launch.py \
  allow_motion:=false enable_motor:=false

ros2 topic hz /left_gripper/width
ros2 topic hz /right_gripper/width
```

确认两路宽度都在约 20 Hz 更新。只有完成空载开合和软件急停验收后，才改成
`allow_motion:=true enable_motor:=true`，并给 bridge 加
`gripper_enabled:=true allow_gripper_motion:=true`。

## 3. 把机械臂移到训练初始位姿

训练数据每个 episode 的初始关节角完全相同：

```text
[-0.07127925, -1.36069359, 0.02806489, 1.88160456, 0.04045673, 0.00094248, 1.57072651]
```

左右臂都使用这一组值，右臂在前、左臂在后的 14 关节顺序是：

```text
[right_joint1..7, left_joint1..7]
```

该 checkpoint 的 200 条训练数据中左臂 7 关节全程保持这组固定值，实际运动只来自
右臂；左臂仍要准确摆到训练 home 并保持，不能停在支撑位。

现场 `support_pose.yaml` 的右臂 `joint4/joint5` 与训练初始位姿差距较大，不要
直接从支撑位启动策略。先用 MoveIt 以 `max_velocity_scaling_factor<=0.05` 把
两臂低速移动到上述目标，确认实际反馈误差在 `0.01--0.03 rad` 内，再启动 VLA
bridge。`joint7` 目标距离上限只有约 `7e-5 rad`，首次建议收到 `1.5700`，并
确认驱动没有超限标志。

## 4. 启动 joint16 bridge

先 dry-run。相机 key 必须与 checkpoint 一致：

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=joint16 \
  execution_mode:=dry_run \
  allow_real_motion:=false \
  gripper_enabled:=false \
  allow_gripper_motion:=false \
  camera_scene_key:=base_0_rgb \
  camera_left_wrist_key:=left_wrist_0_rgb \
  camera_right_wrist_key:=right_wrist_0_rgb \
  max_joint_step_rad:=0.02
```

检查 socket 和状态：

```bash
ls -l /tmp/nero_bimanual_vla_bridge.sock
ros2 node info /dual_nero_bimanual_vla_bridge
```

用插件读一次 16-D observation，确认顺序和有限值：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python - <<'PY'
from lerobot_robot_nero import NeroBimanual, NeroBimanualConfig

with NeroBimanual(NeroBimanualConfig(
        action_schema='joint16',
        scene_camera_key='base_0_rgb',
        left_wrist_camera_key='left_wrist_0_rgb',
        right_wrist_camera_key='right_wrist_0_rgb',
        scene_image_width=224, scene_image_height=224,
        left_wrist_image_width=224, left_wrist_image_height=224,
        right_wrist_image_width=224, right_wrist_image_height=224)) as robot:
    obs = robot.get_observation()
    print({k: v for k, v in obs.items() if not hasattr(v, 'shape')})
    print({k: v.shape for k, v in obs.items() if hasattr(v, 'shape')})
PY
```

确认没有 `feature/schema mismatch` 后，再切到低速 stream：

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=joint16 \
  execution_mode:=stream \
  allow_real_motion:=true \
  gripper_enabled:=false \
  allow_gripper_motion:=false \
  camera_scene_key:=base_0_rgb \
  camera_left_wrist_key:=left_wrist_0_rgb \
  camera_right_wrist_key:=right_wrist_0_rgb \
  max_joint_step_rad:=0.02 \
  max_gripper_step_m:=0.02 \
  stream_command_rate_hz:=50.0 \
  stream_feedback_timeout_sec:=0.75
```

`stream` 会直接写 `/<side>/control/joint_states`，轨迹桥 `dry_run` 不会拦截它。
首次必须清空工作区并有人监护。

## 5. 机器人端启动 rollout client

```bash
cd /home/work/nero_duo_arm
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.scripts.lerobot_rollout \
  --inference.type=remote \
  --inference.endpoint=tcp/<GPU_IP>:7447 \
  --inference.deployment=nero-bimanual-joint16 \
  --inference.mode=chunk \
  --inference.chunk_merge=aligned \
  --inference.semantics=nero-bimanual-joint16-rad-metres-v1 \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
  --robot.action_schema=joint16 \
  --robot.policy_robot_type=nero_dual_joint16 \
  --robot.scene_camera_key=base_0_rgb \
  --robot.left_wrist_camera_key=left_wrist_0_rgb \
  --robot.right_wrist_camera_key=right_wrist_0_rgb \
  --robot.scene_image_width=224 --robot.scene_image_height=224 \
  --robot.left_wrist_image_width=224 --robot.left_wrist_image_height=224 \
  --robot.right_wrist_image_width=224 --robot.right_wrist_image_height=224 \
  --fps=30 \
  --return_to_initial_position=false \
  --task="Use the right arm to put apple into white box." \
  --interactive=true
```

`--task` 必须使用训练集里的原始指令。该数据集共有 11 条同义标签，上面这条在
200 个 episode 中出现最多（30 次）。

进入交互终端后先不要输入 `/start`。确认 bridge 没有 fault、夹爪宽度反馈新鲜、
两臂在训练初始位姿后，再输入：

```text
/start
```

出现异常立刻 `/stop`，并同时用驱动侧急停。

## 6. 首次验收顺序

1. `dry_run` 下观测 state `[16]`、三路图像 `[224,224,3]`，动作回显 `[16]`。
2. `stream` + `gripper_enabled=false`，只观察关节目标，先不发模型动作。
3. 从 home 位姿发送一个 `right_joint1 += 0.01 rad` 的小动作，确认方向和停止。
4. 接 rollout，`max_joint_step_rad=0.02`，先跑 10--20 秒，确认无控制门/fault。
5. 夹爪空载开合通过后，再打开 `gripper_enabled/allow_gripper_motion`，先只跑
   抓取阶段。

## 7. 仿真到真机必须对齐的量

- **初始位姿**：训练固定为上面的 home 状态。支撑位不等于训练初始位姿。
- **物体布局**：200 条训练数据的苹果中心约为 `x=0.518, y=-0.450`，盒子中心
  约为 `x=0.149, y=-0.350`。当前 `demo_nero_apple_box_staged.yml` 若把苹果改到
  `y=-0.35`，会与训练分布相差约 10 cm。
- **相机**：训练是头部 D435 + 左右腕 D405，`base_0_rgb/left_wrist_0_rgb/
  right_wrist_0_rgb`。现场安装位置和朝向要尽量复现仿真视角，不能只用旧
  `side/left_wrist/right_wrist` 标定结果。
- **夹爪宽度**：训练最大宽度 0.10 m，实物 CTAG 约 0.09 m。bridge 会把动作宽度
  clamp 到 `gripper_max_m`，首轮会损失约 1 cm 开度，这是已知 sim2real 偏差。
- **关节方向/零点**：仿真 embodiment 来自同一套 NERO URDF，但仍应先用
  `+0.01 rad` 单关节小动作核对方向和反馈，不能假设符号完全一致。

## 8. 常见失败

- `Feature ... differs`：检查 server YAML 的 name/shape、client 的
  `--robot.*_camera_key`、`--robot.action_schema` 和图像尺寸是否同时为 224。
- `no ... gripper width feedback`：夹爪节点未启动或宽度话题不新鲜。
- `action ... exceeds max_joint_step_rad`：属正常安全限幅，先用 dry-run 看
  `(clamped from ...)`，确认策略没有输出跳变，再逐步放宽。
- `motion fault latched`：`/stop` 后重启 bridge；不要在同一进程里继续发动作。
