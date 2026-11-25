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

> 现场从自己的终端执行时，直接使用文末
> [“9. 四步执行”](#9-四步执行)，每步一条主命令。需要排障时再看
> [“10. 详细命令参考”](#10-详细命令参考排障用)。

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

python3 deployment/scripts/patch_lerobot_position_hold_compat.py \
  --lerobot-root /home/work/lerobot
python3 deployment/scripts/patch_lerobot_position_hold_compat.py \
  --lerobot-root /home/work/lerobot --check

cd /home/work/nero_duo_arm/ros2_ws
colcon build --symlink-install --packages-select dual_nero_vla_bridge dual_nero_webapp
source install/setup.bash
ros2 pkg executables dual_nero_vla_bridge

cd /home/work/nero_duo_arm
./deployment/scripts/test_bimanual_vla_pipeline.sh
```

脚本最后应显示 `OK`。该测试不会驱动真机。

LeRobot 0.6.2 的 `ThreadSafeRobot` 默认只把 `.pos` 结尾的动作当作位置目标。
`joint16` 使用 checkpoint 的原生键名（`right_nero_joint_1`、
`right_gripper_width` 等），因此插件会显式声明
`position_hold_action_keys`，上面的修补脚本让本地 LeRobot 使用该声明。若漏掉
这一步，rollout 会在连接 server 前报：

```text
ValueError: nero_dual_joint16 has no supported local position-hold contract
```

修补脚本是幂等的，并会保留
`src/lerobot/rollout/robot_wrapper.py.bak-joint16` 备份。它不会放宽旧的
`.pos` 规则：没有显式声明的机器人仍走原逻辑，位置与速度混合的动作仍会被拒绝。

## 1. GPU 主机：准备运行时并启动 policy server

该 checkpoint 的 `config.json` 由较新的 RoboTwin 训练运行时写出，带有
`drop_n_last_frames`、`relative_action_type` 和一组 `feature_adapter_*`
训练字段。固定版本的 LeRobot 0.6.2 如果不认识这些字段，会在加载权重前报
`DecodingError`。checkpoint 本身是 `use_feature_adapter=false`，权重中也没有
adapter/memory 参数，因此给 `PI05Config` 加上这些“仅加载用”的惰性字段即可。

每次重装或切换 `/home/zhiyuan/lerobot` 后执行一次：

```bash
cd /home/zhiyuan/nero_duo_arm
python3 deployment/scripts/patch_lerobot_pi05_checkpoint_compat.py \
  --lerobot-root /home/zhiyuan/lerobot
python3 deployment/scripts/patch_lerobot_pi05_checkpoint_compat.py \
  --lerobot-root /home/zhiyuan/lerobot --check
```

脚本是幂等的，并会保留
`configuration_pi05.py.bak-joint16` 备份。若后续升级到原生支持这些字段的
LeRobot，`--check` 会直接通过，不需要再改文件。

启动前确认目标 GPU 有足够显存。该 checkpoint 权重约 9.3 GB，加载后实测占用
约 9.5 GB；`device: cuda` 会使用 `CUDA_VISIBLE_DEVICES` 映射后的第一张卡：

```bash
nvidia-smi --query-gpu=index,name,memory.free,memory.total --format=csv
```

当前工作站 GPU 0 只有约 7.7 GiB 空闲，而 GPU 1 空闲，因此下面的启动命令
使用 `CUDA_VISIBLE_DEVICES=1`。如果 GPU 0 空闲，可以去掉该环境变量。

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
CUDA_VISIBLE_DEVICES=1 \
/home/zhiyuan/miniforge3/envs/lerobot/bin/python \
  -m lerobot.scripts.lerobot_policy_server \
  --config_path=deployment/config/server_nero_joint16.yaml
```

看到以下 ready 日志后保持该终端运行：

```text
Deployment warmed: name=nero-bimanual-joint16 ... action_rate=30.0 Hz horizon=0.833s
Policy server ready: deployment=nero-bimanual-joint16 ...
```

另开终端确认端口已监听：

```bash
ss -ltnp | grep ':7447'
```

如果机器人控制机能直连 GPU 主机的 7447，endpoint 可用
`tcp/10.10.6.11:7447`。当前现场的网络策略会阻断该 TCP 端口，因此推荐在控制机
保持一条 SSH 转发：

```bash
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \
  -L 7447:127.0.0.1:7447 zhiyuan@10.10.6.11
```

另开一个控制机终端确认隧道可用；随后 rollout 的 endpoint 必须使用
`tcp/127.0.0.1:7447`：

```bash
ss -ltnp | grep ':7447'
```

隧道断开时本地端口会一起消失，rollout 将无法连接；先恢复隧道再重试，不要改成
直连 GPU IP。

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
  tcp_offset:='[0.2015, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'

# 本机 realsense2_camera 4.59.1 需要 librealsense2 2.59；系统包若仍是
# 2.58，直接 launch 会报 librealsense2.so.2.59 not found。可先永久升级：
#   sudo apt install ros-humble-librealsense2=2.59.1-1jammy.20261012.181323
# 无法 sudo 时，使用已解包的本地运行库：
export LD_LIBRARY_PATH=/home/work/.local/opt/realsense-2.59/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
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

如果某一侧没有宽度消息，用诊断话题区分串口和夹爪本体：

```bash
ros2 topic echo /left_gripper/status --once
ros2 topic echo /right_gripper/status --once
```

正常时应有 `can_connected: true`、`feedback_fresh: true` 和非空
`feedback_position`。若适配器显示 `can_connected: true` 但反馈为空，检查该侧
夹爪 24 V 供电、CAN 线和接头；不要通过打开 `allow_motion` 绕过反馈检查。

### 每次推理前的夹爪硬检查

策略抓不到苹果时，先看 `/right_gripper/status`，不要先怀疑模型。以下三项
任一不满足，bridge 会拒绝动作，WebApp 也会阻止 Launch/Run：

- `estop_latched: false`；
- `fault_reason` 为空且 `state` 不是 `fault`；
- `enabled: true`（或 `auto_enable_on_command: true`），且宽度反馈新鲜。

普通 WebApp **Stop** / rollout `/stop` 只关闭机械臂控制门，不再调用夹爪
`emergency_stop`。只有显式急停按钮才会锁存夹爪急停。若上一次急停、Action
取消或反馈超时留下了 fault，按下面顺序恢复：

```bash
BASE=http://127.0.0.1:8080

curl -sS -X POST "$BASE/api/gripper" -H 'Content-Type: application/json' \
  -d '{"side":"right","action":"clear_fault"}'
curl -sS -X POST "$BASE/api/gripper" -H 'Content-Type: application/json' \
  -d '{"side":"right","action":"enable","enabled":true}'
curl -sS -X POST "$BASE/api/gripper" -H 'Content-Type: application/json' \
  -d '{"side":"right","action":"position","position":0}'

# 左夹爪同样执行 clear_fault -> enable -> position(0)
```

确认 `driver_state` 不是 `fault`、`estop_latched=false`、`driver_enabled=true`
后再 Launch。若 `clear_fault` 返回 `disable the motor before clearing faults`，
先调用同一接口的 `{"action":"enable","enabled":false}`。

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

现场 `support_pose.yaml` 的左右臂 Park 目标均已改为上述训练 home。启动策略前
仍应确认两臂实际反馈已经到位；如需重新归位，用 MoveIt 以
`max_velocity_scaling_factor<=0.05` 低速移动，确认实际反馈误差在
`0.01--0.03 rad` 内，再启动 VLA bridge。`joint7` 目标距离上限只有约
`7e-5 rad`，首次建议收到 `1.5700`，并确认驱动没有超限标志。

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

插件返回的是 checkpoint 原生键名，不是 `observation.state` 这个总键：

```text
right_nero_joint_1..7, left_nero_joint_1..7,
right_gripper_width, left_gripper_width,
base_0_rgb, left_wrist_0_rgb, right_wrist_0_rgb
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
  --inference.endpoint=tcp/127.0.0.1:7447 \
  --inference.deployment=nero-bimanual-joint16 \
  --inference.mode=chunk \
  --inference.chunk_merge=aligned \
  --inference.semantics=nero-bimanual-joint16-rad-metres-v1 \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
  --robot.action_schema=joint16 \
  --robot.policy_robot_type=nero_dual_joint16 \
  --robot.home_position_file=/home/work/nero_duo_arm/deployment/config/joint16_home.yaml \
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

joint16 的键不以 `.pos` 结尾，但插件显式声明了
`position_hold_action_keys`。`joint16_run_rollout.sh` 同时传入
`deployment/config/joint16_home.yaml` 作为原生复位目标。正常应看到：

```text
Captured initial robot position (16 keys)
Using configured robot reset position (16 keys)
```

因此交互式 `/reset` 会回到训练 home：右 7 关节、左 7 关节和两路夹爪宽度，共
16 维，而不是回到 Launch 时的当前位姿。当前 runbook 使用
`--return_to_initial_position=false`，所以进程退出时不会自动回 home；如需退出后
回位，用 WebApp 的 **Training Home** 按钮或 `joint16_move_home.sh`。若日志只有
`(0 keys)` 或没有 `Using configured robot reset position`，检查插件重装、home YAML
路径和 LeRobot 补丁，不要依赖 `/reset`。

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
- **夹爪宽度**：训练最大宽度 0.10 m；OmniPicker 3 标称行程约 0.12 m，实际写入前必须按本机 TCP/指腹测量重新标定。bridge 会把动作宽度
  clamp 到 `gripper_max_m`，首轮会损失约 1 cm 开度，这是已知 sim2real 偏差。
- **关节方向/零点**：仿真 embodiment 来自同一套 NERO URDF，但仍应先用
  `+0.01 rad` 单关节小动作核对方向和反馈，不能假设符号完全一致。

## 8. 常见失败

- `Feature ... differs`：检查 server YAML 的 name/shape、client 的
  `--robot.*_camera_key`、`--robot.action_schema` 和图像尺寸是否同时为 224。
- `no ... gripper width feedback`：夹爪节点未启动或宽度话题不新鲜。
- `action ... exceeds max_joint_step_rad`：属正常安全限幅，先用 dry-run 看
  `(clamped from ...)`，确认策略没有输出跳变，再逐步放宽。
- `motion fault latched`：先停止 inference，等 WebApp 显示 `stopped`，再点
  **Clear bridge fault**。旧 bridge 不支持该命令时需要重启 bridge。不要在动作
  仍在执行或 stream gate 未关闭时清障。
- `reset FAILED ... bimanual stream action feedback timeout`：旧版 rollout 在
  reset 完成后处于 idle、不再发动作，bridge 把 0.75 秒的动作间隔误判为 client
  掉线并 latch fault。更新后的 rollout 会在 idle 期间持续发送 hold；若仍出现，
  先 Stop、Clear bridge fault，并确认控制机使用的是最新 `/home/work/lerobot`。

## 9. 四步执行

适用于 bridge、相机、joint16 契约已经验收通过后的日常启动。首次部署或修改
bridge 代码后，先按第 10 节做一次 `dry_run` 验收。

### 第 1 步：检查

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/joint16_check.sh
```

最后必须看到 `CHECK OK`。该脚本检查 CAN、插件、position-hold 补丁、ROS 工作区、
Realsense 运行库和离线 pipeline，不会启动真机。

### 第 2 步：启动本地

本地节点完全没有运行时，直接执行：
```bash
cd /home/work/nero_duo_arm
./deployment/scripts/joint16_start_local.sh
```

如果 drivers、cameras、grippers、MoveIt 已经在运行，**不要**重复执行
`joint16_start_local.sh`，也不要为了更新代码杀掉驱动。本次只有 bridge 新增了
`clear_fault`，所以只重启 bridge 即可：

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
pkill -f '[b]imanual_vla_bridge.launch.py' || true
pkill -f '[d]ual_nero_bimanual_vla_bridge' || true
sleep 2
./deployment/scripts/joint16_component.sh bridge
```

`joint16_start_local.sh` 会在 `tmux` 会话 `joint16` 中启动 drivers、cameras、
grippers、MoveIt 和 joint16 stream bridge。查看日志：

```bash
tmux attach -t joint16
```

窗口顺序：`0=drivers`、`1=cameras`、`2=grippers`、`3=moveit`、`4=bridge`。
按 `Ctrl-b d` 退出 tmux 但保持节点运行。

如果现场只剩部分节点在跑（例如 drivers/cameras 已退出，但 grippers、MoveIt、
旧 bridge 还在），最稳妥的是做一次完整本地重启。**执行前确认机械臂有支撑或
急停可用**，因为停掉正在运行的 drivers 会释放力矩：

```bash
pkill -f '[b]imanual_vla_bridge.launch.py' || true
pkill -f '[d]ual_nero_bimanual_vla_bridge' || true
pkill -f '[m]oveit_execution.launch.py' || true
pkill -f '[d]ual_grippers.launch.py' || true
pkill -f '[a]ll_cameras.launch.py' || true
pkill -f '[d]ual_nero_drivers.launch.py' || true
pkill -f '[l]eft_gripper_node' || true
pkill -f '[r]ight_gripper_node' || true
pkill -f '[d]ual_nero_trajectory_bridge' || true
pkill -f '[m]ove_group' || true
pkill -f '[a]gx_arm_ctrl_single_node' || true
sleep 3
./deployment/scripts/joint16_start_local.sh
```

`joint16_start_local.sh` 现在会自动清理只剩死窗口的旧 tmux 会话，并在返回前等待
bridge socket 和 MoveIt `/move_action` 同时就绪。若它报“已有本地节点在运行”，
按上面列出的 PID 清理孤儿节点后再执行，不要直接再开一套。

然后以低速把两臂摆到训练 home：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/joint16_move_home.sh
```

脚本会显示当前角度和目标，要求输入 `MOVE` 后才用 MoveIt 以
`max_velocity_scaling_factor=0.05` 执行。最终应打印
`MAX_HOME_ERROR_RAD <= 0.02`。

如果本地节点已经启动，不要重复执行 `joint16_start_local.sh`，直接执行
`joint16_move_home.sh`。

### 第 3 步：启动服务器

从控制机执行一条命令，同时建立 SSH 隧道并启动 GPU policy server：

```bash
ssh -t -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \
  -L 7447:127.0.0.1:7447 zhiyuan@10.10.6.11 \
  'cd /home/zhiyuan/nero_duo_arm && ./deployment/scripts/joint16_start_server.sh'
```

保持该终端运行，看到 `Policy server ready: deployment=nero-bimanual-joint16`
后再继续。另开终端可确认：

```bash
ss -ltn | grep '127.0.0.1:7447'
```

如果服务器和隧道已经运行，跳过本步。

### 第 4 步：启动 WebApp 并运行推理

第 2 步的 tmux 已经启动了 drivers、cameras、grippers、MoveIt 和 stream bridge，
所以启动 WebApp 时**不要**再加 `start_moveit:=true`，否则会重复启动 MoveIt。

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch dual_nero_webapp webapp.launch.py
```

浏览器打开 `http://<控制机IP>:8080`，使用左侧 **Inference** 面板：

1. **Launch**：在 PTY 中启动 `joint16_run_rollout.sh`，连接 server、加载策略，
   进入交互会话。该按钮**不会**发送 `/start`，机器人不动。
2. 等状态变成 `idle`，日志出现 `Interactive rollout session` 和
   `Captured initial robot position (16 keys)`，并且随后出现
   `Using configured robot reset position (16 keys)`。后者表示 `/reset` 的目标
   已固定为 `deployment/config/joint16_home.yaml` 的训练 home，而不是 Launch
   时的当前位姿。
3. 在 **Task** 输入框修改指令并点 **Set**，等价于 `/subtask <text>`；从下一次
   策略推理开始生效。Launch 时也会使用输入框里的任务。
4. 确认工作区无人、急停可用、bridge 无 fault、两臂在训练 home、夹爪宽度约
   `0.09 m` 后，点 **Run**（红色按钮）发送 `/start`。
5. **Training Home**：inference 未运行时，用 MoveIt 同步把两臂送到训练 home；
   适合 Launch 前检查或 Stop 后手动回位。LIVE 模式下会先逐侧执行受保护的
   关节限位恢复，把失能坠落造成的超限关节收回部署范围，再规划 home；DRY-RUN
   不会执行这段真实恢复运动。该按钮不改变 rollout 状态。
6. **Reset**：发送 `/reset`，暂停推理，并把两臂和夹爪按 16D 原生动作送回训练
   home。它不需要切换到 MoveIt，也不会结束 rollout 会话。
7. **Stop**：发送 `/stop`，关闭 motion gate 并结束 rollout 进程。
8. **Clear bridge fault**：inference 已停止时清除 bridge 的 stop fault。

Launch 前 WebApp 会 preflight：rollout 脚本存在且可执行、bridge socket 存在、
`127.0.0.1:7447` 正在监听、bridge schema 为 `joint16`、bridge 无 latched
motion fault。任一项不满足会直接拒绝，并在面板显示原因。

推理进程存活期间，WebApp 会禁用 Send/Park/gate/motors/gripper 等手动运动按钮，
避免两套控制同时写关节；**E-stop** 和 **Stop both** 始终可用，并会先请求停止
inference。

当前 checkpoint 的训练目标是 apple。修改 Task 虽然可用，但 bottle 等指令属于
分布外输入，成功率不能按 apple 的结果估计。

异常时立即点 **Stop**，并同时使用驱动侧急停。命令行直接运行
`./deployment/scripts/joint16_run_rollout.sh` 仍可作为排障备用路径，见第 10 节。

## 10. 详细命令参考（排障用）

本节是一份从零开始、按终端拆分的完整命令清单。每个 `终端` 保持运行，不要
把所有 `ros2 launch` 放进同一个终端。命令中的 SSH 用户名和 GPU 地址按现场
实际值替换。

### 终端 0：控制机准备、插件和 bridge

```bash
cd /home/work/nero_duo_arm/deployment
sudo ./scripts/enable_can.sh can_left 1000000
sudo ./scripts/enable_can.sh can_right 1000000
ip -details link show can_left
ip -details link show can_right

cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash

env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python -m pip install -e \
    /home/work/nero_duo_arm/lerobot_robot_nero --no-deps \
    --config-settings editable_mode=compat

python3 deployment/scripts/patch_lerobot_position_hold_compat.py \
  --lerobot-root /home/work/lerobot
python3 deployment/scripts/patch_lerobot_position_hold_compat.py \
  --lerobot-root /home/work/lerobot --check

cd /home/work/nero_duo_arm/ros2_ws
colcon build --symlink-install --packages-select dual_nero_vla_bridge
source install/setup.bash

cd /home/work/nero_duo_arm
./deployment/scripts/test_bimanual_vla_pipeline.sh
```

最后一行应输出 `OK`。这一步不驱动真机。

### 终端 1：GPU 主机启动 policy server

先登录 GPU 主机：

```bash
ssh zhiyuan@10.10.6.11
```

在 GPU 主机执行：

```bash
cd /home/zhiyuan/nero_duo_arm

python3 deployment/scripts/patch_lerobot_pi05_checkpoint_compat.py \
  --lerobot-root /home/zhiyuan/lerobot
python3 deployment/scripts/patch_lerobot_pi05_checkpoint_compat.py \
  --lerobot-root /home/zhiyuan/lerobot --check

pkill -f lerobot_policy_server || true
ss -ltnp | grep ':7447' || true

CUDA_VISIBLE_DEVICES=1 \
/home/zhiyuan/miniforge3/envs/lerobot/bin/python \
  -m lerobot.scripts.lerobot_policy_server \
  --config_path=deployment/config/server_nero_joint16.yaml
```

保持该终端运行，直到看到：

```text
Policy server ready: deployment=nero-bimanual-joint16
```

### 终端 2：控制机建立 SSH 隧道

```bash
ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \
  -L 7447:127.0.0.1:7447 zhiyuan@10.10.6.11
```

保持该终端运行。另开终端检查：

```bash
ss -ltnp | grep ':7447'
```

rollout 的 endpoint 必须使用 `tcp/127.0.0.1:7447`。

### 终端 3：控制机启动双臂驱动

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=true \
  tcp_offset:='[0.2015, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'
```

确认双臂状态为 `NORMAL`，无关节故障后再继续。

### 终端 4：控制机启动三路相机

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

export LD_LIBRARY_PATH=/home/work/.local/opt/realsense-2.59/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
ros2 launch dual_nero_cameras all_cameras.launch.py
```

另开终端检查频率：

```bash
source /opt/ros/humble/setup.bash
source /home/work/nero_duo_arm/ros2_ws/install/setup.bash
ros2 topic hz /camera_scene/color/image_raw
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic hz /camera_right_wrist/color/image_raw
```

### 终端 5：控制机启动夹爪只读反馈

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch dual_nero_control dual_grippers.launch.py \
  allow_motion:=false enable_motor:=false
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/work/nero_duo_arm/ros2_ws/install/setup.bash
ros2 topic hz /left_gripper/width
ros2 topic hz /right_gripper/width
ros2 topic echo /left_gripper/status --once
ros2 topic echo /right_gripper/status --once
```

两侧宽度都应持续发布，反馈必须新鲜。当前实物全开约为 `0.09 m`。

### 终端 6：控制机启动 MoveIt 并摆 home

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false rviz:=false
```

左右臂都摆到：

```text
[-0.07127925, -1.36069359, 0.02806489, 1.88160456,
  0.04045673, 0.00094248, 1.57072651]
```

必须用 MoveIt，`max_velocity_scaling_factor<=0.05`。右臂和左臂都要准确到位；
joint16 的动作顺序是右臂 7 关节在前、左臂 7 关节在后。

检查反馈：

```bash
source /opt/ros/humble/setup.bash
source /home/work/nero_duo_arm/ros2_ws/install/setup.bash
ros2 topic echo /left/feedback/joint_states --field position --once
ros2 topic echo /right/feedback/joint_states --field position --once
```

### 终端 7：控制机启动 joint16 bridge

先 dry-run：

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

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

确认观测 state 是 `[16]`、三路图像都是 `[224,224,3]` 后，在该终端按 `Ctrl-C`，
再启动低速 stream：

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

保持 stream 终端运行。

### 终端 8：控制机启动 WebApp 并运行推理

上面的 drivers、cameras、grippers、MoveIt 和 joint16 stream bridge 已由
`joint16_start_local.sh` 在 tmux 中启动。WebApp 只启动面板进程：

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch dual_nero_webapp webapp.launch.py
```

浏览器打开 `http://<控制机IP>:8080`：

- **Inference → Launch**：启动 rollout 并加载策略，不发送 `/start`；
- 等状态为 `idle`，并确认 `Captured initial robot position (16 keys)` 与
  `Using configured robot reset position (16 keys)`；
- 如果夹爪 `estop_latched`、fault 或状态反馈不新鲜，**Launch/Run 会被拒绝**；
  先按“每次推理前的夹爪硬检查”恢复，而不是继续让机械臂空跑；
- 需要改指令时在 **Task** 输入框点 **Set**；
- 现场确认后点 **Run** 发送 `/start`；
- **Reset** 发送 `/reset`，**Stop** 发送 `/stop`；
- inference 停止后可用 **Clear bridge fault** 清 latched stop fault。

当前运行的 bridge 如果是加入 `clear_fault` 之前的旧进程，需要先重启 bridge，
否则 WebApp 的 **Clear bridge fault** 会报 unknown command。

### 可选：命令行直接启动 rollout client（排障用）

```bash
cd /home/work/nero_duo_arm

env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.scripts.lerobot_rollout \
  --inference.type=remote \
  --inference.endpoint=tcp/127.0.0.1:7447 \
  --inference.deployment=nero-bimanual-joint16 \
  --inference.mode=chunk \
  --inference.chunk_merge=aligned \
  --inference.semantics=nero-bimanual-joint16-rad-metres-v1 \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
  --robot.action_schema=joint16 \
  --robot.policy_robot_type=nero_dual_joint16 \
  --robot.home_position_file=/home/work/nero_duo_arm/deployment/config/joint16_home.yaml \
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

看到交互提示后先不要输入 `/start`。确认工作区无人、急停可用、bridge 无 fault、
夹爪宽度反馈新鲜、两臂仍在训练 home，再输入：

```text
/start
```

出现异常立即输入：

```text
/stop
```

并同时使用驱动侧急停。joint16 启动日志应同时出现
`Captured initial robot position (16 keys)` 和
`Using configured robot reset position (16 keys)`；`/reset` 会按后者的 home YAML
回到训练位姿。若缺少第二条日志，检查插件是否已重装、`joint16_run_rollout.sh`
是否传入 `--robot.home_position_file`，此时不要依赖 `/reset`，改用 WebApp 的
**Training Home** 或 MoveIt 低速回位。
