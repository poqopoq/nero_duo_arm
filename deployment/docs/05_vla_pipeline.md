# VLA 链路

> LeRobot 0.6.2 使用 `lerobot.remote_inference.*` 和 Zenoh transport；旧
> `lerobot.async_inference.*` 已移除。默认 `execution_mode:=dry_run`，真实运动
> 需要多道门控。

当前目标是打通 LeRobot `pi05`（π0.5）到 NERO 实机的整条链路。官方
`lerobot/pi05_base` 只是训练基座，不能直接作为 NERO policy；实机必须使用在
20-D NERO 数据上微调、且 checkpoint 特征契约与本仓库一致的 pi0.5。

## 0. 当前服务器核对（2026-11-12）

当前 GPU 主机与文档命令对应的实际环境如下；NERO 控制电脑上的旧
`/home/work/...` 路径不属于 GPU 主机，不能混用。

```text
GPU 主机:  hello-ThinkStation-PX
仓库:      /home/zhiyuan/nero_duo_arm
LeRobot:   0.6.2，/home/zhiyuan/miniforge3/envs/lerobot
GPU:       2 x RTX 4090 24 GB
当前 server: /home/zhiyuan/vla_server/server_nero_xense_smoke.yaml，
             deployment=nero-bimanual-xense-smoke，监听 0.0.0.0:7447
```

- 当前运行的 server 是旧的 SmolVLA plates smoke，不是 pi0.5。启动 pi0.5 前必须先
  停掉它，否则 `7447` 端口会被占用。
- `/home/zhiyuan/models/pi05_base` 存在，但它是 32-D state/action、三路
  `224x224` 基座相机键 `base_0_rgb/left_wrist_0_rgb/right_wrist_0_rgb` 的训练基座，
  不能直接对接本仓库的 20-D rot6d NERO 契约。
- 当前没有可用于 NERO 的 pi0.5 微调 checkpoint；`server_nero_pi05.yaml` 指向的
  `/home/zhiyuan/checkpoints/pi05_nero20/checkpoints/last/pretrained_model`
  尚未生成。

## 1. 架构

```text
GPU 服务器                                  本地 NERO 电脑
lerobot-policy-server / pi0.5 / SmolVLA  <--Zenoh-->  lerobot-rollout remote client
                                                        |
                                                        | Unix socket
                                                        v
                                      dual_nero_vla_bridge (Python 3.10 / ROS 2)
                                                        |
                              +-------------------------+-------------------------+
                              |                                                   |
                  stream: /<side>/control/joint_states          trajectory: FollowJointTrajectory
                              |                                                   |
                              +-------------------------+-------------------------+
                                                        |
                                              /left /right drivers
```

LeRobot 进程不导入 `rclpy`，ROS bridge 不导入 LeRobot；两边只共享长度前缀
JSON + JPEG 的本地 Unix socket 协议。

## 2. 数据契约

### 2.1 单臂 7-D

```text
observation.state = [ee.x.pos, ee.y.pos, ee.z.pos,
                     ee.rx.pos, ee.ry.pos, ee.rz.pos, ee.gripper.pos]
action            = [ee.x.pos, ee.y.pos, ee.z.pos,
                     ee.rx.pos, ee.ry.pos, ee.rz.pos, ee.gripper.pos]
```

- 位置米，旋转 axis-angle 弧度，夹爪 `0=闭合`、`1=张开`。
- `action[t] = state[t+1]`。
- 所有 action feature 以 `.pos` 结尾，满足 LeRobot 0.6.2 remote position-hold 契约。
- 图像默认键 `observation.images.camera1`。

### 2.2 双臂 20-D rot6d（默认）

```text
left_tcp.x, left_tcp.y, left_tcp.z,
left_tcp.r1 ... left_tcp.r6, left_gripper,
right_tcp.x, right_tcp.y, right_tcp.z,
right_tcp.r1 ... right_tcp.r6, right_gripper
```

`r1..r6` 是旋转矩阵前两列的列主序展开 `[m00,m10,m20,m01,m11,m21]`。
相机键默认 `side`、`left_wrist`、`right_wrist`。

### 2.3 双臂 14-D axis-angle（可选）

```text
left:  [x, y, z, rx, ry, rz, gripper]
right: [x, y, z, rx, ry, rz, gripper]
```

只有 checkpoint 明确使用该契约时，才把 bridge 和插件都设为
`action_schema=axis_angle14`。不要混用 20-D 模型和 14-D 插件。

## 3. 构建与安装

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_lerobot_nero_plugin.sh

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dual_nero_vla_bridge
source install/setup.bash
ros2 pkg executables dual_nero_vla_bridge
```

应看到 `dual_nero_vla_bridge vla_bridge`。

## 4. 启动前置

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=true \
  tcp_offset:='[0.2443, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'

ros2 launch dual_nero_cameras all_cameras.launch.py

# 先保持轨迹桥 dry-run
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true rviz:=false
```

### 4.1 TCP 契约

驱动默认 `tcp_offset` 全零时，`/feedback/tcp_pose` 发布的是 `link7`（法兰），
不是夹爪 TCP。双臂 launch 文件现在默认传入 CTAG2F90D 夹爪捏合点：

```text
link7 -> tcp_link = [0.2443, 0.0, -0.0235, -1.5708, 0.0, -1.5708]
```

该值与 `two_nero_tabletop.urdf.xacro` 中 `link7 -> tool0 -> tcp_link` 的合成
变换一致。修改 URDF 的 `*_tcp_z` 或更换夹爪时，必须同步修改 launch 默认值。
自检脚本会比较驱动反馈与 MoveIt `compute_fk` 的结果：

```bash
/usr/bin/python3 deployment/scripts/check_tcp_contract.py
```

预期两侧 `tcp_pose` 与 `*_tcp_link` 的位置差小于 `1 mm`、姿态差小于
`0.01 rad`。若驱动仍等于 `link7`，说明 `tcp_offset` 没有生效，禁止继续做 IK。

bridge 默认把左侧 TCP 数值解释为 `left_base_link` 下，右侧解释为
`right_base_link` 下。若话题 `frame_id` 非空且不匹配，bridge 拒绝该侧反馈。

### 4.2 上电 J4/J7 归位

上电后 `joint4` 可能仍在机械限位附近（约 `2.19--2.20 rad`）；安装夹爪后左臂
`joint7` 也可能越过 `1.5707963` 上限。两者都会让 MoveIt 拒绝目标。先只读检查，
再执行一次带保护的局部归位：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/recover_joint_limits.sh --dry-run
./deployment/scripts/recover_joint_limits.sh
```

脚本默认把 `joint4`/`joint7` 收到 `2.115`/`1.545`，并把其它轻微超限关节
收回到边界内；修正量超过上限时拒绝。它先关闭控制门，调用
`clear_joint_error` 和 `reset_motion`，再使能并核对 7 个电机状态。运动速度
高于 `10%` 时脚本会临时降到 `5%`，归位后恢复原速度；旧驱动不支持运行时
调速时则拒绝移动。使能和开门后它会重新读取反馈再规划，不能用上电前的位置
覆盖刹车释放后的实际姿态。结束后关闭 `/control_enable`，但
**保持电机使能**，避免重力再次把关节拉回物理停止位。失能停止位可以超限，
但不能写进 `support_pose.yaml`。归位过程不访问夹爪。

驱动 launch 默认在上电后自动执行（延迟 `8 s` 等待反馈）：

```bash
ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=true \
  recover_joint_limits:=true \
  recover_joint_limits_delay_sec:=8.0
```

自动归位前必须清空工作区并有人监护；它仍是真实运动，不是上电自检。

## 5. 单臂链路

当前夹爪未连接阶段，VLA bridge 保持 `gripper_enabled:=false`（默认）。动作中的
gripper 通道会被强制为 `0.0`，bridge 不会向夹爪命令 topic 发布消息；观测中的
gripper 也保持 `0.0`，直到收到有效宽度反馈。

### 5.1 dry-run

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=dry_run \
  allow_real_motion:=false \
  side:=left \
  image_key:=camera1 \
  ik_frame_id:=left_base_link
```

检查 socket 和节点：

```bash
ls -l /tmp/nero_vla_bridge.sock
ros2 node info /dual_nero_vla_bridge
```

dry-run 只做动作过滤和打印，不调用 IK、不驱动机械臂。默认限幅：
平移 `0.01 m`、旋转 `0.05 rad`、gripper `0..1`。

### 5.2 IK 干跑

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=ik_dry_run \
  allow_real_motion:=false \
  allow_gripper_motion:=false \
  side:=left \
  ik_frame_id:=left_base_link \
  image_key:=camera1
```

用插件发送一个 `+5 mm` 测试动作：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python - <<'PY'
from lerobot_robot_nero import Nero, NeroConfig

with Nero(NeroConfig(
    bridge_socket='/tmp/nero_vla_bridge.sock',
    image_width=640,
    image_height=480,
)) as robot:
    obs = robot.get_observation()
    action = {name: obs[name] for name in robot.action_features}
    action['ee.x.pos'] += 0.005
    print('applied:', robot.send_action(action))
PY
```

预期 bridge 输出 `IK target ...`，不出现 `Sending trajectory`。

### 5.3 轨迹桥干跑

保持 `moveit_execution.launch.py dry_run:=true`，重启 bridge：

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=trajectory \
  allow_real_motion:=true \
  allow_gripper_motion:=false \
  side:=left \
  ik_frame_id:=left_base_link \
  trajectory_duration_sec:=0.2
```

重复 `+5 mm` 动作。预期 bridge 输出 `Sending trajectory #...`，轨迹桥只发布
仿真 joint state，不驱动电机。

### 5.4 连续流预览与低速验收

`stream` 模式跳过 `FollowJointTrajectory` 的逐段 settle，直接以默认 50 Hz
向 `/<side>/control/joint_states` 发布最新关节目标。没有新 action 时持续重发
最后一个成功 IK 目标，超过 `stream_feedback_timeout_sec` 或
`watchdog_timeout_sec` 后锁存 fault、关闭控制门并急停。`stream` 不经过轨迹桥，
因此 `moveit_execution.launch.py dry_run:=true` **不会**阻止电机运动。

先只验证命令频率，把输出改到预览 topic：

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=stream \
  allow_real_motion:=true \
  allow_gripper_motion:=false \
  side:=left \
  ik_frame_id:=left_base_link \
  control_command_topic:=/left/stream_preview \
  stream_command_rate_hz:=50.0 \
  stream_feedback_timeout_sec:=0.75
```

预期 bridge 打开 `/left/control_enable`，并持续输出 `streaming`；不会出现
`Sending trajectory`。确认 `ros2 topic hz /left/stream_preview` 接近设定频率且
位置保持不变。确认无误后去掉 `control_command_topic`，在支撑位、清空工作区、
人员监护下用 `max_translation_step_m:=0.002` 和低速驱动做首次真实运动。

### 5.5 真机低速

前提：工作区清空、人员监护、软件急停可用、左臂在支撑位附近、所有 Web app
和旧轨迹桥已关闭。`stream` 直接驱动，不需要把轨迹桥切到真机；保留轨迹桥
`dry_run:=true` 也不会拦截 stream 命令。

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=stream \
  allow_real_motion:=true \
  allow_gripper_motion:=false \
  side:=left \
  ik_frame_id:=left_base_link \
  max_translation_step_m:=0.002 \
  max_rotation_step_rad:=0.01 \
  max_joint_step_rad:=0.05 \
  stream_command_rate_hz:=50.0 \
  stream_feedback_timeout_sec:=0.75
```

先只发单个 `+2 mm`，确认方向、幅度和停止行为。接 SmolVLA 时首轮使用
`--fps=5`，通过 `--inference.refill_seconds` 拉长请求间隔，不要直接 30 Hz。
需要逐 action 短轨迹时再切回 `execution_mode:=trajectory`，并把轨迹桥设为
`moveit_execution.launch.py dry_run:=false`。

## 6. 双臂链路

夹爪尚未连接时**不要启动夹爪驱动**，WebApp 也保持 `start_grippers:=false`
（当前默认）。双臂 bridge 继续使用 `gripper_enabled:=false`，只处理机械臂的
14 个关节；20-D 动作布局不变，夹爪两个通道被强制为 `0.0`，不会下发运动命令。

只有夹爪已接入 CAN、且完成空载验收后才启动只读反馈：

```bash
ros2 launch dual_nero_control dual_grippers.launch.py \
  allow_motion:=false enable_motor:=false

ros2 topic hz /left_gripper/width
ros2 topic hz /right_gripper/width
```

### 6.1 dry-run

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=rot6d20 \
  execution_mode:=dry_run \
  allow_real_motion:=false \
  gripper_enabled:=false \
  allow_gripper_motion:=false \
  use_compressed_color:=true \
  camera_scene_key:=side \
  camera_left_wrist_key:=left_wrist \
  camera_right_wrist_key:=right_wrist
```

读取三路图像和 20-D state，再原样回发：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python - <<'PY'
from lerobot_robot_nero import NeroBimanual, NeroBimanualConfig

with NeroBimanual(NeroBimanualConfig()) as robot:
    obs = robot.get_observation()
    print({k: v for k, v in obs.items() if not hasattr(v, 'shape')})
    print('images:', {k: v.shape for k, v in obs.items() if hasattr(v, 'shape')})
    print('applied:', robot.send_action({k: obs[k] for k in robot.action_features}))
PY
```

### 6.2 IK 干跑

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=rot6d20 \
  execution_mode:=ik_dry_run \
  allow_real_motion:=false \
  gripper_enabled:=false \
  allow_gripper_motion:=false
```

发送双臂 `+2 mm` 测试动作：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python - <<'PY'
from lerobot_robot_nero import NeroBimanual, NeroBimanualConfig

with NeroBimanual(NeroBimanualConfig()) as robot:
    obs = robot.get_observation()
    action = {name: obs[name] for name in robot.action_features}
    action['left_tcp.x.pos'] += 0.002
    action['right_tcp.x.pos'] += 0.002
    print('applied:', robot.send_action(action))
PY
```

任一臂 IK、坐标系或关节限位失败时整组拒绝，不允许只动一侧。

### 6.3 轨迹桥干跑

保持 `moveit_execution.launch.py dry_run:=true`，重启 bridge：

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=rot6d20 \
  execution_mode:=trajectory \
  allow_real_motion:=true \
  gripper_enabled:=false \
  allow_gripper_motion:=false \
  max_translation_step_m:=0.005 \
  max_rotation_step_rad:=0.02 \
  max_joint_step_rad:=0.08 \
  trajectory_duration_sec:=0.25
```

预期只生成一条 14 关节 goal，轨迹桥不驱动电机。任一臂失败时锁存 fault 并
请求两侧急停/关闭控制门。

### 6.4 连续流预览与低速验收

双臂 `stream` 使用同一个 14 维 IK 结果，在同一 tick 连续发布
`/left/control/joint_states` 和 `/right/control/joint_states`；任一侧 IK、控制门、
发布或 watchdog 失败都会锁存 fault 并停双侧。

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=rot6d20 \
  execution_mode:=stream \
  allow_real_motion:=true \
  gripper_enabled:=false \
  allow_gripper_motion:=false \
  left_control_command_topic:=/left/stream_preview \
  right_control_command_topic:=/right/stream_preview \
  stream_command_rate_hz:=50.0 \
  stream_feedback_timeout_sec:=0.75
```

先确认两侧预览 topic 各 7 个关节命令同时出现、数值一致成组，停止后两侧不再
发布且控制门关闭。然后去掉预览 topic 覆盖，在双臂都有支撑且可安全失能时做低速
真机验收。双臂 `stream` 同样不受轨迹桥 `dry_run` 保护。

### 6.5 真机低速

前提同单臂，且双臂均有支撑或可安全失能。stream 直接驱动，不受轨迹桥
`dry_run` 保护。

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=rot6d20 \
  execution_mode:=stream \
  allow_real_motion:=true \
  gripper_enabled:=false \
  allow_gripper_motion:=false \
  max_translation_step_m:=0.002 \
  max_rotation_step_rad:=0.01 \
  max_joint_step_rad:=0.05 \
  stream_command_rate_hz:=50.0 \
  stream_feedback_timeout_sec:=0.75
```

先手工发一次双臂 `+2 mm`，确认两侧同时开始/结束、方向正确，停止后两侧控制门
关闭且软件急停生效。夹爪验收先只开一侧：

```bash
ros2 launch dual_nero_control dual_grippers.launch.py \
  allow_motion:=true enable_motor:=false
ros2 service call /left_gripper/enable std_srvs/srv/SetBool "{data: true}"
```

## 7. Policy server / client

仓库内已提供三个 schema 模板。GPU 主机上的仓库路径为
`/home/zhiyuan/nero_duo_arm`，因此不要先复制到别处再引用相对路径：

- `deployment/config/server_nero.yaml`：单臂 7-D axis-angle。
- `deployment/config/server_nero_bimanual.yaml`：双臂 20-D rot6d。
- `deployment/config/server_nero_pi05.yaml`：双臂 20-D rot6d，只接受在 NERO
  20-D 数据上微调后的 pi0.5 checkpoint。

`action_fps` 必须等于 checkpoint 的训练动作频率，并与 client 的 `--fps` 一致。
不能为了“首轮慢一点”随意把 30 Hz 的 checkpoint 写成 5 Hz；那会改变动作时间轴。
首轮真机限速应使用 bridge 的 `max_translation_step_m`、`max_rotation_step_rad`
和 `max_joint_step_rad`。只有确实按 5 Hz 训练过的 checkpoint 才能使用 5.0。

启动前先在 GPU 服务器同时校验 YAML 和 checkpoint 的 canonical feature 契约：

```bash
cd /home/zhiyuan/nero_duo_arm/deployment/config

GPU_PY=/home/zhiyuan/miniforge3/envs/lerobot/bin/python
export GPU_CFG=/home/zhiyuan/nero_duo_arm/deployment/config/server_nero_pi05.yaml

"$GPU_PY" - <<'PY'
import json
import os
from pathlib import Path

import draccus
from lerobot.remote_inference import ServerConfig

cfg = draccus.parse(config_class=ServerConfig, config_path=os.environ['GPU_CFG'])
root = Path(cfg.model.repo_or_path)
if not (root / 'config.json').is_file():
    raise SystemExit(f'checkpoint not found or incomplete: {root}')

policy = json.loads((root / 'config.json').read_text())
canonical = {feature.name: tuple(feature.shape) for feature in cfg.features}

actual = {}
for name, feature in policy['input_features'].items():
    actual[name] = (
        (feature['shape'][1], feature['shape'][2], feature['shape'][0])
        if feature['type'] == 'VISUAL'
        else tuple(feature['shape'])
    )

if canonical != actual:
    raise SystemExit(f'feature mismatch: yaml={canonical} checkpoint={actual}')
output = tuple(policy['output_features']['action']['shape'])
if tuple(cfg.action_feature.shape) != output:
    raise SystemExit(f'action mismatch: yaml={cfg.action_feature.shape} checkpoint={output}')
names = policy.get('action_feature_names')
if names and tuple(names) != tuple(cfg.action_feature.names):
    raise SystemExit('action names differ from checkpoint action_feature_names')

print('OK', cfg.deployment, cfg.robot_type, cfg.model.repo_or_path,
      'fps=', cfg.execution.action_fps, 'action=', output)
PY
```

当前模板的 checkpoint 尚未生成，所以会先报 `checkpoint not found`；如果把
`model.repo_or_path` 指向 `/home/zhiyuan/models/pi05_base`，则会报
`feature mismatch`。只有真正的 NERO pi0.5 微调 checkpoint 才能返回 `OK`。

若 `7447` 已被旧 server 占用，先确认 PID 后停掉旧进程，再启动新 server：

```bash
kill "$(cat /home/zhiyuan/vla_server/server.pid)"

# 等端口释放；应无输出
ss -ltnp | rg ':7447' || true

/home/zhiyuan/miniforge3/envs/lerobot/bin/python \
  -m lerobot.scripts.lerobot_policy_server \
  --config_path=/home/zhiyuan/nero_duo_arm/deployment/config/server_nero_pi05.yaml
```

`server_nero_pi05.yaml` 已包含 `zenoh.listen_endpoints: [tcp/0.0.0.0:7447]`；
只有需要临时换端口时才追加 `--zenoh.listen_endpoints='[tcp/0.0.0.0:7448]'`，
client 也必须同步改 endpoint。旧的直接命令 `lerobot-policy-server` 只有在
`/home/zhiyuan/miniforge3/envs/lerobot/bin` 已加入 `PATH` 时才等价。

`model.repo_or_path`、`execution.action_fps`、`robot_type`、`features`、
`action_feature` 必须和本地 checkpoint/插件一致。单臂 action 名使用
`ee.x.pos ... ee.gripper.pos`；双臂 rot6d 使用 `left_tcp.*/right_tcp.*` 的 20 维契约。
首轮只开放 `chunk`，验证后再加 `rtc_guided`；`rtc_trained` 需要训练时支持
action-prefix conditioning 的 checkpoint。

使用 `server_nero_pi05.yaml` 时，下面双臂 client 必须同时改两处：
`--inference.deployment=nero-bimanual-pi05`，且 `--fps` 等于该 server 的
`execution.action_fps`（当前模板为 `30`）。`robot_type` 与
`--robot.policy_robot_type` 都保持 `xtac_umi_g1`。

先确认本地到 server 的 Zenoh 端口可达：

```bash
nc -vz <SERVER_IP> 7447
```

如果 SSH 可达但 `7447` 连接超时，说明该端口被网络策略阻断，不要误判为
server 未启动。可建立本地端口转发，并把 client endpoint 改为 `127.0.0.1`：

```bash
ssh -N -L 7447:127.0.0.1:7447 <USER>@<SERVER_IP>

# 另一个终端启动 rollout
--inference.endpoint=tcp/127.0.0.1:7447
```

当前双臂 SmolVLA smoke 联调基线（不是 pi0.5）使用
`/home/zhiyuan/checkpoints/xense_plates_nero20_smoke_features/checkpoints/000020/pretrained_model`：
20-D rot6d、三路 `480x640` 图像、`chunk` 50 步、30 Hz。server 的
`execution.action_fps` 和 client 的 `--fps` 必须同时设为 `30`。在该基线上，
10 秒 dry-run 完成 300 个控制 tick、300 次 action 下发、0 starvation；推理
往返约 190--210 ms。

本地单臂 client（0.6.2 remote inference）：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.scripts.lerobot_rollout \
  --inference.type=remote \
  --inference.endpoint=tcp/<SERVER_IP>:7447 \
  --inference.deployment=nero-single \
  --inference.mode=chunk \
  --inference.chunk_merge=aligned \
  --inference.semantics=nero-ee-metres-axis-angle-gripper-normalized-v1 \
  --robot.type=nero_ros \
  --robot.bridge_socket=/tmp/nero_vla_bridge.sock \
  --robot.side=left \
  --robot.camera_key=camera1 \
  --fps=5 \
  --return_to_initial_position=false \
  --task="pick up the block" \
  --interactive=true
```

单臂 action 也包含 axis-angle；`return_to_initial_position=true` 会在退出时对
`ee.rx/ry/rz` 做线性插值，可能穿过无效姿态。因此首次和自动评测都必须显式设为
`false`，退出停止由插件 `disconnect()` 的 `stop` 请求负责。

本地双臂 client：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.scripts.lerobot_rollout \
  --inference.type=remote \
  --inference.endpoint=tcp/<SERVER_IP>:7447 \
  --inference.deployment=nero-bimanual \
  --inference.mode=chunk \
  --inference.chunk_merge=aligned \
  --inference.semantics=nero-bimanual-tcp-metres-rot6d-gripper-normalized-v1 \
  --fps=5 \
  --return_to_initial_position=false \
  --task="pick up the object" \
  --interactive=true \
  --rename_map='{"observation.images.side":"observation.images.camera1","observation.images.left_wrist":"observation.images.camera2","observation.images.right_wrist":"observation.images.camera3"}' \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
  --robot.action_schema=rot6d20 \
  --robot.policy_robot_type=xtac_umi_g1 \
  --robot.scene_camera_key=side \
  --robot.left_wrist_camera_key=left_wrist \
  --robot.right_wrist_camera_key=right_wrist
```

`rename_map` 把本地三路相机映射到 checkpoint 的 `camera1/2/3`。server YAML 中的
`features` 必须使用 checkpoint-facing 名称；不能把数据集侧的 `side`、
`left_wrist`、`right_wrist` 直接写进 server 配置。若 checkpoint 使用其它相机键，
server YAML 和 `--rename_map` 必须一起修改。

单臂和双臂 rollout 评测都必须加 `--return_to_initial_position=false`，因为
axis-angle/rot6d 不能线性插值回初始姿态；退出时由插件 `disconnect()` 请求停止。

### 7.1 语言指令接口

LeRobot 没有“服务器收到一句自然语言后直接控制机械臂”的独立 API。语言指令走
现有 remote-inference 观测链路：

```text
--task / 交互式 /subtask
        -> observation.task + task_version
        -> SmolVLA policy 条件化
        -> action chunk
        -> 本地 bridge -> IK -> NERO 关节命令
```

- `--task="..."` 是启动时的初始指令，随每个 observation 发给服务器。
- `--interactive=true` 后可在终端输入 `/subtask <新指令>`；新指令从下一次推理
  开始生效。它改变的是 policy 条件，不会绕过 policy 直接生成轨迹。
- server YAML 的 `language.enabled` 控制文本生成/子任务查询能力，不是动作推理
  的前置开关。当前 `server_nero*.yaml` 模板均为 `false`；固定任务不需要打开它。
  若要使用 `/autosteer` 让模型生成子任务，则需要服务器支持语言处理器并把
  `language.enabled` 设为 `true`，同时确认该 checkpoint 的 prompt recipe 支持。
- 当前 plates checkpoint 只见过一个任务，建议原样使用：

  `Take the plates from the rack, clean their surfaces using a sponge, and return them to their original positions on the rack.`

  换任意措辞或换任务不应期待泛化；`--task` 文本变化会改变 task version，并可能
  丢弃旧的 action continuation。当前服务器没有经过验收的“文本 -> 机械臂”直连
  接口，真实控制仍必须经过 bridge 的 IK、限位和门控。

### 7.2 推理时具体在哪里输入

只有两个输入位置：

1. **启动 `lerobot-rollout` 时**：`--task="<指令>"` 设置初始任务。
2. **交互终端运行中**：加 `--interactive=true` 后，在同一个终端输入
   `/subtask <新指令>`。它不是 ROS service，也不是服务器 `language/request`。

推荐先 dry-run 验证语言条件化，再切真实执行。双臂 bridge 用 6.1 的 dry-run 命令，
然后在另一个终端启动 rollout：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.scripts.lerobot_rollout \
  --inference.type=remote \
  --inference.endpoint=tcp/127.0.0.1:7447 \
  --inference.deployment=nero-bimanual-xense-smoke \
  --inference.mode=chunk \
  --inference.chunk_merge=aligned \
  --inference.semantics=nero-bimanual-tcp-metres-rot6d-gripper-normalized-v1 \
  --fps=30 \
  --return_to_initial_position=false \
  --task="Take the plates from the rack, clean their surfaces using a sponge, and return them to their original positions on the rack." \
  --interactive=true \
  --rename_map='{"observation.images.side":"observation.images.camera1","observation.images.left_wrist":"observation.images.camera2","observation.images.right_wrist":"observation.images.camera3"}' \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
  --robot.action_schema=rot6d20 \
  --robot.policy_robot_type=xtac_umi_g1 \
  --robot.scene_camera_key=side \
  --robot.left_wrist_camera_key=left_wrist \
  --robot.right_wrist_camera_key=right_wrist
```

程序连接后不会立即运动，rollout 终端会停在交互提示。依次输入：

```text
/help
/start
/subtask "Take the plates from the rack, clean their surfaces using a sponge, and return them to their original positions on the rack."
/stop
```

`/start` 才开始控制循环；`/subtask` 从下一次推理生效，旧的 action continuation
可能被丢弃；`/stop` 停止并关闭控制门。`--fps` 必须等于 server 的
`execution.action_fps`。当前 checkpoint 的动作坐标系尚未标定，以上 dry-run 只能验证
文本进入 policy 的链路，不应直接切到 `stream` 驱动真机。

## 8. 安全门控

真实运动必须同时满足：

1. VLA bridge `execution_mode=stream` 或 `trajectory` 且 `allow_real_motion=true`。
2. `stream` 不经过轨迹桥；`trajectory` 需要 `moveit_execution.launch.py dry_run=false`。
3. `/compute_ik`、驱动控制门可用；`trajectory` 还需要 `FollowJointTrajectory` 可用。
4. 两侧 TCP/关节反馈新鲜；任一侧过期即整组拒绝。
5. 两侧 IK、关节限位、单步检查全部通过；任一失败锁存 fault。
6. `gripper_enabled=false` 时动作夹爪通道强制为 `0.0`，且不向夹爪命令 topic
   发布；只有 `gripper_enabled=true` 且 `allow_gripper_motion=true` 才会发送。
7. 夹爪反馈缺失/过期时本次不发送；夹爪电机还必须先通过
   `/<side>_gripper/enable` 使能。

不要只启动一侧 bridge 或只打开一侧控制门来“先试右臂”；双臂链路设计为原子执行。
`stream` 打开控制门后会直接向驱动发布关节命令，轨迹桥 `dry_run` 对它没有保护；
首次真机只用 `+2 mm`、低步长和有支撑姿态验证。

## 9. 自动测试

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/test_vla_motion_pipeline.sh
./deployment/scripts/test_bimanual_vla_pipeline.sh
```

两脚本分别使用 `ROS_DOMAIN_ID=81/82` 和 `ROS_LOCALHOST_ONLY=1`，可并行执行；
避免共享 `/compute_ik` 与反馈话题造成串扰。

覆盖单臂/双臂 dry-run、IK、短轨迹、stream 持续保持与 latest-wins、stop、
watchdog 锁存、IK 失败、关节单步超限、14 关节单 goal、14-D axis-angle、
夹爪映射和 LeRobot 插件特征。

## 10. 限制与验收

- 左/右夹爪宽度和 gripper action 已接入，默认 `gripper_enabled=false`、
  `allow_gripper_motion=false`；夹爪未连接时保持这两个值不变。
- 夹爪空载 10×10 和低速软件急停已通过；带载抓取、堵转清障、长时热稳定和
  真实 SmolVLA 闭环仍未验收。
- `trajectory` 是逐 action 的短轨迹；`stream` 是 50 Hz 的 latest-wins 绝对关节
  目标流，没有内置插值。模型输出频率低于控制频率时，bridge 会保持最后一个目标。
- IK 只用当前/目标关节状态做碰撞检查，未接入在线点云 PlanningScene。
- 真实双臂 SmolVLA checkpoint 整链路仍未验收；NERO 20-D pi0.5 微调
  checkpoint 尚未生成。

验收标准：

1. `dry_run` 能读取图像和 state，动作维度正确。
2. `ik_dry_run` 返回每侧 7 个关节解，无超限。
3. 任一臂 IK 失败时轨迹桥收不到 goal，fault 锁存。
4. `trajectory` + dry-run 轨迹桥只生成一条 14 关节 goal；`stream` + dry-run
   持续发布两侧 7 关节命令，stop 后立即停止。
5. 真机 `+2 mm` 双侧同步，停止后两侧门关闭且软件急停生效。
6. SmolVLA smoke checkpoint 在 5 Hz remote inference 连续运行 20 秒无
   IK/反馈/控制 fault。
7. pi0.5 微调 checkpoint 先通过第 7 节的 feature 预检，再按训练频率连续运行
   20 秒无 contract/IK/反馈/控制 fault。

## 11. 真机联调记录（2026-11-09）

### 11.1 已修复的 TCP 契约

驱动在 `tcp_offset=0` 时 `/feedback/tcp_pose` 发布的是 `link7`（法兰），
而 bridge/MoveIt 用 `*_tcp_link` 做 IK，两者相差约 `203 mm`、姿态约 `120°`。
`dual_nero_drivers.launch.py` 现在默认：

```text
tcp_offset = [0.2443, 0.0, -0.0235, -1.5708, 0.0, -1.5708]
```

实测 `check_tcp_contract.py`：

```text
left : position_error=0.016 mm rotation_error=0.00003 rad OK
right: position_error=0.013 mm rotation_error=0.00002 rad OK
```

### 11.2 joint4/joint7 与使能保持

两侧 `joint4` 曾为 `2.1956/2.1981`，超出部署限幅 `2.14`，IK 会被拒绝。
用 5% 速度、整组 7 关节命令移到 `2.12` 后 IK 恢复。注意：旧支撑位下右臂失能
后会被重力带回 `joint4≈2.198`；增加夹爪后左臂失能还会把 `joint7` 带到 URDF
上限外。因此复位后必须**保持电机使能**，只关闭 `/control_enable` 门，并让
归位脚本同时回收 `joint4`/`joint7`。重新失能前必须重新执行归位。

### 11.3 SmolVLA smoke remote inference 实测

`lerobot-rollout` 以 30 Hz 连接 `nero-bimanual-xense-smoke` 成功，contract
校验通过，chunk 正常返回。首帧策略动作（原始值，未过滤）：

```text
left : x=+0.1781 y=+0.1907 z=-0.2719 gripper=0.592
right: x=+0.2843 y=-0.0197 z=-0.2737 gripper=0.614
```

当前 NERO 反馈约为 `z=+0.511`，且训练元数据明确要求把源 TCP 变换到
`left_base_link/right_base_link`，而 `xense_nero_conversion.json` 的
`pose_transform` 仍为 `null`。因此该 checkpoint 的动作坐标系尚未标定，
首帧动作相对当前姿态约有 0.4--0.6 m 偏差。

对首帧 2 mm 过滤目标做 `compute_ik`：位置单独可达、姿态单独可达，
两者叠加后返回 `-31 (NO_IK_SOLUTION)`。当前支撑位本身贴近 `joint2/joint4`
下限，策略方向再叠加旋转就超出可达域。这说明**不能靠放宽关节限幅解决**。

### 11.4 SmolVLA smoke 真实执行链验证

在 `stream` 模式、50 Hz、`max_translation_step_m=0.002`、
`max_rotation_step_rad=0.01`、`max_joint_step_rad=0.05`、夹爪禁动下，
手工发送双臂 `+2 mm z`：

```text
left : 0.51111 -> 0.51308 m (+1.97 mm)
right: 0.51092 -> 0.51291 m (+1.99 mm)
```

两侧同 tick、同 stamp、同时停止；停止后控制门关闭。真实执行链
`socket -> IK -> /control/joint_states -> driver -> motor` 已验证。

### 11.5 执行策略动作前仍缺

1. 训练数据到 NERO base 的 `pose_transform` 标定（当前为 `null`）。
2. 与训练工位一致的相机安装和场景；当前 `side` 相机看不到训练里的盘子架。
3. 夹爪只读反馈节点；否则 state 中 gripper 恒为 `0`。
4. 一个不贴关节限位的工作起始姿态，或把策略首帧限制在可达域内。

在这些完成前，只应执行手工小步动作；不要直接把该 checkpoint 的 chunk
接到 `/control/joint_states`。`stream` 每 tick 允许 2 mm，30 Hz 下等效
6 cm/s，未标定动作会持续朝错误方向累积。
