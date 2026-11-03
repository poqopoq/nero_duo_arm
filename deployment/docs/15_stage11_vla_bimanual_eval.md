# Stage 11: 双臂 VLA 真机评测链路

本文把已有的双臂桥接到 LeRobot 0.6 的 `lerobot-rollout`，默认支持 Xense
UMI 原始 HDF5/MKV 转换器产生的 20-D rot6d 数据，同时保留 14-D axis-angle
兼容模式。默认仍是全链路干跑，任何真实运动都需要显式打开多道门控。

> 2026-11-02：三路真实相机 + 双侧 30 Hz 假反馈 + `dry_run` 已完成本地端到端
> 验收。真机 IK/轨迹运动、真实 SmolVLA checkpoint 和夹爪命令仍未验收。

## 1. 数据契约

### 1.1 默认：20-D rot6d

顺序必须与 `hdf52lerobot` 的 `state_layout` 完全一致：

```text
left_tcp.x, left_tcp.y, left_tcp.z,
left_tcp.r1 ... left_tcp.r6,
left_gripper.pos,
right_tcp.x, right_tcp.y, right_tcp.z,
right_tcp.r1 ... right_tcp.r6,
right_gripper.pos
```

- `r1..r6` 是旋转矩阵前两列的列主序展开：
  `[m00, m10, m20, m01, m11, m21]`。
- 位置单位米，夹爪 `0=闭合`、`1=打开`。
- `action[t]` 是下一帧的绝对 TCP 目标。
- 相机键默认：`side`、`left_wrist`、`right_wrist`。

### 1.2 可选：14-D axis-angle

```text
left:  [x, y, z, rx, ry, rz, gripper]
right: [x, y, z, rx, ry, rz, gripper]
```

只有 checkpoint 明确使用该契约时，才把 bridge 和插件都设为
`action_schema=axis_angle14`。不要用 20-D 模型配 14-D 插件，即使只看动作
维度相同也不行。

### 1.3 LeRobot 内部特征名

`lerobot-rollout` 0.6 只把以 `.pos` 或 `.vel` 结尾的机器人特征送入策略。
因此插件的 20 个 action/state 维度使用如下内部命名：

```text
left_tcp.x.pos ... left_tcp.r6.pos, left_gripper.pos,
right_tcp.x.pos ... right_tcp.r6.pos, right_gripper.pos
```

名称可不同，但顺序、维度和单位必须与训练集一致。插件按上述顺序打包为
20-D 张量，不依赖 checkpoint 保存的字段名。

## 2. 链路结构

```text
lerobot-rollout / SmolVLA
        |
        | 20-D action
        v
dual_nero_bimanual_vla_bridge (ROS 2 / Python 3.10)
        |
        +-- /left/feedback/tcp_pose + joint_states
        +-- /right/feedback/tcp_pose + joint_states
        +-- scene + left wrist + right wrist cameras
        |
        +-- left_arm IK ----+
        +-- right_arm IK ----+-- 任一失败：整组拒绝并锁存 fault
                             |
                             v
        一个 14 关节 FollowJointTrajectory goal
                             |
                             v
        dual_nero_trajectory_bridge -> /left /right drivers
```

左右臂 IK 的 `robot_state` 都包含 14 个关节。右臂求解时使用左臂刚求出的
目标关节，减少两侧目标互相穿插；只有两侧 IK、限位和单步检查全部通过后，
才发送一条同步轨迹。

### 2.1 服务器输出与本地接收边界

推理放在 GPU 服务器时，服务器**不输出** ROS topic、关节角或 Unix socket
消息。它只通过 LeRobot 的 `AsyncInference/GetActions` 返回动作块；ROS 相关
转换全部在本地完成。

| 层 | 输出 | 消费者 |
| --- | --- | --- |
| GPU policy server | `list[TimedAction]`，每个 action 为 20-D 绝对动作 | 本地 `lerobot.async_inference.robot_client` |
| 本地 RobotClient | 按 `robot.action_features` 顺序展开的动作字典 | `NeroBimanual` 插件 |
| `NeroBimanual` | `send_action` JSON 帧，包含 20 个 float | 本地 ROS 2 bimanual bridge |
| ROS bimanual bridge | 左右两个 TCP 目标、两次 IK、一条 14 关节轨迹 | trajectory bridge -> drivers |

#### 服务器端输出格式

一次观测对应一个动作块，逻辑形状为：

```text
action_chunk: float32[T, D]
T = actions_per_chunk（服务器可返回的最大步数）
D = 20（rot6d20）或 14（axis_angle14）
```

`T` 个行向量都必须使用第 1 节的动作顺序和单位。默认 `D=20` 时每一行就是：

```text
[left_tcp.x, left_tcp.y, left_tcp.z,
 left_tcp.r1, left_tcp.r2, left_tcp.r3,
 left_tcp.r4, left_tcp.r5, left_tcp.r6,
 left_gripper,
 right_tcp.x, right_tcp.y, right_tcp.z,
 right_tcp.r1, right_tcp.r2, right_tcp.r3,
 right_tcp.r4, right_tcp.r5, right_tcp.r6,
 right_gripper]
```

- 每一行是下一控制时刻的**绝对 TCP 目标**，不是相对增量，也不是关节角。
- 位置单位为米，旋转为训练集定义的 rot6d，夹爪为 `0..1`。
- 归一化、反归一化和 actor 输出的拆分由 LeRobot policy postprocessor 完成；
  服务器返回的必须是已经恢复到物理单位的动作。
- 时间戳和 timestep 由 policy server 自动包装进 `TimedAction`，checkpoint
  不需要自己输出这两个字段。
- 客户端在启动时会把 `policy_type`、checkpoint 路径、`lerobot_features`、
  `actions_per_chunk` 和 device 发给服务器。服务器必须能访问该 checkpoint，
  且 checkpoint 的 action/state 维度、图像键和训练数据完全一致。

服务器没有启动时，本地 `RobotClient` 无法握手；服务器返回空动作块时，本地
队列会等待下一次观测，不会自行生成动作。

#### 本地接收与执行

1. `RobotClient.receive_actions()` 接收动作块，按 timestep 放入本地动作队列。
2. 控制循环按目标 FPS 取出一帧，把张量第 `i` 维映射到
   `NeroBimanual.action_features` 的第 `i` 个名字。
3. `NeroBimanual.send_action()` 检查 20 个值均为有限数，然后向本地
   `/tmp/nero_bimanual_vla_bridge.sock` 发送：

```json
{
  "cmd": "send_action",
  "action": [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.75,
             0.3, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.25]
}
```

4. ROS bridge 对左右动作分别做工作空间、单步平移/旋转和夹爪限幅；把 rot6d
   恢复成四元数；以 `left_base_link`、`right_base_link` 为坐标系，分别请求
   `left_arm`、`right_arm` 的 MoveIt IK。
5. 只有两侧 IK、关节限位和单关节步长检查都通过，bridge 才发送**一条**
   14 关节 `FollowJointTrajectory`：

```text
[left_joint1..left_joint7, right_joint1..right_joint7]
```

6. trajectory bridge 打开两侧 control gate，把同一条轨迹拆给左右驱动执行。
   任一臂 IK/轨迹失败时，bridge 锁存 fault、请求两侧急停；插件会抛错终止
   本地 rollout，避免故障后继续消费动作。

> `lerobot-rollout --policy.path=...` 是**本地加载策略**的模式。若策略实际在
> GPU 服务器上，应使用 `lerobot.async_inference.robot_client` 并指定
> `--server_address`，不要再在本地传 `--policy.path`。

#### 服务器推理模式启动命令

GPU 服务器只需要启动 policy server；checkpoint 路径由本地客户端在握手时
发送，因此该路径必须在服务器上可访问：

```bash
# GPU 服务器
python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080 \
  --fps=5 \
  --obs_queue_timeout=2
```

本地在启动 ROS bridge 后运行 robot client。`--pretrained_name_or_path` 是
服务器的路径或 Hugging Face repo，不是本机路径：

```bash
cd /home/work/nero_duo_arm
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.async_inference.robot_client \
  --server_address=<SERVER_IP>:8080 \
  --policy_type=smolvla \
  --pretrained_name_or_path=/path/on/server/smolvla_checkpoint \
  --policy_device=cuda \
  --actions_per_chunk=10 \
  --fps=5 \
  --task="pick up the object" \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
  --robot.action_schema=rot6d20 \
  --robot.policy_robot_type=bi_rdt_gripper \
  --robot.scene_camera_key=side \
  --robot.left_wrist_camera_key=left_wrist \
  --robot.right_wrist_camera_key=right_wrist \
  --robot.scene_image_width=640 \
  --robot.scene_image_height=480 \
  --robot.left_wrist_image_width=640 \
  --robot.left_wrist_image_height=480 \
  --robot.right_wrist_image_width=640 \
  --robot.right_wrist_image_height=480
```

当前 Scene 和左右腕 Color 均以 `640×480@30` 采集，插件默认也暴露原生
`640×480`，不做有损拉伸。若 checkpoint 使用 `480×480`，SmolVLA 预处理会用
`resize_with_pad` 保持宽高比缩放并 padding。训练和推理必须使用相同的预处理
策略；数据集本身应优先保存相机原生 `640×480`。

本地 client 退出时会调用插件的 `disconnect()`，向 bridge 发送 stop；不要用
`kill -9` 结束本地 client。

## 3. 启动前检查

### 3.0 安装或升级 LeRobot 插件

每次修改 `lerobot_robot_nero` 后都重新安装一次。安装器使用 compatible
editable 模式，保证从仓库根启动 `lerobot-rollout` 时不会被同名项目目录
遮蔽，并会检查 `nero_ros` 和 `nero_bimanual_ros` 两个注册项：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_lerobot_nero_plugin.sh
```

确认 LeRobot CLI 能看到双臂配置：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/lerobot-rollout \
    --robot.type=nero_bimanual_ros --help
```

### 3.1 确认 checkpoint 契约

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python - <<'PY'
from lerobot.configs import PreTrainedConfig

cfg = PreTrainedConfig.from_pretrained('/path/to/checkpoint')
print('input_features:')
for key, value in cfg.input_features.items():
    print(' ', key, value.type, value.shape)
print('action_feature_names:', getattr(cfg, 'action_feature_names', None))
PY
```

检查：

- state/action 是 20 维还是 14 维。
- 图像键是否为 `side/left_wrist/right_wrist`。
- 动作是否绝对位姿；相对动作必须由对应 processor 恢复成绝对目标，否则不能
  直接送入本 bridge。

### 3.2 确认 TCP 坐标系

分别检查两侧反馈和 TF：

```bash
ros2 topic echo --once /left/feedback/tcp_pose
ros2 topic echo --once /right/feedback/tcp_pose
ros2 run tf2_ros tf2_echo left_base_link left_tcp_link
ros2 run tf2_ros tf2_echo right_base_link right_tcp_link
```

bridge 默认把左侧数值解释为 `left_base_link` 下的位姿，右侧解释为
`right_base_link` 下。若话题带非空 `frame_id` 且不等于对应基座，bridge 会
拒绝该侧反馈。当前 `*_tcp_link` 基本在法兰处，真实夹爪 TCP 偏移仍必须标定。

## 4. 第一轮：插件与相机干跑

先只启动驱动、相机和 MoveIt，轨迹桥保持干跑：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=true

ros2 launch dual_nero_cameras all_cameras.launch.py

ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true rviz:=false
```

启动双臂 VLA bridge：

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=rot6d20 \
  execution_mode:=dry_run \
  allow_real_motion:=false \
  use_compressed_color:=true \
  camera_scene_key:=side \
  camera_left_wrist_key:=left_wrist \
  camera_right_wrist_key:=right_wrist
```

双臂 bridge 默认消费三路 `*/color/image_raw/compressed`。如果覆盖任一路
Color 话题，需同时设置对应的 `*_compressed_topic`，或显式设置
`use_compressed_color:=false` 改用 raw Image。

用插件读取三路图像和 20-D state，再原样回发一次：

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

预期：bridge 只打印动作和限幅，不调用 IK、不发送轨迹。

相机采集为 30 FPS；下方 SmolVLA 首轮的 `--fps=5` 是策略/rollout 频率，二者
不是同一个参数。不要因为相机已稳定 30 FPS 就直接把 rollout 提到 30 Hz。
当前 VLA 观测只使用三路 RGB，三路 Depth 默认关闭。

## 5. 第二轮：双臂 IK 干跑

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=rot6d20 \
  execution_mode:=ik_dry_run \
  allow_real_motion:=false
```

发送一个很小的双臂测试动作：

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

预期 bridge 依次出现 left/right IK target，响应中包含 14 个关节目标，且没有
`Sending bimanual trajectory`。如果任一臂 IK、坐标系或关节限位失败，整组被
拒绝，不允许只动一侧。

## 6. 第三轮：轨迹桥干跑

保持 `moveit_execution.launch.py dry_run:=true`，重启 bridge：

```bash
ros2 launch dual_nero_vla_bridge bimanual_vla_bridge.launch.py \
  action_schema:=rot6d20 \
  execution_mode:=trajectory \
  allow_real_motion:=true \
  max_translation_step_m:=0.005 \
  max_rotation_step_rad:=0.02 \
  max_joint_step_rad:=0.08 \
  trajectory_duration_sec:=0.25
```

再次发送上面的 `+2 mm`。预期 bridge 出现 `Sending bimanual trajectory`，
轨迹桥只发布仿真 joint state，不驱动电机。若轨迹桥拒绝 goal、反馈过期或
action 超时，bridge 会锁存 fault，并请求两侧控制门关闭和软件急停。

## 7. 第四轮：真机低速

前提：双臂均有稳定支撑或已确认可安全失能、工作区清空、人员监护、软件急停
入口可用；如现场已安装硬件急停，也必须处于可用状态。关闭所有 Web app、
Servo 和其他会写 `/left/control/joint_states` 或 `right` 的程序。

轨迹桥切到真机：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false rviz:=false
```

使用与上一轮相同的 VLA bridge 真机参数，先手工发送一次 `+2 mm`，确认：

- 两侧运动方向一致且幅度正确。
- 两臂同时开始、同时结束，没有单侧先动。
- 执行期间按 `Ctrl+C` 或 stop 后，两侧控制门关闭、软件急停生效。

通过后再启动 `lerobot-rollout`。

## 8. SmolVLA 评测命令

先使用 5 Hz、短时长、RTC：

```bash
cd /home/work/nero_duo_arm
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/lerobot-rollout \
  --strategy.type=base \
  --policy.path=/path/to/smolvla_checkpoint \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
  --robot.action_schema=rot6d20 \
  --robot.policy_robot_type=bi_rdt_gripper \
  --robot.scene_camera_key=side \
  --robot.left_wrist_camera_key=left_wrist \
  --robot.right_wrist_camera_key=right_wrist \
  --robot.scene_image_width=640 \
  --robot.scene_image_height=480 \
  --robot.left_wrist_image_width=640 \
  --robot.left_wrist_image_height=480 \
  --robot.right_wrist_image_width=640 \
  --robot.right_wrist_image_height=480 \
  --inference.type=rtc \
  --inference.rtc.execution_horizon=10 \
  --inference.rtc.max_guidance_weight=10.0 \
  --fps=5 \
  --duration=20 \
  --task="pick up the object" \
  --return_to_initial_position=false
```

`--return_to_initial_position=false` 很重要：rollout 默认会用线性插值回到启动
时的 state。对 rot6d 或 axis-angle 直接线性插值不是合法的姿态插值，首轮必须
关闭；退出时由插件 `disconnect()` 请求两侧停止。

插件默认读取原生 `640×480`；checkpoint 尺寸不同时由 SmolVLA 的
`resize_with_pad` 完成保持宽高比的缩放和 padding。

## 9. 安全门控

真实运动必须同时满足：

1. VLA bridge：`execution_mode=trajectory` 且 `allow_real_motion=true`。
2. 轨迹桥：`moveit_execution.launch.py dry_run=false`。
3. 驱动控制门可被轨迹桥打开，`/compute_ik` 和 FollowJointTrajectory 可用。
4. 两侧 TCP、关节反馈均新鲜；任一侧过期即整组拒绝。
5. 两侧 IK、关节限位和单步检查均通过；任一侧失败即锁存 fault。

不要只启动一侧 bridge 或只打开一侧控制门来“先试右臂”。本链路的设计前提是
双臂动作原子执行。

## 10. 当前限制

- 夹爪仍只参与 state/action，bridge **不发送夹爪命令**；驱动仍是
  `effector_type=none`。真实抓取前必须增加夹爪控制服务和开合验收。
- 若驱动没有夹爪宽度反馈，state 中夹爪值会是 `0.0`，不要据此做抓取评测。
- 短轨迹是 latest-wins 的绝对关节目标，不是 15/30 Hz 连续伺服。
- IK 只使用当前/目标关节状态做 MoveIt 碰撞检查，未接入在线点云 PlanningScene。
- 尚未用真实双臂 SmolVLA checkpoint 完成整链路验收。

## 11. 自动测试

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/test_bimanual_vla_pipeline.sh
```

覆盖：20-D 限幅、双臂 IK 调用、右臂失败整组拒绝、14 关节单 goal、14-D
axis-angle 模式和 LeRobot 插件 20-D/三相机映射。

### 11.1 本地端到端 dry-run 验收结果（历史配置）

以下结果是在当前 `640×480` 配置之前测得，保留作为历史记录。环境：三路
RealSense 实际相机流；Scene `640×360 YUYV @ 30`，左右腕
`480×270 RGB8 + Z16 @ 30`；VLA bridge `execution_mode=dry_run`；双侧 TCP 和
JointState 由假反馈以 30 Hz 发布。

- 原生 bridge：连续 60 秒、1800 次 `get_observation`，实际 `30.0 Hz`，错误 0；
- bridge 请求延迟：平均 `0.206 ms`，P95 `0.29 ms`，最大 `0.855 ms`；
- 三路图像唯一时间戳帧率：Scene `29.88 FPS`、左腕 `29.91 FPS`、右腕 `29.99 FPS`；
- 三路 JPEG 均可解码，尺寸为 `640×360`、`480×270`、`480×270`；
- 20-D `send_action` 在 dry-run 下返回 `accepted=true`、`executed=false`，
  未调用 IK 或轨迹执行；
- LeRobot `NeroBimanual` 插件连续 30 秒、900 次观测，实际 `30.0 Hz`，状态和
  动作均为 20 维；插件请求平均 `1.541 ms`、P95 `1.943 ms`、最大 `3.572 ms`；
- 插件 480×480 checkpoint 尺寸适配路径通过，三路输出均为 `480×480×3`；
- `./deployment/scripts/test_bimanual_vla_pipeline.sh` 通过：bridge 5 项测试和
  LeRobot 插件 4 项测试全部 OK。

未覆盖：真实双侧 TCP/关节反馈、真实 `/compute_ik`、FollowJointTrajectory
真机执行、真实 SmolVLA checkpoint、夹爪开合。

## 12. 验收标准

1. `dry_run` 能读到三路图像和两侧 state，动作维度为 20。
2. `ik_dry_run` 对当前位姿返回每侧 7 个关节解，无单侧超限。
3. 任意一侧 IK 失败时，轨迹桥收不到 goal，fault 被锁存。
4. `trajectory` + 轨迹桥干跑只生成一条 14 关节 goal。
5. 真机 `+2 mm` 双侧同步，停止后两侧门关闭且软件急停生效。
6. SmolVLA 在 5 Hz 下连续运行 20 秒无 IK/反馈/轨迹 fault，再考虑提高频率。
