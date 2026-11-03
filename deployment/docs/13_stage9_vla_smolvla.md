# Stage 9: SmolVLA VLA 部署链路

本文记录 NERO 双臂的 VLA 部署路径。当前已打通 **观测 -> LeRobot `Robot`
插件 -> 异步策略客户端**，并新增 **VLA 绝对末端动作 -> MoveIt IK -> 短轨迹**
桥接。默认仍是 `execution_mode:=dry_run`；真实运动需要显式双门控，详见
[阶段 10](14_stage10_vla_motion.md)。

> 版本注意：本文命令基于本地 LeRobot 0.6.1 的
> `lerobot.async_inference.*`。工作区中的 LeRobot 0.6.2 已改为新的 remote
> inference API；升级时必须同步迁移 policy server 和 robot client，不能混用。

双臂 20-D rot6d、三相机和 `lerobot-rollout` 实机评测请使用
[阶段 11](15_stage11_vla_bimanual_eval.md)。

## 架构

```text
服务器 GPU（Python 3.12）                   本地 NERO 电脑
┌───────────────────────────┐              ┌──────────────────────────────────┐
│ lerobot.async_inference   │              │ Python 3.12: lerobot             │
│ policy_server             │   gRPC       │   robot_client                   │
│ SmolVLA checkpoint        │<------------>│   lerobot_robot_nero (nero_ros)  │
└───────────────────────────┘              │              │ Unix socket       │
                                           │              v                  │
                                           │ Python 3.10 + ROS 2 Humble       │
                                           │   dual_nero_vla_bridge           │
                                           │   ├─ /left/feedback/tcp_pose     │
                                           │   ├─ /left/feedback/joint_states │
                                           │   ├─ /camera_scene/.../image_raw │
│   └─ 安全检查 / IK / 短轨迹       │
                                           └──────────────────────────────────┘
```

LeRobot 进程不导入 `rclpy`。ROS bridge 不导入 LeRobot。两边只共享一个
长度前缀 JSON + JPEG 的本地 Unix socket 协议。

## 固定数据契约

```text
observation.state = [x, y, z, rx, ry, rz, gripper]
action            = [x, y, z, rx, ry, rz, gripper]
```

- `x/y/z`：米，左臂 TCP 绝对位姿。
- `rx/ry/rz`：axis-angle 旋转向量，弧度。
- `gripper`：`0.0=闭合`，`1.0=打开`；bridge 用 `gripper_max_m` 归一化。
- `action[t] = state[t+1]`。
- 第一版只控制左臂，使用单路场景相机。
- 图像默认键：`observation.images.camera1`。

## 1. 编译 ROS bridge

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dual_nero_vla_bridge
source install/setup.bash
ros2 pkg executables dual_nero_vla_bridge
```

应输出：

```text
dual_nero_vla_bridge vla_bridge
```

## 2. 启动 bridge

先启动驱动和相机，或者至少让 TCP、关节、场景相机话题有数据。然后：

```bash
source /opt/ros/humble/setup.bash
source /home/work/nero_duo_arm/ros2_ws/install/setup.bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=dry_run \
  allow_real_motion:=false \
  side:=left \
  image_topic:=/camera_scene/color/image_raw \
  image_compressed_topic:=/camera_scene/color/image_raw/compressed \
  use_compressed_image:=true \
  image_key:=camera1
```

相机实际以 30 FPS 采集；`vla_bridge` 默认优先消费
`/camera_scene/color/image_raw/compressed`。如果自定义 raw 图像话题，必须同时
设置 `image_compressed_topic`，或显式设置 `use_compressed_image:=false`。

默认 socket 是 `/tmp/nero_vla_bridge.sock`。检查：

```bash
ls -l /tmp/nero_vla_bridge.sock
ros2 node info /dual_nero_vla_bridge
```

## 3. 安装 LeRobot 插件

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_lerobot_nero_plugin.sh
```

默认使用现有环境 `/home/work/miniforge3/envs/lerobot`。脚本会清理 ROS 注入的
`PYTHONPATH` 等变量，避免 Python 3.12 环境加载 Humble 的 Python 3.10
site-packages。

## 4. 不接策略的闭环自检

在另一个终端，启动 bridge 后执行：

```bash
env -u PYTHONPATH \
    -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH \
    -u ROS_DISTRO \
    -u ROS_VERSION \
    -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python - <<'PY'
from lerobot_robot_nero import Nero, NeroConfig

with Nero(NeroConfig(
    bridge_socket='/tmp/nero_vla_bridge.sock',
    side='left',
    camera_key='camera1',
    image_width=640,
    image_height=480,
)) as robot:
    observation = robot.get_observation()
    print({key: value for key, value in observation.items() if key != 'camera1'})
    print('image', observation['camera1'].shape, observation['camera1'].dtype)
    applied = robot.send_action({name: observation[name] for name in robot.action_features})
    print('applied', applied)
PY
```

当前 Scene Color 的原始尺寸是 `640×480`，所以示例直接使用 `640×480`，避免
额外缩放。若 SmolVLA checkpoint 的训练尺寸不同，应以 checkpoint 为准；插件
会用 `INTER_AREA` 把图像缩放到 `image_width`/`image_height`。`NeroConfig` 的
`image_fps` 只是插件配置字段，当前不驱动 RealSense 采集循环；相机 30 FPS 由
`all_cameras.launch.py` 决定。SmolVLA 会在模型预处理中把图像保持宽高比缩放
并 padding 到 checkpoint 输入尺寸。

`execution_mode:=dry_run` 时 `send_action` 只做安全过滤并打印，不调用 IK、
不驱动机械臂。bridge 默认限制：

- 单步平移：`0.01 m`
- 单步旋转：`0.05 rad`
- 工作空间：`[-2,-2,-0.2]` 到 `[2,2,2]` 米
- gripper 范围：`0..1`

## 5. 服务器启动 SmolVLA policy server

在 GPU 服务器的 LeRobot 环境中：

```bash
python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080 \
  --fps=15 \
  --inference_latency=0.066 \
  --obs_queue_timeout=2
```

第一次握手时，客户端会把 policy 类型、checkpoint 路径和机器人特征发给
server。server 需要能访问该 checkpoint。

这里的 `--fps=15` 是策略服务器/控制循环频率，不是相机采集频率。当前相机
采集仍为 30 FPS；首轮推理建议保持 5–15 Hz，不要因为相机是 30 FPS 就直接
把策略频率设为 30 Hz。

## 6. 本地启动 robot client

```bash
env -u PYTHONPATH \
    -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH \
    -u ROS_DISTRO \
    -u ROS_VERSION \
    -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.async_inference.robot_client \
  --robot.type=nero_ros \
  --robot.bridge_socket=/tmp/nero_vla_bridge.sock \
  --robot.side=left \
  --robot.camera_key=camera1 \
  --robot.image_width=640 \
  --robot.image_height=360 \
  --robot.request_timeout_s=2.0 \
  --server_address=<SERVER_IP>:8080 \
  --policy_type=smolvla \
  --pretrained_name_or_path=<CHECKPOINT_OR_HF_REPO> \
  --actions_per_chunk=50 \
  --fps=15 \
  --task="pick up the block"
```

训练数据推荐直接用 `umi_to_lerobot.py --contract nero-ros` 生成；它会输出
`observation.images.camera1`、7D `observation.state/action` 和
`robot_type=nero_ros`，与这里的 bridge 和插件默认值一致。如果使用
`umi-sroi` 默认契约，则图像键是 `observation.images.camera`，需要把 bridge
的 `image_key` 和客户端 `--robot.camera_key` 同步改为 `camera`。训练集和推理
端的键名、state 维度必须完全一致。

## 7. 训练数据要求

SmolVLA 训练集至少需要：

```text
observation.state          float32[7]
observation.images.camera1 image H x W x 3
action                     float32[7]
task                       language instruction
```

使用 UMI 数据训练时，转换命令必须加 `--contract nero-ros`：

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /path/to/umi_session.zarr.zip \
  -o /home/work/datasets/lerobot/umi_nero \
  --repo-id local/umi_nero \
  --fps 30 \
  --task "pick up the block" \
  --contract nero-ros \
  --gripper-min 0.0 \
  --gripper-max 0.09 \
  --pose-transform /path/to/T_robot_from_umi.json
```

正式训练前检查：

1. `info.json` 中 `action` 和 `observation.state` 都是 7D，名称与
   `STATE_NAMES` 一致。
2. 图像键是 `observation.images.camera1`，尺寸与推理相机一致。
3. `action[t] = state[t+1]`。
4. UMI 坐标已经用实测 `T_robot_from_umi` 变换到 NERO 左臂基座坐标系。

GPU 服务器上的 SmolVLA 微调示例：

```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=local/umi_nero \
  --dataset.root=/data/lerobot/umi_nero \
  --policy.repo_id=${HF_USER}/nero_umi_smolvla \
  --batch_size=32 \
  --steps=20000 \
  --output_dir=/data/checkpoints/nero_umi_smolvla \
  --job_name=nero_umi_smolvla \
  --policy.device=cuda
```

`batch_size`/`steps` 需要按 GPU 显存和数据量调整。完整的上传、训练和部署
顺序见 [UMI → SmolVLA → NERO 端到端流程](16_umi_nero_e2e.md)。

## 8. 实机运动桥

`ik_dry_run` 和 `trajectory` 两种模式、启动顺序、双门控和验收步骤见
[阶段 10：VLA 动作到 NERO 运动执行](14_stage10_vla_motion.md)。

## 9. 当前状态

当前已验证：

- ROS 假 TCP/关节/相机话题 -> bridge -> Python 3.12 `Nero` 插件闭环。
- 四元数转 axis-angle、gripper 归一化、图像 JPEG 传输和单步安全夹紧。
- `RobotConfig` 自动发现 `nero_ros`，`RobotClient` 生成 7D state 与
  `camera1` 图像特征。
- 绝对 TCP action -> `/compute_ik` -> 关节限位/单步检查。
- `ik_dry_run` 报告关节解，不发送轨迹。
- `trajectory` 以 latest-wins 异步方式发送短 `FollowJointTrajectory`。
- 假 IK 服务 + 假轨迹 action server 的 5 项接口测试通过。

尚未实现：

- 真实夹爪控制。
- 3 相机训练/推理特征。
- 15/30 Hz 连续伺服；当前短轨迹采用显式降频/latest-wins。
- 真机整链路验收（目前只验证到假 IK/假 action server）。

