# VLA 链路

> LeRobot 0.6.1 使用 `lerobot.async_inference.*`。工作区中的 0.6.2 已改为新的
> remote inference API；升级时必须同时迁移 policy server 和 robot client。
> 默认 `execution_mode:=dry_run`，真实运动需要多道门控。

## 1. 架构

```text
GPU 服务器                             本地 NERO 电脑
policy_server / SmolVLA  <--gRPC-->  robot_client (Python 3.12)
                                           |
                                           | Unix socket
                                           v
                                   dual_nero_vla_bridge (Python 3.10 / ROS 2)
                                           |
                         +-----------------+-----------------+
                         |                                   |
                  left_arm IK                         right_arm IK
                         |                                   |
                         +-------- 14-joint trajectory -----+
                                           |
                              dual_nero_trajectory_bridge
                                           |
                                  /left /right drivers
```

LeRobot 进程不导入 `rclpy`，ROS bridge 不导入 LeRobot；两边只共享长度前缀
JSON + JPEG 的本地 Unix socket 协议。

## 2. 数据契约

### 2.1 单臂 7-D

```text
observation.state = [x, y, z, rx, ry, rz, gripper]
action            = [x, y, z, rx, ry, rz, gripper]
```

- 位置米，旋转 axis-angle 弧度，夹爪 `0=闭合`、`1=张开`。
- `action[t] = state[t+1]`。
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
  speed_percent:=5 auto_enable:=true

ros2 launch dual_nero_cameras all_cameras.launch.py

# 先保持轨迹桥 dry-run
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true rviz:=false
```

确认 TCP 坐标系和 IK：

```bash
ros2 topic echo --once /left/feedback/tcp_pose
ros2 topic echo --once /right/feedback/tcp_pose
ros2 run tf2_ros tf2_echo left_base_link left_tcp_link
ros2 run tf2_ros tf2_echo right_base_link right_tcp_link
ros2 service list | grep '/compute_ik'
ros2 action list | grep follow_joint_trajectory
```

bridge 默认把左侧 TCP 数值解释为 `left_base_link` 下，右侧解释为
`right_base_link` 下。若话题 `frame_id` 非空且不匹配，bridge 拒绝该侧反馈。

## 5. 单臂链路

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
    action['ee.x'] += 0.005
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

### 5.4 真机低速

前提：工作区清空、人员监护、软件急停可用、左臂在支撑位附近、所有 Web app
和旧轨迹桥已关闭。

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false rviz:=false
```

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=trajectory \
  allow_real_motion:=true \
  allow_gripper_motion:=false \
  side:=left \
  ik_frame_id:=left_base_link \
  max_translation_step_m:=0.005 \
  max_rotation_step_rad:=0.02 \
  max_joint_step_rad:=0.08 \
  trajectory_duration_sec:=0.25
```

先只发单个 `+2 mm`，确认方向、幅度和停止行为。接 SmolVLA 时首轮使用
`--fps=5 --actions_per_chunk=10`，不要直接 30 Hz。

## 6. 双臂链路

先只读启动左右夹爪：

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
  allow_gripper_motion:=false \
  max_translation_step_m:=0.005 \
  max_rotation_step_rad:=0.02 \
  max_joint_step_rad:=0.08 \
  trajectory_duration_sec:=0.25
```

预期只生成一条 14 关节 goal，轨迹桥不驱动电机。任一臂失败时锁存 fault 并
请求两侧急停/关闭控制门。

### 6.4 真机低速

前提同单臂，且双臂均有支撑或可安全失能。轨迹桥切到真机：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false rviz:=false
```

先手工发一次双臂 `+2 mm`，确认两侧同时开始/结束、方向正确，停止后两侧控制门
关闭且软件急停生效。夹爪验收先只开一侧：

```bash
ros2 launch dual_nero_control dual_grippers.launch.py \
  allow_motion:=true enable_motor:=false
ros2 service call /left_gripper/enable std_srvs/srv/SetBool "{data: true}"
```

## 7. Policy server / client

GPU 服务器：

```bash
python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080 \
  --fps=5 \
  --obs_queue_timeout=2
```

本地单臂 client：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.async_inference.robot_client \
  --robot.type=nero_ros \
  --robot.bridge_socket=/tmp/nero_vla_bridge.sock \
  --robot.side=left \
  --robot.camera_key=camera1 \
  --server_address=<SERVER_IP>:8080 \
  --policy_type=smolvla \
  --pretrained_name_or_path=<CHECKPOINT_OR_HF_REPO> \
  --actions_per_chunk=10 \
  --fps=5 \
  --task="pick up the block"
```

本地双臂 client：

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.async_inference.robot_client \
  --server_address=<SERVER_IP>:8080 \
  --policy_type=smolvla \
  --pretrained_name_or_path=/path/on/server/checkpoint \
  --actions_per_chunk=10 \
  --fps=5 \
  --task="pick up the object" \
  --robot.type=nero_bimanual_ros \
  --robot.bridge_socket=/tmp/nero_bimanual_vla_bridge.sock \
  --robot.action_schema=rot6d20 \
  --robot.policy_robot_type=bi_rdt_gripper \
  --robot.scene_camera_key=side \
  --robot.left_wrist_camera_key=left_wrist \
  --robot.right_wrist_camera_key=right_wrist
```

本地双臂 rollout 评测使用 `lerobot-rollout`。首轮必须加
`--return_to_initial_position=false`，因为 rot6d/axis-angle 不能线性插值回初始
姿态；退出时由插件 `disconnect()` 请求两侧停止。

## 8. 安全门控

真实运动必须同时满足：

1. VLA bridge `execution_mode=trajectory` 且 `allow_real_motion=true`。
2. 轨迹桥 `moveit_execution.launch.py dry_run=false`。
3. `/compute_ik` 和 `FollowJointTrajectory` 可用，驱动控制门可打开。
4. 两侧 TCP/关节反馈新鲜；任一侧过期即整组拒绝。
5. 两侧 IK、关节限位、单步检查全部通过；任一失败锁存 fault。
6. 夹爪只在 `allow_gripper_motion=true` 时发送；反馈缺失/过期时本次不发送。
7. 夹爪电机必须先通过 `/<side>_gripper/enable` 使能。

不要只启动一侧 bridge 或只打开一侧控制门来“先试右臂”；双臂链路设计为原子执行。

## 9. 自动测试

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/test_vla_motion_pipeline.sh
./deployment/scripts/test_bimanual_vla_pipeline.sh
```

两脚本分别使用 `ROS_DOMAIN_ID=81/82` 和 `ROS_LOCALHOST_ONLY=1`，可并行执行；
避免共享 `/compute_ik` 与反馈话题造成串扰。

覆盖单臂 dry-run/IK/短轨迹、IK 失败锁存、关节单步超限、双臂 20-D 限幅、
14 关节单 goal、14-D axis-angle、夹爪映射和 LeRobot 插件特征。

## 10. 限制与验收

- 左/右夹爪宽度和 gripper action 已接入，默认 `allow_gripper_motion=false`。
- 夹爪空载 10×10 和低速软件急停已通过；带载抓取、堵转清障、长时热稳定和
  真实 SmolVLA 闭环仍未验收。
- 短轨迹是 latest-wins 绝对关节目标，不是 15/30 Hz 连续伺服。
- IK 只用当前/目标关节状态做碰撞检查，未接入在线点云 PlanningScene。
- 真实双臂 SmolVLA checkpoint 整链路仍未验收。

验收标准：

1. `dry_run` 能读取图像和 state，动作维度正确。
2. `ik_dry_run` 返回每侧 7 个关节解，无超限。
3. 任一臂 IK 失败时轨迹桥收不到 goal，fault 锁存。
4. `trajectory` + dry-run 轨迹桥只生成一条 14 关节 goal。
5. 真机 `+2 mm` 双侧同步，停止后两侧门关闭且软件急停生效。
6. SmolVLA 在 5 Hz 连续运行 20 秒无 IK/反馈/轨迹 fault。
