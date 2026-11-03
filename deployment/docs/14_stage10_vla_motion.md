# Stage 10: VLA 动作到 NERO 运动执行

本文实现 SmolVLA 输出到 NERO 左臂的第一版运动链路：

```text
action[7] = [x, y, z, rx, ry, rz, gripper]
                     |
                     v
             工作空间/单步限幅
                     |
                     v
       /compute_ik (MoveIt, left_arm)
                     |
                     v
          关节限位/单步验证
                     |
                     v
 /dual_arm_controller/follow_joint_trajectory
                     |
                     v
        /dual_nero_trajectory_bridge -> driver
```

这是一个可验收的首版链路，不是高频 MoveIt Servo。bridge 采用
latest-wins：新的 IK 目标覆盖尚未执行的旧目标，同一时刻只有一个
`FollowJointTrajectory` goal。默认短轨迹时长 `0.2 s`，因此首轮推理建议
`--fps=5` 或 `10`。15/30 Hz 连续跟踪需要下一阶段接入 MoveIt Servo 或
专用流式关节伺服。

## 1. 执行模式

| `execution_mode` | 调用 IK | 发送轨迹 | 真实运动 | 用途 |
| --- | --- | --- | --- | --- |
| `dry_run` | 否 | 否 | 否 | 只验证插件、相机和动作限幅 |
| `ik_dry_run` | 是 | 否 | 否 | 验证坐标系、IK 解、关节限位 |
| `trajectory` | 是 | 是 | 取决于轨迹桥 | 先接干跑轨迹桥，最后接真机 |

`trajectory` 还要求 `allow_real_motion:=true`。真正驱动机械臂还需要
`dual_nero_trajectory_bridge` 以 `dry_run:=false` 启动。两个门控缺一不可。

## 2. 固定接口

bridge 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `execution_mode` | `dry_run` | `dry_run/ik_dry_run/trajectory` |
| `allow_real_motion` | `false` | `trajectory` 模式的第二道门控 |
| `ik_service` | `/compute_ik` | MoveIt IK 服务 |
| `ik_group_name` | `left_arm` | 由 `side` 自动生成 |
| `ik_link_name` | `left_tcp_link` | 由 `side` 自动生成 |
| `ik_frame_id` | `left_base_link` | IK 位姿坐标系 |
| `ik_timeout_sec` | `0.1` | 单次 IK 求解超时 |
| `max_joint_step_rad` | `0.15` | 相对当前反馈的单关节最大步长 |
| `trajectory_action` | `/dual_arm_controller/follow_joint_trajectory` | 轨迹桥 action |
| `trajectory_duration_sec` | `0.2` | 每个短轨迹的时长 |

安全处理：

- TCP 或关节反馈超过 `observation_timeout_sec` 时拒绝动作。
- 平移/旋转先经过 `0.01 m`、`0.05 rad` 单步限幅。
- IK 失败、关节超限、关节跳变、轨迹失败、轨迹超时或 action 超时都会锁存
  motion fault，并请求软件急停/关闭控制门。
- fault 锁存后 bridge 拒绝后续动作。必须排除原因、重新启动 bridge，并重新
  确认机器人状态；不要自动恢复。

## 3. 前置检查

必须先确认 TCP 位姿与 IK 坐标系一致。当前驱动的
`/left/feedback/tcp_pose` 没有填写 `frame_id`，而 bridge 默认把数值解释为
`left_base_link` 下的位姿。上机前要确认：

```bash
ros2 topic echo --once /left/feedback/tcp_pose
ros2 run tf2_ros tf2_echo left_base_link left_tcp_link
```

若驱动实际发布的是世界坐标或其他基座坐标，必须先做固定变换，不能直接让 IK
使用。如果 TCP 话题带有非空的 `frame_id` 且不等于 `ik_frame_id`，bridge 会
拒绝该反馈。当前 `left_tcp_link` 基本位于法兰处，真实夹爪 TCP 偏移仍需标定。

驱动、相机和 MoveIt：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=true

ros2 launch dual_nero_cameras all_cameras.launch.py

# move_group + 轨迹桥；先保持轨迹桥 dry-run
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true rviz:=false
```

检查 IK 和 action：

```bash
ros2 service list | grep '/compute_ik'
ros2 action list | grep follow_joint_trajectory
ros2 node list | grep -E 'move_group|trajectory_bridge'
```

## 4. 第一轮：IK 干跑

启动 bridge：

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=ik_dry_run \
  allow_real_motion:=false \
  side:=left \
  ik_frame_id:=left_base_link \
  image_topic:=/camera_scene/color/image_raw \
  image_compressed_topic:=/camera_scene/color/image_raw/compressed \
  use_compressed_image:=true \
  image_key:=camera1
```

单臂 bridge 默认消费 JPEG 压缩流。自定义 raw 图像话题时，必须同时覆盖
`image_compressed_topic`，或设置 `use_compressed_image:=false`。

用插件发送一个 `+5 mm` 的测试动作：

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

当前 Scene Color 原始尺寸为 `640×480`。如果 SmolVLA checkpoint 使用其它输入
尺寸，由模型预处理保持宽高比缩放并 padding 到 checkpoint 尺寸；训练和推理
必须使用同一预处理策略。

bridge 终端应出现 `IK target ...`，且没有 `Sending trajectory`。如果出现
坐标系、关节限位或单步超限错误，先修坐标/模型，不要进入下一轮。

## 5. 第二轮：轨迹桥干跑

保持 `moveit_execution.launch.py dry_run:=true`，重启 VLA bridge：

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=trajectory \
  allow_real_motion:=true \
  side:=left \
  ik_frame_id:=left_base_link \
  trajectory_duration_sec:=0.2
```

重复上一节的 `+5 mm` 动作。预期：

- VLA bridge 输出 `Sending trajectory #...`。
- 轨迹桥处于 `dry_run=True`，只插值/发布仿真 joint states，不驱动电机。
- 轨迹桥终端报告 trajectory completed。

此时若轨迹桥拒绝 goal、反馈过期或 action 不可用，VLA bridge 会锁存 fault。

## 6. 第三轮：真机低速

前提：工作区清空、人员监护、硬件急停可用、左臂处于稳定支撑位、软件急停已
实测。先关闭所有 Web app 和旧的轨迹桥，避免两个节点同时控制。

轨迹桥真机模式：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false rviz:=false
```

VLA bridge 真机模式：

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=trajectory \
  allow_real_motion:=true \
  side:=left \
  ik_frame_id:=left_base_link \
  max_translation_step_m:=0.005 \
  max_rotation_step_rad:=0.02 \
  max_joint_step_rad:=0.08 \
  trajectory_duration_sec:=0.25
```

先只发送单个 `+2 mm` 动作，确认方向、幅度和停止行为。通过后再接 SmolVLA，
首轮建议：

```bash
--fps=5 --actions_per_chunk=10
```

不要一开始使用 `--fps=30`。当前链路会丢弃来不及执行的中间动作，30 Hz 下会
表现为明显跳步。

## 7. 当前限制

- 本文的单臂 bridge 只控制左臂。双臂原子执行、20-D rot6d 和三相机评测见
  [阶段 11](15_stage11_vla_bimanual_eval.md)。
- 只使用单路场景相机 `camera1`。
- `gripper` 参与 state/action，但当前 bridge **不发送夹爪命令**。驱动仍是
  `effector_type:=none`；安装 AGX 夹爪后还需要增加夹爪控制接口和开合验收。
- 短轨迹是绝对关节目标，latest-wins；不是 15/30 Hz 连续笛卡尔伺服。
- 未接入 PlanningScene 在线碰撞更新，只依赖 IK 的 `avoid_collisions`。
- 尚未完成真实 SmolVLA checkpoint 的整链路验收。

## 8. 自动测试

假 `/compute_ik` 与假 FollowJointTrajectory server：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/test_vla_motion_pipeline.sh
```

测试覆盖：dry-run 不调用 IK、IK 干跑字段、短轨迹内容、IK 失败锁存、关节
单步超限拒绝。

## 9. 验收标准

1. `ik_dry_run` 对当前位姿返回与反馈关节接近的解。
2. `+5 mm` 动作在 IK 中被限制为不超过 `0.01 m`，关节解不超限。
3. 轨迹桥干跑完成，无 action 超时，无 fault。
4. 真机单步 `+2 mm` 运动方向正确，停止后可立即软件急停。
5. SmolVLA 在 5 Hz 下输出连续、无 IK fault；再考虑提高频率或接入 Servo。
