# MoveIt 与标准支撑位

> 当前状态：规划、碰撞检查、轨迹桥 dry-run、单臂/双臂真机执行、真实轨迹取消
> 联锁和稳定 park pose 已通过。相机点云和动态障碍尚未接入。

## 1. 规划组

| 规划组 | 类型 | 末端/关节 |
| --- | --- | --- |
| `left_arm` | 单链 | `left_tcp_link` |
| `right_arm` | 单链 | `right_tcp_link` |
| `both_arms` | 组合 | `left_arm` + `right_arm` |
| `left_gripper` | 单关节 | `left_ctag_left_1_joint` |
| `right_gripper` | 单关节 | `right_ctag_left_1_joint` |
| `both_grippers` | 组合 | `left_gripper` + `right_gripper` |

夹爪关节范围 `[-0.91, 0] rad`：`0` 全开，`-0.91` 全闭。TCP 相对 `tool0`
沿工具轴偏移 `0.1708 m`。

## 2. 启动规划

MoveIt 默认给机器人、场景物体和附着物体统一增加 `0.025 m`（25 mm）
碰撞包络，用于吸收基座位置、桌面高度和连杆尺寸的现场测量误差。该包络在
规划阶段生效，不等同于真实障碍物检测。

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py
```

无界面：

```bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py \
  gui:=false rviz:=false
```

现场复测误差较大时可临时增大包络；这会降低可达空间，必须重新验证目标位姿：

```bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py \
  collision_padding:=0.03
```

执行入口和 Web app 使用同名参数。例如：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true collision_padding:=0.03
```

RViz 手动规划：

1. `MotionPlanning -> Context -> Planning Group` 选择 `left_arm`、
   `right_arm` 或 `both_arms`。
2. `Start State` 设为 `Current`。
3. 设置 `Goal State` 或拖动 `Joints` 滑条。
4. 先点 `Plan`；确认无碰撞后再 `Execute`。

## 3. 轨迹桥 dry-run

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py dry_run:=true
```

dry-run 行为：

- 接收 `/dual_arm_controller/follow_joint_trajectory`。
- 按 50 Hz 插值并更新模拟 `/joint_states`。
- 不向驱动发布控制命令，不使能电机。

规划与碰撞检查：

```bash
cd /home/work/nero_duo_arm
/usr/bin/python3 deployment/scripts/test_dual_moveit_plan.py
/usr/bin/python3 deployment/scripts/test_dual_collision.py
/usr/bin/python3 deployment/scripts/test_moveit_execution_dry_run.py --group left_arm
/usr/bin/python3 deployment/scripts/test_moveit_execution_dry_run.py --group both_arms
/usr/bin/python3 deployment/scripts/test_trajectory_bridge_dry_run.py
/usr/bin/python3 deployment/scripts/test_trajectory_cancel_dry_run.py
```

## 4. 标准支撑位

文件：`deployment/config/support_pose.yaml`。

```text
left_joint1:  -1.574653504
left_joint2:  -1.757092771
left_joint3:  -0.045413467
left_joint4:   2.140000000
left_joint5:  -0.013561208
left_joint6:  -0.009267698
left_joint7:   0.122714100

right_joint1:  1.562331480
right_joint2: -1.752886528
right_joint3: -0.050841441
right_joint4:  2.140000000
right_joint5: -0.014311700
right_joint6: -0.007627089
right_joint7:  0.118996548
```

约束：

- `joint2` 下限 `-1.760` 是唯一保留的项目放宽项，不得继续放宽。
- `joint4` 原生上限为 `2.146755`；部署保守限幅 `2.14`，不要改回 `2.20`。
- `joint6` 使用官方范围 `[-0.7330383, 0.8726646]`。
- 支撑位通过 MoveIt `check_state_validity`，要求 `valid=True, contacts=0`。
- 复位后保持电机使能、只关闭 `/control_enable` 门。右臂失能后会被重力带回
  `joint4≈2.198`，再次执行前必须重新归位到 `2.12`。
- 驱动必须带 `tcp_offset=[0.2018, 0.0, -0.0235, -1.5708, 0.0, -1.5708]`；
  `reset_to_standard_support.sh` 已内置该参数。TCP 自检见
  `deployment/scripts/check_tcp_contract.py`。

## 5. 上电归位与复位到支撑位

### 5.1 上电 joint4 局部归位

上电反馈可能让 `joint4` 停在 `2.19--2.20 rad`，超过部署上限 `2.14`，此时
直接做 FK/IK 或整体 Park 会失败。先执行只改 `joint4` 的低速归位：

```bash
cd /home/work/nero_duo_arm

# 只读检查，不使能、不运动
./deployment/scripts/recover_joint_limits.sh --dry-run

# 真实局部归位；结束后关闭控制门，但保持电机使能
./deployment/scripts/recover_joint_limits.sh --target 2.115 --cap-margin 0.03
```

脚本会拒绝陈旧反馈、其它关节超限或修正量过大的情况，不访问夹爪。驱动 launch
也可以自动执行：

```bash
ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=true \
  recover_joint_limits:=true recover_joint_limits_delay_sec:=8.0
```

归位是真实运动，必须有支撑、清空工作区和现场监护。不要用 `--disable`，也不要
在归位结束后关闭电机使能。

### 5.2 整体复位到支撑位

```bash
cd /home/work/nero_duo_arm

# 只规划，不使能、不运动
./deployment/scripts/reset_to_standard_support.sh --plan-only

# 真实复位；到位后软件急停，默认保持电机使能
./deployment/scripts/reset_to_standard_support.sh

# 完整失能验证；仅在现场确认支撑和误差后使用
./deployment/scripts/reset_to_standard_support.sh --disable
```

脚本会自行启动左右驱动和 MoveIt，执行前必须关闭 Web app、其他 MoveIt 和
占用 CAN/action 的程序。`--disable` 在失能前检查关节限位，失能后等待并复测
漂移。

## 6. 真机执行与取消

先确认工作区无人、机械臂有支撑、软件急停可用、左右均在支撑位附近：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false rviz:=false
```

真实轨迹取消联锁测试：

```bash
./deployment/scripts/test_trajectory_cancel_real.sh
```

该脚本只移动两侧 `joint7` 各约 `0.04 rad`，检测到运动后取消，随后自动执行
`reset_to_standard_support.sh --disable`。要求两侧软件急停各触发至少一次、
两侧控制门关闭，并最终回到支撑位且 14 个电机失能。

## 7. 已知限制

- NERO 驱动没有标准 `FollowJointTrajectory` action，必须经轨迹桥。
- 碰撞检查只覆盖 URDF、桌面和静态 planning scene；未接入相机点云/OctoMap。
- `collision_padding` 只能覆盖有界建模误差；未建模的动态障碍、工装变形和
  超过包络的安装偏差仍可能导致真机碰撞，不能替代现场急停和工作区隔离。
- 短轨迹是 latest-wins 的绝对关节目标，不是高频连续伺服。
- 夹爪 `GripperCommand` 默认 `allow_motion=false`，真机夹爪步骤见夹爪文档。
