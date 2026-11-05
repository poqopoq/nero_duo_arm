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

## 5. 复位到支撑位

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
- 短轨迹是 latest-wins 的绝对关节目标，不是高频连续伺服。
- 夹爪 `GripperCommand` 默认 `allow_motion=false`，真机夹爪步骤见夹爪文档。
