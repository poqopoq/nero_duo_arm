# 阶段 5：MoveIt 2 双臂规划

> 开始日期：2026-11-02  
> 前置模型：`dual_nero_description`  
> 当前状态：**规划、碰撞、轨迹桥 dry-run 和左臂真机执行均已通过（6/7）**；夹爪规划组待完成。

## 1. 规划组

| 规划组 | 类型 | 基座 | 末端 |
| --- | --- | --- | --- |
| `left_arm` | 单链 | `left_base_link` | `left_tcp_link` |
| `right_arm` | 单链 | `right_base_link` | `right_tcp_link` |
| `both_arms` | 子组组合 | — | `left_arm` + `right_arm` |

夹爪尚未选型，因此暂不创建 `left_gripper`、`right_gripper` 规划组。

## 2. 坐标系

```text
world                         桌面平面、两基座中心中点
├── left_base_mount           xyz=[0,+0.35,0], yaw=π
├── right_base_mount          xyz=[0,-0.35,0], yaw=π
└── table_slab
```

`left_tcp_link` 和 `right_tcp_link` 当前位于法兰处，作为临时规划末端；夹爪确定后更新 TCP 偏移。

## 3. 第一阶段目标

- [x] 安装 MoveIt 2
- [x] 创建 `dual_nero_moveit_config`
- [x] 生成 SRDF：`left_arm`、`right_arm`、`both_arms`
- [x] 配置关节限制和 KDL 运动学
- [x] 加入桌面、双臂碰撞体
- [x] RViz 加载双臂模型
- [x] 执行纯规划测试，不向真机发送轨迹
- [x] 验证左右臂和双臂碰撞检测
- [ ] 夹爪规划组（夹爪未定）
- [x] MoveIt 轨迹到 NERO 执行桥接（dry-run）

## 4. 后续执行集成

真实 NERO 驱动当前使用：

```text
/left/control/joint_states
/right/control/joint_states
```

它不是标准 `FollowJointTrajectory` action。真实执行前需要：

- 添加轨迹到 JointState 的桥接节点，或
- 增加专用 MoveIt controller adapter，并保持软件急停和控制门。


## 5. 当前实现

MoveIt 配置包：

```text
dual_nero_ws/src/dual_nero_moveit_config
```

启动：

```bash
cd /home/a/workspace/dual_nero_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py
```

无界面模式：

```bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py gui:=false rviz:=false
```

## 6. 纯规划结果

测试目标：

- `left_arm`：左臂小角度关节目标
- `both_arms`：左右臂同时小角度关节目标

结果：

```text
left_arm:   error_code=1, trajectory_points=36
both_arms:  error_code=1, trajectory_points=36
```

`error_code=1` 为 MoveIt `SUCCESS`。

测试脚本：

```text
dual_nero_deployment/scripts/test_dual_moveit_plan.py
```

## 7. 碰撞检测结果

通过 `check_state_validity` 验证：

```text
normal_zero:       valid=True,  contacts=0
left_into_table:   valid=False, contacts=1
```

说明桌面碰撞体生效，MoveIt 能检测机械臂伸入桌面的状态。

测试脚本：

```text
dual_nero_deployment/scripts/test_dual_collision.py
```

## 8. 当前限制

- 当前 MoveIt 只规划，不执行。
- NERO 驱动没有标准 `FollowJointTrajectory` action。
- `moveit_simple_controller_manager` 当前没有配置真实控制器。
- 夹爪未选型，左右夹爪组未创建。
- 相机尚未加入 MoveIt 的 Octomap/PlanningScene。

## 9. 下一步

实现安全的 MoveIt 轨迹执行桥：

1. 接收 MoveIt `FollowJointTrajectory`。
2. 按时间插值生成 `/left/control/joint_states`、`/right/control/joint_states`。
3. 默认 `control_enabled=false`，只做 dry-run。
4. dry-run 通过后，再在 5% 速度下做极小轨迹执行测试。
5. 同时支持双臂软件急停并关闭两侧控制门。

## 10. RViz 手动规划方法

启动：

```bash
cd /home/a/workspace/dual_nero_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py
```

RViz 操作：

1. 左侧 Displays 中确认 `MotionPlanning` 已启用。
2. 展开 `MotionPlanning -> Context`，将 `Planning Group` 选择为：
   - `left_arm`
   - `right_arm`
   - `both_arms`
3. 打开 `Planning` 面板。
4. 将 `Start State` 设为 `Current`。
5. 简单测试时，将 `Goal State` 设为 `<random valid>`。
6. 点击 `Plan`，只规划，不要点击 `Execute`。
7. 规划成功后可在 Trajectory 显示中播放轨迹或拖动时间滑块。

指定关节目标：

1. 打开 `Joints` 面板。
2. 选择 Goal 状态。
3. 拖动对应关节滑块。
4. 返回 `Planning -> Plan`。

当前控制器尚未接入，因此 `Execute` 和 `Plan & Execute` 会失败，这是预期行为。


## 11. 轨迹桥 dry-run

包：

```text
dual_nero_ws/src/dual_nero_control
```

动作接口：

```text
/dual_arm_controller/follow_joint_trajectory
control_msgs/action/FollowJointTrajectory
```

控制器配置：

```text
dual_nero_moveit_config/config/moveit_controllers.yaml
```

启动：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py dry_run:=true
```

dry-run 行为：

- 桥接收 MoveIt 轨迹。
- 按 50 Hz 插值。
- 更新模拟 `/joint_states`。
- 不向 `/left/control/joint_states` 或 `/right/control/joint_states` 发布命令。
- 不使能机械臂。

实测：

```text
left_arm MoveGroup execution:  error_code=1, SUCCESS
both_arms MoveGroup execution: error_code=1, SUCCESS
```

MoveIt 日志确认：

```text
Trajectory completed in 2.704s; dry_run=True
Controller 'dual_arm_controller' successfully finished
```

## 12. 剩余工作

- [ ] 夹爪选型后增加左右夹爪规划组。
- [x] 真实 NERO 左臂执行验证。
- [ ] 真实执行前确认 `speed_percent=5`、工作空间无人、软件急停可操作。
- [x] 真实执行时桥切换到 `dry_run:=false`。
- [ ] 验证取消轨迹时左右控制门关闭并触发软件急停。


## 13. 左臂真实 MoveIt 执行

测试配置：

```text
规划组: left_arm
joint7 目标增量: +0.05 rad
速度百分比: 5%
MoveIt 速度缩放: 0.05
右臂: 不动
dry_run: false
```

实测：

```text
left_joint7 start:   0.160745 rad
left_joint7 target:  0.210745 rad
left_joint7 end:     0.216752 rad
actual delta:        0.056008 rad
MoveIt error_code:   1 (SUCCESS)
trajectory time:     3.355 s
left disable:        True
right disable:       True
```

执行日志确认：

```text
External control gate opened
Trajectory completed in 3.355s; dry_run=False
Controller 'dual_arm_controller' successfully finished
External control gate closed
```

## 14. 下一步

- [ ] 双臂 `both_arms` 真实低速执行。
- [ ] 真实轨迹取消时验证软件急停和两侧控制门关闭。
- [ ] 夹爪选型后增加左右夹爪规划组和真实 TCP。
- [ ] 相机 Hub 到位后恢复三机标定。


## 15. 双臂真实执行与返回支撑位测试

测试配置：

```text
规划组: both_arms
左 joint7 目标增量: +0.03 rad
右 joint7 目标增量: -0.03 rad
速度: 5%
MoveIt 速度缩放: 0.05
```

轨迹执行：

```text
主轨迹 error_code:   1 (SUCCESS)
返回轨迹 error_code: 1 (SUCCESS)
```

返回后误差：

```text
support pose 最大允许误差: 0.03 rad
实际最大误差:              0.058137 rad
AT_SUPPORT:                False
```

安全处理：

```text
LEFT_EMERGENCY_STOP:   True
RIGHT_EMERGENCY_STOP:  True
LEFT_DISABLE:          未执行
RIGHT_DISABLE:         未执行
当前左右电机:          enabled（保持位置）
```

结论：返回轨迹虽然执行成功，但实际姿态未进入支撑位容差，因此**没有失能电机**。安全联锁按预期工作。

当前需要用只读监控驱动保持运行，现场确认机械臂是否已经由支撑结构承重。确认安全后才能失能电机。

支撑位快照：

```text
dual_nero_deployment/config/support_pose.yaml
```

## 16. 标准支撑位姿

文件：

```text
dual_nero_deployment/config/support_pose.yaml
```

生成方法：

- 以当前左右臂实际停靠姿态为参考。
- `joint1` 左右对称化为 `∓π/2`。
- `joint2..joint7` 取左右测量平均值，清理微小不对称。
- 写入 MoveIt 的 `left_arm/support`、`right_arm/support`、`both_arms/support`。
- 通过 MoveIt `check_state_validity`：`valid=True, contacts=0`。

标准值：

```text
left_joint1:  -1.570796327
left_joint2:  -1.739692000
left_joint3:  -0.042874000
left_joint4:   2.137226000
left_joint5:  -0.013753000
left_joint6:  -0.001231000
left_joint7:   0.123124000

right_joint1:  1.570796327
right_joint2: -1.739692000
right_joint3: -0.042874000
right_joint4:  2.137226000
right_joint5: -0.013753000
right_joint6: -0.001231000
right_joint7:  0.123124000
```

用途：

- RViz 中作为 `Goal State -> support`。
- 真实执行结束后作为返回目标。
- 取消或异常轨迹的安全返回目标。
- 仅当实际关节误差进入阈值后才允许急停失能。

## 17. 返回标准支撑位时右臂反馈中断

左臂使用标准支撑位目标后已到达：

```text
left final:
[-1.570814, -1.739692, -0.042865, 2.136946,
 -0.013753, -0.001222, 0.123063]
```

右臂未执行命令，原因：

```text
right.agx_arm_ctrl_single_node: Agx_arm is not connected
right.agx_arm_ctrl_single_node: Agx_arm feedback lost
```

CAN 检查：

```text
can_right: UP, ERROR-ACTIVE
candump can_right: 无任何帧
can_left: 正常持续帧
```

安全状态：

- 左臂：enabled，保持标准支撑位。
- 右臂：反馈不可用，未发送失能命令。
- 左右电机：均未失能。
- 只读左臂监控驱动仍在运行。
- 禁止在右臂无反馈时执行返回、急停失能或断电操作。

恢复步骤：

1. 检查右臂控制器电源。
2. 检查右臂到 can_right 适配器的 CAN 线。
3. 必要时重新上电右臂。
4. 恢复后用 `check_nero_feedback.sh can_right` 验证。
5. 再执行右臂返回标准支撑位；左右均满足误差阈值后才允许急停失能。

## 18. 右臂无 CAN 帧隔离结果

右 USB-CAN 适配器已单独 detach/attach：

```text
Windows BUSID: 1-2
Linux serial:  002200384648570B20303731
can_right:     UP, 1 Mbit/s, ERROR-ACTIVE
```

但：

```text
candump can_right: 无任何帧
RX packets: 0
```

说明：

- 右适配器、USB/IP 和 Linux gs_usb 已恢复。
- 右 NERO 控制器没有向 CAN 总线发送数据。
- 左臂 can_left 仍正常。
- 左臂保持在标准支撑位并使能。
- 右臂未执行任何失能命令。

处理要求：

1. 先确认右臂已由支撑结构承重。
2. 检查右臂控制器电源、指示灯和 CAN 线。
3. 在确保支撑安全的前提下重新上电右臂。
4. 恢复后重新执行 `candump can_right` 和右臂反馈检查。

## 19. 右适配器 loopback 失败

右臂重新上电后仍无 CAN 帧，右臂只读驱动主动查询也超时。

对 `can_right` 执行内部 loopback：

```text
can_right: UP, LOOPBACK
cansend: 无回显
TX errors: 3
TX dropped: 5
```

说明右 USB-CAN 适配器/USB 会话当前不能正常完成收发。左适配器仍正常，左臂保持在标准支撑位。

下一步：

1. 物理拔插右 USB-CAN 适配器，序列号 `002200384648570B20303731`。
2. 不要触碰左 USB-CAN 适配器和左臂。
3. 重新直通后再次 loopback 和 candump。
4. 若仍失败，更换右 USB-CAN 适配器或 USB 线。

## 20. 安全落位完成

右 USB-CAN 适配器物理拔插后恢复：`can_right` 收到 NERO 周期帧，右臂只读反馈恢复。右臂重新上电后 7 个电机为 `disabled`，实际姿态接近标准支撑位。

左臂最终检查：

```text
left current:
[-1.570657, -1.739692, -0.042848, 2.136894,
 -0.013753, -0.001222, 0.123046]

max support error: 0.000332 rad
LEFT_EMERGENCY_STOP: True
LEFT_DISABLE: True
RIGHT_MOTOR_STATUS: 7 x disabled
```

处理后已关闭左右只读监控驱动，无 MoveIt、RViz 或轨迹桥进程残留。

当前安全状态：

- 左臂：软件急停后已失能。
- 右臂：已失能。
- `can_left`、`can_right`：UP。
- 标准支撑位姿已保存并加入 MoveIt。

## 21. 标准支撑位安全落位通过

右适配器物理拔插并重新直通后，`can_right` 恢复周期帧，右臂反馈恢复。

最终标准支撑位对齐结果：

```text
left final:
[-1.570831, -1.739587, -0.042848, 2.136894,
 -0.013753, -0.001222, 0.123116]

right final:
[1.570796, -1.739674, -0.042813, 2.137156,
 -0.013753, -0.001239, 0.123150]

maximum support error: 0.000454 rad
AT_SUPPORT: True
```

安全流程执行结果：

```text
LEFT_EMERGENCY_STOP:  True
RIGHT_EMERGENCY_STOP: True
LEFT_DISABLE:         True
RIGHT_DISABLE:        True
```

当前安全状态：

- 左右臂均失能。
- 左右臂均位于标准支撑位。
- 无驱动、MoveIt、轨迹桥或 RViz 控制进程运行。
- 标准支撑流程已验证。

## 22. 轨迹取消 dry-run

测试：

```text
轨迹时长: 5 s
取消时间: 0.5 s
dry_run: true
```

第一次测试发现默认单线程执行器在轨迹回调阻塞时无法处理取消请求，轨迹仍执行到 5 秒。已修复：

- 轨迹桥改用 `MultiThreadedExecutor(num_threads=4)`。
- 服务等待不再嵌套 `spin_until_future_complete`。
- 取消请求可并发处理。

复测结果：

```text
CANCEL_RESPONSE: True
GOAL_STATUS:     5 (CANCELED)
ERROR_CODE:      -4
```

日志：

```text
Trajectory cancelled; dry_run=True
```

说明取消链路在 dry-run 下有效，且没有向真机发送命令。

真实轨迹中途取消仍需在后续真机安全测试中验证。
