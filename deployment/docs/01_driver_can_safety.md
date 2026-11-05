# 驱动、CAN 与软件急停

> 适用于原生 Ubuntu + ROS 2 Humble。所有真机运动前先读本页安全边界。

## 1. 启用 CAN

```bash
cd /home/work/nero_duo_arm/deployment
sudo ./scripts/enable_can.sh can_left 1000000
sudo ./scripts/enable_can.sh can_right 1000000
ip -details link show can_left
ip -details link show can_right
```

只读检查单侧关节反馈：

```bash
./scripts/check_nero_feedback.sh can_left
./scripts/check_nero_feedback.sh can_right
```

脚本以 `arm_type:=nero auto_enable:=false effector_type:=none` 启动单个驱动，
收到一次 `/feedback/joint_states` 后退出，不发送运动命令。

## 2. 双 NERO namespace

左右驱动参数必须保持：

```text
left:  namespace:=left  can_port:=can_left
right: namespace:=right can_port:=can_right
arm_type:=nero
auto_enable:=false
effector_type:=none
```

验证左右隔离：

```bash
./scripts/stage2_check_dual_namespace.sh
```

预期关键话题：

```text
/left/control/joint_states
/right/control/joint_states
/left/feedback/joint_states
/right/feedback/joint_states
/left/feedback/tcp_pose
/right/feedback/tcp_pose
```

驱动在 `JointState.name` 中仍发布 `joint1..joint7`。namespace 只隔离话题，
不修改关节名；双臂模型和 MoveIt 使用 `left_joint1..7`、`right_joint1..7`，
由描述/桥接层完成前缀转换。

## 3. 软件急停

现场没有物理急停。当前停止路径：

| 路径 | 用途 |
| --- | --- |
| 官方 NERO Web 端急停 | 独立于 ROS 的现场停止入口 |
| `/left/emergency_stop` | 左臂软件急停 |
| `/right/emergency_stop` | 右臂软件急停 |
| `/left/control_enable`、`/right/control_enable` | 关闭控制门，阻止后续命令 |

驱动收到 `emergency_stop` 后读取当前关节角并执行 `move_j(current_positions)`，
让机械臂保持当前位置；它不是硬件动力切断。

双臂联动急停必须同时：

1. 调用 `/left/emergency_stop` 和 `/right/emergency_stop`。
2. 将两侧 `control_enable` 设为 `false`。
3. 锁存急停状态，恢复时重新使能并重新打开控制门。

验证命令：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/stage2_dual_soft_estop_test.sh
```

该脚本以 5% 速度小幅移动两侧 `joint7`，检测到实际位移后同时急停，并检查
关节稳定和 14 个电机最终失能。真机执行前必须清空工作区并人工监护。

## 4. 安全边界

- 软件急停依赖上位机、ROS 节点和 CAN 通信持续可用；CAN 断开、进程崩溃或
  系统死机时不可用。
- 不要同时启动两套驱动、两套 MoveIt 或两个占用 `8080` 的 Web app。
- 首次运行、更换硬件或修改参数后，先使用 `moveit_dry_run:=true`。
- 未完成 dry-run、控制门、急停、Park 和现场安全确认前，不执行真机运动。
- 失能前必须确认机械臂已由支撑结构承重，且关节误差在阈值内。

## 5. 已知验收状态

- 左右 CAN 稳定命名、1 Mbit/s 通信和 7 关节反馈：通过。
- 左右 namespace 隔离：通过。
- 单臂与双臂联动软件急停、急停后 14 个电机失能：通过。
- 官方 Web 端急停链路：仍需现场操作确认。
