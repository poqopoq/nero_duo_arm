# OmniPicker 3 夹爪与 CAN FD

> 当前默认驱动为 AGILINK OmniPicker 3 (OP3)。CAN 协议依据 AGIBOT
> [OmniPicker 产品手册](https://www.agibot.com/DOCS/OS/omnipicker-manual)
> 第 6.3 节，并参考
> [AgibotTech/agilink_omnihand_sdk](https://github.com/AgibotTech/agilink_omnihand_sdk)
> 的 OP3 ROS 2 文档；官方 URDF/网格来自产品手册附录。旧版 CTAG2F90-D 驱动仍保留为
> `ctag_left_gripper_node` / `ctag_right_gripper_node`。

官方模型文件已经复制到 `dual_nero_description/meshes/omnipicker3/`，来源记录
见 `meshes/omnipicker3/SOURCE.md`。

## 1. 硬件与 CAN FD 参数

- 固件必须支持 CAN，官网要求 CAN 协议固件不低于 `3.3.0`。
- OP3 默认通信模式是 CAN/CAN-FD；使用版本 30 的 DIP 开关时，引脚 3、4
  打开，其余关闭。
- CAN FD 仲裁域：`1 Mbps`，采样点 `80%`。
- CAN FD 数据域：`5 Mbps`，采样点 `75%`。
- 控制帧使用 8 字节数据和标准 CAN ID；ID 对应设备的 `can_node_id`，默认
  `1`。同一 CAN 网段中的左右夹爪必须配置为不同 ID。

本项目支持两种主机侧连接：

| `transport` | 用途 | 地址参数 |
|---|---|---|
| `zqwl` | ZQWL CDC-ACM CAN FD 适配器，节点自行配置 1M/5M | `port` |
| `socketcan` | ZLG、HCAN、PEAK 等提供 SocketCAN 接口的适配器 | `can_interface` |

### 1.1 ZQWL 串口权限

```bash
sudo usermod -aG dialout "$USER"
newgrp dialout
```

节点和 `gripper_can.py` 都会按现有 ZQWL 协议自动配置 CAN FD。不要同时运行
旧 CTAG 节点和新 OP3 节点，因为两者会争用同一个 CDC-ACM 设备。

### 1.2 SocketCAN 配置

节点只使用已经配置好的 SocketCAN 接口，不调用 `sudo`。每次 USB 适配器
重新枚举或接口重建后执行：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can \
  fd on bitrate 1000000 dbitrate 5000000 \
  sample-point 0.8 dsample-point 0.75
sudo ip link set can0 up
ip -details link show can0
```

左右夹爪在独立接口时通常使用 `can0`、`can1`。

## 2. 直接控制

`gripper.sh` 不依赖 ROS，适合首次接线和 CAN FD 排查。先停止占用适配器的
ROS 节点。

ZQWL：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/gripper.sh right open
./deployment/scripts/gripper.sh right close
./deployment/scripts/gripper.sh right position --target 500
```

SocketCAN：

```bash
./deployment/scripts/gripper.sh right open \
  --transport socketcan --interface can1
```

常用参数：

```bash
./deployment/scripts/gripper.sh right open \
  --speed 30 --force 60 --acceleration 60 --deceleration 60
```

`0` 表示张开，`1000` 表示闭合。脚本按官方 8 位位置字段换算，并默认发送
CAN FD + BRS。旧固件需要经典 CAN 兼容模式时使用 `--classic-can`。

只读状态不要求运动：

```bash
./deployment/scripts/gripper.sh right status \
  --transport socketcan --interface can1
```

节点还会按 `feedback_rate` 发送 OP3 SDK 使用的只读位置查询
`0x01130100 | can_node_id`。因此 `allow_motion:=false` 时位置、开度和诊断仍会更新。
官方简化状态帧则在收到控制命令后上报故障和运动状态。

## 3. ROS 2 启动

单只夹爪：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch dual_nero_control right_gripper.launch.py \
  allow_motion:=false
```

确认 CAN、ID 和反馈链路后，再显式允许运动：

```bash
ros2 launch dual_nero_control right_gripper.launch.py \
  allow_motion:=true
```

SocketCAN 示例：

```bash
ros2 launch dual_nero_control right_gripper.launch.py \
  transport:=socketcan can_interface:=can1 allow_motion:=true
```

双臂夹爪：

```bash
ros2 launch dual_nero_control dual_grippers.launch.py \
  allow_motion:=false
```

MoveIt：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false gripper:=true left_gripper:=true \
  gripper_allow_motion:=true left_gripper_allow_motion:=true
```

左右 ID 不重复时，MoveIt 启动参数也可以分别设置：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  gripper:=true left_gripper:=true \
  gripper_allow_motion:=true left_gripper_allow_motion:=true
```

当前 MoveIt 使用 `*_op3_wide1_joint` 作为唯一主动关节，其余指节通过 mimic
跟随。模型网格来自官网 URDF，闭合外形约 `88.1 × 153 mm`，张开外形约
`125 × 143 mm`；`wide1` 的 `0.39 rad` 对应张开姿态。网格到捏合点的
`0.130 m` 和传感器栈后的 `0.1725 m` 是初始估计，仍需枢轴标定确认。

## 4. ROS 2 接口

接口与旧夹爪保持兼容：

| 名称 | 类型 | 说明 |
|---|---|---|
| `/<side>_gripper/position_cmd` | `std_msgs/Int32` | `0=张开`，`1000=闭合`；会限制到 `command_max` |
| `/<side>_gripper/speed_cmd` | `std_msgs/Int32` | 速度百分比 `1..100` |
| `/<side>_gripper/position` | `std_msgs/Int32` | 上游位置换算到 `0..1000` |
| `/<side>_gripper/width` | `std_msgs/Float32` | 线性开度估计，配置的最大开度默认 120 mm |
| `/<side>_gripper/state` | `sensor_msgs/JointState` | 关节状态 |
| `/joint_states` | `sensor_msgs/JointState` | MoveIt/RViz 使用的共享状态 |
| `/<side>_gripper/status` | `diagnostic_msgs/DiagnosticArray` | 状态、故障、协议字节和反馈年龄 |
| `/<side>_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | MoveIt 夹爪动作 |
| `/<side>_gripper/enable` | `std_srvs/SetBool` | 启用或停止逻辑命令流 |
| `/<side>_gripper/emergency_stop` | `std_srvs/Empty` | 锁存软件急停并停止命令流 |
| `/<side>_gripper/clear_fault` | `std_srvs/Trigger` | 清除软件故障/急停锁存 |
| `/<side>_gripper/clear_obstruction` | `std_srvs/Trigger` | 向张开方向移动 120 counts |

运动参数：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `allow_motion` | `false` | 第一层运动许可 |
| `can_fd` | `true` | 发送 CAN FD 帧 |
| `brs` | `true` | 数据相位使用 5 Mbps |
| `feedback_rate` | `20 Hz` | 控制/状态刷新率 |
| `feedback_timeout_sec` | `1.0` | 运动中反馈超时 |
| `motion_speed_percent` | `50` | 官方 `Vel Cmd` 百分比 |
| `motion_force_percent` | `100` | 官方 `Force Cmd` 百分比 |
| `motion_acceleration_percent` | `100` | 官方 `Acc Cmd` 百分比 |
| `motion_deceleration_percent` | `100` | 官方 `Dec Cmd` 百分比 |
| `position_tolerance` | `8` | `0..1000` 位置容差 |
| `joint_name` | `<side>_op3_wide1_joint` | URDF/MoveIt 主关节 |
| `joint_closed_rad` / `joint_open_rad` | `0.0` / `0.39` | 官方网格闭合/张开关节角 |
| `parallel_gripper_width_mm` | `120` | `width` 话题的标称开度上限 |

`Force Cmd` 是厂商协议的力/力矩百分比，不是 N 或 Nm 的闭环传感器读数。
触觉数据和 SDK 的毫米行程/gentle-grip 扩展接口没有接入这个简化协议。

## 5. 官方协议摘要

下表来自官网 OmniPicker 手册第 6.3 节。

下位控制帧，CAN ID = `can_node_id`：

| D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 |
|---|---|---|---|---|---|---|---|
| 0 | Pos | Vel | Force | Acc | Dec | 0 | 0 |

位置 `0` 为闭合、`0xFF` 为完全张开；其余字节 `0..0xFF` 映射最大值百分比。
50% 位置的完整帧示例：

```text
ID=1  DATA=00 7F FF FF FF FF 00 00
```

上位状态帧：

| D0 | D1 | D2 | D3 | D4 | D5-D7 |
|---|---|---|---|---|---|
| Fault | State | Pos | Vel | Force | 原控制帧 D5-D7 |

故障码：`0 none`、`1 overheat`、`2 overspeed`、`3 initialization`、
`4 limit`。状态码：`0 reached`、`1 moving`、`2 stalled`、`3 object dropped`。

## 6. 急停边界

OP3 简化 CAN 协议没有提供“物理失能电机”的确认字段。本节点的
`emergency_stop` 会立即停止下发目标、锁存软件急停并阻止后续运动；它不能
替代硬件急停、24 V 电源切断或已验证的驱动器失能回路。上电后夹爪会按固件
逻辑自动使能并可能自动闭合，首次接线必须清空手指行程。

## 7. 首验顺序

1. 断开机械臂规划，执行 `allow_motion:=false`，确认适配器打开和诊断正常。
2. 核对左右 `can_node_id`，确保 ID 不冲突。
3. 用 `gripper.sh` 做 10% 以内的小幅往返，确认位置字节方向。
4. 低速、低力执行 `open -> close -> open`，记录实际 120 mm 行程中的接触点。
5. 再执行 `test_gripper_estop.py` 和 MoveIt 小幅动作验收。
6. 完成机械几何/TCP 标定后，才允许把夹爪位置纳入抓取策略和自动数据采集。
