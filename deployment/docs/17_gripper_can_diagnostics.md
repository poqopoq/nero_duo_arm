# CTAG2F90-D 双夹爪与 ZQWL-CANFD

> 左右各一只 CRT CTAG2F90-D 两指夹爪，经 ZQWL `UCANFD-100E` 单通道
> USB-CANFD 连接。默认 `allow_motion=false`，只读反馈。

## 1. 最简开合

直接通过 CAN 控制，不需要启动 ROS。先停止占用同一串口的夹爪 ROS 节点：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/gripper.sh right open
./deployment/scripts/gripper.sh right close
```

左夹爪把 `right` 改成 `left`。`open` 发送位置 `0`，`close` 发送安全闭合上限
`970`；脚本自动使能、写目标、读取反馈，退出前失能。

调整速度（`1..100%`）：

```bash
./deployment/scripts/gripper.sh right open --speed 10
./deployment/scripts/gripper.sh right close --speed 10
```

参数保存在 `deployment/config/gripper_can.json`：

```json
{
  "left":  {"speed": 25, "max_current": 200},
  "right": {"speed": 25, "max_current": 200}
}
```

命令行 `--max-current` 只覆盖本次命令，不改配置文件。脚本每次运行会临时写入
`0x0533`，回读确认后运动，退出时恢复控制器原值。`0x0533` 生效约有 1 秒延迟。

保持电机和 CAN 通道使能（调试用）：

```bash
./deployment/scripts/gripper.sh right open --keep-enabled
```

该模式退出时仍恢复 `0x0533`，但不会自动失能；调试结束必须手动停止并使能状态
恢复，不能长期堵转。

## 2. 设备映射

以 `/dev/serial/by-id/` 为准，`/dev/ttyACM*` 重新插拔后可能变化。

| 侧别 | 稳定串口 | 本次枚举 | 关节 |
| --- | --- | --- | --- |
| 左 | `usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00` | `/dev/ttyACM1` | `left_ctag_left_1_joint` |
| 右 | `usb-ZQWL-CANFD_ZQWL-CANFD_6A649697220A-if00` | `/dev/ttyACM0` | `right_ctag_left_1_joint` |

默认通信参数：CAN FD，仲裁域 `1 Mbps`，数据域 `5 Mbps`，寄存器 `0x0A0A = 0x0503`。
厂商 ROS 示例使用经典 CAN `1M` 兼容模式；两种模式当前都已通过只读验证。

## 3. 协议

29-bit 扩展帧 ID：

```text
bit0..6   设备 ID（默认 1）
bit7      读写标志（1=写，0=读）
bit8..23  寄存器地址
```

设备 ID=1 时：

| 操作 | CAN ID | 数据 | 说明 |
| --- | --- | --- | --- |
| 使能 | `0x00010081` | `01 00` / `00 00` | `0x0100` |
| 目标位置 | `0x00010281` | high 2B + low 2B，小端 | `0x0102/0x0103` |
| 触发运动 | `0x00010881` | `01 00` | `0x0108` |
| 读实际位置 | `0x00041801` | `00 00 00 00` | `0x0418/0x0419` |

`position=1000` 编码为 `00 00 E8 03`；反馈按同样顺序解码为
`(high << 16) | low`。

厂商寄存器表把 `0x0101` 的速度/力矩/力控模式标为“未实现”。当前提供原始
电流、电机状态和故障遥测，但没有电流到牛顿值的标定，也没有闭环力控。

## 4. 主机权限

用户已加入 `dialout` 但当前 shell 未刷新时：

```bash
newgrp dialout
```

或使用包装脚本自动切换：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/run_in_dialout.sh \
  ros2 launch dual_nero_control left_gripper.launch.py
```

## 5. 构建与测试

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dual_nero_control dual_nero_moveit_config
source install/setup.bash
```

协议、帧编码、运动学映射和 PTY 传输测试：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/test_gripper_control.sh
```

只读启动：

```bash
ros2 launch dual_nero_control dual_grippers.launch.py
```

或分别启动：

```bash
ros2 launch dual_nero_control left_gripper.launch.py
ros2 launch dual_nero_control right_gripper.launch.py
```

检查接口：

```bash
ros2 action list | grep gripper
ros2 topic list | grep gripper
ros2 service list | grep -E 'gripper/(enable|emergency_stop|clear_fault|clear_obstruction)'
ros2 topic hz /left_gripper/position
ros2 topic hz /right_gripper/position
ros2 topic hz /left_gripper/status
ros2 topic hz /right_gripper/status
ros2 topic echo --once /right_gripper/status
```

`allow_motion=false` 时请求使能应被拒绝，不会写 CAN 使能/位置/触发帧：

```bash
ros2 service call /left_gripper/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /right_gripper/enable std_srvs/srv/SetBool "{data: true}"
```

## 6. 驱动接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/<side>_gripper/position` | `std_msgs/Int32` | 原始反馈，0=张开、1000=机械全闭 |
| `/<side>_gripper/width` | `std_msgs/Float32` | 指腹开口，米 |
| `/<side>_gripper/state` | `sensor_msgs/JointState` | 0=张开，约 `-0.90017`=命令闭合上限 |
| `/<side>_gripper/status` | `diagnostic_msgs/DiagnosticArray` | 约 20 Hz，含 enabled/fault/stall/电流/温度 |
| `/<side>_gripper/position_cmd` | `std_msgs/Int32` | 手动位置命令 `0..990` |
| `/<side>_gripper/enable` | `std_srvs/SetBool` | 使能/失能 |
| `/<side>_gripper/emergency_stop` | `std_srvs/Empty` | 锁存故障并写失能 |
| `/<side>_gripper/clear_fault` | `std_srvs/Trigger` | 复位 `0x0110` 报警并清锁存 |
| `/<side>_gripper/clear_obstruction` | `std_srvs/Trigger` | 停止、失能、复位、使能并张开 |
| `/<side>_gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | MoveIt 夹爪接口 |

常用参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `allow_motion` | `false` | 总运动门 |
| `enable_motor` | `false` | 启动时是否使能 |
| `goal_timeout_sec` | `10.0` | 位置命令反馈超时 |
| `feedback_timeout_sec` | `1.0` | 反馈新鲜度超时 |
| `latch_motion_timeout` | `true` | 超时锁存并失能 |
| `cancel_disables_motor` | `true` | Action 取消后失能 |
| `position_tolerance` | `25` | 到达容差 |
| `command_max` | `990` | 闭合命令上限 |
| `treat_closed_stall_as_grasp` | `true` | 闭合接触按夹持结束 |
| `motion_speed_percent` | `25` | `0x0104` 速度 `1..100` |
| `motion_torque_percent` | `100` | `0x0105` 力矩上限 `1..100` |
| `motion_ramp_ms` | `200` | `0x0106` 斜坡 `0..1000 ms` |
| `max_current` | `200` | `0x0533` 最大电流 `1..1000` |

运行期修改：

```bash
ros2 param set /right_gripper_node motion_speed_percent 20
ros2 param set /right_gripper_node motion_ramp_ms 300
ros2 param set /right_gripper_node max_current 250
```

`max_current=200` 仍堵转时按 50 逐级提高，先短行程验证电流和温升，不要直接
升到 1000。

## 7. 故障与清障

以下情况锁存故障并写失能：

- 使能后反馈超时；
- 张开方向在 `goal_timeout_sec` 内未到达；
- Action 取消；
- 位置/触发 CAN 写失败；
- 反馈位置超限；
- 调用 `emergency_stop`。

闭合方向若由 `0x0401` 确认“零速 + 力矩到达”，按夹持结束：
`motion_stalled=true`、保持 `enabled`、不锁存 fault。该策略由
`treat_closed_stall_as_grasp` 控制。

清除故障要求电机已失能且反馈新鲜：

```bash
ros2 service call /right_gripper/emergency_stop std_srvs/srv/Empty
ros2 service call /right_gripper/enable std_srvs/srv/SetBool "{data: false}"
ros2 service call /right_gripper/clear_fault std_srvs/srv/Trigger
ros2 service call /right_gripper/enable std_srvs/srv/SetBool "{data: true}"
```

最后一条只有 `allow_motion:=true` 时才成功。清故障不会自动使能或运动。

Web 的 `Recover arm` 只复位 NERO 机械臂，不清夹爪故障；夹爪堵转使用该侧
`Clear obstruction`，流程为停止 → 失能 → 复位硬件报警 → 使能 → 张开。

## 8. 真实开合验收

左右分别验收，先只开运动权限、保持电机失能：

```bash
ros2 launch dual_nero_control left_gripper.launch.py \
  allow_motion:=true enable_motor:=false

ros2 launch dual_nero_control right_gripper.launch.py \
  allow_motion:=true enable_motor:=false
```

确认反馈稳定在约 20 Hz 后使能：

```bash
ros2 topic hz /left_gripper/position
ros2 service call /left_gripper/enable std_srvs/srv/SetBool "{data: true}"
```

小幅单次命令：

```bash
ros2 topic pub --once /right_gripper/position_cmd std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /right_gripper/position_cmd std_msgs/msg/Int32 "{data: 100}"
```

每次只发一条，确认宽度和关节反馈变化后再发下一条。完成后回张开位并失能：

```bash
ros2 service call /right_gripper/enable std_srvs/srv/SetBool "{data: false}"
```

`GripperCommand` 的 `position` 是关节弧度，不是 `0..990`。允许下限约
`-0.90017 rad`；更低目标会被拒绝。

```bash
ros2 service call /right_gripper/enable std_srvs/srv/SetBool "{data: true}"
ros2 action send_goal /right_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: -0.20, max_effort: 0.0}}"
ros2 service call /right_gripper/enable std_srvs/srv/SetBool "{data: false}"
```

## 9. 10×10 与急停验收

先启动允许运动的驱动，不要在 Web 已启动同一驱动时重复启动：

```bash
source /opt/ros/humble/setup.bash
source /home/work/nero_duo_arm/ros2_ws/install/setup.bash
ros2 launch dual_nero_control dual_grippers.launch.py allow_motion:=true
```

10 个位置 × 10 个速度，两侧共 200 组：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/test_gripper_sweep.sh --sides both
```

自定义范围：

```bash
./deployment/scripts/test_gripper_sweep.sh \
  --sides right \
  --positions 0,110,220,330,440,550,660,770,880,990 \
  --speeds 10,20,30,40,50,60,70,80,90,100 \
  --position-tolerance 25 \
  --timeout-sec 10 \
  --csv /tmp/right_gripper_sweep.csv
```

急停验收：

```bash
./deployment/scripts/test_gripper_estop.sh \
  --sides both --speed 10 --trigger-travel 40 \
  --post-stop-window 1.5 --max-stop-travel 20 \
  --csv deployment/acceptance/gripper_estop_20261104.csv
```

脚本确认实际位移后调用 `/<side>_gripper/emergency_stop`，验证故障锁存、
拒绝再次使能、停止后位移不超过 `20` 计数、`clear_fault` 可清除且电机保持
失能，最后恢复运动并返回起始位置。

## 10. MoveIt 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `gripper` / `left_gripper` | `false` | 是否启动右/左夹爪驱动 |
| `gripper_allow_motion` / `left_gripper_allow_motion` | `false` | 是否允许真实使能/位置写入 |
| `gripper_enable_motor` / `left_gripper_enable_motor` | `false` | 启动时是否使能 |
| `gripper_port` / `left_gripper_port` | 现场 by-id | 左右 ZQWL 串口 |
| `gripper_can_fd` / `gripper_brs` | `false` | 右夹爪 CAN FD/BRS |
| `left_gripper_can_fd` / `left_gripper_brs` | `false` | 左夹爪 CAN FD/BRS |
| `gripper_command_max` | `990` | 闭合命令上限 |
| `gripper_max_current` | `200` | `0x0533` 最大电流 |

只读：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true rviz:=true \
  gripper:=true left_gripper:=true \
  gripper_allow_motion:=false left_gripper_allow_motion:=false
```

完成物理开合验收后：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=false rviz:=true \
  gripper:=true left_gripper:=true \
  gripper_allow_motion:=true left_gripper_allow_motion:=true \
  gripper_enable_motor:=true left_gripper_enable_motor:=true
```

## 11. 实物 TCP 标定

`tcp_offset` 当前默认值来自 URDF 和厂商几何，不等于实物夹爪的最终夹持中心。
更换夹爪、拆装指爪或发现 `check_tcp_contract.py` 与实际抓取点偏差后，必须做
枢轴（pivot）标定。

准备：

1. 驱动、`robot_state_publisher` 和 MoveIt/URDF 已启动；
2. 确认 `world -> <side>_link7` TF 连续可用；
3. 工作区清空，机械臂低速、有人监护；
4. 选一个固定枢轴点。枢轴点必须代表最终抓取点：可直接用闭合指爪尖，或在
   两指之间夹一根刚性指针，并把指针长度从结果中扣除。

确认 TF：

```bash
ros2 run tf2_ros tf2_echo world right_link7
```

运行标定（脚本只读 TF，不控制机械臂）：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dual_nero_calibration calibrate_gripper_tcp \
  --side right --samples 12 --strict
```

每个位姿保持枢轴点不动，只改变工具姿态；按 Enter 采样。脚本会拒绝与已有姿态
旋转差小于 `5 deg` 的样本，并要求总旋转跨度至少 `15 deg`。建议至少 8 个位姿，
覆盖两个以上非平行旋转轴。完成后输出：

```text
OUTPUT .../deployment/calibration_results/gripper_tcp_right.yaml
TCP_XYZ_LINK7_M [...]
DRIVER_TCP_OFFSET [...]
URDF_ARGS {'right_tcp_x': ..., 'right_tcp_y': ..., 'right_tcp_z': ...}
PIVOT_RMS_MM 0.8
CALIBRATION PASS
```

验收阈值：枢轴 RMS `< 2 mm`，旋转跨度 `>= 15 deg`，条件数 `< 1e5`。
结果同时给出两组必须同步修改的值：

- 驱动 `dual_nero_drivers.launch.py` 的 `DEFAULT_TCP_OFFSET` /
  launch 参数 `tcp_offset`；
- `dual_nero_description/urdf/two_nero_tabletop.urdf.xacro` 的
  `*_tcp_x/y/z` 默认值（Web 3D 和 MoveIt 使用同一套 URDF）。

修改后重新启动驱动和 MoveIt，并运行：

```bash
/usr/bin/python3 deployment/scripts/check_tcp_contract.py
```

只有 `tcp_pose` 与 MoveIt FK 一致且实际抓取点复测通过，才把标定结果用于 VLA
或自动抓取。左右夹爪分别标定，不要把一侧结果直接复制到另一侧。

## 12. 夹爪开度标定

`/<side>_gripper/width` 默认用线性模型把原始位置 `0..1000` 映射到 `0..90 mm`。
指爪、垫片或夹持物体改变后，用卡尺在至少两个相距 `100` 计数的位置测量实际
指腹开口。

先低速移动到目标位置，等待 `/position` 稳定，再用卡尺测量：

```bash
ros2 topic pub --once /right_gripper/position_cmd std_msgs/msg/Int32 "{data: 250}"
ros2 topic echo --once /right_gripper/position
# 卡尺读数，例如 67.2 mm
```

记录多组 `原始位置=开口毫米`，拟合并生成驱动参数：

```bash
ros2 run dual_nero_calibration calibrate_gripper_opening \
  --side right \
  --measurement 0=89.4 \
  --measurement 250=67.2 \
  --measurement 500=45.1 \
  --measurement 750=22.7
```

输出 YAML 包含 `opening_min_mm`、`opening_max_mm`、线性斜率和拟合残差。把
`node_parameters` 中四个值写入该侧夹爪 launch/参数文件：

```text
position_min
position_max
opening_min_mm
opening_max_mm
```

拟合 RMS 大于 `1 mm` 时不要直接套用线性模型；先检查指爪是否松动、机械零位
是否变化，或把工作区间分段标定。开度标定只修正宽度反馈，不改变 CAN 原始
`0..1000` 位置含义。

## 13. 已知限制

- 位置开环命令 + 位置反馈确认；速度、斜坡和力矩上限使用厂商寄存器；有原始
  电流/温度/故障遥测，但没有校准力值或闭环力控。
- 闭合接触后默认保持使能，`motion_stalled=true` 时仍可能持续施力升温；长时
  夹持必须监视 `user_current_raw`、`temperature_c` 和 `drop_alarm`。
- `motion_torque_percent` 是百分比限幅，不是 N 为单位的 `max_effort`。
- `GripperCommand` 取消会写失能并锁存故障，但已发出的 CAN 位置/触发命令
  无法撤回。
- 夹爪没有物理急停；ROS/Web/VLA 软件急停会写失能，但不能替代物理断电。
- 不要随意写 `0x0A0A` 改波特率；先用只读请求确认当前 `1M/5M` 或经典 `1M`。
