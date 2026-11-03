# 阶段 1：官方 NERO ROS 2 驱动安装记录

> 记录日期：2026-10-30  
> 工作空间：`/home/work/nero_duo_arm/ros2_ws`
> ROS：Humble（系统 Python 3.10）  
> 状态：5/5；左臂稳定命名、通信和关节反馈已验证

## 1. 已完成

- [x] 创建 `dual_nero_ws`
- [x] 获取官方 AgileX `agx_arm_ros` ROS 2 分支
  - 仓库：https://github.com/agilexrobotics/agx_arm_ros.git
  - 提交：`286633e0d2edb038ee3a1047d9feb7f614a1c076`
  - 位置：`dual_nero_ws/src/agx_arm_ros`
- [x] 初始化 `agx_arm_urdf` 子模块
  - 提交：`680fa925cd00db7e2628563d2edc5540963045de`
- [x] 安装 `pyAgxArm 1.0.0`、`python-can 4.6.1`
- [x] `colcon build --symlink-install` 成功
- [x] ROS 包可见：`agx_arm_msgs`、`agx_arm_description`、`agx_arm_ctrl`、`agx_arm_moveit`
- [x] 安装用户级 `pip3`，未使用 Conda、未覆盖 ROS 系统包
- [x] 左臂 CAN 以 1,000,000 bit/s 启用并收到 NERO 反馈
- [x] 驱动识别固件 `v121 / 1.21`，发布 `joint1` 至 `joint7`

## 2. 两台 USB-CAN 映射

当前 WSL 通过 USB/IP 接入两台 candleLight：

| Linux 接口 | USB 路径 | 适配器序列号 | 状态 |
| --- | --- | --- | --- |
| `can_left` | 当前 `1-1:1.0` | `004500404648571620303731` | 左臂；1 Mbit/s，反馈正常 |
| `can_right` | 未接入 | `002200384648570B20303731` | 右臂候选；规则已预置 |

物理左右已确认规则：`004500...3731 -> can_left`。该规则已写入 systemd `.link` 文件并通过重新枚举验证。

Windows USB/IP：

| BUSID | 序列号 | 状态 |
| --- | --- | --- |
| `1-2` | `004500404648571620303731` | Attached |
| `6-1` | `002200384648570B20303731` | Attached |

## 3. 已执行：只启用左臂 CAN

```bash
cd /home/work/nero_duo_arm/deployment
sudo ./scripts/enable_can.sh can_left 1000000
```

该脚本只设置 `can_left` 为 1,000,000 bit/s 并启用接口，不启动驱动、不使能机械臂。

## 4. 已通过：只读检查左臂关节反馈

```bash
cd /home/work/nero_duo_arm/deployment
./scripts/check_nero_feedback.sh can_left
```

脚本以：

```text
arm_type:=nero
auto_enable:=false
effector_type:=none
```

启动单个驱动，只等待一次 `/feedback/joint_states`，随后退出。此阶段未发送运动命令。

实测结果：

```text
name: joint1 ... joint7
position: [1.5505, -1.7513, -0.0009, 2.1962, 0.0127, 0.0278, 0.1461]
velocity: [0, 0, 0, 0, 0, 0, 0]
effort: [0, 0, 0, 0, 0, 0, 0]
firmware: v121, 1.21, v121
```

注意：日志中的 `control_enabled: True` 表示驱动已具备控制通道，不等同于电机已经使能；本次参数为 `auto_enable:=false`。

## 5. 阶段 1 剩余退出条件

- [x] 确认物理左臂对应 `can_left`
- [x] 启动候选接口，确认 NERO 反馈
- [x] 记录 NERO 型号、固件版本、关节数量和关节名
- [ ] 验证急停确实切断/禁止运动
- [x] 将左臂映射写入阶段记录

## 6. 注意

USB/IP 设备在 WSL 重启、Windows 重启或适配器重新插拔后可能解除 Attached 状态。恢复时先检查：

```powershell
usbipd list
```

然后重新执行 `usbipd attach --wsl --busid <BUSID>`。阶段 2 应将稳定命名建立在适配器序列号上，而不是枚举顺序上。
