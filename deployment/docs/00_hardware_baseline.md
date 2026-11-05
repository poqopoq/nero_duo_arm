# 硬件与系统基线

> 更新：2026-11-04  
> 环境：原生 Ubuntu 22.04.5 + ROS 2 Humble + Python 3.10  
> 工作区：`/home/work/nero_duo_arm`

## 1. 主机与工具链

| 项目 | 当前值 |
| --- | --- |
| 内核 | `6.8.0-138-generic` |
| 架构 | `x86_64` |
| CPU / 内存 | 16 逻辑核 / 15 GiB |
| ROS | `ROS_DISTRO=humble` |
| Python | `/usr/bin/python3` 3.10.12 |
| 关键工具 | `colcon`、`rosdep`、`xacro`、`can-utils`、`v4l2-ctl` |

## 2. CAN 固定映射

| 逻辑接口 | 适配器序列号 | 用途 | 速率 |
| --- | --- | --- | --- |
| `can_left` | `004500404648571620303731` | 左 NERO | 1,000,000 bit/s |
| `can_right` | `002200384648570B20303731` | 右 NERO | 1,000,000 bit/s |

不要依赖 `can0/can1` 枚举顺序。接口缺失时重新执行稳定命名脚本并重新插拔：

```bash
cd /home/work/nero_duo_arm
sudo ./deployment/scripts/install_can_stable_names.sh
sudo ./deployment/scripts/enable_can.sh can_left 1000000
sudo ./deployment/scripts/enable_can.sh can_right 1000000
ip -details link show can_left
ip -details link show can_right
```

## 3. 相机固定映射

| 角色 | 型号 | 序列号 | 当前配置 |
| --- | --- | --- | --- |
| `camera_scene` | D435 | `263422075415` | Color `640×480@30`，Depth 默认关闭 |
| `camera_left_wrist` | D405 | `260522270991` | Color `640×480@30`，Depth 默认关闭 |
| `camera_right_wrist` | D405 | `261122272091` | Color `640×480@30`，Depth 默认关闭 |

三路 Color 同时发布 JPEG 压缩流，例如
`/camera_scene/color/image_raw/compressed`。VLA/LeRobot 默认只消费 Color。

```bash
./deployment/scripts/check_realsense.sh
ros2 topic hz /camera_scene/color/image_raw
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic hz /camera_right_wrist/color/image_raw
```

## 4. 机械与坐标参数

```text
world: 桌面平面、左右基座中心中点；+X 指向桌内，+Y 指向左，+Z 向上
table front edge: world.x = -0.06 m
left base:  xyz = [0, +0.35, 0], yaw = pi
right base: xyz = [0, -0.35, 0], yaw = pi
基座中心距: 0.70 m
TCP: tool0 沿工具轴 +0.1708 m，到 CTAG2F90D 两指夹持中心
```

模型检查：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_description display.launch.py
```

## 5. 上电前检查

- 确认左右 CAN 均为 `state UP`，且 `candump` 有周期帧。
- 确认三台相机序列号和话题正确，USB 3.x 设备速率为 `5000` 或更高。
- 确认工作区无人、机械臂有支撑、软件急停入口可用。
- 首次启动使用 `moveit_dry_run:=true`、`auto_enable:=false`。

## 6. 仍待现场确认

- D435 更换 USB 3.x 线后的正式 10 分钟稳定性。
- 三台相机最终安装位置和线缆余量。
- 夹爪实物 TCP 偏移标定。
- 桌面尺寸、基座位置与朝向的最终签字记录。
