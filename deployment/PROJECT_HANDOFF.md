# Dual NERO 项目交接

> 更新：2026-11-04
>
> 环境：原生 Ubuntu 22.04.5 + ROS 2 Humble
> 工作区：`/home/work/nero_duo_arm`

## 1. 项目结构

```text
/home/work/nero_duo_arm/
├── ros2_ws/src/
│   ├── agx_arm_ros/              # AgileX 官方 NERO 驱动
│   ├── dual_nero_description/
│   ├── dual_nero_cameras/
│   ├── dual_nero_control/
│   ├── dual_nero_moveit_config/
│   ├── dual_nero_calibration/
│   └── dual_nero_webapp/
├── dependencies/pyAgxArm/        # 官方 Python SDK
├── references/cc_agx_arm_ros/    # 参考仓库
└── deployment/                   # 文档、配置、验收脚本
```

## 2. 固定映射

```text
左 NERO CAN: 004500404648571620303731 -> can_left, 1 Mbit/s
右 NERO CAN: 002200384648570B20303731 -> can_right, 1 Mbit/s
固件:        v121 / 1.21

左腕 D405: 260522270991
右腕 D405: 261122272091
D435:      263422075415
```

机械坐标：

```text
world: 桌面平面、两基座中心中点，+X 指向桌内，+Y 指向左
left base:  xyz=[0, +0.35, 0], yaw=pi
right base: xyz=[0, -0.35, 0], yaw=pi
基座中心距: 0.70 m
TCP: tool0 沿工具轴 +0.1708 m
```

## 3. 当前状态

已通过：

- 左右 CAN 稳定命名、1 Mbit/s 通信、7 关节反馈。
- 左右 namespace 隔离、单臂/双臂软件急停、14 电机失能。
- 双臂 MoveIt 规划、碰撞检查、轨迹桥 dry-run、单臂/双臂真机低速执行。
- 真实轨迹取消联锁、稳定 park pose 复位与失能验证。
- 三路 Color `640×480@30` 短测和 Web app 端到端接口。
- 夹爪只读反馈、真实开合、MoveIt `GripperCommand` 小幅往返。
- 夹爪空载 10×10 位置/速度 `200/200`、低速软件急停 `4/4`。
- 单臂/双臂 VLA bridge 的 dry-run、IK 干跑和假 action server 测试。

仍未验收：

- 带载抓取、真实堵塞清障、长时热稳定、夹爪 CAN 掉线恢复。
- 夹爪实物 TCP 标定、D435 USB3 下正式 10 分钟稳定性。
- Web/MoveIt 全链路真机联锁、真实 SmolVLA checkpoint 整链路。
- 真实物理急停。

## 4. 安全状态

- 左右臂当前应位于 `deployment/config/support_pose.yaml` 记录的重力稳定
  park pose；`joint2` 下限 `-1.760` 是唯一保留的项目放宽项。
- `joint4` 上限 `2.14`，`joint6` 范围 `[-0.7330383, 0.8726646]`。
- 软件急停：`/left/emergency_stop`、`/right/emergency_stop` 加两侧
  `control_enable=false`；没有物理急停。
- 失能前必须确认机械臂有支撑、关节误差在阈值内、硬件超限标志为 false。

## 5. 原生 Ubuntu 快速复验

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_native_dependencies.sh
sudo ./deployment/scripts/install_can_stable_names.sh
./deployment/scripts/install_realsense_udev.sh

cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

cd /home/work/nero_duo_arm/deployment
sudo ./scripts/enable_can.sh can_left 1000000
sudo ./scripts/enable_can.sh can_right 1000000
./scripts/check_nero_feedback.sh can_left
./scripts/check_nero_feedback.sh can_right
./scripts/check_realsense.sh
```

MoveIt 只读检查：

```bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py gui:=false rviz:=false
python3 deployment/scripts/test_dual_moveit_plan.py
python3 deployment/scripts/test_dual_collision.py
```

## 6. 文档索引

```text
docs/00_hardware_baseline.md          硬件、CAN、相机、坐标
docs/01_driver_can_safety.md          驱动、namespace、软件急停
docs/02_sensors_and_calibration.md    相机接入与标定
docs/03_moveit_and_park.md            MoveIt、轨迹桥、支撑位
docs/04_umi_lerobot.md                UMI 转 LeRobot、训练数据
docs/05_vla_pipeline.md               单臂/双臂 VLA 链路
docs/11_stage8_webapp.md              Web 控制台
docs/12_usage_and_commands.md         使用手册与命令速查
docs/17_gripper_can_diagnostics.md    夹爪协议与验收
```
