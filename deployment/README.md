# Dual NERO 部署

工作区：`/home/work/nero_duo_arm`。本目录包含部署文档、配置和验收脚本。

## 文档

- [硬件与系统基线](docs/00_hardware_baseline.md) — CAN/相机固定映射、坐标参数
- [驱动、CAN 与软件急停](docs/01_driver_can_safety.md) — namespace、反馈检查、急停
- [相机接入与标定](docs/02_sensors_and_calibration.md) — 三路 RealSense、外参标定
- [MoveIt 与标准支撑位](docs/03_moveit_and_park.md) — 规划、轨迹桥、Park/复位
- [UMI 数据到 LeRobot](docs/04_umi_lerobot.md) — zarr/HDF5 转换、训练数据
- [VLA 链路](docs/05_vla_pipeline.md) — 单臂/双臂 bridge、policy server、client
- [Web 控制台](docs/11_stage8_webapp.md) — 启动、安全顺序、相机
- [使用手册与命令速查](docs/12_usage_and_commands.md) — 日常命令入口
- [夹爪与 ZQWL-CANFD](docs/17_gripper_can_diagnostics.md) — 协议、只读、验收
- [项目交接](PROJECT_HANDOFF.md) — 当前状态、映射、快速复验

## 快速检查

```bash
cd /home/work/nero_duo_arm/deployment
./scripts/check_stage0.sh
./scripts/check_realsense.sh
sudo ./scripts/enable_can.sh can_left 1000000
sudo ./scripts/enable_can.sh can_right 1000000
./scripts/check_nero_feedback.sh can_left
./scripts/check_nero_feedback.sh can_right
```

脚本只读，不安装软件、不修改网络、不启动运动。真机运动前先读
`docs/12_usage_and_commands.md` 的安全边界。
