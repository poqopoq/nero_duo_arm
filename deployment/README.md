# Dual NERO 部署记录

本目录包含分阶段部署文档与只读检查脚本。参考源码在工作区 `cc_agx_arm_ros/`，本目录不修改参考仓库。

## 阶段

- [阶段 0：系统与硬件基线](docs/00_stage0_system_hardware_baseline.md) — 进行中，硬件项待现场确认
- [参考仓库审计](docs/01_reference_repo_audit.md)
- [阶段 1：官方驱动安装](docs/02_stage1_driver_setup.md) — 5/5，左臂通信已验证
- [阶段 2：CAN 稳定命名与 namespace](docs/03_stage2_dual_can_namespace.md) — 完成：左右独立控制与双臂联动软件急停通过
- [软件急停验证](docs/04_software_estop_validation.md) — 单臂与双臂联动软件急停全部通过
- [阶段 3：桌面双臂模型参数](docs/05_stage3_model_parameters.md) — 参数化 Xacro 已建立，细节待实测
- [阶段 4：三台 RealSense 接入](docs/06_stage4_realsense_setup.md) — 暂停，等待新 USB Hub
- [阶段 5：MoveIt 2 双臂规划](docs/07_stage5_moveit_plan.md) — 规划、执行桥、真实轨迹取消联锁、单臂/双臂真机执行及稳定失能 park pose 通过；夹爪规划组待定
- [阶段 6：相机外参标定框架](docs/08_stage6_camera_calibration.md) — 12×9 棋盘格采集/求解/TF 发布框架已建立
- [阶段 7：UMI 数据转 LeRobot v3](docs/10_stage7_umi_lerobot.md) — 单臂 UMI zarr 转换器与 Xense 原始 HDF5/MKV 路径说明
- [阶段 8：双臂 Web 控制台](docs/11_stage8_webapp.md) — MoveIt `/move_action` 规划执行、3D 模型、三路相机与安全联锁；相机点云防碰撞待接入
- [完整使用手册与命令速查](docs/12_usage_and_commands.md) — Web 控制台逐项说明、REST/ROS 命令、相机标定、UMI 转换和故障排查
- [阶段 9：SmolVLA VLA 部署链路](docs/13_stage9_vla_smolvla.md) — ROS bridge + `lerobot_robot_nero` 闭环，观测、动作契约与策略客户端
- [阶段 10：VLA 动作到 NERO 运动执行](docs/14_stage10_vla_motion.md) — `dry_run/ik_dry_run/trajectory` 三模式、MoveIt IK、短轨迹异步执行与安全门控
- [阶段 11：双臂 VLA 真机评测](docs/15_stage11_vla_bimanual_eval.md) — 20-D rot6d / 14-D axis-angle、双臂原子 IK、三相机、`lerobot-rollout` 评测与真机门控

## 运行阶段 0 检查

```bash
cd /home/work/nero_duo_arm/deployment
./scripts/check_stage0.sh | tee docs/00_stage0_auto_snapshot.md
```

脚本只读，不安装软件、不修改网络、不启用 CAN、不访问硬件控制接口。
