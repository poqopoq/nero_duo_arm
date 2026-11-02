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
- [阶段 5：MoveIt 2 双臂规划](docs/07_stage5_moveit_plan.md) — 规划、执行桥、取消和标准支撑安全流程通过（6/7）

## 运行阶段 0 检查

```bash
cd /home/a/workspace/dual_nero_deployment
./scripts/check_stage0.sh | tee docs/00_stage0_auto_snapshot.md
```

脚本只读，不安装软件、不修改网络、不启用 CAN、不访问硬件控制接口。

- [阶段 6：相机外参标定框架](docs/08_stage6_camera_calibration.md) — 12×9 棋盘格采集/求解/TF 发布框架已建立
