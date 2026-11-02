# Dual NERO 项目交接与原生 Ubuntu 迁移

> 更新日期：2026-11-02  
> 当前环境：WSL2 + Ubuntu 22.04.5 + ROS 2 Humble  
> 目标环境：原生 Ubuntu 22.04 + ROS 2 Humble

## 1. 项目目录

```text
/home/a/workspace/
├── dual_nero_ws/                 # ROS 2 工作空间
│   ├── src/
│   │   ├── agx_arm_ros/          # AgileX 官方 NERO 驱动
│   │   ├── dual_nero_description/
│   │   ├── dual_nero_cameras/
│   │   ├── dual_nero_control/
│   │   ├── dual_nero_moveit_config/
│   │   └── dual_nero_calibration/
│   └── dependencies/pyAgxArm/    # 官方 Python SDK 源码
├── dual_nero_deployment/         # 文档、配置、验证脚本
└── cc_agx_arm_ros/               # CloudChef 双臂参考仓库
```

## 2. 硬件与固定映射

### NERO

```text
左臂 CAN 适配器: 004500404648571620303731 -> can_left
右臂 CAN 适配器: 002200384648570B20303731 -> can_right
CAN bitrate:     1,000,000 bit/s
固件:            v121 / 1.21
```

### RealSense

```text
左腕 D405: 260522270991
右腕 D405: 261122272091
D435:      263422075415
固件:      5.15.1.55
```

注意：Windows PnP 显示的另一组 USB 序列号与 RealSense `rs-enumerate-devices` 序列号不同。ROS 驱动和本项目的相机配置使用 `rs-enumerate-devices` 序列号。

### 当前 Hub

```text
Hub: Genesys Logic 05e3:0626
类型: USB 3.1 Hub，5 Gbps 级别
三相机枚举: 均 5000 Mbps
```

当前 Hub 下三机 15 FPS 并行测试仍出现 incomplete frame，需要换 USB 3.2 Gen2 10 Gbps 外接供电 Hub，或使用原生 Ubuntu 直连。

## 3. 机械与坐标参数

```text
world:
  桌面平面、左右基座中心中点
  +X 指向桌内
  +Y 指向左侧
  +Z 向上

table front edge:
  world.x = -0.06 m

left base:
  xyz = [0, +0.35, 0]
  yaw = pi

right base:
  xyz = [0, -0.35, 0]
  yaw = pi
```

两基座中心距为 700 mm。

## 4. 已完成步骤

### 阶段 0：系统和硬件基线

- [x] Ubuntu 22.04 和 ROS 2 Humble
- [x] 两台 candleLight USB-CAN 适配器
- [x] 左右 NERO 反馈
- [x] 软件急停方案和验证
- [ ] 三台相机稳定并行
- [ ] 相机最终安装
- [ ] 桌面和基座尺寸正式签字

### 阶段 1：官方驱动

- [x] 创建 `dual_nero_ws`
- [x] 下载官方 `agx_arm_ros` 驱动
- [x] 初始化 `agx_arm_urdf` 子模块
- [x] 安装 `pyAgxArm`、`python-can`
- [x] Humble 编译通过
- [x] 左右 NERO 通信和关节反馈

官方驱动提交：

```text
286633e0d2edb038ee3a1047d9feb7f614a1c076
```

URDF 子模块提交：

```text
680fa925cd00db7e2628563d2edc5540963045de
```

### 阶段 2：双 NERO 独立控制

- [x] `can_left` / `can_right` 序列号稳定命名
- [x] `/left` / `/right` namespace
- [x] 左右关节反馈隔离
- [x] 单臂低速运动
- [x] 双臂联动软件急停
- [x] 测试后 14 个电机失能

### 阶段 3：双臂模型和 TF

- [x] 桌面固定式双臂 Xacro
- [x] `world -> table -> left/right base`
- [x] 两基座 700 mm，yaw=pi
- [x] 无夹爪模型
- [x] `tool0` / `tcp_link` 占位
- [ ] 自定义夹爪模型
- [ ] 最终 TCP 偏移
- [ ] 相机安装 TF（相机未安装）

### 阶段 4：三台 RealSense

- [x] `realsense2_camera`、description、msgs 安装
- [x] RealSense udev 规则
- [x] 左右 D405 单机验证
- [x] 左右 D405 并行验证
- [x] D435 单机验证
- [x] 三机在 USB3 5000 Mbps 下枚举
- [ ] 三机稳定 15 FPS 并行
- [ ] 最终 10 Gbps Hub 或原生 Ubuntu 验证

### 阶段 5：MoveIt 2

- [x] MoveIt 2 安装
- [x] `left_arm`、`right_arm`、`both_arms` 规划组
- [x] KDL 运动学
- [x] OMPL 规划
- [x] 桌面/双臂碰撞检测
- [x] `left_arm` 和 `both_arms` 纯规划
- [x] MoveIt → FollowJointTrajectory 轨迹桥
- [x] 轨迹桥 dry-run
- [x] 轨迹取消 dry-run
- [x] 左臂真实低速执行
- [x] 双臂真实低速执行和标准支撑位返回
- [x] 标准支撑位误差 < 0.03 rad 后才急停失能
- [ ] 夹爪规划组（夹爪未定）

### 阶段 6：相机标定框架

- [x] 标定包 `dual_nero_calibration`
- [x] 12×9 棋盘格配置（11×8 内角点，15 mm）
- [x] 固定相机外参求解
- [x] 腕部手眼标定求解
- [x] static TF YAML 输出和发布
- [x] 合成数据求解器验证
- [ ] D435 世界外参
- [ ] 左 D405 手眼外参
- [ ] 右 D405 手眼外参

### 阶段 7：数据采集与 LeRobot

- [ ] 数据采集节点
- [ ] 图像/关节/夹爪/末端位姿同步
- [ ] ROS 2 bag
- [ ] LeRobot Dataset 转换
- [ ] π0.5 数据加载验证

## 5. 标准支撑位姿

文件：

```text
dual_nero_deployment/config/support_pose.yaml
```

MoveIt 状态：

```text
left_arm/support
right_arm/support
both_arms/support
```

安全规则：

1. 任何真机执行结束后先返回 `support`。
2. 支撑误差必须 <= `0.03 rad`。
3. 到位后再调用软件急停。
4. 软件急停成功后再失能电机。
5. 返回失败时只软件急停，电机保持使能，等待人工支撑。

## 6. 已知问题

1. WSL USB/IP 下 CAN 适配器偶发失步，物理拔插通常恢复。
2. 当前 5 Gbps Hub 下三台 RealSense 15 FPS 并行仍出现 incomplete frame。
3. 相机尚未安装，不能进行最终外参标定。
4. 夹爪未选型，MoveIt 暂无夹爪规划组。
5. 没有实体急停；当前按项目决定使用官方 Web 端和 ROS 软件急停。
6. WSL 不作为正式数据采集和实时控制平台。

## 7. 原生 Ubuntu 迁移步骤

### 7.1 复制项目

建议复制：

```text
/home/a/workspace/dual_nero_ws/src
/home/a/workspace/dual_nero_ws/dependencies
/home/a/workspace/dual_nero_deployment
```

不需要复制：

```text
dual_nero_ws/build
dual_nero_ws/install
dual_nero_ws/log
```

保持路径为 `/home/a/workspace` 可以避免修改项目中的绝对路径。

### 7.2 安装系统依赖

```bash
cd /home/a/workspace
sudo ./dual_nero_deployment/scripts/install_native_dependencies.sh
```

如果脚本提示 ROS 2 未安装，先安装 ROS 2 Humble Desktop。

### 7.3 安装 udev 规则

```bash
sudo ./dual_nero_deployment/scripts/install_can_stable_names.sh
./dual_nero_deployment/scripts/install_realsense_udev.sh
```

重新插拔 CAN 和相机。

### 7.4 编译工作空间

```bash
cd /home/a/workspace/dual_nero_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 7.5 基础复验

```bash
cd /home/a/workspace/dual_nero_deployment
./scripts/check_stage0.sh | tee docs/00_stage0_auto_snapshot_native.md
./scripts/check_realsense.sh
```

CAN：

```bash
sudo ./scripts/enable_can.sh can_left 1000000
sudo ./scripts/enable_can.sh can_right 1000000
./scripts/check_nero_feedback.sh can_left
./scripts/check_nero_feedback.sh can_right
```

MoveIt：

```bash
cd /home/a/workspace/dual_nero_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py
```

### 7.6 原生 Ubuntu 上的验收顺序

1. 三台相机均以 5000 Mbps 枚举。
2. 三机 15 FPS 并行至少 10 分钟无 incomplete frame / USB 断开。
3. 相机安装到最终位置。
4. 左腕 D405 手眼标定。
5. 右腕 D405 手眼标定。
6. D435 world 外参标定。
7. 发布并验证三段 static TF。
8. 进入数据采集和 LeRobot。

## 8. 关键文档

```text
docs/00_stage0_system_hardware_baseline.md
docs/02_stage1_driver_setup.md
docs/03_stage2_dual_can_namespace.md
docs/04_software_estop_validation.md
docs/05_stage3_model_parameters.md
docs/06_stage4_realsense_setup.md
docs/07_stage5_moveit_plan.md
docs/08_stage6_camera_calibration.md
```

## 9. 关键安全状态

当前左右臂已失能并位于标准支撑位。迁移后首次上电前必须重新检查：

- 急停入口
- 机械支撑
- CAN 映射
- 关节反馈
- 标准支撑位姿
- 控制门初始关闭

不要在新机器上直接执行真机轨迹。
