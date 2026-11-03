# Dual NERO 项目交接与原生 Ubuntu 迁移

> 更新日期：2026-11-03
> 当前环境：原生 Ubuntu 22.04.5 + ROS 2 Humble
> 当前工作区：`/home/work/nero_duo_arm`

## 1. 项目目录

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
├── dependencies/pyAgxArm/        # 官方 Python SDK 源码
├── references/cc_agx_arm_ros/    # CloudChef 双臂参考仓库
└── deployment/                   # 文档、配置、验证脚本
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

### 当前 USB 连接（原生 Ubuntu）

```text
左腕 D405: 5000 Mbps / USB 3.20
右腕 D405: 5000 Mbps / USB 3.20
D435:      480 Mbps / USB 2.10（临时 USB 2.0 线）
```

2026-11-01 在原生 Ubuntu 上完成 120 秒三机功能测试：三路 Color、Depth、
Aligned Depth 均约 15 FPS，未出现 incomplete frame 或帧超时。D435 仍使用
USB 2.0 线，因此该结果不能替代 5000 Mbps 下的正式 10 分钟验收。

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
- [x] 原生 Ubuntu USB2 临时三机功能测试（120 秒，约 15 FPS）
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
- [x] 真实轨迹取消、双侧软件急停和控制门关闭
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
- [x] 单臂 UMI replay-buffer zarr/HDF5 → LeRobot v3 转换器（canonical/嵌套/扁平 HDF5，7D 绝对 EE action）
- [x] 原始双臂 UMI HDF5/MKV → Xense `hdf52lerobot` 路径文档化
- [x] 用真实公开 UMI session 验证 zarr 与 canonical HDF5 转换（前 5 集冒烟）
- [ ] 用 NERO 标定后的真实 session 完成可部署数据转换
- [ ] π0.5 数据加载验证

UMI 转换说明见 `deployment/docs/10_stage7_umi_lerobot.md`，完整训练/部署流程见
`deployment/docs/16_umi_nero_e2e.md`。`umi-sroi` 契约保持发布格式：
`action = [ee.x, ee.y, ee.z, ee.wx, ee.wy, ee.wz, ee.gripper_pos]`，不写
`observation.state`。本地 NERO 部署使用 `--contract nero-ros`：
`action/state = [ee.x, ee.y, ee.z, ee.rx, ee.ry, ee.rz, ee.gripper]`，
图像键 `observation.images.camera1`，`robot_type=nero_ros`。夹爪为 0=闭合、
1=张开，`action[t]` 默认是 `t+1` 的绝对位姿。

### 阶段 8：Web 控制台

- [x] `dual_nero_webapp` FastAPI/WebSocket package
- [x] 双臂关节滑条、反馈和 3D xacro 模型
- [x] Send/Park 经 MoveIt `/move_action` 规划、静态碰撞检查并执行
- [x] Web app 控制门、使能/失能、单侧/双臂软件急停
- [x] 三路 RealSense Color/Depth MJPEG 画面
- [x] 无硬件 HTTP、URDF 与静态页面冒烟测试
- [x] MoveIt dry-run 下的 Park、急停取消和 Stop both 测试
- [ ] 真机联锁、关节方向和长时间运行验收
- [ ] 相机点云/OctoMap 实时动态障碍接入 MoveIt

说明见 `deployment/docs/11_stage8_webapp.md`。

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

### 7.1 确认工作区

当前仓库布局：

```text
/home/work/nero_duo_arm/ros2_ws/src
/home/work/nero_duo_arm/dependencies/pyAgxArm
/home/work/nero_duo_arm/deployment
```

不需要复制：

```text
dual_nero_ws/build
dual_nero_ws/install
dual_nero_ws/log
```

脚本默认从仓库位置推导路径，也可以通过 `NERO_DUO_ROOT`、
`NERO_DUO_WS`、`NERO_DUO_DEPLOY` 覆盖。

### 7.2 安装系统依赖

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_native_dependencies.sh
```

如果脚本提示 ROS 2 未安装，先安装 ROS 2 Humble Desktop。

### 7.3 安装 udev 规则

```bash
sudo ./deployment/scripts/install_can_stable_names.sh
./deployment/scripts/install_realsense_udev.sh
```

重新插拔 CAN 和相机。

### 7.4 编译工作空间

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 7.5 基础复验

```bash
cd /home/work/nero_duo_arm/deployment
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
cd /home/work/nero_duo_arm/ros2_ws
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
docs/10_stage7_umi_lerobot.md
docs/11_stage8_webapp.md
docs/12_usage_and_commands.md
```

## 9. 关键安全状态

2026-11-01 原生 Ubuntu 首次只读检查发现左右臂均不在旧主机记录的
标准支撑位：

| 侧 | 相对 `support_pose.yaml` 的最大误差 |
| --- | --- |
| 左臂 | 0.0595 rad（3.41°） |
| 右臂 | 0.5926 rad（33.95°） |

2026-11-01 再次只读复核，误差未变化。无头 MoveIt `check_state_validity`
显示实时双臂姿态与 `support_pose.yaml` 姿态均为 `valid=True, contacts=0`；
因此当前差异是物理/坐标系确认问题，不能仅凭碰撞检查直接恢复运动。

随后使用新的
`deployment/scripts/reset_to_standard_support.sh` 完成了 MoveIt 低速复位。
第一次复位到达旧支撑位后自动失能，确认旧支撑位不是重力稳定位：

```text
复位后（失能前）最大误差: 0.009774 rad
失能后最大误差: 左 0.059341 rad / 右 0.060650 rad
结论: 旧支撑位不能无动力保持，关节 2/4 会下垂
```

为获得真正稳定的停靠位，先禁用双电机并记录自然平衡位置。最终连续 20 秒
无漂移，MoveIt 碰撞检查为 `valid=True, contacts=0`：

```text
left:  [-1.574653504, -1.757092771, -0.045413467, 2.196549224,
        -0.013561208, -0.009267698, 0.122714100]
right: [ 1.562331480, -1.752886528, -0.050841441, 2.197805861,
        -0.014311700, -0.007627089, 0.118996548]
```

该稳定位姿已写入 `support_pose.yaml` 和 SRDF 的三个 `support` 状态。旧 URDF
的 `joint2` 下限 `-1.74`、`joint4` 上限 `2.14` 会钳制实测平衡位；仅将本项目
`dual_nero_description` 的两个已验证方向放宽为 `-1.760` 和 `2.200`。官方
URDF 和 SDK 预设未修改，硬件关节超限标志在目标位保持全 `false`。

最终完整 `enable -> stable park -> software E-stop -> disable` 验证：

```text
规划轨迹:                 23 点，2.154 s，5% 速度缩放
MoveIt error_code:        1
失能前最大误差:           0.009634 rad
失能前硬件超限标志:       左右 7/7 均为 false
左右软件急停:             True / True
失能结果:                 True / True
失能后 5 秒最大误差:      0.009268 rad
再保持 5 秒的姿态漂移:    0.000087 rad
14 个电机实际状态:        全部 disabled
驱动退出后 20 秒复测:     最大误差 0.009268 rad，变化 <= 0.000174 rad
AT_SUPPORT:               True
```

当前状态：左右臂已失能，稳定停靠在新 park pose，无下垂，ROS 驱动已退出。
复位脚本默认保持使能；`--plan-only` 只做规划预检，`--disable` 才执行完整
失能验证。失能前若硬件状态不可用或任一关节超限标志为 `true`，脚本会软件
急停并保持电机使能，不会盲目断电。旧的
`return_to_standard_support_and_disable.py` 仍不应在本机直接使用。

迁移后首次上电前必须重新检查：

- 急停入口
- 机械支撑
- CAN 映射
- 关节反馈
- 标准支撑位姿
- 控制门初始关闭

不要在新机器上直接执行真机轨迹。
