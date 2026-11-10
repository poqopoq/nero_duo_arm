# 使用手册与命令速查

> 工作区：`/home/work/nero_duo_arm`
>
> 环境：原生 Ubuntu 22.04 + ROS 2 Humble
> 细节文档：硬件 `00`、驱动/急停 `01`、相机 `02`、MoveIt `03`、UMI `04`、VLA `05`、夹爪 `17`、夹爪六维力 `18`。

## 0. 安全边界

- 当前没有物理急停，依赖 NERO 驱动 `emergency_stop` 服务和软件控制门。
- MoveIt 只检查 URDF、桌面和静态 planning scene；相机点云和动态障碍未接入。
- 每次真机运动前清空工作区、确认机械臂有支撑、确认急停入口可用。
- 首次运行或改参数后先用 `moveit_dry_run:=true` 和 `auto_enable:=false`。
- 不要同时启动两套驱动、两套 MoveIt 或两个占用 `8080` 的 Web app。
- 标记 **真机运动** 的命令会改变姿态；没有现场监护不要执行。

## 1. 环境

首次部署：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_native_dependencies.sh
sudo ./deployment/scripts/install_can_stable_names.sh
./deployment/scripts/install_realsense_udev.sh
```

每个新终端：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

确认包：

```bash
ros2 pkg prefix dual_nero_webapp
ros2 pkg prefix dual_nero_moveit_config
ros2 pkg prefix dual_nero_calibration
```

重新编译：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启用 CAN：

```bash
cd /home/work/nero_duo_arm/deployment
sudo ./scripts/enable_can.sh can_left 1000000
sudo ./scripts/enable_can.sh can_right 1000000
ip -details link show can_left
ip -details link show can_right
```

不要依赖 `can0/can1` 枚举顺序。接口缺失时先执行稳定命名并重新插拔。

## 2. 启动

### 2.1 推荐：dry-run

启动左右驱动、三路相机、MoveIt 和 Web app；电机失能，轨迹只模拟：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_grippers:=false start_cameras:=true start_moveit:=true \
  moveit_dry_run:=true auto_enable:=false
```

浏览器打开 `http://<机器人主机IP>:8080`。查看 IP：

```bash
hostname -I
```

旧终端没有刷新 `dialout` 组权限时，用包装脚本：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/webapp.sh \
  start_drivers:=true start_cameras:=true start_moveit:=true \
  moveit_dry_run:=true auto_enable:=false
```

只允许本机访问：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true \
  moveit_dry_run:=true auto_enable:=false http_host:=127.0.0.1
```

### 2.2 真机运动

只有在 dry-run、控制门、急停、Park 和现场安全确认全部通过后使用：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_grippers:=false start_cameras:=true start_moveit:=true \
  moveit_dry_run:=false auto_enable:=false \
  recover_joint_limits:=true recover_joint_limits_delay_sec:=8.0
```

默认 `speed_percent=5`。需要 40% 时显式加 `speed_percent:=40`。页面顶部应显示
`MoveIt: ready (LIVE)`；显示 `DRY-RUN` 时不会驱动真机。

常用平滑控制命令（夹爪未连接时保持关闭）：

```bash
./deployment/scripts/run_in_dialout.sh ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_grippers:=true start_cameras:=true start_moveit:=true \
  moveit_dry_run:=false auto_enable:=false speed_percent:=100 \
  moveit_velocity_scaling:=0.10 moveit_acceleration_scaling:=0.10 \
  trajectory_command_rate_hz:=100.0 trajectory_interpolation:=cubic \
  trajectory_settle_timeout_sec:=1.0 trajectory_settle_tolerance_rad:=0.01 \
  gripper_allow_motion:=true gripper_enable_motor:=true
```
### 2.3 分终端启动

驱动：

```bash
# dry-run：5% 速度，电机保持失能
ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=false \
  tcp_offset:='[0.2018, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'

# 真机运动：40% 速度并自动使能
ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=40 auto_enable:=true \
  recover_joint_limits:=true recover_joint_limits_delay_sec:=8.0 \
  tcp_offset:='[0.2018, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'
```

`tcp_offset` 默认已是 CTAG2F90D 捏合点；显式写出是为了和部署记录对齐。
更换夹爪或修改 `*_tcp_z` 后必须同步调整，否则驱动 `tcp_pose` 与 MoveIt
`*_tcp_link` 不一致，IK 会选到错误分支。

上电后如果 `joint4` 接近 `2.19--2.20 rad`，先做只读检查和局部归位：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/recover_joint_limits.sh --dry-run
./deployment/scripts/recover_joint_limits.sh --target 2.115 --cap-margin 0.03
```

该脚本只调整 `joint4`，结束后关闭控制门但保持电机使能；不要顺手失能，否则
右臂会被重力拉回限位。夹爪未连接时保持 `start_grippers:=false`，VLA bridge
保持 `gripper_enabled:=false`。

相机：

```bash
ros2 launch dual_nero_cameras all_cameras.launch.py
```

MoveIt + 轨迹桥：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true rviz:=false
```

仅规划：

```bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py
```

只启动 Web app：

```bash
ros2 launch dual_nero_webapp webapp.launch.py
```

### 2.4 常用参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `start_drivers` | `false` | 同时启动左右驱动 |
| `start_cameras` | `false` | 同时启动三路相机 |
| `start_moveit` | `false` | 启动 `move_group` 和轨迹桥 |
| `moveit_dry_run` | `true` | `false` 才控制真机 |
| `auto_enable` | `true` | 启动时使能电机；首次调试设为 `false` |
| `speed_percent` | `5` | NERO 速度百分比 |
| `recover_joint_limits` | `false` | 驱动启动后执行 joint4 局部归位并保持使能 |
| `recover_joint_limits_delay_sec` | `8.0` | 等待反馈后再执行归位 |
| `start_grippers` | `false` | 是否启动夹爪 CAN 驱动；未连接夹爪时保持关闭 |
| `moveit_velocity_scaling` | `0.05` | MoveIt 速度缩放 |
| `moveit_acceleration_scaling` | `0.05` | MoveIt 加速度缩放 |
| `trajectory_command_rate_hz` | `50.0` | 轨迹桥指令频率 |
| `trajectory_interpolation` | `linear` | `linear` 或 `cubic` |
| `trajectory_settle_timeout_sec` | `1.0` | 终点保持等待 |
| `trajectory_settle_tolerance_rad` | `0.01` | 终点误差容限 |
| `use_compressed_color` | `true` | Web app 订阅 JPEG 压缩流 |
| `http_host` / `http_port` | `0.0.0.0` / `8080` | Web app 监听地址和端口 |
| `enable_cameras` | `true` | 是否订阅相机 |
| `gripper_allow_motion` | `false` | 是否允许夹爪真实运动 |
| `gripper_enable_motor` | `false` | 启动时是否使能夹爪 |
| `gripper_speed_percent` | `25` | 夹爪速度 `1..100%` |
| `gripper_ramp_ms` | `200` | 加速度斜坡 `0..1000 ms` |
| `gripper_max_current` | `200` | 最大电流软限位 `1..1000` |
| `gripper_command_max` | `990` | 闭合命令上限 |

## 3. 推荐操作顺序

1. 启动 dry-run，确认两侧 `fb live`、三路相机和 `MoveIt: ready (DRY-RUN)`。
2. 打开对应侧 `Open gate`。
3. `Sync from robot` 同步滑条。
4. 单侧极小 `Send`，确认方向和停止行为。
5. 双臂目标确认后 `Both Send`。
6. 结束后 `Both Park`，确认到位，再 `Stop both` 和 `Disable motors`。
7. Park 失败时不要直接失能；先 `Stop both`，人工支撑后再处理。

## 4. HTTP 控制

先设置地址并检查健康：

```bash
BASE=http://127.0.0.1:8080
curl -s "$BASE/api/health" | python3 -m json.tool
curl -s "$BASE/api/state" | python3 -m json.tool
curl -s "$BASE/api/motion" | python3 -m json.tool
```

查询接口：

| 命令 | 用途 |
| --- | --- |
| `curl -s "$BASE/api/config"` | 关节名、限位、Park、相机元数据 |
| `curl -s "$BASE/api/state"` | 反馈、反馈年龄、门和电机状态 |
| `curl -s "$BASE/api/motion"` | MoveIt 可用性、忙状态、dry-run/真机 |
| `curl -s "$BASE/api/cameras"` | 相机帧率、分辨率、帧龄 |
| `curl -s "$BASE/api/gripper"` | 夹爪范围、许可和实时状态 |
| `curl -s "$BASE/robot_urdf"` | 展开后的 URDF |

控制门：

```bash
curl -s -X POST "$BASE/api/control_gate" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","enabled":true}'
```

电机使能/失能：

```bash
curl -s -X POST "$BASE/api/enable" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","enabled":true}'

curl -s -X POST "$BASE/api/enable" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","enabled":false}'
```

单臂运动（`positions` 必须正好 7 个数，J1..J7，单位 rad）：

```bash
LEFT='[<J1>,<J2>,<J3>,<J4>,<J5>,<J6>,<J7>]'
curl -s -X POST "$BASE/api/move" \
  -H 'Content-Type: application/json' \
  -d "{\"side\":\"left\",\"positions\":$LEFT}"
```

双臂同步：

```bash
LEFT='[<left_J1>,<left_J2>,<left_J3>,<left_J4>,<left_J5>,<left_J6>,<left_J7>]'
RIGHT='[<right_J1>,<right_J2>,<right_J3>,<right_J4>,<right_J5>,<right_J6>,<right_J7>]'
curl -s -X POST "$BASE/api/move" \
  -H 'Content-Type: application/json' \
  -d "{\"side\":\"both\",\"positions\":{\"left\":$LEFT,\"right\":$RIGHT}}"
```

Park：

```bash
curl -s -X POST "$BASE/api/park" \
  -H 'Content-Type: application/json' \
  -d '{"side":"both"}'
```

取消、急停、清故障、恢复：

```bash
curl -s -X POST "$BASE/api/cancel_motion" \
  -H 'Content-Type: application/json' -d '{}'

curl -s -X POST "$BASE/api/emergency_stop" \
  -H 'Content-Type: application/json' -d '{"side":"left"}'

curl -s -X POST "$BASE/api/emergency_stop_all" \
  -H 'Content-Type: application/json' -d '{}'

curl -s -X POST "$BASE/api/clear_faults" \
  -H 'Content-Type: application/json' -d '{"side":"left"}'

curl -s -X POST "$BASE/api/recover" \
  -H 'Content-Type: application/json' -d '{"side":"left"}'
```

夹爪：

```bash
curl -s "$BASE/api/gripper" | python3 -m json.tool

curl -s -X POST "$BASE/api/gripper" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","action":"position","position":500}'

curl -s -X POST "$BASE/api/gripper" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","action":"configure","speed_percent":25}'

curl -s -X POST "$BASE/api/gripper" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","action":"clear_obstruction"}'
```

WebSocket：`ws://<机器人IP>:8080/ws`，约 10 Hz 推送状态。除非开发客户端，
优先使用 UI 或 REST。

## 5. ROS 检查

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 action info /move_action
ros2 action info /dual_arm_controller/follow_joint_trajectory
```

关键话题：

```text
/left/feedback/joint_states
/right/feedback/joint_states
/left/feedback/arm_status
/right/feedback/arm_status
/left/feedback/motor_status
/right/feedback/motor_status
/left/control/joint_states
/right/control/joint_states
/joint_states
```

```bash
ros2 topic echo --once /left/feedback/joint_states sensor_msgs/msg/JointState
ros2 topic echo --once /right/feedback/joint_states sensor_msgs/msg/JointState
ros2 topic hz /left/feedback/joint_states
ros2 topic hz /right/feedback/joint_states
```

驱动服务（把 `left` 换成 `right` 操作右臂）：

```bash
ros2 service call /left/control_enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /left/enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /left/clear_joint_error std_srvs/srv/Trigger "{}"
ros2 service call /left/reset_motion std_srvs/srv/Trigger "{}"
ros2 service call /left/emergency_stop std_srvs/srv/Empty "{}"
```

**不要用 `/move_home` 代替 Park。** 它绕过 MoveIt 直接去全零位，不是已验证的
重力稳定支撑姿态。除专门的低速急停验收脚本外，不要直接向
`/left/control/joint_states` 发布命令。

关节限位（Web app 服务端截断值，rad）：

| 关节 | 下限 | 上限 |
| --- | ---: | ---: |
| J1 | -2.705260 | 2.705260 |
| J2 | -1.760000 | 1.740000 |
| J3 | -2.750000 | 2.750000 |
| J4 | -1.010000 | 2.140000 |
| J5 | -2.750000 | 2.750000 |
| J6 | -0.7330383 | 0.8726646 |
| J7 | -1.5707963 | 1.5707963 |

J2 下限 `-1.760000` 是唯一保留的项目放宽项。其余限位等于或严于 NERO 原生值；
J4 上限 `2.14` 是部署保守限幅（原生 `2.146755`），不是原生上限。

## 6. 相机

```bash
ros2 launch dual_nero_cameras all_cameras.launch.py
ros2 topic hz /camera_scene/color/image_raw
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic hz /camera_right_wrist/color/image_raw
ros2 topic hz /camera_scene/color/image_raw/compressed
ros2 topic hz /camera_left_wrist/color/image_raw/compressed
ros2 topic hz /camera_right_wrist/color/image_raw/compressed
```

Depth 默认关闭；需要时加 `enable_scene_depth:=true` 或
`enable_wrist_depth:=true`，并单独复测帧率。安装、USB 和标定命令见
`02_sensors_and_calibration.md`。

## 7. Park 与复位

上电后先执行上文的 `recover_joint_limits.sh`，再进入整体复位/Park；两者不要
同时运行，且都要求工作区清空和现场监护。

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/reset_to_standard_support.sh --plan-only
./deployment/scripts/reset_to_standard_support.sh
./deployment/scripts/reset_to_standard_support.sh --disable
```

脚本自行启停驱动和 MoveIt，执行前关闭 Web app 和其他控制进程。支撑位数值和
安全规则见 `03_moveit_and_park.md`。

## 8. 验收与开发脚本

先关闭 Web app 和已有 MoveIt，只保留脚本需要的前置进程。

只读/规划：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/check_stage0.sh
./deployment/scripts/check_realsense.sh
./deployment/scripts/check_nero_feedback.sh can_left
./deployment/scripts/check_nero_feedback.sh can_right
./deployment/scripts/stage2_check_dual_namespace.sh
/usr/bin/python3 deployment/scripts/test_dual_moveit_plan.py
/usr/bin/python3 deployment/scripts/test_dual_collision.py
/usr/bin/python3 deployment/scripts/test_moveit_execution_dry_run.py --group left_arm
/usr/bin/python3 deployment/scripts/test_moveit_execution_dry_run.py --group both_arms
/usr/bin/python3 deployment/scripts/test_trajectory_bridge_dry_run.py
/usr/bin/python3 deployment/scripts/test_trajectory_cancel_dry_run.py
```

真机运动（必须人工监护）：

```bash
/usr/bin/python3 deployment/scripts/soft_estop_motion_test.py --namespace left
/usr/bin/python3 deployment/scripts/soft_estop_motion_test.py --namespace right
/usr/bin/python3 deployment/scripts/soft_estop_motion_test.py --both
./deployment/scripts/reset_to_standard_support.sh --plan-only
./deployment/scripts/test_trajectory_cancel_real.sh
```

夹爪验收：

```bash
./deployment/scripts/test_gripper_control.sh
./deployment/scripts/test_gripper_ft_sensor.sh
./deployment/scripts/test_gripper_sweep.sh --sides both
./deployment/scripts/test_gripper_estop.sh --sides both --speed 10 --trigger-travel 40
```

夹爪六维力传感器：

```bash
ros2 launch dual_nero_gripper_ft_sensor gripper_ft.launch.py \
  serial_port:=/dev/ttyUSB0 baud_rate:=921600
ros2 topic echo /gripper_ft/data
ros2 topic hz /gripper_ft/data
ros2 service call /gripper_ft_node/get_version std_srvs/srv/Trigger "{}"
# 先停止节点，再扫描串口与传感器 SN 的对应关系
sg dialout -c '/usr/bin/python3 \
  deployment/scripts/identify_gripper_ft_sensors.py'
```

## 9. UMI 与 VLA

- UMI 转换、回读、上传和训练：`04_umi_lerobot.md`
- 单臂/双臂 VLA bridge、policy server、robot client：`05_vla_pipeline.md`
- 夹爪协议、只读启动和验收：`17_gripper_can_diagnostics.md`
- 夹爪六维力传感器与 ROS 2 接口：`18_gripper_ft_sensor.md`

## 10. 常见问题

### 页面 `disconnected`

```bash
ss -ltnp | grep ':8080'
ros2 node list | grep dual_nero_webapp
curl -s http://127.0.0.1:8080/api/health
```

### `MoveIt: unavailable`

```bash
ros2 action list | grep -E 'move_action|follow_joint_trajectory'
ros2 node list | grep -E 'move_group|trajectory_bridge'
```

启动 `start_moveit:=true` 或单独启动 `moveit_execution.launch.py`。

### 点击 Send 没运动

1. 页面是否为 `MoveIt: ready (LIVE)`。
2. 当前侧是否 `gate open`、`motors enabled`、`fb live`。
3. `app.motion.available` 是否为 `true`。
4. 目标是否在关节限位内。
5. 是否有前一次 motion busy 或取消锁存。

服务端每次新运动前会调用 `/dual_nero_trajectory_bridge/clear_cancel`；不可用时
会拒绝运动。

### Park 不可用

```bash
curl -s http://127.0.0.1:8080/api/config \
  | python3 -m json.tool | grep -A20 '"park"'
```

检查支撑位文件是否存在、两侧是否各有 7 个有限数值、是否超出关节限位。

### 相机 `offline/stale`

```bash
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic echo --once /camera_left_wrist/color/camera_info
./deployment/scripts/check_realsense.sh
```

检查序列号、USB 速率、话题名和相机参数。

## 11. 尚未实现

- 相机点云、OctoMap、动态障碍和 planning scene 编辑。
- Cartesian/IK/命名位姿目标和浏览器轨迹预览。
- 夹爪带载抓取、真实堵塞清障、长时热稳定和 CAN 掉线恢复。
- Web/MoveIt 全链路真机联锁验收。
- 真实物理急停和完整硬件诊断面板。
- 真实双臂 SmolVLA checkpoint 整链路验收。
