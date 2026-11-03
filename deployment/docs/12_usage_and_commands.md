# Dual NERO 完整使用手册与命令速查

> 适用版本：2026-11-02，原生 Ubuntu 22.04 + ROS 2 Humble  
> 工作区：`/home/work/nero_duo_arm`  
> 范围：启动、Web 控制台、MoveIt、相机、标定、UMI 数据转换、验收脚本和故障排查

## 0. 先读：安全边界

- 当前系统没有物理急停，依靠 NERO 驱动 `emergency_stop` 服务和软件控制门。
- MoveIt 当前只对 URDF、桌面和静态 planning scene 做碰撞检查；相机点云、
  OctoMap 和实时动态障碍尚未接入。
- 每次真机运动前必须清空工作区、确认机械臂有支撑、确认急停入口可用。
- 第一次运行、更换硬件或修改参数后，先使用 `moveit_dry_run:=true`。只有
  dry-run 和联锁检查通过后，才使用 `moveit_dry_run:=false`。
- 不要同时启动两套驱动、两套 MoveIt 或两个使用 `8080` 端口的 Web app。
- 本文中标记 **真机运动** 的命令会改变机械臂姿态；没有现场监护时不要执行。

## 1. 路径与环境

### 1.1 首次部署依赖

在原生 Ubuntu 22.04 + ROS 2 Humble 上执行：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_native_dependencies.sh
sudo ./deployment/scripts/install_can_stable_names.sh
./deployment/scripts/install_realsense_udev.sh
```

`install_native_dependencies.sh` 不能使用 `sudo` 运行；脚本内部会在需要时调用
`sudo`。CAN 稳定命名和 RealSense udev 安装后重新插拔对应 USB 设备。

### 1.2 每个终端的环境

每个新终端都需要加载 ROS 和工作区：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

确认关键命令可用：

```bash
ros2 pkg prefix dual_nero_webapp
ros2 pkg prefix dual_nero_moveit_config
ros2 pkg prefix dual_nero_calibration
```

如果包不存在，重新编译：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 1.3 CAN 上电前检查

启用左右 CAN（每条命令都需要 root）：

```bash
cd /home/work/nero_duo_arm/deployment
sudo ./scripts/enable_can.sh can_left 1000000
sudo ./scripts/enable_can.sh can_right 1000000
```

只读确认接口已 UP：

```bash
ip -details link show can_left
ip -details link show can_right
```

不要依赖 `can0`、`can1` 的枚举顺序；本项目固定使用 `can_left` 和
`can_right`。如果接口不存在，重新检查 USB 适配器和稳定命名 udev 规则。

## 2. 启动方式

### 2.1 推荐：第一次启动使用 dry-run

该命令启动左右驱动、三路相机、MoveIt 和 Web app。`auto_enable:=false`
让电机保持失能，`moveit_dry_run:=true` 让轨迹桥只模拟执行：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true \
  moveit_dry_run:=true auto_enable:=false
```

浏览器打开：

```text
http://<机器人主机IP>:8080
```

查看机器人主机 IP：

```bash
hostname -I
```

只有本机访问时：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true \
  moveit_dry_run:=true auto_enable:=false http_host:=127.0.0.1
```

### 2.2 真机运动

只有在 dry-run、控制门、急停、Park 和现场安全确认全部通过后使用：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true \
  moveit_dry_run:=false auto_enable:=false
```

启动后页面顶部应显示 `MoveIt: ready (LIVE)`。如果显示
`MoveIt: ready (DRY-RUN)`，Send/Park 不会驱动真机。

### 2.3 分终端启动

驱动：

```bash
ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=false
```

相机：

```bash
ros2 launch dual_nero_cameras all_cameras.launch.py
```

当前默认配置为三路 Color `640×480 @ 30`；三路 Depth 默认全部关闭，因为
LeRobot/VLA 只消费 RGB。三路 Color 同时发布原始 Image 和 JPEG CompressedImage。
相机采集是 30 FPS；VLA 文档中的 `--fps=5`/`15` 是策略或 rollout 频率。

MoveIt 规划和执行桥：

```bash
ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true rviz:=false
```

仅规划、不执行（没有轨迹桥和真实控制器）：

```bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py
```

无界面规划：

```bash
ros2 launch dual_nero_moveit_config moveit_planning.launch.py gui:=false rviz:=false
```

只启动 Web app（驱动、相机和 MoveIt 已由其他终端启动）：

```bash
ros2 launch dual_nero_webapp webapp.launch.py
```

### 2.4 常用启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `start_drivers` | `false` | 是否同时启动左右 NERO 驱动 |
| `start_cameras` | `false` | 是否同时启动三路 RealSense |
| `start_moveit` | `false` | 是否启动 `move_group` 和轨迹桥 |
| `moveit_dry_run` | `true` | `true` 不向驱动发送轨迹；`false` 才控制真机 |
| `moveit_rviz` | `false` | MoveIt 启动时是否打开 RViz |
| `auto_enable` | `true` | 驱动启动时是否自动使能电机；首次调试建议显式设为 `false` |
| `use_compressed_color` | `true` | Web app 的 Color 默认订阅 JPEG 压缩流；设为 `false` 时改用 raw Image |
| `speed_percent` | `5` | NERO 驱动速度百分比 |
| `http_host` | `0.0.0.0` | Web app 监听地址 |
| `http_port` | `8080` | Web app 端口 |
| `initial_gate_open` | `false` | 只改变 Web app 本地门；驱动门仍独立默认关闭 |
| `enable_cameras` | `true` | 是否订阅并转发相机话题 |
| `can_left` / `can_right` | `can_left` / `can_right` | 左右 CAN 接口名 |

## 3. Web 控制台逐项说明

### 3.1 顶部状态栏

| 控件 | 功能 |
| --- | --- |
| `connected/disconnected` | 浏览器 WebSocket 与 Web app 的连接状态 |
| `gates: left/right` | 两侧 Web app 控制门当前状态 |
| `MoveIt: ready (DRY-RUN/LIVE)` | MoveIt action 可用性及执行模式 |
| `Cancel motion` | 取消当前轨迹桥目标和 MoveIt goal；不关门、不失能 |
| `Stop both` | 并行发送左右硬件急停，并关闭两侧控制门、取消运动 |
| 时钟 | Web app 服务器时间 |

`Cancel motion` 用于普通取消。人员、碰撞或失控风险出现时使用对应侧的
`E-stop` 或顶部 `Stop both`，不要只点 `Cancel motion`。

### 3.2 3D 视图

- 左键拖动旋转视角。
- 滚轮缩放。
- 关节滑条变化时，3D 模型同步预览目标姿态；预览本身不会移动真机。
- 3D 视图依赖 CDN 加载 Three.js；CDN 不可用时关节控制和相机仍可工作。

### 3.3 左臂/右臂面板

状态标签：

| 标签 | 含义 |
| --- | --- |
| `fb live` / `fb stale` | 关节反馈是否新鲜；运动要求小于 1.5 秒 |
| `gate open/closed` | 当前侧控制门 |
| `motors enabled/disabled` | 当前侧电机使能状态 |
| `arm ok` | 驱动报告 `arm_status=0` |
| `brakes set` | 电机失能时 `arm_status=6`，通常是抱闸未释放 |
| `brake stuck` | 已使能但仍有 `arm_status=6`，需要检查抱闸 |
| `fault: ...` | 关节驱动故障、通信错误或硬件限位标志 |

关节控制：

- `J1` 至 `J7` 是目标关节角，单位 rad。
- 滑条右侧第一列是命令值 `cmd`，第二列是机器人反馈 `fb`。
- 拖动滑条只改变目标和 3D 预览，不会立即运动。
- `Send` 才把 7 个目标交给 MoveIt 规划、碰撞检查并执行。

按钮：

| 按钮 | 功能 | 前置条件/注意事项 |
| --- | --- | --- |
| `Send` | 当前侧 7 关节目标送入 MoveIt 执行 | 门开、电机使能、反馈小于 1.5 秒、MoveIt 可用 |
| `Park` | 当前侧回到 `support_pose.yaml` 中的支撑姿态 | 同上，并弹出确认框 |
| `Sync from robot` | 把最新反馈复制到滑条 | 7 个关节反馈必须齐全 |
| `Zero sliders` | 把滑条和 3D 预览置零 | 不会移动真机；点击 `Send` 才会运动 |
| `Open/Close gate` | 调用 `/<side>/control_enable` | 关门会取消当前侧运动 |
| `Enable/Disable motors` | 调用 `/<side>/enable_agx_arm` | 使能会产生力矩；失能会弹窗确认 |
| `E-stop` | 当前侧硬件软件急停，随后关门并取消运动 | 急停不等于自动失能 |
| `Clear faults` | 调用 `/<side>/clear_joint_error` | 只清关节故障，不复位、不使能、不运动 |
| `Recover arm` | 依次复位运动控制器、清故障、重新使能 | 门必须关闭、反馈小于 2 秒、机械臂必须有人支撑 |

`Recover arm` 不会自动回到 Park。恢复后仍需重新打开门，再执行
`Sync from robot` 和 `Park`。

### 3.4 Both arms

| 按钮 | 功能 |
| --- | --- |
| `Both Send` | 将左右各 7 个目标合成一个 `both_arms` 的 14 关节 MoveIt goal |
| `Both Park` | 左右支撑姿态合成一个 14 关节同步 Park 轨迹 |

双臂动作要求两侧同时满足门开、电机使能、反馈新鲜和 MoveIt 可用。它不是
两个独立请求，而是一条同步轨迹，因此两臂不会各自抢先运动。

### 3.5 相机

- `Scene`、`Left wrist`、`Right wrist` 对应三路相机；Color 默认使用
  JPEG CompressedImage。Depth 默认不发布，启用后使用 raw Image。
- `color/depth` 按钮切换彩色和深度 MJPEG。
- 状态显示 `live/offline/stale`，底部显示采集 FPS、分辨率和帧龄。
- 画面断开时页面会自动重试；持续 `stale` 时检查 ROS 话题和 USB。
- 三路 Depth 当前默认关闭，因此 Depth 页面显示空流是预期行为；需要时分别用
  `enable_scene_depth:=true` 或 `enable_wrist_depth:=true` 开启并复测稳定性。
- 相机画面只用于观察，当前没有接入 MoveIt 动态避障。

## 4. 推荐操作顺序

### 4.1 启动和首次运动

1. 清空桌面和双臂运动范围，确认人员不进入工作区。
2. 启动 dry-run，打开页面并确认 `connected`。
3. 确认两侧 `fb live`、`gates: closed`、`MoveIt: ready (DRY-RUN)`。
4. 在 dry-run 下依次测试 `Sync from robot`、`Open gate`、`Enable motors`、
   小幅 `Send` 和 `Park`。dry-run 返回会明确写出 `no driver command sent`。
5. 关闭页面动作、关闭终端，重新按 **真机运动** 命令启动。
6. 两侧分别点击 `Open gate`。
7. 两侧分别点击 `Enable motors`，确认状态为 `motors enabled`。
8. 点击 `Sync from robot`，确认滑条与反馈一致。
9. 先执行单侧极小 `Send`，确认方向、速度和急停；再做双臂动作。
10. 需要双臂同步时，先确认两个滑条目标，再点 `Both Send`。

### 4.2 停止和退出

1. `Both Park`，等待执行完成，并用反馈确认确实到位。
2. 点击 `Stop both`，确认两侧控制门关闭。
3. 分别点击 `Disable motors`。
4. 在启动终端按 `Ctrl-C`。

如果 Park 失败，不要直接失能；先 `Stop both`，人工支撑机械臂，再根据
反馈决定是否失能。

## 5. HTTP 控制命令

下面的示例默认 Web app 在机器人主机本机运行。远程机器把
`127.0.0.1` 换成机器人 IP。先确认健康状态：

```bash
BASE=http://127.0.0.1:8080
curl -s "$BASE/api/health" | python3 -m json.tool
curl -s "$BASE/api/state" | python3 -m json.tool
curl -s "$BASE/api/motion" | python3 -m json.tool
```

所有 POST 都需要 JSON 请求头。若安装了 `jq`，可把
`python3 -m json.tool` 换成 `jq .`。

### 5.1 查询接口

| 命令 | 用途 |
| --- | --- |
| `curl -s "$BASE/api/config"` | 关节名、限位、Park 可用性、相机元数据 |
| `curl -s "$BASE/api/state"` | 两侧反馈、反馈年龄、门和电机状态、健康诊断 |
| `curl -s "$BASE/api/motion"` | MoveIt 是否可用、是否忙、dry-run/真机 |
| `curl -s "$BASE/api/cameras"` | 每路相机帧率、分辨率、帧龄 |
| `curl -s "$BASE/robot_urdf"` | 浏览器使用的展开后 URDF |

### 5.2 控制门

打开：

```bash
curl -s -X POST "$BASE/api/control_gate" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","enabled":true}'
```

关闭：

```bash
curl -s -X POST "$BASE/api/control_gate" \
  -H 'Content-Type: application/json' \
  -d '{"side":"right","enabled":false}'
```

`side` 只能是 `left` 或 `right`。

### 5.3 电机使能/失能

```bash
curl -s -X POST "$BASE/api/enable" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","enabled":true}'

curl -s -X POST "$BASE/api/enable" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left","enabled":false}'
```

使能会产生力矩。失能会取消该侧活动运动。

### 5.4 单臂 MoveIt 运动

`positions` 必须正好有 7 个数，顺序为 J1 到 J7，单位 rad。示例中的
`<J1>` 到 `<J7>` 必须替换为经过确认的目标值：

```bash
LEFT='[<J1>,<J2>,<J3>,<J4>,<J5>,<J6>,<J7>]'

curl -s -X POST "$BASE/api/move" \
  -H 'Content-Type: application/json' \
  -d "{\"side\":\"left\",\"positions\":$LEFT}"
```

右臂把 `"left"` 改为 `"right"`。服务端会把超出保守限位的值截断，但截断
不能替代现场检查。

### 5.5 双臂同步运动

```bash
LEFT='[<left_J1>,<left_J2>,<left_J3>,<left_J4>,<left_J5>,<left_J6>,<left_J7>]'
RIGHT='[<right_J1>,<right_J2>,<right_J3>,<right_J4>,<right_J5>,<right_J6>,<right_J7>]'

curl -s -X POST "$BASE/api/move" \
  -H 'Content-Type: application/json' \
  -d "{\"side\":\"both\",\"positions\":{\"left\":$LEFT,\"right\":$RIGHT}}"
```

### 5.6 Park

单侧：

```bash
curl -s -X POST "$BASE/api/park" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left"}'
```

双臂：

```bash
curl -s -X POST "$BASE/api/park" \
  -H 'Content-Type: application/json' \
  -d '{"side":"both"}'
```

Park 使用安装后的
`share/dual_nero_webapp/config/support_pose.yaml`。源文件是
`ros2_ws/src/dual_nero_webapp/config/support_pose.yaml`，修改后需要重新编译或
确认 symlink-install 已同步。

### 5.7 取消、急停和故障恢复

取消当前运动：

```bash
curl -s -X POST "$BASE/api/cancel_motion" \
  -H 'Content-Type: application/json' -d '{}'
```

单侧软件急停：

```bash
curl -s -X POST "$BASE/api/emergency_stop" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left"}'
```

双侧软件急停：

```bash
curl -s -X POST "$BASE/api/emergency_stop_all" \
  -H 'Content-Type: application/json' -d '{}'
```

清除关节故障（不运动、不复位、不使能）：

```bash
curl -s -X POST "$BASE/api/clear_faults" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left"}'
```

恢复机械臂（必须关门并支撑机械臂）：

```bash
curl -s -X POST "$BASE/api/recover" \
  -H 'Content-Type: application/json' \
  -d '{"side":"left"}'
```

### 5.8 WebSocket

`ws://<机器人主机IP>:8080/ws` 每秒约 10 次推送：

```json
{"type":"state","data":{"left":{},"right":{}},"motion":{}}
```

也可以从 WebSocket 发送运动请求。下面是结构示意，`<J1>` 等不是可执行值，
必须替换为经过确认的 7 个目标关节角：

```json
{"side":"left","positions":[<J1>,<J2>,<J3>,<J4>,<J5>,<J6>,<J7>]}
{"side":"both","positions":{"left":[<left_J1>,<left_J2>,<left_J3>,<left_J4>,<left_J5>,<left_J6>,<left_J7>],"right":[<right_J1>,<right_J2>,<right_J3>,<right_J4>,<right_J5>,<right_J6>,<right_J7>]}}
```

Web UI 自身使用 REST 提交运动。除非开发客户端，否则优先使用 UI 或
`/api/move`、`/api/park`。

## 6. ROS 2 检查和底层控制

### 6.1 节点、话题、服务和 action

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

只读查看一次反馈：

```bash
ros2 topic echo --once /left/feedback/joint_states sensor_msgs/msg/JointState
ros2 topic echo --once /right/feedback/joint_states sensor_msgs/msg/JointState
```

查看频率：

```bash
ros2 topic hz /left/feedback/joint_states
ros2 topic hz /right/feedback/joint_states
```

### 6.2 驱动服务

以下服务由每个 namespace 分别提供：

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/left/control_enable` | `std_srvs/SetBool` | 打开/关闭驱动控制门 |
| `/left/enable_agx_arm` | `std_srvs/SetBool` | 使能/失能电机 |
| `/left/emergency_stop` | `std_srvs/Empty` | 软件急停 |
| `/left/clear_joint_error` | `std_srvs/Trigger` | 清除关节驱动故障 |
| `/left/reset_motion` | `std_srvs/Trigger` | 复位运动控制器；门必须关闭 |
| `/left/move_home` | `std_srvs/Empty` | 驱动层回到全零位；Web app 不使用 |

把 `left` 换成 `right` 即可操作右臂。示例：

```bash
ros2 service call /left/control_enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /left/enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /left/clear_joint_error std_srvs/srv/Trigger "{}"
ros2 service call /left/reset_motion std_srvs/srv/Trigger "{}"
ros2 service call /left/emergency_stop std_srvs/srv/Empty "{}"
```

**不要用 `/move_home` 代替 Park。** 它绕过 MoveIt，直接去全零位，不是
经过验证的重力稳定支撑姿态。

直接向 `/left/control/joint_states` 发布 `JointState` 会绕过 MoveIt 碰撞
检查和 Web app 互锁。除专门的低速急停验收脚本外，不要把话题直发作为正常
控制方式。

### 6.3 关节限位

Web app 服务端会按以下保守范围截断 J1-J7（rad）：

| 关节 | 下限 | 上限 |
| --- | ---: | ---: |
| J1 | -2.705260 | 2.705260 |
| J2 | -1.760000 | 1.740000 |
| J3 | -2.750000 | 2.750000 |
| J4 | -1.010000 | 2.200000 |
| J5 | -2.750000 | 2.750000 |
| J6 | -0.7330383 | 0.8726646 |
| J7 | -1.5707963 | 1.5707963 |

## 7. 相机

### 7.1 启动和检查

当前默认 30 FPS 配置：

| 相机 | Color | Depth |
| --- | --- | --- |
| Scene / D435 | `640×480 YUYV @ 30` | 默认关闭 |
| Left wrist / D405 | `640×480 RGB8 @ 30` | 默认关闭 |
| Right wrist / D405 | `640×480 RGB8 @ 30` | 默认关闭 |

三台均设置 `enable_sync=false`、关闭 Infra 和 PointCloud；Color 由
`color_jpeg_relay` 以 quality 78 发布压缩流。

```bash
ros2 launch dual_nero_cameras all_cameras.launch.py
```

开启 Scene Depth 或双腕 Depth（会改变带宽配置，不能沿用当前 Color 30 FPS
验收结果）：

```bash
ros2 launch dual_nero_cameras all_cameras.launch.py \
  enable_scene_depth:=true

ros2 launch dual_nero_cameras all_cameras.launch.py \
  enable_wrist_depth:=true
```

两只腕部相机单独启动（默认 RGB-only）：

```bash
ros2 launch dual_nero_cameras d405_pair.launch.py
```

只读枚举 USB 和 RealSense 序列号：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/check_realsense.sh
```

确认三路彩色话题：

```bash
ros2 topic hz /camera_scene/color/image_raw
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic hz /camera_right_wrist/color/image_raw
```

确认三路 JPEG 压缩流：

```bash
ros2 topic hz /camera_scene/color/image_raw/compressed
ros2 topic hz /camera_left_wrist/color/image_raw/compressed
ros2 topic hz /camera_right_wrist/color/image_raw/compressed
```

如果临时开启双腕 Depth，再检查对应话题：

```bash
ros2 topic hz /camera_left_wrist/depth/image_rect_raw
ros2 topic hz /camera_right_wrist/depth/image_rect_raw
```

Web app 默认 Color 话题：

```text
/camera_scene/color/image_raw/compressed
/camera_left_wrist/color/image_raw/compressed
/camera_right_wrist/color/image_raw/compressed
```

启用 Depth 后额外订阅：

```text
/camera_scene/aligned_depth_to_color/image_raw
/camera_left_wrist/aligned_depth_to_color/image_raw
/camera_right_wrist/aligned_depth_to_color/image_raw
```

通过 Web app 查看实际 FPS 和分辨率：

```bash
curl -s http://127.0.0.1:8080/api/cameras | python3 -m json.tool
```

回退到 raw Color，或覆盖自定义 Color 话题：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  scene_color_topic:=/my_camera/color/image_raw \
  scene_color_compressed_topic:=/my_camera/color/image_raw/compressed \
  enable_cameras:=true

ros2 launch dual_nero_webapp webapp.launch.py \
  use_compressed_color:=false

ros2 launch dual_nero_webapp webapp.launch.py enable_cameras:=false
```

如果只覆盖 `scene_color_topic` 而保持 `use_compressed_color:=true`，Web app
仍会订阅默认压缩话题。自定义 raw 话题时，要么同时设置对应的
`*_color_compressed_topic`，要么显式设置 `use_compressed_color:=false`。

### 7.2 相机标定

先确认标定板配置：

```text
ros2_ws/src/dual_nero_calibration/config/calibration_board.yaml
```

当前为 12×9 方格、11×8 内角点、15 mm 边长。尺寸不对会直接导致外参错误。

固定场景相机采集和求解：

```bash
ros2 launch dual_nero_calibration capture.launch.py camera:=scene samples:=20

ros2 run dual_nero_calibration solve_extrinsics \
  --camera scene \
  --session /home/work/nero_duo_arm/deployment/calibration_data/scene/<timestamp>
```

腕部手眼标定：

```bash
ros2 launch dual_nero_calibration capture.launch.py camera:=left_wrist samples:=20
ros2 run dual_nero_calibration solve_extrinsics \
  --camera left_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/left_wrist/<timestamp>

ros2 launch dual_nero_calibration capture.launch.py camera:=right_wrist samples:=20
ros2 run dual_nero_calibration solve_extrinsics \
  --camera right_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/right_wrist/<timestamp>
```

发布外参 TF：

```bash
ros2 launch dual_nero_calibration publish_extrinsic.launch.py \
  yaml_files:="/path/camera_scene_extrinsic.yaml,/path/camera_left_wrist_extrinsic.yaml,/path/camera_right_wrist_extrinsic.yaml"
```

结果默认写入 `deployment/calibration_results/`。手眼建议验收阈值：
平移 RMS < 3 mm、旋转 RMS < 0.5°、重投影 RMS < 1 px。

## 8. Park、标准支撑位和自动复位

标准支撑位文件：

```text
deployment/config/support_pose.yaml
ros2_ws/src/dual_nero_webapp/config/support_pose.yaml
```

两份必须保持一致。自动复位脚本会自行启动两侧驱动和 MoveIt，因此运行前
必须先关闭现有 Web app、驱动和 MoveIt：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/reset_to_standard_support.sh --plan-only
```

完整执行并最终失能：

```bash
./deployment/scripts/reset_to_standard_support.sh --disable
```

不加 `--disable` 时，脚本只执行复位并保持电机使能，不执行完整失能验证：

```bash
./deployment/scripts/reset_to_standard_support.sh
```

脚本要求 `can_left`、`can_right` 已启用。失败时优先软件急停并保持人工支撑，
不要绕过脚本中的稳定性和硬件限位检查。

## 9. UMI 数据转换到 LeRobot v3

该阶段只转换数据，不需要机械臂运动。LeRobot 与 ROS 使用不同的 Python
环境，LeRobot 安装到 `/home/work/.venvs/umi_lerobot`。

安装或重建环境：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_umi_lerobot_env.sh
```

先检查输入数据和夹爪宽度范围：

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /path/to/umi_session.zarr.zip \
  --fps 30 \
  --task "pick up the block" \
  --dry-run
```

转换：

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /path/to/umi_session.zarr.zip \
  -o /home/work/datasets/lerobot/umi_pick \
  --repo-id local/umi_pick \
  --fps 30 \
  --task "pick up the block" \
  --gripper-min 0.0 \
  --gripper-max 0.09 \
  --pose-transform /path/to/T_robot_from_umi.json
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `--episodes 0,2-4` | 只转换指定 episode |
| `--max-episodes N` | 限制转换数量，用于冒烟测试 |
| `--camera camera1_rgb` | 选择相机；可重复指定 |
| `--task-file tasks.json` | 按 episode 提供语言指令 |
| `--robot-index 1` | 双臂 zarr 中只取一个 robot |
| `--no-videos` | 保存逐帧图像而不是 MP4 |
| `--resize 224x224` | 训练前缩放图像 |
| `--overwrite` | 把已有输出目录移到时间戳 `.bak` 后重写 |
| `--include-state` | 额外写入 `observation.state`；默认 UMI 契约不写 |

当前转换输出约定：

```text
action = [ee.x, ee.y, ee.z, ee.wx, ee.wy, ee.wz, ee.gripper_pos]
```

位置单位为米，旋转为轴角/旋转向量，夹爪 `0=闭合`、`1=张开`；
`action[t]` 默认目标帧为 `t+1`。

当前仓库已实现 LeRobot `nero_ros` 插件和 VLA 绝对末端动作到 MoveIt IK /
短轨迹的 ROS bridge；SmolVLA 训练命令、真实 checkpoint 推理验收和夹爪命令
仍未完成。运动执行边界和上机步骤见
[阶段 10：VLA 动作到 NERO 运动执行](14_stage10_vla_motion.md)。

## 10. 验收和开发脚本

先关闭 Web app 和已有 MoveIt，只保留脚本需要的前置进程。下面脚本分为
只读/规划测试和真机运动测试，不要混跑。

### 10.1 只读和规划

```bash
cd /home/work/nero_duo_arm

./deployment/scripts/check_stage0.sh
./deployment/scripts/check_realsense.sh
./deployment/scripts/check_nero_feedback.sh can_left
./deployment/scripts/check_nero_feedback.sh can_right
./deployment/scripts/stage2_check_dual_namespace.sh
```

MoveIt 规划与碰撞检查：

```bash
python3 deployment/scripts/test_dual_moveit_plan.py
python3 deployment/scripts/test_dual_collision.py
```

轨迹桥 dry-run：

```bash
python3 deployment/scripts/test_moveit_execution_dry_run.py --group left_arm
python3 deployment/scripts/test_moveit_execution_dry_run.py --group both_arms
python3 deployment/scripts/test_trajectory_bridge_dry_run.py
python3 deployment/scripts/test_trajectory_cancel_dry_run.py
```

### 10.2 真机运动测试

**真机运动。** `soft_estop_motion_test.py` 会直接发布控制话题：

```bash
python3 deployment/scripts/soft_estop_motion_test.py --namespace left
python3 deployment/scripts/soft_estop_motion_test.py --namespace right
python3 deployment/scripts/dual_soft_estop_motion_test.py
```

`reset_to_standard_support.sh` 会自行启停驱动和 MoveIt，执行前必须确认
没有其他控制进程占用 CAN 和 action：

```bash
./deployment/scripts/reset_to_standard_support.sh --plan-only
```

真实轨迹取消脚本同样会启动自己的驱动和 MoveIt：

```bash
./deployment/scripts/test_trajectory_cancel_real.sh
```

只有在人员监护、工作区清空、稳定支撑姿态和软件急停全部确认后才执行这些
真机脚本。

## 11. 常见问题

### 页面显示 `disconnected`

```bash
ss -ltnp | grep ':8080'
ros2 node list | grep dual_nero_webapp
curl -s http://127.0.0.1:8080/api/health
```

检查 Web app 是否启动、端口是否冲突、浏览器访问的 IP 是否正确。

### `MoveIt: unavailable`

```bash
ros2 action list | grep -E 'move_action|follow_joint_trajectory'
ros2 node list | grep -E 'move_group|trajectory_bridge'
```

启动 `start_moveit:=true`，或单独启动
`dual_nero_moveit_config moveit_execution.launch.py`。

### 点击 `Send` 没有运动

按顺序检查：

1. 页面是否显示 `MoveIt: ready (LIVE)`；如果还是 `DRY-RUN`，不会驱动真机。
2. 当前侧是否 `gate open`、`motors enabled`、`fb live`。
3. `app.motion.available` 是否为 `true`。
4. 目标是否在关节限位内。
5. 是否弹出错误 toast；同时查看 Web app 终端中的 MoveIt 错误码。
6. 是否仍有前一次 motion busy 或轨迹桥取消锁存。

服务端每次新运动前都会调用
`/dual_nero_trajectory_bridge/clear_cancel`；如果它不可用，会拒绝运动。

### `Park` 按钮不可用

```bash
curl -s http://127.0.0.1:8080/api/config \
  | python3 -m json.tool | grep -A20 '"park"'
```

检查支撑位文件是否存在、两侧是否各有 7 个有限数值、是否超出关节限位。
修改支撑位后确认安装目录已同步。

### `brakes set`、`brake stuck` 或失能后震颤

- 电机失能时 `brakes set` 通常正常。
- 电机使能后仍显示 `brake stuck`，检查抱闸和机械支撑。
- 失能后有短促下沉或震颤，先人工支撑，不要反复使能/失能。
- `Clear faults` 只清错误位，不解决抱闸保持或机械间隙。
- `Recover arm` 会重新使能并可能突然产生力矩；门必须关闭。

### 相机 `offline/stale`

```bash
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic echo --once /camera_left_wrist/color/camera_info
./deployment/scripts/check_realsense.sh
```

检查序列号、USB 速率、话题名和 Web app 参数。三路相机同时运行时还要检查
是否有 `Incomplete video frame` 或 USB 断开。

### 3D 模型空白

打开浏览器开发者工具，检查 CDN 的 Three.js/urdf-loader 请求。CDN 不可用时
3D 预览缺失，但关节控制和相机不受影响。

### 取消后下一次运动被拒

等待当前 MoveIt result 返回；确认轨迹桥 `clear_cancel` 服务存在。必要时
重新启动轨迹桥和 Web app，不要直接连续点击 Send。

## 12. 尚未实现

- 相机点云、OctoMap、实时动态障碍和 MoveIt planning scene 编辑；
- Cartesian/IK/命名位姿目标和浏览器轨迹预览；
- 夹爪规划组与夹爪控制；
- Web 端速度/加速度调节（当前运动缩放固定约 `0.05`）；
- 登录、TLS 和权限控制；
- 真实双臂 SmolVLA checkpoint 的整链路验收（当前只验证到假 IK / action server）；
- UMI zarr 转换器仍默认 7-D 轴角，20-D rot6d 双臂训练数据与坐标系对齐需使用
  Xense 原始链路并单独验收；
- 真实物理急停和完整硬件诊断面板。

