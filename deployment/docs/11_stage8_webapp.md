# 阶段 8：双臂 Web 控制台

> 记录日期：2026-11-02  
> 状态：**Send/Park 已接入 MoveIt `/move_action`；dry-run 联锁、Park、取消与 Stop both 已通过，待真机验收**

完整按钮说明、HTTP/ROS 控制命令和故障排查见
[完整使用手册与命令速查](12_usage_and_commands.md)。

## 1. 新增内容

新增 ROS 2 package：

```text
ros2_ws/src/dual_nero_webapp/
```

功能：

- 浏览器中的双臂 3D 模型，使用与 ROS 相同的
  `two_nero_tabletop.urdf.xacro`；
- 左右臂各 7 个关节滑条和实时反馈；
- **Send** 和 **Park** 均经 MoveIt `/move_action` 规划、静态碰撞检查并由
  `dual_arm_controller` 执行；
- 支持左右臂独立运动和 **Both Send / Both Park** 同步双臂运动；
- 左右臂及双臂 **Park** 均从安装后的 `config/support_pose.yaml` 读取支撑姿态；
- 双侧控制门、使能/失能和软件急停；
- 场景、左腕、右腕三路 RealSense 画面，可切换 Color/Depth；
- 相机画廊按屏幕宽度自动换行，完整显示画面，页面可纵向滚动。

Web app 不再直接发布关节目标。每次运动先做控制门、电机使能、反馈新鲜度和
关节限位检查，再向 MoveIt 发送 `MoveGroup` goal。MoveIt 使用 `left_arm` /
`right_arm` / `both_arms` 规划组，规划场景包含完整双臂机器人、桌面和已配置的
静态障碍，然后通过 `dual_nero_control` 轨迹桥和 `dual_arm_controller` 执行。
双臂运动是**一个包含 14 个关节约束的 MoveIt goal**，两臂沿同一条同步轨迹执行，
不是同时发起两个独立请求。

当前碰撞检查是**静态**的：URDF 机器人、桌面和静态 planning scene。相机点云、
OctoMap 和实时动态障碍尚未接入；现场仍需保持工作区无人、无障碍。

轨迹执行中的取消由 `/dual_nero_trajectory_bridge/cancel_motion` 完成。只取消
MoveIt 的 `/move_action` goal 不能可靠中断已经下发给控制器的轨迹，因此
Web app 会先取消轨迹桥，再取消 MoveIt goal。轨迹桥会锁存取消状态，防止
刚启动的执行回调重新开门；Web app 在每次新运动前调用
`/dual_nero_trajectory_bridge/clear_cancel` 清除锁存。

## 2. 安装与编译

安装脚本已加入 `python3-fastapi`、`python3-uvicorn` 和 WebSocket 所需的
`python3-wsproto`：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_native_dependencies.sh
```

编译：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_nero_control dual_nero_webapp --symlink-install
source install/setup.bash
```

## 3. 启动

只启动 Web app（驱动、相机和 MoveIt 由其他终端启动）：

```bash
ros2 launch dual_nero_webapp webapp.launch.py
```

连同驱动、相机和 MoveIt 一起启动（MoveIt 默认 dry-run）：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true
```

`moveit_dry_run` 默认 `true`，轨迹桥只模拟执行，不向驱动发送控制目标。确认真机
方向和联锁后再改为：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true \
  start_moveit:=true moveit_dry_run:=false
```

没有 MoveIt 时，页面显示 `MoveIt: unavailable`，Send/Park 会被拒绝；不会回退
到无碰撞检查的直接关节控制。

页面顶部会明确显示 `MoveIt: ready (DRY-RUN)` 或 `MoveIt: ready (LIVE)`。
dry-run 下 Send/Park 完成后提示会写明 **no driver command sent**，避免把模拟
成功误认为机械臂已经运动。

浏览器打开：

```text
http://<机器人主机IP>:8080
http://10.10.11.146:8080
```

默认监听 `0.0.0.0`。只允许本机访问时：

```bash
ros2 launch dual_nero_webapp webapp.launch.py http_host:=127.0.0.1
```

## 4. 安全操作顺序

1. 确认桌面、双臂和相机附近无人，急停方式可用。
2. 启动驱动后确认两侧反馈为 `fb live`。
3. 点击对应侧的 **Open gate**，打开 Web app 和驱动控制门。
4. 用 **Sync from robot** 将滑条同步到当前姿态。
5. 修改目标关节后点击 **Send**。目标先在服务端按 NERO 驱动/URDF 的保守
   关节限位截断，再由 MoveIt 规划、碰撞检查并执行。
   左右臂目标都确认后，可点击 **Both Send** 执行一条 14 关节同步轨迹。
6. 需要回到支撑姿态时点击该侧 **Park**，并确认弹窗。Park 仅在控制门已开、
   电机已标记使能、反馈小于 1.5 秒且配置姿态通过关节限位校验时发送。
   **Both Park** 要求两侧均满足同一条件。
7. 需要停止时立即点击该侧 **E-stop**，或使用顶部 **Stop both**。顺序为：
   先发送驱动 `emergency_stop`，再关闭控制门，取消轨迹桥执行并取消 MoveIt
   goal。**Stop both** 并行下发左右硬件急停，避免串行等待。

Park 将支撑姿态作为 MoveIt 目标执行。接口成功返回表示 MoveIt 已完成规划和
执行；仍需以新反馈确认实际到位。静态模型之外的现场障碍不会被检测。

每侧状态栏还会显示 `arm_status`/`motor_status` 诊断摘要。电机失能时
`arm_status=6`（关节抱闸未打开）显示为 `brakes set`，这是失能状态的正常
反馈；若电机使能后仍显示 `brake stuck`，才需要检查抱闸。驱动新增：

- `/<side>/clear_joint_error`：发送关节驱动故障清除命令，不改变使能、抱闸
  或运动状态；
- `/<side>/reset_motion`：控制门关闭时复位运动控制器状态；
- Webapp **Clear faults** 只调用清错命令；
- Webapp **Recover arm** 必须先关闭控制门并确认机械臂有支撑，之后依次复位
  运动控制器、清除关节故障并重新使能电机，不会自动运动到 Park。

失能后出现短促下沉或震颤时，不要反复使能/失能；应先支撑机械臂，再从
`arm_status`、`motor_status` 和左右关节差值判断是抱闸保持不足、机械间隙，
还是控制器状态问题。

关闭控制门、失能电机、单侧急停和 Stop both 都会取消正在执行的轨迹桥目标与
MoveIt goal。顶部另有独立的 **Cancel motion** 按钮，只取消当前运动，不关闭
控制门或失能电机。

驱动 `dual_nero_drivers.launch.py` 默认 `control_enabled=false`，因此仅启动
Web app 并不会自动允许运动。Web app 自身的控制门也默认关闭，形成第二道
软件互锁。

## 5. 相机话题

Web app 不直接打开 RealSense 设备，而是订阅 `dual_nero_cameras` 已经发布的
ROS 话题：

```text
/camera_scene/color/image_raw
/camera_scene/aligned_depth_to_color/image_raw
/camera_left_wrist/color/image_raw
/camera_left_wrist/aligned_depth_to_color/image_raw
/camera_right_wrist/color/image_raw
/camera_right_wrist/aligned_depth_to_color/image_raw
```

如果想确认相机是否真正出帧：

```bash
ros2 topic hz /camera_scene/color/image_raw
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic hz /camera_right_wrist/color/image_raw
```

## 6. 已验证

- `colcon build --packages-select dual_nero_webapp` 通过；
- `colcon build --packages-select dual_nero_control dual_nero_webapp` 通过；
- `ament_flake8`（control + webapp）、Python 编译和浏览器 JavaScript 语法检查
  通过；
- 无相机、无驱动时 `/api/health`、`/api/config`、`/api/state` 正常；
- `/robot_urdf` 成功展开 `two_nero_tabletop.urdf.xacro`；
- 首页静态资源可访问；
- 控制门关闭时 WebSocket 命令被拒绝；
- `/api/park` 在控制门关闭、电机未使能或反馈过期时拒绝发送；
- `/api/config` 正确报告左右 Park 姿态是否可用；
- 使用假 ROS Image 话题验证 Color/Depth MJPEG 均返回有效 JPEG 帧；
- 相机话题 launch 参数覆盖生效。
- `--symlink-install` 下 `/static/*` 与 `agx_arm_description` 网格资源可正常加载。
- MoveIt dry-run + 假驱动验证左臂 Park 成功，返回 7 个支撑位姿关节值；
- 右臂 Park 执行中调用 E-stop，硬件急停服务先被调用，随后轨迹桥取消；
  HTTP 返回约 55 ms，Park 请求返回 `CONTROL_FAILED`，没有残留 busy 状态；
- Stop both 并行调用两侧硬件急停，左右均返回 `stop_confirmed=true` 且
  `gate_closed=true`；
- 轨迹桥日志确认 `Trajectory cancelled by cancel service`，MoveIt 报告
  `PATH_TOLERANCE_VIOLATED`，随后控制器停止；下一次运动前日志确认
  `Trajectory cancellation latch cleared`。

## 7. 待真机验收

- [ ] 两侧控制门与急停按钮的真机联锁；
- [ ] 左右 Park 姿态、运动方向、速度与到位反馈的真机验收；
- [ ] 滑条命令方向与关节限位现场确认；
- [ ] 三路相机 Color/Depth 完整画面、帧率和延迟；
- [ ] 小屏和低分辨率下相机画廊与页面滚动；
- [ ] 长时间运行下 FastAPI/MJPEG 与 ROS 回调的稳定性。

## 8. 尚未实现

- 相机点云、OctoMap 和实时动态障碍防碰撞；
- 相机 TF 接入 MoveIt 的 `world` 规划树和 planning scene 障碍物管理界面；
- 规划前轨迹预览、碰撞预检查 UI、浏览器内轨迹可视化；
- 笛卡尔目标、IK 目标、自定义命名姿态；
- 夹爪规划组和夹爪 Web 控制；
- 速度/加速度调节界面；当前固定为 `0.05`；
- MoveIt 执行进度、真实电机诊断、温度/错误状态展示；
- 登录认证、TLS 和权限控制。

## 9. 已知限制

- Web app 没有登录认证，只应在可信局域网使用。
- 3D 视图依赖 CDN 的 Three.js/urdf-loader；CDN 不可用时关节控制仍可用。
- MJPEG 使用三路并发视频连接，带宽高于 WebRTC，但不额外争抢相机设备。
- Web app 的控制门状态是进程内状态；如果其他工具直接调用驱动
  `control_enable`，以 ROS 服务结果为准。
- 非 dry-run 轨迹结束后，轨迹桥会关闭驱动控制门，但 Web app 的本地
  `gate_open` 可能仍显示为打开；再次运动前应切换一次控制门或重启 Web app。
- 当前没有 3D sensor plugin，MoveIt 会记录
  `No 3D sensor plugin(s) defined for octomap updates`；相机 frame 也尚未
  连通到 `world`。因此现有防碰撞只覆盖静态模型。
