# Web 控制台

> Web app 通过 MoveIt `/move_action` 规划执行，默认 dry-run。页面不直接发布
> 关节目标，也不回退到无碰撞检查的直接控制。

## 1. 功能

- 双臂 3D 模型，约 10 Hz 跟随反馈；滑条操作时临时显示目标预览。
- 左右 7 关节滑条、Send、Park、Both Send / Both Park。
- Park 从 `config/support_pose.yaml` 读取支撑姿态。
- 双侧控制门、使能/失能、软件急停、Stop both。
- 夹爪开合/任意位置、Speed、Clear obstruction，与机械臂急停联动。
- 三路 RealSense Color/Depth 画廊；Depth 默认关闭。

## 2. 安装与编译

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_native_dependencies.sh

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_nero_control dual_nero_webapp --symlink-install
source install/setup.bash
```

## 3. 启动

只启动 Web app：

```bash
ros2 launch dual_nero_webapp webapp.launch.py
```

连同驱动、相机、MoveIt 一起启动（默认 dry-run）：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true
```

真机模式：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_cameras:=true start_moveit:=true \
  moveit_dry_run:=false auto_enable:=false
```

浏览器打开 `http://<机器人主机IP>:8080`。只允许本机访问时加
`http_host:=127.0.0.1`。页面顶部会显示 `MoveIt: ready (DRY-RUN)` 或
`MoveIt: ready (LIVE)`；dry-run 下 Send/Park 不会驱动真机。

## 4. 安全操作顺序

1. 确认工作区无人、急停入口可用。
2. 启动驱动，确认两侧反馈 `fb live`。
3. 打开对应侧 `Open gate`。
4. `Sync from robot` 同步滑条。
5. 修改目标后 `Send`；双臂目标确认后 `Both Send`。
6. 运动前确认 `MoveIt: ready (LIVE)`、`gate open`、`motors enabled`。
7. 异常时先 `E-stop` / `Stop both`，不要连续点击 Send。

## 5. 相机

Web app 默认消费三路 Color；Depth 默认关闭。检查：

```bash
ros2 topic hz /camera_scene/color/image_raw
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic hz /camera_right_wrist/color/image_raw
curl -s http://127.0.0.1:8080/api/cameras | python3 -m json.tool
```

需要 Depth 时分别加 `enable_scene_depth:=true` 或
`enable_wrist_depth:=true`，并单独复测画面和帧率。

## 6. 已知限制

- 无登录认证，只应在可信局域网使用。
- 3D 视图依赖 CDN 的 Three.js/urdf-loader；CDN 不可用时关节控制仍可用。
- 碰撞检查只覆盖 URDF、桌面和静态 planning scene，不含相机点云/OctoMap。
- 非 dry-run 轨迹结束后驱动控制门会被轨迹桥关闭，页面 `gate_open` 可能仍显示
  打开；再次运动前切换一次控制门或重启 Web app。
- 速度/加速度界面固定约 `0.05`，暂无闭环力控和在线障碍管理。
