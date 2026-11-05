# 相机接入与标定

> 当前配置：三路 Color `640×480@30`，Depth 默认关闭。标定需在相机最终安装后执行。

## 1. 安装

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_native_dependencies.sh
./deployment/scripts/install_realsense_udev.sh
```

已安装包：

```text
ros-humble-realsense2-camera
ros-humble-realsense2-description
ros-humble-realsense2-camera-msgs
ros-humble-librealsense2
v4l-utils
```

安装 udev 规则后重新插拔相机。

## 2. 设备与话题

| 角色 | 型号 | 序列号 | 话题前缀 |
| --- | --- | --- | --- |
| 场景 | D435 | `263422075415` | `/camera_scene` |
| 左腕 | D405 | `260522270991` | `/camera_left_wrist` |
| 右腕 | D405 | `261122272091` | `/camera_right_wrist` |

每路发布：

```text
/<role>/color/image_raw
/<role>/color/image_raw/compressed
/<role>/color/camera_info
/<role>/depth/image_rect_raw        # 默认关闭
/<role>/aligned_depth_to_color/image_raw  # 默认关闭
```

枚举和检查：

```bash
rs-enumerate-devices -s
./deployment/scripts/check_realsense.sh
```

## 3. 启动三路相机

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_cameras all_cameras.launch.py
```

检查 Color 和压缩流：

```bash
ros2 topic hz /camera_scene/color/image_raw
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic hz /camera_right_wrist/color/image_raw
ros2 topic hz /camera_scene/color/image_raw/compressed
ros2 topic hz /camera_left_wrist/color/image_raw/compressed
ros2 topic hz /camera_right_wrist/color/image_raw/compressed
```

需要 Depth 时单独开启：

```bash
ros2 launch dual_nero_cameras all_cameras.launch.py enable_scene_depth:=true
ros2 launch dual_nero_cameras all_cameras.launch.py enable_wrist_depth:=true
```

USB 要求：

- 优先使用 USB 3.x 直连，避免 USB2 Hub 或充电线。
- 三台同时运行时检查 `Incomplete video frame`、帧超时和 USB 带宽。
- D435 在 USB2 下必须关闭 Infra 流，否则 Depth 可能持续超时。

## 4. 标定包

```text
ros2_ws/src/dual_nero_calibration
```

可执行程序：

```text
capture_chessboard   # 采集棋盘格与 TF 数据
solve_extrinsics     # 求解固定相机或手眼外参
publish_extrinsic    # 发布 static TF
```

输出 TF：

| 相机 | Parent | Child |
| --- | --- | --- |
| D435 | `world` | `camera_scene_link` |
| 左腕 D405 | `left_tool0` | `camera_left_wrist_link` |
| 右腕 D405 | `right_tool0` | `camera_right_wrist_link` |

## 5. 标定板

当前配置：

```text
棋盘方格: 12 x 9
OpenCV 内角点: 11 x 8
方格边长: 0.015 m
```

配置文件：`dual_nero_ws/src/dual_nero_calibration/config/calibration_board.yaml`。
尺寸填错会直接导致尺度错误。

## 6. D435 场景相机标定

1. 固定标定板并记录它在 `world` 中的位姿，写入
   `config/calibration_targets.yaml` 的 `scene.board_pose_in_world`。
2. 采集 15–20 个不同角度：

```bash
ros2 launch dual_nero_calibration capture.launch.py \
  camera:=scene samples:=20
```

3. 求解：

```bash
ros2 run dual_nero_calibration solve_extrinsics \
  --camera scene \
  --session /home/work/nero_duo_arm/deployment/calibration_data/scene/<timestamp>
```

## 7. 腕部 D405 手眼标定

标定板固定不动，移动机械臂到 15–20 个不同位姿，每个位姿保证标定板完整可见。
采集节点要求相邻位姿平移至少 `2 cm` 或旋转至少 `2°`。

```bash
ros2 launch dual_nero_calibration capture.launch.py \
  camera:=left_wrist samples:=20

ros2 run dual_nero_calibration solve_extrinsics \
  --camera left_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/left_wrist/<timestamp>
```

右腕把 `left_wrist` 改为 `right_wrist`。

## 8. 发布标定 TF

```bash
ros2 launch dual_nero_calibration publish_extrinsic.launch.py \
  yaml_files:="/path/camera_scene_extrinsic.yaml,/path/camera_left_wrist_extrinsic.yaml,/path/camera_right_wrist_extrinsic.yaml"
```

验收阈值：

```text
手眼平移一致性 RMS: < 3 mm
手眼旋转一致性 RMS: < 0.5 deg
重投影 RMS:         < 1.0 px
```

相机安装位置变化后，原手眼矩阵失效，必须重新标定。
