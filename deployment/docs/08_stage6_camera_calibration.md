# 阶段 6：相机外参标定框架

> 开始日期：2026-11-02  
> 状态：棋盘格标定框架已建立；相机尚未安装到最终位置，暂不能进行最终外参标定。

## 1. 标定包

```text
dual_nero_ws/src/dual_nero_calibration
```

可执行程序：

```text
capture_charuco    # 采集 ChArUco 图像与机器人/TF 数据
solve_extrinsics   # 求解固定相机或手眼外参
publish_extrinsic  # 将解算结果发布为 static TF
```

## 2. 坐标定义

```text
world: 桌面平面、左右基座中心中点，+X 指向桌内，+Y 指向左
```

输出 TF：

| 相机 | Parent | Child |
| --- | --- | --- |
| D435 场景相机 | `world` | `camera_scene_link` |
| 左腕 D405 | `left_tool0` | `camera_left_wrist_link` |
| 右腕 D405 | `right_tool0` | `camera_right_wrist_link` |

Realsense 驱动继续发布 `camera_*_link -> camera_*_color_optical_frame` 的固定光学坐标变换。

## 3. 标定板

当前使用棋盘格：

```text
棋盘方格: 12 x 9
OpenCV 内角点: 11 x 8
方格边长: 0.015 m（15 mm）
```

配置：

```text
dual_nero_ws/src/dual_nero_calibration/config/calibration_board.yaml
```

如果“12 x 9”指的是内角点数量，而不是方格数量，需要改为：

```yaml
pattern_cols: 12
pattern_rows: 9
```

已按 `12×9 方格、15 mm 边长` 配置。棋盘格尺寸错误会直接导致外参尺度和深度错位。

采集程序：

```text
capture_chessboard
```

框架仍保留 ChArUco 兼容分支，但当前默认 `type: chessboard`。

## 4. 启动相机

三台相机稳定后：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_cameras all_cameras.launch.py
```

## 5. D435 场景相机标定

1. 将标定板固定在桌面已知位置。
2. 修改 `config/calibration_targets.yaml` 中：

```yaml
scene:
  board_pose_in_world:
    xyz: [x, y, z]
    rpy: [roll, pitch, yaw]
```

3. 采集 15–20 个不同角度的标定板图像：

```bash
ros2 launch dual_nero_calibration capture.launch.py \
  camera:=scene samples:=20
```

4. 求解：

```bash
ros2 run dual_nero_calibration solve_extrinsics \
  --camera scene \
  --session /home/work/nero_duo_arm/deployment/calibration_data/scene/<timestamp>
```

## 6. 左右腕 D405 手眼标定

标定板固定在桌面/工作区内，保持不动。

对每一侧：

1. 通过 MoveIt 或手动示教移动机械臂到 15–20 个不同位置。
2. 每个位置确保标定板完整出现在腕部相机中。
3. 采集：

```bash
ros2 launch dual_nero_calibration capture.launch.py \
  camera:=left_wrist samples:=20
```

右腕：

```bash
ros2 launch dual_nero_calibration capture.launch.py \
  camera:=right_wrist samples:=20
```

4. 求解：

```bash
ros2 run dual_nero_calibration solve_extrinsics \
  --camera left_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/left_wrist/<timestamp>

ros2 run dual_nero_calibration solve_extrinsics \
  --camera right_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/right_wrist/<timestamp>
```

采集节点会自动要求机器人位姿变化：

```text
平移 >= 2 cm 或旋转 >= 2°
```

棋盘格必须完整出现在图像中；OpenCV 棋盘格检测不支持只看到部分角点。

## 7. 输出

默认输出：

```text
dual_nero_deployment/calibration_results/<camera>_extrinsic.yaml
```

其中包含：

- parent/child frame
- translation
- quaternion
- RPY
- 样本数
- 平移/旋转一致性 RMS
- 生成时间

## 8. 发布标定 TF

```bash
ros2 launch dual_nero_calibration publish_extrinsic.launch.py \
  yaml_files:="/path/camera_scene_extrinsic.yaml,/path/camera_left_wrist_extrinsic.yaml,/path/camera_right_wrist_extrinsic.yaml"
```

## 9. 验收建议

```text
手眼平移一致性 RMS:       < 3 mm
手眼旋转一致性 RMS:       < 0.5 deg
重投影 RMS:               < 1.0 px
```

如果误差偏大：

- 检查标定板打印尺寸。
- 增加样本数量和姿态多样性。
- 避免标定板模糊、反光或只出现在图像边缘。
- 检查机器人工具坐标 frame 是否正确。
- 检查 camera_link -> optical frame 是否来自当前驱动配置。

## 10. 合成自测

已使用已知 `T_tool_camera` 合成 12 组手眼数据验证求解器：

```text
translation_error_m: 1.23e-16
rotation_error_deg:  1.05e-14
```

说明 OpenCV `calibrateHandEye` 的坐标方向处理正确。

## 11. 当前阻塞

- 新 USB 3.2 Gen2 Hub 尚未到位。
- 三相机同时运行仍可能 incomplete frame。
- 标定板实物尺寸和场景相机安装位置尚需现场确认。

## 12. 框架验证

采集节点启动与超时退出已验证：

```text
Capturing 1 samples for scene
Calibration capture timeout
capture finished
```

完整合成数据 CLI 测试：

```text
samples: 10
TRANSLATION [0.1, -0.02, 0.06]
TRANSLATION_RMS_M: 2.74e-16
ROTATION_RMS_DEG: 3.36e-14
REPROJECTION_RMS_PX: 0.0
```

输出 YAML 已正确写入，包括 parent/child frame、平移、四元数、RPY、样本数和误差指标。


## 13. 当前前置条件

当前 D435、左 D405、右 D405 尚未安装到机械臂或最终场景支架上。

因此：

- 不执行最终手眼/世界外参标定。
- 不生成会进入生产 TF 树的相机外参。
- 相机安装改变后，已标定的手眼矩阵会失效。

安装要求：

- 左 D405：通过刚性支架固定在左腕/工具附近，不遮挡夹爪和相机视野。
- 右 D405：通过刚性支架固定在右腕/工具附近，左右安装方式尽量对称。
- D435：固定在最终场景位置，视野覆盖桌面作业区。
- 线缆需留出运动余量，但不能牵拉相机或改变安装角度。

安装完成并确认支架不会松动后，按以下顺序标定：

1. 左腕 D405 手眼标定。
2. 右腕 D405 手眼标定。
3. D435 世界外参标定。
