# 相机接入与标定

> 当前配置：三路 Color `640×480@30`，Depth 默认关闭。标定需在相机最终安装后执行。

## 0. 标定顺序

建议按以下顺序执行，前一步不通过就不要进入下一步：

1. 确认标定板参数（第 5 节）；
2. 检查三路相机、机器人 TF、夹爪反馈（第 6 节）；
3. 标定左右腕 D405 手眼外参（第 8 节）；
4. 如需固定场景相机，再标 D435 外参（第 7 节）；
5. 标定左右夹爪 TCP（第 10 节）；
6. 标定左右夹爪开度（第 11 节）；
7. 写入驱动/URDF/夹爪参数，重启并验收（第 9、10、11 节）。

任一 `CALIBRATION` 不是 `PASS`，或夹爪没有新鲜反馈时，不要发布结果或投入
自动抓取。

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
capture_charuco      # 采集棋盘格/ChArUco 与 TF 数据
capture_chessboard   # capture_charuco 的兼容别名
solve_extrinsics     # 求解固定相机或手眼外参，并检查数据质量
publish_extrinsic    # 发布 static TF
```

RealSense 的 `camera_info` 已包含出厂内参；当前流程标定的是相机外参。
只有更换镜头、修改分辨率后内参明显漂移，或需要自定义内参时，才另外做
OpenCV 内参标定。

输出 TF：

| 相机 | Parent | Child |
| --- | --- | --- |
| D435 | `world` | `camera_scene_link` |
| 左腕 D405 | `left_tool0` | `camera_left_wrist_link` |
| 右腕 D405 | `right_tool0` | `camera_right_wrist_link` |

## 5. 标定板参数

现场标定板规格：`12×9`，方格边长为 `15 mm`，类型为普通棋盘格。

先确认 `12×9` 指的是“方格数”还是“OpenCV 内角点”：

| 12×9 的含义 | `pattern_cols` | `pattern_rows` | 说明 |
| --- | ---: | ---: | --- |
| 12×9 个棋盘方格 | `11` | `8` | 当前仓库默认值；内角点比方格少一行一列 |
| 12×9 个内角点 | `12` | `9` | 实际棋盘是 13×10 个方格 |

配置文件：`ros2_ws/src/dual_nero_calibration/config/calibration_board.yaml`。
当前内容按“12×9 个方格”配置：

```yaml
board:
  type: chessboard
  pattern_cols: 11
  pattern_rows: 8
  square_length_m: 0.015
```

启动采集节点时，日志必须能看到与实体板一致的尺寸，例如：

```text
Using chessboard inner corners (11, 8), square=0.0150 m
```

如果日志里的内角点数与实体板不一致，先改配置再采集；不要通过调阈值“强行”
检测。尺寸或角点数填错会直接造成尺度错误或位姿错误。

`type: chessboard` 使用 `pattern_cols` / `pattern_rows`；`type: charuco` 使用
`squares_x` / `squares_y` / `marker_length_m`。切换板型后必须重新采集，
不能混用旧样本。

## 6. 每次标定前的检查

所有标定命令都在同一个终端执行：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 6.1 相机

三路相机必须都有图像。首次启动后才插上的相机不会被已启动的节点接管，
需要重启相机 launch：

```bash
rs-enumerate-devices -s
ros2 launch dual_nero_cameras all_cameras.launch.py
```

另一个终端检查：

```bash
ros2 topic hz /camera_scene/color/image_raw
ros2 topic hz /camera_left_wrist/color/image_raw
ros2 topic hz /camera_right_wrist/color/image_raw
```

正常应为约 `30 Hz`，且 `ros2 topic info -v <topic>` 的 Publisher count 为 1。

### 6.2 机器人 TF

腕部手眼标定必须有连续的工具链路：

```bash
ros2 run tf2_ros tf2_echo world left_link7
ros2 run tf2_ros tf2_echo world right_link7
```

两条命令都能持续输出变换后再开始采样。夹爪 TCP 标定也依赖这两个 TF。

如果报 `Invalid frame ID "world"`，说明 `robot_state_publisher` 没有启动，
通常是因为启动 Web 时用了 `start_moveit:=false`。标定前启动完整栈：

```bash
ros2 launch dual_nero_webapp webapp.launch.py \
  start_drivers:=true start_grippers:=false start_cameras:=true \
  start_moveit:=true moveit_dry_run:=true auto_enable:=false
```

`world` 由 `robot_state_publisher` 按 URDF 发布；`left_link7` / `right_link7`
还需要两侧驱动持续发布 `/joint_states`。如果只有 `world` 没有 `*_link7`，
检查 `/joint_states` 和驱动是否在线。

### 6.3 夹爪反馈

夹爪 TCP 和开度标定都要求原始位置反馈新鲜：

```bash
ros2 topic hz /left_gripper/position
ros2 topic hz /right_gripper/position
ros2 topic echo --once /left_gripper/status | grep -A1 feedback_position
ros2 topic echo --once /right_gripper/status | grep -A1 feedback_position
```

正常应为约 `20 Hz`，`feedback_position` 有数值，且
`feedback_fresh` 为 `true`。如果 `/position` 没有数据、`feedback_position`
为空或 `feedback_fresh=false`，先按 `17_gripper_can_diagnostics.md` 处理
CAN/串口/供电，不要继续做 TCP 或开度标定。

## 7. D435 场景相机外参

场景相机是固定相机，求解需要标定板在 `world` 下的真实位姿。只标腕部相机时
可以跳过本节。

### 7.1 固定标定板

把板平放在桌面并夹紧，保证标定期间不会移动。OpenCV 的板坐标系定义如下：

- 原点：第一个内角点，不是板的外边框；
- `+X`：沿第一行内角点方向；
- `+Y`：沿第一列内角点方向；
- `+Z`：垂直板面。

`calibration_targets.yaml` 中的 `board_pose_in_world` 表示
`T_world_board`，`rpy` 是固定 `XYZ` 欧拉角，单位是弧度。平放板上电时不要
默认把 `rpy` 写成 `[0, 0, 0]`，应以实测坐标轴为准。

### 7.2 用机器人 TCP 测三点

用尖头 TCP 依次触碰并记录 `world` 下的三个点：

```text
p0 = 第一个内角点
px = 同一行、沿 +X 方向的另一个内角点
py = 同一列、沿 +Y 方向的另一个内角点
```

TCP 位姿以 TF 为准。在示教/低速模式下触碰板面时，读取
`world -> <side>_tcp_link` 的平移，取接触瞬间的稳定值：

```bash
ros2 run tf2_ros tf2_echo world left_tcp_link
ros2 run tf2_ros tf2_echo world right_tcp_link
```

驱动的 `/<side>/feedback/tcp_pose` 位于驱动自身基座坐标，且消息
`header.frame_id` 为空，不能直接当作 `world` 坐标使用。若使用加长指针测量，
还要把指针尖端相对 `*_tcp_link` 的偏移补进去。

用三点构造 `T_world_board`：

```bash
/usr/bin/python3 - <<'PY'
import numpy as np
from scipy.spatial.transform import Rotation

p0 = np.array([0.0, 0.0, 0.0])   # 换成实测 p0
px = np.array([0.0, 0.0, 0.0])   # 换成实测 px
py = np.array([0.0, 0.0, 0.0])   # 换成实测 py

x = px - p0
x /= np.linalg.norm(x)
z = np.cross(x, py - p0)
z /= np.linalg.norm(z)
y = np.cross(z, x)

R = np.column_stack([x, y, z])
print('xyz =', p0.tolist())
print('rpy =', Rotation.from_matrix(R).as_euler('xyz').tolist())
print('quat_xyzw =', Rotation.from_matrix(R).as_quat().tolist())
PY
```

把输出的 `xyz` 和 `rpy` 写入
`ros2_ws/src/dual_nero_calibration/config/calibration_targets.yaml`：

```yaml
cameras:
  scene:
    board_pose_in_world:
      xyz: [X, Y, Z]          # 实测，不是占位值
      rpy: [ROLL, PITCH, YAW] # 实测，单位 rad
```

### 7.3 采集与求解

保持板不移动，从 15–20 个不同角度采集：

```bash
ros2 launch dual_nero_calibration capture.launch.py \
  camera:=scene samples:=20

ros2 run dual_nero_calibration solve_extrinsics \
  --camera scene \
  --session /home/work/nero_duo_arm/deployment/calibration_data/scene/<timestamp> \
  --strict
```

结果默认写入
`deployment/calibration_results/scene_extrinsic.yaml`。如果
`CALIBRATION` 不是 `PASS`，不要发布该结果。

## 8. 腕部 D405 手眼标定

腕部相机是 eye-in-hand：标定板固定不动，机械臂带动相机改变位姿。标定板
在世界中的精确位置不参与求解，但仍必须夹紧，并在采集期间保持不动。

### 8.1 采集

先标左侧：

```bash
ros2 launch dual_nero_calibration capture.launch.py \
  camera:=left_wrist samples:=20
```

人工或通过 MoveIt 低速移动左臂，在每个位姿停稳，让标定板完整出现在画面中。
采集节点要求相邻位姿满足以下至少一项：

```text
平移 >= 2 cm
旋转 >= 2 deg
```

采集节点还会逐帧检查检测质量，不合格的帧不会保存：

```text
相邻角点最小间距 >= 2 px
该帧 solvePnP 重投影 RMS <= 1.0 px
```

如果日志反复出现
`board rejected: adjacent corners too close` 或
`board rejected: reprojection RMS ...`，说明板太小、太远、反光或运动模糊，
应把板移近并调整光照，而不是放宽阈值。建议棋盘格在画面中的单个方格边长
至少约 15–20 px，并让标定板占据画面的较大区域。

建议至少 15 个有效样本，并覆盖多个方向的旋转；不要只平移。每个位姿停稳后等
日志出现 `captured N/20`，再移动到下一个位姿。达到样本数后节点自动退出。

右臂重复：

```bash
ros2 launch dual_nero_calibration capture.launch.py \
  camera:=right_wrist samples:=20
```

### 8.1.1 从 ROS 2 bag 离线自动选帧

如果不想在现场实时采集，可以先录 bag，再离线自动挑选合格且姿态分散的帧：

```bash
ros2 bag record -o right_wrist_calib \
  /camera_right_wrist/color/image_raw \
  /camera_right_wrist/color/camera_info \
  /tf /tf_static
```

录制期间保持标定板固定，低速移动机械臂，覆盖平移和多个旋转方向。录完后：

```bash
ros2 run dual_nero_calibration extract_from_bag \
  --camera right_wrist \
  --bag /home/work/right_wrist_calib \
  --samples 20 \
  --strict
```

工具会逐帧检测棋盘格，执行与实时采集相同的质量门限，然后按机器人位姿做
最远点采样，优先选择重投影误差低且旋转/平移差异大的帧。默认输出到
`deployment/calibration_data/right_wrist/bag_<bag名>_<时间戳>/`，目录结构与
实时采集完全一致，可以直接交给 `solve_extrinsics`。

bag 很大时可加 `--stride 2` 隔帧检查。若 bag 的话题或 TF 做过重映射：

```bash
--image-topic /实际图像话题 \
--info-topic /实际CameraInfo话题 \
--tf-topic /实际TF话题
```

注意：普通 mp4/avi 只有图像，没有机器人 `world -> tool0` 和
`camera_link -> optical` 的 TF，不能用于腕部手眼外参求解。要离线标定腕部
相机，必须录制同时包含图像、CameraInfo 和 `/tf` 的 ROS 2 bag。

### 8.1.2 手动逐帧采集

如果希望完全手动控制，每次把机械臂调好、停稳后再记录一帧：

```bash
ros2 run dual_nero_calibration capture_manual \
  --camera right_wrist \
  --samples 20
```

终端操作：

```text
Enter  记录当前帧
s      显示当前棋盘格检测和机器人运动状态
d      删除上一帧记录
q      结束并写入 metadata.yaml
```

记录前至少停稳约 0.3 s。节点会检查最近 0.3 s 的机器人运动，默认要求
平移 `< 1 mm`、旋转 `< 0.2 deg`，同时要求该帧重投影 RMS `<= 1.0 px`。
不满足时不会保存，并会打印具体原因。停止时间或阈值可通过
`--max-stationary-translation-mm`、`--max-stationary-rotation-deg` 调整，
但不要为了让运动中的帧通过而放宽。

记录时还会拦住"换了个姿态但几乎没动"的帧：与已有样本相比，旋转
`< 3 deg` 且平移 `< 10 mm` 的位姿会被拒绝，并提示要再多动一点。
这两个门限用 `--min-pose-rotation-deg`、`--min-pose-translation-mm` 调整；
确实需要保留近似位姿时加 `--allow-near-duplicate`。状态行里的
`spread=NN.Ndeg` 是当前已记录样本之间的最大姿态夹角，用来判断旋转覆盖够不够
（求解门槛是 `>= 10 deg`，实际建议做到 40 deg 以上）。

也可以在另一个终端用 service 触发单帧记录：

```bash
ros2 service call /capture_manual/capture std_srvs/srv/Trigger
```

输出目录为 `deployment/calibration_data/<camera>/manual_<时间戳>/`，格式与
自动采集一致。结束后按 8.2 节求解该目录。

### 8.1.3 自动走点采集（可选）

默认仍用手动逐帧采集；这一节只是不想一直手动摆位时的可选项。

`capture_auto` 会自己挑 20 个位姿并驱动机械臂走点，运动全部经过现有的
`/dual_arm_controller/follow_joint_trajectory`（MoveIt 轨迹桥），不绕过门禁、
限位和 settle 逻辑。

先只规划、不动机械臂，确认位姿合理：

```bash
ros2 run dual_nero_calibration capture_auto \
  --camera right_wrist \
  --samples 20
```

输出会列出每个位姿相对当前姿态的关节增量（度）和视野判定。确认后加
`--execute` 真正走点：

```bash
ros2 run dual_nero_calibration capture_auto \
  --camera right_wrist \
  --samples 20 \
  --execute
```

流程：

1. 在当前位姿拍一帧作为种子，得到标定板在 `world` 中的位置；
2. 用 URDF 正运动学在关节空间采样周边位姿，丢掉标定板会出画、太远/太近、
   倾角过大或离关节限位太近的候选；
3. 按姿态最远点采样选出 19 个（连同种子共 20 个）；
4. 逐个发送单点轨迹，停稳后检测棋盘格并保存：

```text
<session>/sample_0000.npz           角点 + 位姿，供 solve_extrinsics 使用
<session>/sample_0000.png           原始图像
<session>/sample_0000_detected.png  画出角点的图像
<session>/metadata.yaml
```

输出目录为 `deployment/calibration_data/<camera>/auto_<时间戳>/`，之后按 8.2
节求解。`--no-save-images` 可以只存 npz。

安全与自检：

- 默认关节速度 `0.35 rad/s`、单步关节增量上限 `22 deg`、关节限位留 4 deg；
  `--joint-speed-rad-s`、`--max-joint-step-deg`、`--limit-margin-deg` 可调。
- 必须有人现场监护；Ctrl-C 会取消当前轨迹目标。
- 启动时比较 URDF 正运动学与 `world -> tool0` 的 TF，差超过 10 mm / 1 deg
  直接退出，提示描述文件的 xacro 参数不一致。
- 轨迹返回成功但关节没到位（例如轨迹桥在 `dry_run`）会报错并停止，避免录到
  一组完全相同的位姿。
- 单个位姿检测失败只跳过该位姿，不中断整批。

规划需要一个近似的 `tool0 -> camera_link` 外参来预测视野，按以下顺序取：
已发布的 TF、`--extrinsic` 指定的 YAML、`calibration_results/` 里最新的
`<camera>_extrinsic*.yaml`。这个外参只用于预测画面，不参与最终求解。

### 8.2 求解

左侧：

```bash
ros2 run dual_nero_calibration solve_extrinsics \
  --camera left_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/left_wrist/<timestamp> \
  --strict
```

右侧：

```bash
ros2 run dual_nero_calibration solve_extrinsics \
  --camera right_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/right_wrist/<timestamp> \
  --strict
```

默认使用 OpenCV `PARK`。需要对比时可加：

```bash
--method TSAI|HORAUD|ANDREFF|DANIILIDIS
```

同一个批次内不要修改标定板参数或相机分辨率。

如果求解出现下面这类结果：

```text
TRANSLATION_RMS_M 0.157267397
ROTATION_RMS_DEG 16.165526
REPROJECTION_RMS_PX 5.550098
```

这不是“阈值太严”，而是角点数据已经损坏。该批次必须整体废弃并重新采集，
不要在旧目录上删几个样本后继续求解，也不要为了 `PASS` 调大阈值。旧版采集
脚本曾在 `findChessboardCornersSB` 之后又做固定 `11×11` 的 `cornerSubPix`，
当方格在画面中只有约 7–16 px 时会把角点吸附到相邻角点，造成相邻角点几乎
重合。修复后的采集脚本会直接使用 SB 的亚像素结果，并在保存前拒绝这类帧。

如果重投影和单帧重复性都很好，一致性却稳定卡在几毫米到 1 度：

```text
TRANSLATION_RMS_M 0.009386867
ROTATION_RMS_DEG 0.935808
REPROJECTION_RMS_PX 0.100039
CHECK translation_rms FAIL
CHECK rotation_rms FAIL
```

先加 `--report` 看逐帧残差：

```bash
ros2 run dual_nero_calibration solve_extrinsics \
  --camera right_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/right_wrist/<timestamp> \
  --report
```

输出中的 `ratio = 平移残差 / (旋转残差 x 板距)`。`ratio` 中位数接近 1 时，
残差是**姿态误差**（上例约 0.94 deg），平移只是它乘以板距的结果，与角点、
焦距、标定板尺寸无关。这种情况继续调检测参数或加样本没有用，问题在机械链：

- 机械臂关节零点或绝对定位精度；
- 相机支架、夹爪连接、线缆在跨姿态时的柔性变形；
- 采集期间标定板被碰动。

判别方法：让机械臂从**不同方向**回到同一个标称位姿，每个位姿停稳后连续记录
2-3 帧（用 8.1.2 节的 `capture_manual`），再带 `--report` 求解。报告里的
`same-pose repeatability` 会自动把这些帧分组，分别给出 `T_base_tool` 和
`T_cam_board` 的重复性：

```text
same-pose repeatability:
  samples [0, 1, 2]: T_base_tool 1.850 mm / 0.210 deg, T_cam_board 0.060 mm / 0.010 deg
```

- `T_base_tool` 已差到毫米级 / 0.2 deg 以上 → 机械臂重复定位或绝对精度问题；
- `T_base_tool` 基本不变、`T_cam_board` 变化明显 → 相机支架或标定板移动。

### 8.2.1 多采几批并筛选一致帧

可以多采一些再筛，但不能按“单帧残差最小”逐帧挑：那样会偏向某一小簇视角，
残差看起来很好，外参仍然不可重复。`--robust` 的做法是寻找**最大的自洽子集**：
每轮抽 8 帧求一次外参，再统计整批中有多少帧支持同一个结果，最后只在这些
一致帧上重新求解。它不会放宽 `--max-*` 验收阈值。

建议每批采 30–40 帧，采 2–3 批独立会话。批次之间可以重新摆机械臂，
但不要碰相机支架和标定板：

```bash
ros2 run dual_nero_calibration capture_manual \
  --camera right_wrist \
  --samples 35

ros2 run dual_nero_calibration solve_extrinsics \
  --camera right_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/right_wrist/<session_a> \
  --robust --report --strict
```

再拿另一批做独立复核：

```bash
ros2 run dual_nero_calibration solve_extrinsics \
  --camera right_wrist \
  --session /home/work/nero_duo_arm/deployment/calibration_data/right_wrist/<session_b> \
  --robust --compare /home/work/nero_duo_arm/deployment/calibration_data/right_wrist/<session_a> \
  --strict
```

关注这些输出：

- `INLIERS 24/35 (69%)`：一致帧比例，建议 `>= 60%`；
- `INLIER_SPREAD_DEG`：一致帧的旋转跨度，建议 `>= 30 deg`；
- `COMPETING_FRAMES`：剩余帧还能被另一个外参解释的数量，`> 0` 说明批次内
  存在竞争模型，不能当作已经收敛；
- `COMPARE_TRANSLATION_DIFF_MM` / `COMPARE_ROTATION_DIFF_DEG`：两批独立数据
  给出的外参差异，建议分别 `<= 3 mm`、`<= 0.3 deg`。

如果 `--robust` 输出 `no consistent subset`，或者两批差异仍接近单批自身的
残差（当前右腕就是这样），说明外参没有可重复性。此时筛帧解决不了问题，
必须回到机械侧排查：支架刚性、相机是否松动、机械臂绝对精度或标定板是否移动。

机械一致性修好之前，任何算法都无法把外参压到 3 mm / 0.5 deg 以内，也不要靠
放宽 `--max-*` 换一个 `PASS`。

## 9. 相机标定验收与发布

`solve_extrinsics` 会输出逐项检查和总状态。验收阈值：

```text
样本数:           >= 6
旋转跨度:         >= 10 deg
平移一致性 RMS:   < 3 mm
旋转一致性 RMS:   < 0.5 deg
重投影 RMS:       < 1.0 px
```

加 `--strict` 后任一项失败会返回非零退出码。只有输出
`CALIBRATION PASS` 才继续发布。

未通过时结果不会覆盖已发布的外参，而是写到
`deployment/calibration_results/<camera>_extrinsic.<session>.warn.yaml`，并在 YAML 中
记录 `status: WARN` 和 `failed_checks`。`publish_extrinsic` 默认拒绝发布
`status` 不是 `PASS` 的文件；只在排查问题时临时加 `allow_degraded:=true`：

```bash
ros2 launch dual_nero_calibration publish_extrinsic.launch.py \
  yaml_files:="/home/work/nero_duo_arm/deployment/calibration_results/right_wrist_extrinsic.<session>.warn.yaml" \
  allow_degraded:=true
```

发布三路外参前，先停止已有同名 static TF 发布节点，避免重复 TF：

```bash
ros2 launch dual_nero_calibration publish_extrinsic.launch.py \
  yaml_files:="/home/work/nero_duo_arm/deployment/calibration_results/scene_extrinsic.yaml,/home/work/nero_duo_arm/deployment/calibration_results/left_wrist_extrinsic.yaml,/home/work/nero_duo_arm/deployment/calibration_results/right_wrist_extrinsic.yaml"
```

如果只标了腕部相机，只传对应两个文件。检查 TF：

```bash
ros2 run tf2_ros tf2_echo world camera_scene_link
ros2 run tf2_ros tf2_echo left_tool0 camera_left_wrist_link
ros2 run tf2_ros tf2_echo right_tool0 camera_right_wrist_link
```

## 10. 夹爪实物 TCP 标定

夹爪 TCP 标定与相机标定相互独立。工具只读 TF，不控制机械臂；机械臂必须由
人工或 MoveIt 低速移动，并全程有人监护。

### 10.1 准备枢轴点

选一个代表最终抓取点的固定枢轴：

- 直接用闭合后的指爪尖；或
- 在两指之间夹一根刚性指针，标定后从结果中扣除指针长度。

确认工具链路：

```bash
ros2 run tf2_ros tf2_echo world right_link7
```

### 10.2 采集与求解

```bash
ros2 run dual_nero_calibration calibrate_gripper_tcp \
  --side right --samples 12 --strict
```

每个位姿让枢轴点保持不动，只改变工具姿态；按 Enter 采样。脚本会：

- 拒绝与所有已采姿态旋转差小于 `5 deg` 的样本；
- 要求总旋转跨度至少 `15 deg`；
- 检查枢轴 RMS `< 2 mm`、条件数 `< 1e5`。

左侧把 `right` 改为 `left`。左右必须分别标定，不能互相复制。

输出文件：

```text
deployment/calibration_results/gripper_tcp_right.yaml
deployment/calibration_results/gripper_tcp_left.yaml
```

终端同时打印：

```text
DRIVER_TCP_OFFSET [...]
URDF_ARGS {'right_tcp_x': ..., 'right_tcp_y': ..., 'right_tcp_z': ...}
CALIBRATION PASS
```

### 10.3 写入驱动和 URDF

将 `DRIVER_TCP_OFFSET` 同步到：

```text
ros2_ws/src/dual_nero_control/launch/dual_nero_drivers.launch.py
  DEFAULT_TCP_OFFSET
```

将 `URDF_ARGS` 中的三个值同步到：

```text
ros2_ws/src/dual_nero_description/urdf/two_nero_tabletop.urdf.xacro
  <side>_tcp_x / <side>_tcp_y / <side>_tcp_z
```

两处必须使用同一组标定结果。修改后重新编译并重启驱动、MoveIt 和 Web：

```bash
cd /home/work/nero_duo_arm/ros2_ws
colcon build --symlink-install \
  --packages-select dual_nero_control dual_nero_description \
  dual_nero_moveit_config dual_nero_webapp
source install/setup.bash
```

然后运行只读一致性检查：

```bash
/usr/bin/python3 deployment/scripts/check_tcp_contract.py
```

驱动 `tcp_pose` 与 MoveIt `*_tcp_link` 的误差应接近 0；否则不要用于自动抓取。

## 11. 夹爪开度标定

开度标定只修正 `/<side>_gripper/width` 的毫米映射，不改变 CAN 原始位置
`0..1000` 的含义。开始前必须确认 `/<side>_gripper/position` 为约 `20 Hz`，
且 `feedback_fresh=true`。

### 11.1 测量

以右侧为例，先以只允许运动、但启动时不使能的方式启动驱动：

```bash
ros2 launch dual_nero_control right_gripper.launch.py \
  allow_motion:=true enable_motor:=false
```

使能后低速移动到多个位置，每次等 `/position` 稳定，再用卡尺测量两指腹之间的
实际开口：

```bash
ros2 service call /right_gripper/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /right_gripper/position_cmd std_msgs/msg/Int32 "{data: 0}"
ros2 topic echo --once /right_gripper/position
```

再依次测试 `250`、`500`、`750`、`990` 等位置。建议至少 4 组，且覆盖
工作区间；不要只取两个相邻点。

记录成：

```text
RAW_POSITION=OPENING_MM
```

### 11.2 拟合

```bash
ros2 run dual_nero_calibration calibrate_gripper_opening \
  --side right \
  --measurement 0=89.4 \
  --measurement 250=67.2 \
  --measurement 500=45.1 \
  --measurement 750=22.7
```

上面的数字只是格式示例，必须替换成本次卡尺实测值。输出文件：

```text
deployment/calibration_results/gripper_opening_right.yaml
```

终端会打印斜率、截距、`opening_min_mm`、`opening_max_mm` 和拟合残差。
`FIT_RMS_MM > 1` 时不要直接使用线性结果，先检查指爪松动、零位变化或机械
非线性，必要时分段标定。

### 11.3 写入夹爪节点

把输出 YAML 的 `node_parameters` 四个参数写入对应夹爪节点的 `parameters`：

```text
position_min
position_max
opening_min_mm
opening_max_mm
```

当前 `right_gripper.launch.py` / `left_gripper.launch.py` 没有把这四项暴露成
launch 参数，需要直接在该侧 launch 的 `parameters` 字典中增加，或使用现场
参数 YAML 文件。修改后重启节点。

标定完成后回张开位并失能：

```bash
ros2 topic pub --once /right_gripper/position_cmd std_msgs/msg/Int32 "{data: 0}"
ros2 service call /right_gripper/enable std_srvs/srv/SetBool "{data: false}"
```

## 12. 常见失败

| 现象 | 检查 |
| --- | --- |
| `No sample_*.npz files` | `--session` 是否指到本次时间戳目录 |
| 一直采集不到样本 | 相机话题是否有帧、标定板是否完整、`CameraInfo` 是否到达 |
| 日志出现 `board rejected: adjacent corners too close` | 板太远/太小或模糊；移近标定板，单格投影建议 >= 15–20 px |
| 日志出现 `board rejected: reprojection RMS ...` | 该帧角点质量差；改善光照/反光/运动模糊，不要调大阈值 |
| 日志内角点数与实体板不一致 | `pattern_cols` / `pattern_rows` 是否按方格数减一 |
| `Invalid frame ID "world"` | 未启动 `robot_state_publisher`；启动 Web/MoveIt，或把 `start_moveit:=true` |
| `Robot tool TF unavailable` | 驱动、`robot_state_publisher` 和 MoveIt/URDF 是否已启动 |
| `Camera link/optical TF unavailable` | RealSense 是否发布对应 optical frame |
| 手眼 RMS 很大 | 先逐样本查角点间距和重投影；旧批次可能已损坏，应重采。再查姿态跨度、板尺寸、TCP/URDF |
| 固定相机外参漂移 | `scene.board_pose_in_world` 是否实测、板是否移动 |
| 三路相机缺少一路 | 先执行 `rs-enumerate-devices -s`，确认序列号和 USB 3.x 连接 |
| 夹爪 `/position` 无数据 | 先查 CAN、串口、供电和 `feedback_fresh`，不要继续标定 |
| 开度拟合 RMS 大 | 指爪松动、测量点太少或机械非线性，改用更多点/分段 |

相机或夹爪拆装、碰撞、线缆拉扯或安装位置变化后，必须重新标定；旧外参不能
继续用于抓取或 VLA 数据采集。
