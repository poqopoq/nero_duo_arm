# 阶段 4：三台 RealSense 接入

> 记录日期：2026-11-02  
> 状态：**暂停，等待 USB 3.2 Gen2 外接供电 Hub**。三台相机已识别，WSL USB/IP 下三机高带宽并行仍不稳定。

## 1. 已安装

```text
ros-humble-realsense2-camera       4.58.4
ros-humble-realsense2-description  4.58.4
ros-humble-realsense2-camera-msgs  4.58.4
ros-humble-librealsense2           2.58.4
v4l-utils
```

工具：

```text
/opt/ros/humble/bin/rs-enumerate-devices
/opt/ros/humble/bin/rs-capture
/usr/bin/v4l2-ctl
```

已安装官方 udev 规则：

```text
/etc/udev/rules.d/99-realsense-libusb.rules
```

## 2. 当前枚举结果

```text
rs-enumerate-devices -s
No device detected. Is it plugged in?
```

Linux 当前也没有 `/dev/video*`，说明三台相机尚未接入或尚未直通到 WSL。

## 3. 下一步：逐台接入

建议顺序：

1. 只连接 D435。
2. 执行 `rs-enumerate-devices -s`，记录型号和序列号。
3. 只连接左腕 D405，记录序列号。
4. 断开左腕，连接右腕 D405，记录序列号。
5. 三台同时连接，确认三台都能枚举。
6. 再启动三个独立的 `realsense2_camera` 节点。

不要一次连接三台后直接修改配置，否则无法可靠区分两台同型号 D405。

## 4. 角色命名

预定：

| 角色 | 型号 | 序列号 |
| --- | --- | --- |
| `camera_scene` | D435 | `263422075415` |
| `camera_left_wrist` | D405 | `260522270991` |
| `camera_right_wrist` | D405 | `261122272091` |

每台相机使用独立 namespace：

```text
/camera_scene/color/image_raw
/camera_scene/depth/image_rect_raw
/camera_scene/color/camera_info

/camera_left_wrist/color/image_raw
/camera_left_wrist/depth/image_rect_raw
/camera_left_wrist/color/camera_info

/camera_right_wrist/color/image_raw
/camera_right_wrist/depth/image_rect_raw
/camera_right_wrist/color/camera_info
```

## 5. 注意

- RealSense 应优先使用 USB 3.0+ 直连。
- 通过 WSL USB/IP 运行三路 RGB-D 可能出现带宽或稳定性问题；正式采集建议使用原生 Ubuntu。
- 三台同时运行后需检查时间戳、丢帧率和 USB 带宽。


## 6. 左腕 D405 实测

```text
型号: RealSense D405
VID:PID: 8086:0b5b
序列号: 260522270991
固件: 5.15.1.55
Linux: /dev/video0..video5
当前 USB: 3.2, 5000 Mbps
早期测试 USB: 2.1, 480 Mbps
```

当前测试配置：

```text
Color: 640x480 RGB8 @ 15 FPS
Depth: 640x480 Z16 @ 15 FPS
align_depth.enable: true
```

实测：

```text
Color FPS: 约 14 FPS
Depth FPS: 约 14 FPS
```

ROS 驱动启动后曾发布：

```text
/camera_left_wrist/camera_left_wrist/color/image_raw
/camera_left_wrist/camera_left_wrist/color/camera_info
/camera_left_wrist/camera_left_wrist/depth/image_rect_raw
/camera_left_wrist/camera_left_wrist/depth/camera_info
/camera_left_wrist/camera_left_wrist/aligned_depth_to_color/image_raw
/camera_left_wrist/camera_left_wrist/aligned_depth_to_color/camera_info
```

## 7. USB2 限制

`realsense2_camera` 报告：

```text
Device USB type: 2.1
Reduced performance is expected.
```

正式三相机运行前必须确认 D405 以 USB 3.x（5000 Mbps 或更高）枚举。需要检查：

- USB 3.x 数据线，不能使用 USB2 或充电线。
- 直连电脑 USB3 接口，避免经过 USB2 Hub。
- WSL 中对应 USB 设备的 `speed` 应为 `5000` 或 `10000`。
- 三台相机同时运行时需要检查 USB 控制器带宽分配。

## 8. 当前下一步

右腕 D405 接入后，执行：

```bash
/home/work/nero_duo_arm/deployment/scripts/check_realsense.sh
```

左腕序列号已记录，因此可以直接从两台 D405 的枚举结果中识别右腕序列号。


## 9. 右腕 D405 实测

```text
型号: RealSense D405
序列号: 261122272091
固件: 5.15.1.55
VID:PID: 8086:0b5b
USB: 3.2, 5000 Mbps
```

ROS 话题前缀：

```text
/camera_right_wrist/color/image_raw
/camera_right_wrist/color/camera_info
/camera_right_wrist/depth/image_rect_raw
/camera_right_wrist/depth/camera_info
/camera_right_wrist/aligned_depth_to_color/image_raw
/camera_right_wrist/aligned_depth_to_color/camera_info
```

### 带宽测试

| 配置 | 结果 |
| --- | --- |
| 640×480 @ 30 FPS | 大量 `Incomplete video frame detected`，无稳定图像 |
| 640×480 @ 15 FPS | 仍出现 incomplete frame |
| 480×270 @ 15 FPS | 稳定，Color/Depth 约 14.7 FPS，无 incomplete frame |

当前 WSL USB/IP 路径下建议 D405 使用：

```text
Color: 480x270 RGB8 @ 15 FPS
Depth: 480x270 Z16 @ 15 FPS
align_depth.enable: true
```

## 10. 双 D405 并行结果

左腕早期曾在 USB2.1 下运行；当前左右均为 USB3.2。双机同时运行时需要：

1. 两台 D405 同时连接到 Windows/WSL。
2. 再次运行 `check_realsense.sh`，确认两个序列号同时存在。
3. 分别用 `260522270991` 和 `261122272091` 启动两个独立节点。
4. 两台都先使用 480×270 @ 15 FPS。
5. 检查是否出现 incomplete frame、时间戳跳变或 topic 中断。

若双机同时运行不稳定，优先改为原生 Ubuntu；WSL USB/IP 不适合作为三路 RGB-D 的正式采集环境。


## 11. 双 D405 并行验证

两台相机同时连接后：

```text
左腕: 260522270991, USB 3.2
右腕: 261122272091, USB 3.2
```

稳定配置：

```text
Color: 480x270 RGB8 @ 15 FPS
Depth: 480x270 Z16 @ 15 FPS
align_depth.enable: true
```

实测：

```text
左腕 Color: 14.742 FPS
右腕 Color: 14.762 FPS
左腕 Depth: 约 14.75 FPS
右腕 Depth: 约 14.74 FPS
Incomplete frame: 0
```

启动包：

```text
dual_nero_ws/src/dual_nero_cameras
```

启动命令：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_cameras d405_pair.launch.py
```

固定话题：

```text
/camera_left_wrist/color/image_raw
/camera_left_wrist/depth/image_rect_raw
/camera_left_wrist/aligned_depth_to_color/image_raw
/camera_right_wrist/color/image_raw
/camera_right_wrist/depth/image_rect_raw
/camera_right_wrist/aligned_depth_to_color/image_raw
```

## 12. 当前下一步

接入 D435 场景相机：

1. 使用 USB 3.x 接口连接 D435。
2. 通过 usbipd 直通到 WSL。
3. 读取 D435 序列号。
4. 使用独立 `camera_scene` 节点测试 RGB、Depth、CameraInfo。
5. 三台相机同时运行，检查带宽、帧率和时间戳。


## 13. D435 实测

```text
型号: RealSense D435
序列号: 263422075415
固件: 5.15.1.55
VID:PID: 8086:0b07
USB: 2.1, 480 Mbps（当前）
```

单机配置 `RGB 640x480@15 + Depth 480x270@15` 可发布话题，但 USB2 下 Depth 仅约 10–11 FPS，并出现一次 global timestamp 读取错误。

三台同时启动时：

- 三个节点均能建立。
- D405 序列号绑定已改为直接 Node 参数，确认左右不串设备。
- D435 当前 RGB 640x480@15、Depth 480x270@15。
- 双 D405 在 15 FPS 下仍会间歇出现 `Incomplete video frame detected`。
- 曾出现左腕 D405 因 `VIDIOC_S_FMT` I/O 错误从 USB/IP 断开。

## 14. 当前结论

WSL USB/IP 不适合作为三路 RealSense 的正式数据采集环境。建议：

1. D435 改用 USB 3.x 数据线和接口，确认 Linux `speed=5000`。
2. 三台相机尽量分散到不同 USB3 控制器。
3. 若三机仍需降帧或持续 incomplete frame，改为原生 Ubuntu。
4. 当前开发阶段可将双 D405 降到 480x270@5 FPS 做功能联调，但不能代表最终采集性能。

## 15. 当前下一步

先将 D435 改接 USB3，并保持两台 D405 USB3 连接。连接后执行：

```bash
/home/work/nero_duo_arm/deployment/scripts/check_realsense.sh
```

确认三台设备全部为 `5000 Mbps` 后，再进行一次三机低带宽联合测试。


## 16. 待办锁定：新 Hub 到位后复测

当前不继续调相机。新 USB 3.2 Gen2 外接供电 Hub 到位后：

1. Hub 接到电脑 USB4/USB-C 端口。
2. 三台相机接同一 Hub 的不同 USB3 端口。
3. 两台 CAN 适配器不接该 Hub。
4. 执行 `check_realsense.sh`，确认三台均为 5000 Mbps 或更高。
5. 启动 `all_cameras.launch.py`，连续运行至少 10 分钟。
6. 检查 incomplete frame、USB 断开、帧率和时间戳。
7. 若仍不稳定，改用原生 Ubuntu。

在新 Hub 到位前，阶段 4 功能开发暂停，继续阶段 5。

## 17. 新 Hub 检测结果

Hub：

```text
VID:PID: 05e3:0626
类型: Genesys Logic USB3.1 Hub
Windows 描述: 通用 SuperSpeed USB 集线器 / USB3.1 Hub
Linux 侧三台相机: 5000 Mbps, USB 3.20
```

三台相机：

```text
D405 左腕: 260522270991
D405 右腕: 261122272091
D435 场景: 263422075415
```

三机同时启动测试：

```text
D435 Color: 约 13.8 FPS
D405 左 Color: 约 14.7 FPS
D405 右 Color: 约 12.9 FPS
Incomplete video frame: 39 次
dmesg: uvcvideo Non-zero status (-32)
```

结论：

- 新 Hub 解决了 D435 之前以 USB2 枚举的问题。
- 该 Hub 是 USB3.1 5 Gbps 上行，不是推荐的 USB3.2 Gen2 10 Gbps。
- 三台相机在 480x270/640x480 15 FPS 混合配置下仍不稳定，不能作为正式三路采集环境。
- `uvcvideo -32` 表明仍有 USB/IP/URB 传输错误。

下一步：

1. 换 USB3.2 Gen2 10 Gbps、外接供电 Hub。
2. 或将三台相机迁到原生 Ubuntu 主机。
3. 在最终硬件确定前，单相机标定和单相机功能验证仍可继续。


## 18. 原生 Ubuntu USB2 功能验证（2026-11-01）

当前临时连接：

```text
左腕 D405: 5000 Mbps / USB 3.20
右腕 D405: 5000 Mbps / USB 3.20
D435:      480 Mbps / USB 2.10（USB 2.0 线）
```

测试配置：

```text
D435 Color: 640x480 @ 15 FPS
D435 Depth: 480x270 @ 15 FPS
D405 Color: 480x270 @ 15 FPS
D405 Depth: 480x270 @ 15 FPS
align_depth.enable: true
enable_infra / enable_infra1 / enable_infra2: false
```

D435 在 USB2 下必须关闭两个红外输出。驱动默认打开红外流时，深度帧会
持续出现 `Frames didn't arrived within 5 seconds`；关闭后深度恢复正常。

120 秒三机并行结果：

```text
D435 Color:          约 15.0 FPS
D435 Depth:          约 15.0 FPS
D435 Aligned Depth:  约 15.0 FPS
D405 左 Color:       约 15.0 FPS
D405 左 Depth:       约 15.0 FPS
D405 左 Aligned:     约 15.0 FPS
D405 右 Color:       约 15.0 FPS
D405 右 Depth:       约 15.0 FPS
D405 右 Aligned:     约 15.0 FPS
Incomplete frame / 帧超时: 0
```

结论：USB2 临时配置已通过三机功能验证。由于 D435 仍以 480 Mbps 枚举，
继续保留 USB3 数据线和 10 分钟正式验收待办。
