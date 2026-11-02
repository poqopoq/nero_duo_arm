# 阶段 0：硬件与系统基线记录表

> 记录日期：2026-10-30（Asia/Shanghai）  
> 当前检查对象：`DESKTOP-9Q57OPK` 上的 WSL2 环境  
> 结论：**阶段 0 尚未全部通过。** ROS 2 Humble、两台 CAN 适配器、左右 NERO 反馈和软件急停已就绪；三台 RealSense、电源细节和安装尺寸仍需现场确认。

## 1. 自动确认结果

| 检查项 | 当前结果 | 状态 | 说明 |
| --- | --- | --- | --- |
| Ubuntu 22.04 | 22.04.5 LTS (Jammy) | 通过 | 满足 ROS 2 Humble 的目标系统版本 |
| 运行平台 | WSL2，内核 6.18.54.1-microsoft-standard-WSL2 | 风险 | USB-CAN 已通过 USB/IP 正常工作；实时控制稳定性仍需长时压力验证 |
| ROS 2 Humble | 已安装 `ros-humble-desktop`，`ROS_DISTRO=humble` | 通过 | `rclpy`、ROS 2 CLI、`colcon` 可用 |
| ROS 工具链 | `colcon`、`rosdep`、`xacro`、`can-utils`、`ethtool` 已安装 | 通过 | 阶段 1 最小依赖已补齐；MoveIt/RealSense 尚未安装 |
| Conda | 未安装 | 未通过 | 建议只用于 LeRobot/PyTorch 等非 ROS 依赖隔离 |
| CAN 工具 | `candump`、`cansend`、`ethtool` 可用 | 通过 | `can_left` 使用 `gs_usb`，1 Mbit/s 通信正常 |
| USB-CAN 适配器 | 两台 candleLight 同时可见，`gs_usb` 驱动正常 | 通过 | 稳定命名为 `can_left`、`can_right` |
| NERO 通信 | 左右均收到 7 关节反馈，固件 `v121 / 1.21` | 通过 | 左右 namespace 和联动软件急停均验证通过 |
| RealSense | Linux 可见 0 台；Windows PNP 未发现 D435/D405 | 阻塞 | 需连接三台相机后读取序列号 |
| 串口设备 | 无 `/dev/ttyUSB*`、`/dev/ttyACM*` | 待确认 | 若使用夹爪/转接器，需要补充 |
| 计算资源 | 16 逻辑 CPU，WSL 内存 11 GiB，工作区可用约 949 GiB | 通过 | 当前开发环境资源足够 |
| 主机时间 | Asia/Shanghai | 通过 | 后续需持续关注 NTP 和 ROS 时间戳 |

Windows 主机为 MECHREVO WUJIE14XA，AMD Ryzen 7 8845HS，Windows 11。Windows 已识别两台 candleLight CAN 适配器，并通过 `usbipd-win 5.3.0` 直通到 WSL；RealSense 仍未连接。

## 2. 需要现场填写

| 项目 | 左臂 | 右臂 | 备注 |
| --- | --- | --- | --- |
| NERO 序列号/型号 | 待填 | 待填 | 以铭牌和官方工具输出为准 |
| CAN 适配器型号 | candleLight `004500...3731` | candleLight `002200...3731` | VID:PID `1d50:606f`，按序列号绑定 |
| USB 物理端口 | BUSID `6-1` | BUSID `1-2` | BUSID 会变化，不作为稳定标识 |
| 期望稳定接口名 | `can_left`（建议） | `can_right`（建议） | 建议按适配器序列号绑定 |
| CAN 波特率 | 1,000,000 bit/s | 1,000,000 bit/s | `can_left` 已验证 1 Mbit/s 可通信 |
| 电源规格与状态 | 待填 | 待填 | 电压、电流、接地、上电顺序 |
| 急停方式 | 官方 Web + ROS 软件急停 | 官方 Web + ROS 软件急停 | 双臂独立与联动软件急停已实测 |
| 通信接口 | `can_left` | `can_right` | 基于适配器序列号命名 |
| 基座安装位置 | 待测 | 待测 | 相对 world/桌面的 XYZ |
| 基座朝向 | 待测 | 待测 | 建议记录 RPY |
| 桌面尺寸 | 待测 | — | 长、宽、厚 |
| 桌面高度 | 待测 | — | world 到桌面的高度 |
| 两基座间距 | 待测 | 待测 | 同一边到另一边 |

## 3. 三台相机记录

| 相机 | 角色 | 序列号 | 安装位置 | 分辨率/FPS | 状态 |
| --- | --- | --- | --- | --- | --- |
| D435 | 场景/全局相机 | 待填 | 待测 | 待定 | 未连接 |
| D405 #1 | 左腕相机 | 待填 | 左 TCP 附近 | 待定 | 未连接 |
| D405 #2 | 右腕相机 | 待填 | 右 TCP 附近 | 待定 | 未连接 |

安装完成后执行：

```bash
lsusb
rs-enumerate-devices -s
ls -l /dev/v4l/by-id/
```

## 4. 阶段 0 退出条件

- [x] WSL2 + USBIP 已能同时看到两台 CAN 适配器；是否作为最终控制主机仍待长时稳定性评估。
- [x] ROS 2 Humble 安装完成，`rclpy` 和 ROS 2 CLI 可用。
- [x] 左右臂适配器已固定映射为 `can_left`、`can_right`。
- [ ] 两台 NERO 的型号、序列号、电源、急停和通信接口已记录并现场验证。
- [ ] D435 和两台 D405 同时可见，序列号与左右角色已记录。
- [ ] 桌面尺寸、桌面高度、左右基座位置和朝向已完成测量。
- [ ] 完成本表第 2、3 节填写，并签字确认。

## 5. 当前下一步

阶段 1、左臂稳定命名、`/left` namespace、低速运动和软件急停均已完成。右臂接入后验证 `can_right`、`/right` namespace 和双臂联动急停。
