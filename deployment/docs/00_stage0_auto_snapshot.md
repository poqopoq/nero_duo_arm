# Stage 0 自动检查快照

- 采集时间: `2026-10-30 18:05:30 CST`
- 主机名: `DESKTOP-9Q57OPK`
- 执行用户: `a`
- 执行目录: `/home/a/workspace`

## 操作系统与运行环境

- 发行版: `Ubuntu 22.04.5 LTS`
- VERSION_ID: `22.04`
- 内核: `6.18.54.1-microsoft-standard-WSL2`
- 架构: `x86_64`
- 运行平台: `WSL2 (硬件直通需额外配置，不推荐作为真机实时控制主机)`

## 计算与存储资源

- 逻辑 CPU: `16`
- 内存: `11Gi`
- 工作区可用空间: `946G`

## ROS 2 / 工具链

- ROS_DISTRO: `humble`
- ros2 命令: `/opt/ros/humble/bin/ros2`
- Humble setup: `present`
- colcon: `/usr/bin/colcon`
- rosdep: `/usr/bin/rosdep`
- xacro: `/opt/ros/humble/bin/xacro`
- rclpy: `/opt/ros/humble`
- pip3: `/home/a/.local/bin/pip3`
- apt: `can-utils 2020.11.0-1`
- apt: `ros-humble-desktop 0.10.0-1jammy.20260916.003705`
- apt: `ros-humble-ros-base 0.10.0-1jammy.20260915.225919`

## Python 与 Conda

- python3 路径: `/usr/bin/python3`
- python3 版本: `Python 3.10.12`
- conda: `missing`
- conda 版本: `missing`

> ROS 2 Humble 的 `rclpy`/C++ 节点应使用系统 Python 和 APT 依赖。Conda 建议只隔离 LeRobot/PyTorch，不替代 ROS 2 主环境。

## USB 设备

- lsusb: `/usr/bin/lsusb`
- `Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub`
- `Bus 001 Device 007: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter`
- `Bus 001 Device 006: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter`
- `Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub`

- 匹配到的 USB-CAN/CAN 适配器数量: `2`
- 匹配到的 Intel RealSense 设备数量: `0`
- candleLight CAN 适配器数量: `2`

## CAN 接口

- ip 命令: `/usr/sbin/ip`

### CAN link

```text
9: can_left: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 16 maxmtu 16 
    can state ERROR-ACTIVE restart-ms 0 
	  bitrate 1000000 sample-point 0.750 
	  tq 62 prop-seg 5 phase-seg1 6 phase-seg2 4 sjw 2
	  gs_usb: tseg1 1..16 tseg2 1..8 sjw 1..4 brp 1..1024 brp-inc 1
	  clock 48000000 numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 parentbus usb parentdev 1-1:1.0 
10: can_right: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 16 maxmtu 16 
    can state ERROR-ACTIVE restart-ms 0 
	  bitrate 1000000 sample-point 0.750 
	  tq 62 prop-seg 5 phase-seg1 6 phase-seg2 4 sjw 2
	  gs_usb: tseg1 1..16 tseg2 1..8 sjw 1..4 brp 1..1024 brp-inc 1
	  clock 48000000 numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 parentbus usb parentdev 1-2:1.0 
```

### CAN 内核模块（已加载）

```text
can_raw                20480  0
can                    20480  1 can_raw
gs_usb                 24576  0
can_dev                49152  1 gs_usb
```
- candump: `/usr/bin/candump`

## 串口设备与稳定映射

- `/dev/serial/by-id`: missing
- `ls: cannot access '/dev/ttyUSB*': No such file or directory`
- `ls: cannot access '/dev/ttyACM*': No such file or directory`

## 视频设备与稳定映射

- `/dev/v4l/by-id`: missing
- `ls: cannot access '/dev/video*': No such file or directory`
- v4l2-ctl: `missing`
- rs-enumerate-devices: `missing`

## udev 规则

- /etc/udev/rules.d/99-realsense-libusb.rules: `missing`
- /etc/udev/rules.d/99-agx-can.rules: `missing`
- /etc/udev/rules.d/99-nero-can.rules: `missing`

## 网络与主机时间


```text
lo               UNKNOWN        127.0.0.1/8 10.255.255.254/32 ::1/128 
eth0             DOWN           
eth1             UP             10.10.11.141/24 fe80::17c4:508:d8cd:98e9/64 
loopback0        UP             
can_left         UP             
can_right        UP             
```
- 时间: `2026-10-30 18:05:30 CST`
- NTP 同步: `yes`
