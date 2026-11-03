# Stage 0 自动检查快照

- 采集时间: `2026-11-02 14:32:35 CST`
- 主机名: `jywgworkstation2`
- 执行用户: `work`
- 执行目录: `/home/work/nero_duo_arm`

## 操作系统与运行环境

- 发行版: `Ubuntu 22.04.5 LTS`
- VERSION_ID: `22.04`
- 内核: `6.8.0-138-generic`
- 架构: `x86_64`
- 运行平台: `原生 Linux`

## 计算与存储资源

- 逻辑 CPU: `16`
- 内存: `15Gi`
- 工作区可用空间: `71G`

## ROS 2 / 工具链

- ROS_DISTRO: `humble`
- ros2 命令: `/opt/ros/humble/bin/ros2`
- Humble setup: `present`
- colcon: `/usr/bin/colcon`
- rosdep: `/usr/bin/rosdep`
- xacro: `/opt/ros/humble/bin/xacro`
- rclpy: `/opt/ros/humble`
- pip3: `/usr/bin/pip3`
- apt: `can-utils 2020.11.0-1`
- apt: `ros-humble-desktop 0.10.0-1jammy.20260916.003705`
- apt: `ros-humble-realsense2-camera 4.58.4-1jammy.20260915.211150`
- apt: `ros-humble-realsense2-description 4.58.4-1jammy.20260908.002050`
- apt: `ros-humble-ros-base 0.10.0-1jammy.20260915.225919`

## Python 与 Conda

- python3 路径: `/usr/bin/python3`
- python3 版本: `Python 3.10.12`
- conda: `missing`
- conda 版本: `missing`

> ROS 2 Humble 的 `rclpy`/C++ 节点应使用系统 Python 和 APT 依赖。Conda 建议只隔离 LeRobot/PyTorch，不替代 ROS 2 主环境。

## USB 设备

- lsusb: `/usr/bin/lsusb`
- `Bus 002 Device 004: ID 8086:0b5b Intel Corp. Intel(R) RealSense(TM) Depth Camera 405 `
- `Bus 002 Device 003: ID 8086:0b5b Intel Corp. Intel(R) RealSense(TM) Depth Camera 405 `
- `Bus 002 Device 002: ID 05e3:0626 Genesys Logic, Inc. USB3.1 Hub`
- `Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub`
- `Bus 001 Device 005: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter`
- `Bus 001 Device 004: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter`
- `Bus 001 Device 003: ID 30fa:2031 INSTANT USB Keyboard`
- `Bus 001 Device 007: ID 8087:0026 Intel Corp. AX201 Bluetooth`
- `Bus 001 Device 006: ID 30fa:0301 Instant USB Optical Mouse`
- `Bus 001 Device 002: ID 05e3:0610 Genesys Logic, Inc. Hub`
- `Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub`

- 匹配到的 USB-CAN/CAN 适配器数量: `2`
- 匹配到的 Intel RealSense 设备数量: `3`
- candleLight CAN 适配器数量: `2`

## CAN 接口

- ip 命令: `/usr/sbin/ip`

### CAN link

```text
3: can_left: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 
    can state ERROR-ACTIVE restart-ms 0 
	  bitrate 1000000 sample-point 0.750 
	  tq 62 prop-seg 5 phase-seg1 6 phase-seg2 4 sjw 2
	  gs_usb: tseg1 1..16 tseg2 1..8 sjw 1..4 brp 1..1024 brp-inc 1
	  clock 48000000 numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 parentbus usb parentdev 1-6:1.0 
4: can_right: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can  promiscuity 0 minmtu 0 maxmtu 0 
    can state ERROR-ACTIVE restart-ms 0 
	  bitrate 1000000 sample-point 0.750 
	  tq 62 prop-seg 5 phase-seg1 6 phase-seg2 4 sjw 2
	  gs_usb: tseg1 1..16 tseg2 1..8 sjw 1..4 brp 1..1024 brp-inc 1
	  clock 48000000 numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 parentbus usb parentdev 1-8:1.0 
```

### CAN 内核模块（已加载）

```text
can_raw                16384  0
can                    24576  1 can_raw
gs_usb                 24576  0
can_dev                53248  1 gs_usb
```
- candump: `/usr/bin/candump`

## 串口设备与稳定映射

- `/dev/serial/by-id`: missing
- `ls: cannot access '/dev/ttyUSB*': No such file or directory`
- `ls: cannot access '/dev/ttyACM*': No such file or directory`

## 视频设备与稳定映射

- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_262843072244-video-index5 -> ../../video11`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_261523072912-video-index5 -> ../../video5`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_261523072912-video-index3 -> ../../video3`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_262843072244-video-index1 -> ../../video7`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_261523072912-video-index2 -> ../../video2`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_261523072912-video-index0 -> ../../video0`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_262843072244-video-index3 -> ../../video9`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_262843072244-video-index0 -> ../../video6`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_262843072244-video-index2 -> ../../video8`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_262843072244-video-index4 -> ../../video10`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_261523072912-video-index1 -> ../../video1`
- `usb-Intel_R__RealSense_TM__Depth_Camera_405_Intel_R__RealSense_TM__Depth_Camera_405_261523072912-video-index4 -> ../../video4`
- `crw-rw-rw-+ 1 root plugdev 81,  0 Nov  1 22:11 /dev/video0`
- `crw-rw-rw-+ 1 root plugdev 81,  1 Nov  1 22:11 /dev/video1`
- `crw-rw-rw-+ 1 root plugdev 81, 10 Nov  1 22:11 /dev/video10`
- `crw-rw-rw-+ 1 root plugdev 81, 11 Nov  1 22:11 /dev/video11`
- `crw-rw-rw-+ 1 root plugdev 81,  2 Nov  1 22:11 /dev/video2`
- `crw-rw-rw-+ 1 root plugdev 81,  3 Nov  1 22:11 /dev/video3`
- `crw-rw-rw-+ 1 root plugdev 81,  4 Nov  1 22:11 /dev/video4`
- `crw-rw-rw-+ 1 root plugdev 81,  5 Nov  1 22:11 /dev/video5`
- `crw-rw-rw-+ 1 root plugdev 81,  6 Nov  1 22:11 /dev/video6`
- `crw-rw-rw-+ 1 root plugdev 81,  7 Nov  1 22:11 /dev/video7`
- `crw-rw-rw-+ 1 root plugdev 81,  8 Nov  1 22:11 /dev/video8`
- `crw-rw-rw-+ 1 root plugdev 81,  9 Nov  1 22:11 /dev/video9`
- v4l2-ctl: `/usr/bin/v4l2-ctl`
- rs-enumerate-devices: `/opt/ros/humble/bin/rs-enumerate-devices`

## udev 规则

- /etc/udev/rules.d/99-realsense-libusb.rules: `present`
- /etc/udev/rules.d/99-agx-can.rules: `missing`
- /etc/udev/rules.d/99-nero-can.rules: `missing`

## 网络与主机时间


```text
lo               UNKNOWN        127.0.0.1/8 ::1/128 
enp2s0           UP             10.10.10.46/24 fe80::97a6:f456:d830:8913/64 
can_left         UP             
can_right        UP             
wlo1             UP             10.10.11.146/24 fe80::b415:1d10:99a6:d26b/64 
```
- 时间: `2026-11-02 14:32:35 CST`
- NTP 同步: `yes`
