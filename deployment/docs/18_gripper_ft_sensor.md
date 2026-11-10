# 夹爪六维力传感器

> 夹爪指尖六维力/力矩传感器，UART TTL，协议 V0.1.0。
> ROS 2 包：`dual_nero_gripper_ft_sensor`、`dual_nero_gripper_ft_sensor_msgs`。

## 1. 接口定位

传感器相对夹爪独立工作，不属于 ZQWL CAN 链路。ROS 2 节点只读串口并发布
六维力数据；启停上传、读取版本/SN、修改采样率和波特率通过服务完成，不会
使能或驱动夹爪。

协议测量帧包含 6 个 little-endian `float`，ROS 消息保持接收顺序：

```text
std_msgs/Header header
string sensor_id
string serial_number
float32[6] data
```

`sensor_id` 是人为配置的逻辑编号，`serial_number` 是从传感器 `0x04` 命令读回
的硬件 SN。两者随消息发布，因此录制数据后仍能确认来源。

V0.1.0 文档没有给出各轴的名称、正方向、单位和零点，因此不能仅凭数组下标
推定 `Fx/Fy/Fz/Mx/My/Mz`。使用到抓取力控制或学习状态前，必须完成轴映射和
量纲标定。

## 2. 接线与设备

传感器使用外部 5 V 供电：

| 传感器线色 | 连接 |
| --- | --- |
| 红色 `+5V` | 外部 5 V 正极 |
| 黑色 `TTL TX` | USB-TTL 适配器 RX |
| 黄色 `TTL RX` | USB-TTL 适配器 TX |
| 绿色 `GND` | 外部 5 V 负极和适配器 GND |

不要用 3.3 V TTL 电源替代 5 V 传感器供电；TX/RX 必须交叉，所有设备共地。

查看稳定串口路径：

```bash
ls -l /dev/serial/by-id/
```

只有一个 CH340 时可以把 launch 的 `serial_port` 设为
`/dev/serial/by-id/usb-*`。多个 CH340 通常没有唯一序列号，`by-id` 会重名；
此时用 `/dev/serial/by-path/pci-*-usb-*-port*`，传感器身份依靠硬件 SN 区分。
串口权限不足时，将用户加入 `dialout` 组并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

加入后需要重新登录；如果 `getent group dialout` 已包含当前用户、但 `id` 尚未
显示该组，可临时执行 `newgrp dialout`，或在自动化脚本中用
`sg dialout -c '<command>'`。

部分 Ubuntu 系统上的 `brltty` 会误占用 CH340 USB 串口，使设备只剩
`usbfs` 而没有 `/dev/ttyUSB*`，或在日志中出现大量 `Ignored Byte`。确认
不需要盲文设备后执行：

```bash
sudo systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
```

随后重新插拔传感器 USB。

## 3. 构建

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt install python3-serial
colcon build --symlink-install --packages-select \
  dual_nero_gripper_ft_sensor_msgs dual_nero_gripper_ft_sensor
source install/setup.bash
```

## 4. 启动

单只传感器：

```bash
ros2 launch dual_nero_gripper_ft_sensor gripper_ft.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=921600 \
  frame_id:=gripper_ft_sensor
```

也可以使用安装后的参数文件：

```bash
ros2 run dual_nero_gripper_ft_sensor gripper_ft_node \
  --ros-args --params-file \
  install/dual_nero_gripper_ft_sensor/share/dual_nero_gripper_ft_sensor/config/gripper_ft.yaml
```

默认接口：

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/gripper_ft/data` | `dual_nero_gripper_ft_sensor_msgs/msg/SixAxisForce` | 发布 | 6 维力/力矩读数 |
| `/gripper_ft/raw` | `std_msgs/msg/UInt8MultiArray` | 发布 | 原始 24 字节数据区，默认关闭 |
| `/gripper_ft_node/start_streaming` | `std_srvs/srv/Trigger` | 服务 | 恢复周期上传 |
| `/gripper_ft_node/stop_streaming` | `std_srvs/srv/Trigger` | 服务 | 停止周期上传 |
| `/gripper_ft_node/get_version` | `std_srvs/srv/Trigger` | 服务 | 固件版本 |
| `/gripper_ft_node/get_serial_number` | `std_srvs/srv/Trigger` | 服务 | 传感器 SN |
| `/gripper_ft_node/set_sample_rate` | `dual_nero_gripper_ft_sensor_msgs/srv/SetSampleRate` | 服务 | 500 / 1000 Hz |
| `/gripper_ft_node/set_baud_rate` | `dual_nero_gripper_ft_sensor_msgs/srv/SetBaudRate` | 服务 | 115200 / 230400 / 460800 / 921600 |

使用两只传感器时，分别指定节点名、串口和话题，避免服务与话题冲突：

```bash
ros2 launch dual_nero_gripper_ft_sensor gripper_ft.launch.py \
  serial_port:=/dev/serial/by-id/<left-sensor> \
  node_name:=left_gripper_ft_node \
  sensor_id:=left \
  expected_serial_number:=<left-sensor-sn> \
  require_serial_match:=true \
  data_topic:=/left_gripper/ft \
  raw_topic:=/left_gripper/ft_raw \
  frame_id:=left_gripper_ft_sensor

ros2 launch dual_nero_gripper_ft_sensor gripper_ft.launch.py \
  serial_port:=/dev/serial/by-id/<right-sensor> \
  node_name:=right_gripper_ft_node \
  sensor_id:=right \
  expected_serial_number:=<right-sensor-sn> \
  require_serial_match:=true \
  data_topic:=/right_gripper/ft \
  raw_topic:=/right_gripper/ft_raw \
  frame_id:=right_gripper_ft_sensor
```

### 4.1 传感器编号与 SN 识别

USB 口和 CH340 适配器不能可靠代表传感器身份：更换 USB 口后设备路径会变化，
而常见 CH340 适配器没有唯一 USB 序列号。传感器协议提供 `0x04` SN 查询，因此
编号必须绑定传感器 SN，而不是绑定 `/dev/ttyUSB*` 或 USB 口。

先停止所有占用串口的节点，再扫描当前连接：

```bash
cd /home/work/nero_duo_arm
sg dialout -c '/usr/bin/python3 \
  deployment/scripts/identify_gripper_ft_sensors.py'
```

输出包含 `port`、稳定物理端口 `by-path`、`serial_number` 和 `firmware`。例如：

```text
port        by-path                                           serial_number       firmware
/dev/ttyUSB0 /dev/serial/by-path/pci-0000:00:14.0-usb-0:5.3:1.0-port0  F6-020-...  0.1.0
```

状态 `DATA_ONLY_NO_SN` 表示黑色 TX 到适配器 RX 的数据链路正常，但传感器
黄色 RX 没有接到适配器 TX，因此只能读数据、不能读 SN 或发送命令；状态
`NO_DATA_NO_SN` 还表示数据链路本身没有建立。

将扫描到的 SN 写入对应 launch 的 `expected_serial_number`，并设置不同的
`sensor_id` 和话题。节点启动时自动查询 SN：

- SN 匹配：正常发布，数据消息携带配置的 `sensor_id` 和实际 `serial_number`。
- SN 不匹配：`require_serial_match:=true` 时拒绝发布，避免 USB 换口后把左右
  传感器数据标反；日志同时打印实际 SN 和期望 SN。
- SN 查询超时：检查传感器黄色 RX 是否接到适配器 TX。单向连接仍可输出六维
  数据，但无法读取 SN、版本或设置采样率。

更换 USB 口后重新运行扫描工具，更新每路 `serial_port` 即可。逻辑编号仍由 SN
决定，不需要根据 `ttyUSB*` 顺序猜左右。

## 5. 检查读数

```bash
ros2 topic hz /gripper_ft/data
ros2 topic echo --once /gripper_ft/data
ros2 service call /gripper_ft_node/get_version std_srvs/srv/Trigger "{}"
ros2 service call /gripper_ft_node/get_serial_number std_srvs/srv/Trigger "{}"
```

确认采样率：

```bash
ros2 service call /gripper_ft_node/set_sample_rate \
  dual_nero_gripper_ft_sensor_msgs/srv/SetSampleRate "{sample_rate_hz: 1000}"
```

修改波特率会同时修改传感器和本节点串口设置：

```bash
ros2 service call /gripper_ft_node/set_baud_rate \
  dual_nero_gripper_ft_sensor_msgs/srv/SetBaudRate "{baud_rate: 460800}"
```

若服务提示传感器已应答但串口重开失败，重启节点并把 `baud_rate` 改成新值。

## 6. 协议与实现

```text
AA 55 | msgType(1B) | dataLen(1B) | data[dataLen] | CRC16(2B)
```

- 测量：`msgType=0x01`，`dataLen=24`，6 个 little-endian `float`。
- 启动上传：`0x09`；停止上传：`0x0A`。
- 版本：`0x03`；SN：`0x04`。
- 设置波特率：`0x02`，数据字节为 `115200=0`、`230400=1`、`460800=2`、
  `921600=3`。
- 设置采样率：`0x0F`，`500 Hz=4`、`1000 Hz=5`。
- CRC 为 Modbus CRC16；文档称“大端”，但全部样例在线路上为低字节在前，
  实现按样例和标准 Modbus RTU 字节序处理。
- V0.1.0 没有定义通用“寄存器读/写”命令或寄存器地址表；身份读取使用
  `0x04`，请求固定为 `AA 55 04 00 33 2C`。任意寄存器读取需要厂商提供额外
  的寄存器协议，不能替代串口 TX 命令链路。

节点具备字节流重同步、CRC 校验、命令应答超时、重复应答容忍和串口自动重连。
传感器上电后默认持续上传，因此不需要先调用启动服务。

## 7. 标定与 TF

在把数据用于抓取力反馈前至少完成：

1. 空载零点采集，记录 6 维均值与噪声；
2. 确认 6 个通道与传感器坐标轴的对应关系；
3. 确认力的单位（N、kgf 或原始计数）和力矩单位（N·m 或原始计数）；
4. 确认各轴正方向与夹爪/末端坐标系的关系；
5. 如需参与规划或学习，再发布对应静态 TF，并统一 `frame_id`。

当前 ROS 接口保留原始读数，不对零点和轴方向做隐式修正。

## 8. 测试与故障排查

协议测试不依赖硬件：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/test_gripper_ft_sensor.sh
```

| 现象 | 检查 |
| --- | --- |
| 打不开串口 | 路径是否存在、`dialout` 权限、是否被其他程序占用 |
| CH340 没有生成 `/dev/ttyUSB*` | 检查是否被 `brltty` 占用，停止并屏蔽 `brltty-udev.service` 后重插 |
| 无 `/gripper_ft/data` | `ros2 topic list`、节点日志、传感器是否上电 |
| 有报文但无有效数据 | 检查波特率是否为 921600、TX/RX 是否交叉 |
| 读数正常但命令服务超时 | 检查传感器黄色 RX 是否接到适配器 TX；只能接收时传感器仍会周期上传 |
| CRC 错误持续增加 | 检查共地、线缆、电平兼容和串口参数 |
| 服务超时 | 传感器仍在上传但未应答命令；检查版本和命令支持 |
| 采样率偏高导致处理压力 | 改用 500 Hz 或降低下游订阅/发布频率 |

传感器是夹爪末端附件，安装和走线不得干涉夹爪开合、机械臂运动或相机视野。
