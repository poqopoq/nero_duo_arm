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

不启动 ROS 时，可以直接监测串口帧率、数值变化率和触摸响应。脚本会独占
串口，运行前先停止占用对应端口的 ROS 节点：

```bash
cd /home/work/nero_duo_arm
/usr/bin/python3 deployment/scripts/monitor_gripper_ft_sensors.py \
  /dev/ttyUSB1 /dev/ttyUSB3 /dev/ttyUSB4 --duration 30
```

不传端口时默认监测全部 `/dev/ttyUSB*`。持续触摸传感器 2–3 秒，出现
`<== TOUCH` 的那一路即当前传感器；`rate` 应约为 1000 Hz，`changed` 接近
`rate` 表示每个新帧的六轴数值都在变化，`span` 是该显示窗口内的峰谷变化。
脚本结束时会输出每路平均帧率、最大变化轴、CRC 错误和总帧数。

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

URDF 中传感器按 `25 mm` 厚、连接块按 `17.5 mm` 厚建模，合计 `42.5 mm`。
CTAG2F90D 自身从安装面到闭合指尖约 `222.5 mm`，因此整条末端从法兰到指尖
为 `265 mm`。原 `170.8 mm` 是安装面到两指捏合点的距离；叠加后 TCP 到法兰
为 `213.3 mm`。传感器和连接块的质量、惯量目前是名义值，称重后应回填 URDF。

在把数据用于抓取力反馈前至少完成：

1. 空载零点采集，记录 6 维均值与噪声；
2. 确认 6 个通道与传感器坐标轴的对应关系；
3. 确认力的单位（N、kgf 或原始计数）和力矩单位（N·m 或原始计数）；
4. 确认各轴正方向与夹爪/末端坐标系的关系；
5. 如需参与规划或学习，再发布对应静态 TF，并统一 `frame_id`。

当前 ROS 接口保留原始读数，不对零点和轴方向做隐式修正。

## 8. 低延迟 1 kHz 实时力控

### 8.1 驱动路径

节点默认使用低延迟配置：

- 串口读取在首字节到达后立即返回，并排空当前 `in_waiting` 数据，不再因为
  `read(4096)` 加 50 ms 超时而一次积压约 50 帧。
- `/gripper_ft/data` 使用 `best_effort`、`KEEP_LAST(1)` QoS，旧样本不会在
  DDS 或订阅端排队。
- 保持 `921600` 波特率和传感器 `1000 Hz` 输出。1000 Hz 时线路占用约
  `30 kB/s`，约为 921600 8N1 容量的 33%。

当前机器上以 `/dev/ttyUSB1` 实测 5 s：平均 `1000.0 Hz`，ROS 端到达间隔
P50 `1.006 ms`、P99 `1.736 ms`、最大 `1.965 ms`，没有超过 5 ms 的间隔，
串口 CRC 错误为 0。

修改采样率仍依赖双向 UART。若扫描结果为 `DATA_ONLY_NO_SN`，必须先连接
传感器黄色 RX 到适配器 TX，否则 `set_sample_rate` 服务不会得到应答。

### 8.2 控制环设计

1 kHz 力控不应把耗时的控制计算直接放在 ROS 订阅回调里。推荐订阅回调只校验
并保存最新样本，再由独立 1 kHz 定时器读取：

```text
serial worker -> /gripper_ft/data (best_effort, depth 1)
              -> atomic latest-sample mailbox
              -> 1 kHz control timer -> command path
```

控制定时器每次读取最新样本并检查样本年龄。建议超过 `3–5 ms` 就判定为超时，
使用保持、软停机或受控衰减，不能把旧力值无限重复用于闭环。不要使用
`KEEP_LAST(10)` 以上深度，也不要在回调中执行 `ros2 topic echo`、写大量日志、
保存大文件或等待服务。

订阅端应使用与发布端兼容的 QoS，例如：

```python
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

control_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
self.create_subscription(
    SixAxisForce, '/gripper_ft/data', latest_sample_callback,
    control_qos)
```

### 8.3 系统实时性

当前数据链路已经达到软实时 1 kHz。进一步降低尾部延迟可做以下调整：

- 保持 USB 设备常供电，禁用对应 USB 口自动挂起：向
  `/sys/bus/usb/devices/<bus-port>/power/control` 写入 `on`。
- 将 CPU governor 从 `powersave` 改为 `performance`，并避免深度 C-state。
- 用 `taskset`/cpuset 将串口节点、DDS 和控制循环隔开到专用 CPU，避免与
  相机、建图、训练或网页进程争用。
- 用 `chrt -f 80` 等 SCHED_FIFO 策略运行串口线程和控制线程；需要相应权限，
  并保留看门狗和超时保护。
- 对硬实时或确定性要求高的场景使用 PREEMPT_RT，并把实时控制环做成 C++、
  直接串口或共享内存路径。Python GC、GIL、ROS 2 执行器和 DDS 调度无法提供
  严格的 1 ms 最坏情况保证。

消息 `header.stamp` 是驱动收到帧的时间，不是传感器内部采样时刻；协议没有
样本时间戳。闭环应优先使用“最新样本 + 样本年龄”，不要把相邻消息时间差当作
传感器时钟。

## 9. 测试与故障排查

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
