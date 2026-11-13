# dual_nero_gripper_ft_sensor

夹爪六维力/力矩传感器 UART TTL 接口。节点解析精一微感 V0.1.0 的
`AA 55` 协议帧，将 6 个 little-endian `float` 读数发布为
`dual_nero_gripper_ft_sensor_msgs/msg/SixAxisForce`，并提供自动上传、设备
信息、采样率和波特率服务。

## 硬件连接

传感器使用外部 5 V 供电，UART TTL 默认波特率 921600：

| 传感器 | USB-TTL 适配器 |
|--------|----------------|
| 红色 +5V | 外部 5 V（不要接 3.3 V TTL 供电） |
| 黑色 TTL TX | RX |
| 黄色 TTL RX | TX |
| 绿色 GND | GND |

传感器 TX 必须交叉接到适配器 RX；传感器、适配器和主机必须共地。

## 消息

```text
std_msgs/Header header
string sensor_id
string serial_number
float32[6] data
```

`sensor_id` 是配置的逻辑编号，`serial_number` 是节点启动时通过 `0x04` 命令
读取的硬件 SN。编号应绑定 SN，不能依赖会随 USB 口变化的 `ttyUSB*` 路径。

`data[0..5]` 是传感器返回的 6 维力/力矩读数，保持协议接收顺序。V0.1.0
协议没有定义各轴名称、正方向和单位，因此本接口不擅自把数组映射为
`Fx/Fy/Fz/Mx/My/Mz`。完成实物标定后，应在使用端按传感器坐标和量纲解释。

## 话题与服务

| 名称 | 类型 | 说明 |
|------|------|------|
| `/gripper_ft/data` | `dual_nero_gripper_ft_sensor_msgs/msg/SixAxisForce` | 六维力读数 |
| `/gripper_ft/raw` | `std_msgs/msg/UInt8MultiArray` | 原始 24 字节数据区，仅 `publish_raw:=true` 时发布 |
| `/gripper_ft_node/start_streaming` | `std_srvs/srv/Trigger` | 启动周期自动上传 |
| `/gripper_ft_node/stop_streaming` | `std_srvs/srv/Trigger` | 停止周期自动上传 |
| `/gripper_ft_node/get_version` | `std_srvs/srv/Trigger` | 返回固件版本，如 `0.1.0` |
| `/gripper_ft_node/get_serial_number` | `std_srvs/srv/Trigger` | 返回传感器 SN |
| `/gripper_ft_node/set_sample_rate` | `dual_nero_gripper_ft_sensor_msgs/srv/SetSampleRate` | 支持 500 / 1000 Hz |
| `/gripper_ft_node/set_baud_rate` | `dual_nero_gripper_ft_sensor_msgs/srv/SetBaudRate` | 支持 115200 / 230400 / 460800 / 921600 |

服务使用节点私有名称；节点名可通过 launch 参数改变。

## 构建

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt install python3-serial
colcon build --symlink-install --packages-select \
    dual_nero_gripper_ft_sensor_msgs dual_nero_gripper_ft_sensor
source install/setup.bash
```

串口权限：

```bash
sudo usermod -aG dialout "$USER"
```

加入 `dialout` 后需要重新登录。单只 CH340 可使用 `by-id`；多个 CH340 通常
没有唯一序列号，应使用 `by-path` 并以传感器 SN 区分身份。

## 运行

```bash
ros2 launch dual_nero_gripper_ft_sensor gripper_ft.launch.py \
    serial_port:=/dev/ttyUSB0 baud_rate:=921600 \
    sensor_id:=left expected_serial_number:=<sensor-sn>
```

查看读数：

```bash
ros2 topic echo /gripper_ft/data
ros2 topic hz /gripper_ft/data
```

启动、停止自动上传：

```bash
ros2 service call /gripper_ft_node/start_streaming std_srvs/srv/Trigger "{}"
ros2 service call /gripper_ft_node/stop_streaming std_srvs/srv/Trigger "{}"
```

读取版本和 SN：

```bash
ros2 service call /gripper_ft_node/get_version std_srvs/srv/Trigger "{}"
ros2 service call /gripper_ft_node/get_serial_number std_srvs/srv/Trigger "{}"
```

设置采样率和波特率：

```bash
ros2 service call /gripper_ft_node/set_sample_rate \
    dual_nero_gripper_ft_sensor_msgs/srv/SetSampleRate "{sample_rate_hz: 1000}"

ros2 service call /gripper_ft_node/set_baud_rate \
    dual_nero_gripper_ft_sensor_msgs/srv/SetBaudRate "{baud_rate: 460800}"
```

设置波特率成功后，节点会关闭原串口并以新波特率重新打开；`baud_rate` 参数
同步更新。若服务失败但设备已经切换波特率，需要以新波特率重启节点。

## 低延迟 1 kHz

节点默认按低延迟方式工作：串口收到首字节后立即读取当前内核缓冲区，不再
等待 50 ms 的串口超时；`/gripper_ft/data` 默认使用 `best_effort`、
`keep_last(1)` QoS。当前硬件实测平均 `1000 Hz`，ROS 端到达间隔 P99 约
`1.7 ms`。

实时力控订阅端应使用 best-effort、depth 1 QoS，并把回调收到的最后一帧放入
线程安全的最新值槽位，由独立 1 kHz 控制定时器读取；不要在每个回调里做重计算
或阻塞式日志。控制器还应检查样本年龄，例如超过 `3–5 ms` 时按超时处理。

`ros2 topic hz` 可以观察到约 1000 Hz，但平均帧率不能证明低延迟；验收时应
同时统计到达间隔 P99、最大间隔、CRC 错误和样本年龄。当前 Python + ROS 2
链路属于软实时，若要求硬实时或抖动小于 1 ms，应把串口采集和力控环放到
PREEMPT_RT 下的 C++/共享内存路径，而不是依赖 Python topic 回调。

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | 传感器串口 |
| `baud_rate` | `921600` | 串口波特率 |
| `frame_id` | `gripper_ft_sensor` | 消息 Header 坐标系 |
| `sensor_id` | 节点名 | 消息中的逻辑传感器编号 |
| `expected_serial_number` | 空 | 期望的硬件 SN；空表示接受任意传感器 |
| `query_serial_number_on_start` | `true` | 连接后自动查询 `0x04` SN |
| `require_serial_match` | `false` | SN 不匹配时是否拒绝发布数据 |
| `data_topic` | `/gripper_ft/data` | 六维力话题 |
| `raw_topic` | `/gripper_ft/raw` | 原始数据话题 |
| `publish_raw` | `false` | 是否发布原始 24 字节数据区 |
| `qos_reliability` | `best_effort` | 力数据 QoS；实时控制用 `best_effort`，可靠记录可改 `reliable` |
| `qos_depth` | `1` | 力数据 QoS 历史深度；实时控制保持 1，避免排队旧样本 |
| `response_timeout` | `1.0` | 命令应答超时，秒 |
| `reconnect_interval` | `2.0` | 串口重连间隔，秒 |

## 协议说明

- 帧格式：`AA 55 | msgType(1B) | dataLen(1B) | data | CRC16(2B)`。
- 测量帧：`msgType=0x01`，`dataLen=24`，数据为 6 个 little-endian `float`。
- CRC 使用 Modbus CRC16，校验范围为帧头到数据区末尾。
- 文档文字称 CRC 为“大端”，但所有样例在线上均为低字节在前，即标准
  Modbus RTU 字节序；实现和测试以样例为准。
- 传感器上电后默认周期上传，不调用启动服务也会持续发布。
- 串口断开时节点保持运行并自动重连；未连接时命令服务返回失败。
- 传感器 SN 识别需要双向 UART；只有数据单向连接时 `serial_number` 为空。

扫描当前端口与传感器 SN：

```bash
cd /home/work/nero_duo_arm
sg dialout -c '/usr/bin/python3 \
  deployment/scripts/identify_gripper_ft_sensors.py'
```

## 测试

协议层不依赖 ROS 和真实硬件：

```bash
cd /home/work/nero_duo_arm/ros2_ws/src/dual_nero_gripper_ft_sensor
PYTHONPATH=. /usr/bin/python3 -m pytest -q test
```

完整部署与上机检查见 `deployment/docs/18_gripper_ft_sensor.md`。
