# 软件急停验证记录

> 记录日期：2026-10-30  
> 决策：现场不安装实体急停，采用官方 NERO Web 端和 ROS `/emergency_stop` 作为当前停止机制。  
> 状态：左臂、右臂和双臂联动软件急停均已通过。

## 1. 采用的急停路径

| 路径 | 当前用途 |
| --- | --- |
| 官方 NERO Web 端急停 | 现场人工停止入口 |
| ROS `/left/emergency_stop` | 左臂软件急停 |
| ROS `/right/emergency_stop` | 右臂软件急停，已验证 |
| `/left/control_enable`、`/right/control_enable` | 急停后关闭控制门，阻止后续命令 |

## 2. 驱动行为

ROS `/left/emergency_stop` 服务读取当前关节角，并调用：

```text
move_j(current_joint_positions)
```

作用是让机械臂保持当前位置。它不是硬件动力切断，但在本项目当前部署中作为约定的软件停止机制。

## 3. 左臂实测

测试配置：

```text
namespace: /left
can_port: can_left
arm_type: nero
speed_percent: 5
control_enabled: false（开始时关闭）
```

测试流程：

1. 使能 7 个关节。
2. 打开 `/left/control_enable`。
3. 仅向安全方向移动 `joint7`，目标增量 `+0.15 rad`。
4. 运动 0.35 秒后调用 `/left/emergency_stop`。
5. 连续观察 2.5 秒。
6. 关闭控制门并失能 7 个电机。

实测结果：

```text
joint7_start:            0.103830 rad
joint7_target:           0.253830 rad
joint7_at_stop:          0.149418 rad
joint7_final:            0.149383 rad
moved_before_stop:       0.045588 rad
post_stop_change:        0.000035 rad
late_window_drift:       0.000070 rad
software_stop_service:   success
post_test_motor_status:  7 x disabled
```

判定：**通过**。机械臂发生低速运动，软件急停后位置稳定，测试结束后电机全部失能。

右臂单独测试：

```text
joint7_start:            0.146171 rad
moved_before_stop:       0.010961 rad
post_stop_change:        0.000052 rad
late_window_drift:       0.000157 rad
software_stop_service:   success
post_test_motor_status:  7 x disabled
```

## 4. 已知限制

- 软件急停依赖上位机、ROS 节点和 CAN 通信持续可用。
- CAN 断开、进程崩溃或系统死机时，ROS 软件急停不可用。
- 官方 NERO Web 端急停是独立于 ROS 节点的第二入口。
- 不得将软件急停描述为硬件安全回路。

## 5. 双臂联动验证

软件急停不得只停止单臂。已按以下方式实现和验证：

1. 同时调用 `/left/emergency_stop` 和 `/right/emergency_stop`。
2. 同时将 `/left/control_enable` 和 `/right/control_enable` 设为 `false`。
3. 急停状态应锁存，防止松开按钮后自动继续执行旧目标。
4. 恢复时必须重新使能并重新打开控制门。
5. 在 WebApp 中加入一键双臂急停入口，或复用官方 Web 端联动急停。

## 6. 双臂联动实测

```text
left:  moved_before_stop=0.010210 rad
       post_stop_change=0.000942 rad
       late_window_drift=0.000140 rad
right: moved_before_stop=0.010664 rad
       post_stop_change=0.000035 rad
       late_window_drift=0.000140 rad
```

两侧同时调用急停成功，两侧控制门关闭，最终 14 个电机全部 `disabled`。

## 7. 验收清单

- [x] 左臂软件急停服务可用
- [x] 左臂低速运动
- [x] 急停后关节位置稳定
- [x] 急停后电机失能
- [x] 右臂软件急停
- [x] 左右臂联动急停
- [x] 左右臂急停后共 14 个电机失能
- [ ] 官方 Web 端急停链路现场操作确认（由现场使用者确认）
