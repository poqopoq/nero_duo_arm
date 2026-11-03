# 阶段 2：双 NERO 独立控制准备

> 记录日期：2026-10-30  
> 当前硬件：左右臂均接入  
> 状态：**完成（5/5）**；左右反馈、namespace、独立低速运动及双臂联动软件急停均通过

## 1. CAN 稳定命名

通过 `/etc/systemd/network/*.link` 按 USB 适配器序列号命名，避免依赖 `can0/can1` 枚举顺序。

| 逻辑名称 | 适配器序列号 | 当前状态 |
| --- | --- | --- |
| `can_left` | `004500404648571620303731` | 规则生效；1 Mbit/s；反馈正常 |
| `can_right` | `002200384648570B20303731` | 规则生效；1 Mbit/s；反馈正常 |

安装脚本：

```bash
/home/work/nero_duo_arm/deployment/scripts/install_can_stable_names.sh
```

## 2. 已确认

- [x] 左右机械臂均已接入
- [x] 左臂适配器序列号 `004500404648571620303731` 映射为 `can_left`
- [x] 右臂适配器序列号 `002200384648570B20303731` 映射为 `can_right`
- [x] 左右驱动固件均为 `v121 / 1.21`
- [x] `/left/feedback/joint_states` 和 `/left/control/*` 话题验证通过
- [x] 右臂及 `can_right` 现场验证
- [x] 左右驱动可同时运行，话题和节点互不冲突
- [x] 左右臂分别完成 5% 速度的低速运动与软件急停
- [x] 双臂联动软件急停通过
- [x] 测试后左右共 14 个电机全部失能

## 3. 已解决异常

稳定命名后的第一次 USB 重挂载导致 candleLight 在 Windows 侧进入 `0000:0002 设备描述符请求失败`，Linux 侧 `gs_usb` 返回 `err=-32 (EPIPE)`。物理重插 USB-CAN 适配器后恢复：

- Windows 正常识别 VID:PID `1d50:606f`。
- Linux `gs_usb` 正常 probe。
- 接口自动命名为 `can_left`。
- 被动捕获到 NERO 周期 CAN 帧。
- 7 关节反馈恢复。

## 4. 已执行验证

启用并只读验证：

```bash
cd /home/work/nero_duo_arm/deployment
sudo ./scripts/enable_can.sh can_left 1000000
sudo ./scripts/enable_can.sh can_right 1000000
./scripts/check_nero_feedback.sh can_left
./scripts/check_nero_feedback.sh can_right
```

验证左右 namespace：

```bash
./scripts/stage2_check_dual_namespace.sh
```

预期话题：

```text
/left/control/joint_states
/right/control/joint_states
/left/feedback/joint_states
/right/feedback/joint_states
/left/feedback/tcp_pose
/right/feedback/tcp_pose
```

实测 namespace 话题：

```text
/left/control/joint_states
/right/control/joint_states
/left/control/move_j
/right/control/move_j
/left/feedback/joint_states
/right/feedback/joint_states
/left/feedback/tcp_pose
/right/feedback/tcp_pose
```

驱动参数必须保持：

```text
left:  namespace:=left  can_port:=can_left
right: namespace:=right can_port:=can_right
arm_type:=nero
auto_enable:=false
effector_type:=none
```

## 5. 关节前缀设计

官方驱动在 `JointState.name` 中发布 `joint1..joint7`。namespace 只能隔离话题，不会自动修改 `JointState.name`。后续双臂模型需要：

- 驱动输入：继续使用未加前缀的 `joint1..joint7`。
- `/joint_states` 汇总：转换为 `left_joint1..left_joint7`、`right_joint1..right_joint7`。
- 或者为每侧增加 prefix/remap 节点，在汇总前完成名称转换。

话题隔离已验证；`JointState.name` 仍是 `joint1..joint7`，阶段 3 必须完成前缀转换。

## 6. 安全边界

现场不安装实体急停，按项目决定使用官方 NERO Web 端和 ROS `/emergency_stop` 作为软件停止机制；详见 `04_software_estop_validation.md`。

- [x] 左臂反馈恢复
- [x] `/left` namespace 验证通过
- [x] 左臂低速运动测试
- [x] 左臂 ROS 软件急停测试
- [x] 急停后关节稳定且电机失能
- [x] 右臂软件急停
- [x] 左右臂联动急停

## 7. 双臂联动急停实测

测试方式：左右臂 `joint7` 同时小幅运动，检测到两侧均移动至少 `0.01 rad` 后，同时调用 `/left/emergency_stop` 和 `/right/emergency_stop`，并关闭两侧控制门。

```text
left:  moved_before_stop=0.010210 rad
       post_stop_change=0.000942 rad
       late_window_drift=0.000140 rad
right: moved_before_stop=0.010664 rad
       post_stop_change=0.000035 rad
       late_window_drift=0.000140 rad
```

结果：两侧急停服务成功，关节稳定，最终 14 个电机全部 `disabled`。
