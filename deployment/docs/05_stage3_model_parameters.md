# 阶段 3：桌面双臂模型参数

> 记录日期：2026-10-30  
> 布局：两台 NERO 水平正装、面向桌内，基座后缘贴桌面前沿。

## 1. 已确认参数

| 参数 | 数值 | 说明 |
| --- | --- | --- |
| world 位置 | 桌面平面、两基座中心中点 | `world.z=0`，不再设置桌面高度 |
| 两基座中心距 | `0.70 m` | 现场确定值 |
| 基座中心距前沿 | `0.060 m` | 桌面前沿位于 `world.x=-0.06 m` |
| 基座朝向 | `yaw=π` | 基座旋转 180°，夹爪工作侧朝向桌内 |
| 左侧位置 | `[0, +0.35, 0]` | 相对 world，`+Y` 为左 |
| 右侧位置 | `[0, -0.35, 0]` | 相对 world，`-Y` 为右 |
| 桌面尺寸占位 | `1.80 m x 0.90 m x 0.03 m` | 用户表示暂定足够大 |
| 末端执行器 | 暂不安装 | 默认 `include_gripper=false` |
| TCP | 暂用空偏移 | `tcp_link` 默认与 `tool0` 重合 |

## 2. 模型坐标系

```text
world                          z = 0（桌面平面，两基座中心中点）
├── table_slab                 x = depth/2 - 0.06, z = -thickness/2
├── left_base_mount            xyz = [0, +0.35, 0], yaw = 0
│   └── left_base_link ... left_joint1..7, left_tool0, left_tcp_link
└── right_base_mount           xyz = [0, -0.35, 0], yaw = 0
    └── right_base_link ... right_joint1..7, right_tool0, right_tcp_link
```

## 3. 可达性与间距依据

- 官方 URDF 计算水平最大可达半径：约 `0.611 m`
- 最大垂直伸展：约 `0.749 m`
- 基座外形包围盒：约 `109 mm x 80 mm x 86 mm`
- 700 mm 中心距时，两个 0.611 m 工作圆在中心方向约有 522 mm 重叠区；碰撞风险较高，必须依赖碰撞检测和受限动作。

## 4. 当前模型包

位置：

```text
dual_nero_ws/src/dual_nero_description
```

文件：

- `urdf/nero_arm_macro.xacro`：带 `left_/right_` 前缀的单臂宏。
- `urdf/two_nero_tabletop.urdf.xacro`：桌面、固定基座和双臂组合模型。
- `launch/display.launch.py`：RViz 模型检查。
- `rviz/dual_nero.rviz`：RViz 配置。

运行：

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_nero_description display.launch.py
```

## 5. 尚未确定

- [x] 基座中心到桌沿的实际距离：60 mm。
- [x] 暂不安装夹爪，默认模型不包含夹爪几何。
- [ ] 确定末端执行器后填写 TCP 偏移。
- D435 与两台 D405 的安装 XYZ/RPY。
- 基座实际 yaw 已按朝向桌面设置为 π。
- 桌面真实宽度、深度和厚度。


## 6. TCP 说明

TCP（Tool Center Point）是工具实际工作点的坐标系，例如：

- 夹爪两指之间的抓取中心。
- 吸盘接触面中心。
- 焊枪、螺丝刀或探针的尖端。

TCP 与法兰 `tool0` 不是同一个点。夹爪安装后通常需要配置：

```text
tool0 -> tcp_link
xyz = [x, y, z]  # 米
rpy = [roll, pitch, yaw]  # 弧度
```

当前没有确定夹爪，因此：

```text
left_tool0  -> left_tcp_link:  xyz=[0,0,0], rpy=[0,0,0]
right_tool0 -> right_tcp_link: xyz=[0,0,0], rpy=[0,0,0]
```

这只表示“TCP 暂等于法兰”，不是最终工具位置。未来可通过 launch 参数覆盖：

```bash
ros2 launch dual_nero_description display.launch.py   include_gripper:=false   left_tcp_xyz:='0.0 0.0 0.10'   left_tcp_rpy:='0.0 0.0 0.0'
```

`include_gripper:=true` 时使用参考仓库中的 Pika 兼容夹爪几何，但具体 TCP 仍应由实际夹爪尺寸确定。
