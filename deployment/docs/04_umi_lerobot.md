# UMI 数据到 LeRobot

> 转换环境：`/home/work/.venvs/umi_lerobot`（Python 3.10 / LeRobot 0.4.4）  
> 训练/部署环境：LeRobot 0.6.2（Python 3.12）
> 本流程只转换数据，不需要机械臂运动。

## 1. 输入类型

| 输入 | 转换器 | 输出契约 |
| --- | --- | --- |
| canonical UMI zarr（`.zarr`/`.zarr.zip`） | `deployment/scripts/umi_to_lerobot.py` | 7-D EE，`umi-sroi` 或 `nero-ros` |
| canonical/嵌套/扁平 UMI HDF5（`.h5`/`.hdf5`） | 同上 | 7-D EE |
| Xense 原始双臂 HDF5/MKV | `hdf52lerobot` | 20-D rot6d |

canonical 数据结构：

```text
data/cameraN_rgb                 (T, H, W, 3) uint8
data/robot0_eef_pos              (T, 3) float32, m
data/robot0_eef_rot_axis_angle   (T, 3) float32, rad
data/robot0_gripper_width        (T, 1) float32, m
meta/episode_ends                (N,) int64
```

HDF5 使用同样字段；也支持单个外层 group `session/data/...` 和根目录扁平单集。
任意每集一个 group 的自定义布局不支持。

## 2. 数据契约

默认 `umi-sroi`：

```text
observation.images.camera   video
action                      float32[7]
```

本地 NERO 单臂使用 `--contract nero-ros`：

```text
observation.state = [x, y, z, rx, ry, rz, gripper]
action            = [x, y, z, rx, ry, rz, gripper]
observation.images.camera1
robot_type        = nero_ros
```

单位：位置米，旋转 axis-angle 弧度，夹爪 `0=闭合`、`1=张开`；
`action[t]` 是 `t+1` 的绝对位姿。

Xense 原始双臂转换输出 20-D rot6d：

```text
{left,right}_tcp.x/y/z
{left,right}_tcp.r1..r6
{left,right}_gripper.pos
```

## 3. 安装转换环境

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_umi_lerobot_env.sh
```

环境位于 `/home/work/.venvs/umi_lerobot`，不要求 GPU。

## 4. 先 dry-run

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /path/to/umi_session.zarr.zip \
  --fps 30 \
  --task "pick up the block" \
  --contract nero-ros \
  --dry-run
```

HDF5 只需替换输入路径。dry-run 校验 schema 并打印原始夹爪宽度范围，不写数据集。

## 5. 坐标变换

UMI 位姿通常在该次采集的 SLAM/world frame。训练前必须测量
`T_robot_from_umi`，将 UMI 坐标映射到 NERO 基座/桌面坐标。JSON 格式：

```json
{
  "matrix": [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
  ]
}
```

平移单位为米，左上 3×3 必须是合法旋转。上面的单位矩阵只是格式示例。

## 6. 夹爪归一化

```bash
--gripper-min 0.0 --gripper-max 0.09
```

优先使用实测物理开合范围。`--gripper-mode auto` 使用数据观测到的 min/max，
当演示没有完整开合时可能错误。每次转换会写
`meta/gripper_normalization.json`。

## 7. 转换

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /path/to/umi_session.zarr.zip \
  -o /home/work/datasets/lerobot/umi_pick \
  --repo-id local/umi_pick \
  --fps 30 \
  --task "pick up the block" \
  --contract nero-ros \
  --gripper-min 0.0 \
  --gripper-max 0.09 \
  --pose-transform /path/to/T_robot_from_umi.json
```

HDF5 把输入改为 `/path/to/umi_session.h5` 即可。

常用参数：

| 参数 | 说明 |
| --- | --- |
| `--episodes 0,2-4` | 只转换指定 episode |
| `--max-episodes N` | 限制转换数量，用于冒烟 |
| `--camera camera1_rgb` | 选择相机；可重复 |
| `--task-file tasks.json` | 每集提供语言指令 |
| `--robot-index 1` | 双臂 zarr 中只取一侧 |
| `--no-videos` | 保存逐帧图像而不是 MP4 |
| `--overwrite` | 已有输出移到时间戳 `.bak` 后重写 |
| `--contract nero-ros` | NERO 7-D state/action、`camera1` |
| `--contract umi-sroi` | 保持发布 UMI/SROI 契约（默认） |
| `--include-state` / `--no-state` | 显式覆盖 state 写入 |

## 8. 回读检查

```bash
/home/work/.venvs/umi_lerobot/bin/python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = "/home/work/datasets/lerobot/umi_pick"
ds = LeRobotDataset(repo_id="local/umi_pick", root=root)
sample = ds[0]
print("episodes", ds.meta.total_episodes)
print("frames", ds.meta.total_frames)
print("robot_type", ds.meta.robot_type)
print("action", sample["action"].shape)
print("state", sample["observation.state"].shape)
print("image", sample["observation.images.camera1"].shape)
assert tuple(sample["action"].shape) == (7,)
assert tuple(sample["observation.state"].shape) == (7,)
PY
```

## 9. 上传与训练

```bash
rsync -avP /home/work/datasets/lerobot/umi_pick/ \
  <USER>@<SERVER>:/data/lerobot/umi_pick/
```

服务器上先做短跑：

```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=local/umi_pick \
  --dataset.root=/data/lerobot/umi_pick \
  --policy.repo_id=${HF_USER}/nero_umi_pick_smolvla \
  --batch_size=8 \
  --steps=200 \
  --save_freq=100 \
  --output_dir=/data/checkpoints/nero_umi_pick_smoke \
  --job_name=nero_umi_pick_smoke \
  --policy.device=cuda
```

确认 loss 和显存正常后，再增加 `batch_size`/`steps`。训练集已经是 `camera1`，
不需要额外 `--rename_map`；如果报相机键不匹配，先检查 `info.json`。

## 10. 本地部署顺序

1. 启动驱动、相机、MoveIt（轨迹桥先 dry-run）。
2. 启动 VLA bridge，先 `execution_mode:=dry_run`。
3. 启动 `lerobot-policy-server` 和 `lerobot-rollout` remote client，`fps=5`。
4. 依次切换 `ik_dry_run`、`stream` 预览 topic、`stream + allow_real_motion`
   低速真机；需要逐 action 短轨迹时再验证 `trajectory + allow_real_motion`。
5. 真机阶段必须现场监护、清空工作区、确认软件急停。

完整 VLA 命令见 `05_vla_pipeline.md`。

## 11. 验证状态与限制

- canonical zarr、canonical/嵌套/扁平 HDF5、图像/视频、坐标变换和夹爪归一化
  测试通过。
- 公开样本 `ZIMBOT/umi-3d-dataset-curtain` 已验证 5 集、398 帧、7-D
  action/state 和 `camera1` 回读。
- 公开样本没有 NERO 标定，只能验证数据链路，不能直接上真机。
- 5 集冒烟数据不足以训练可用策略；真实 checkpoint 整链路仍未验收。
- 相机分辨率、视角和安装位置与 UMI 采集设备不同，会显著影响迁移效果。
