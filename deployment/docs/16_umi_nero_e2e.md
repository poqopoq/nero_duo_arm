# UMI → LeRobot → SmolVLA → NERO 端到端流程

> 更新日期：2026-11-03  
> 转换环境：`/home/work/.venvs/umi_lerobot`（Python 3.10 / LeRobot 0.4.4）  
> 训练/部署环境：LeRobot 0.6.1（Python 3.12，与本地插件环境一致）

本文把四件事串起来：

1. 下载公开 UMI 数据。
2. 将 zarr 或 HDF5 转成 LeRobot v3。
3. 在 GPU 服务器微调 SmolVLA。
4. 在本地 NERO 单臂上分阶段部署。

## 0. 当前验证状态

已实际验证：

- 公开样本 `ZIMBOT/umi-3d-dataset-curtain` 下载完成，1.0 GiB，743 集、
  47,168 帧。
- zarr 样本转 LeRobot：前 5 集、398 帧，7D `action/state` 与
  `observation.images.camera1` 回读成功。
- 从同一真实样本切出的 canonical HDF5 和扁平单集 HDF5：读取、dry-run 和
  canonical HDF5 转 LeRobot 均成功。
- 转换器测试：19 项通过，覆盖 zarr、canonical/嵌套/扁平 HDF5、两种契约、
  视频/图像写出和元数据。

尚未验证：

- 本机没有 GPU，未执行 SmolVLA 训练。
- 公开样本没有 NERO 坐标标定，只验证数据链路，不能直接驱动机器人。
- 真实 checkpoint 的整链路推理、真实夹爪命令和 UMI→NERO 坐标标定仍需现场
  完成。

## 1. 下载公开 UMI 样本

```bash
mkdir -p /home/work/datasets/umi/raw
curl -L --fail --retry 3 \
  -o /home/work/datasets/umi/raw/dataset_curtain_umi_3d.zarr.zip \
  https://huggingface.co/datasets/ZIMBOT/umi-3d-dataset-curtain/resolve/main/dataset_curtain_umi_3d.zarr.zip

sha256sum /home/work/datasets/umi/raw/dataset_curtain_umi_3d.zarr.zip
```

预期 SHA256：

```text
d89f3365434ea43435de5c41fba376f01719abcaef3569e418ba37c60e824b65
```

样本字段：

```text
camera0_rgb                   (47168, 224, 224, 3) uint8, JPEG XL
robot0_eef_pos                (47168, 3) float32
robot0_eef_rot_axis_angle     (47168, 3) float32
robot0_gripper_width          (47168, 1) float32
meta/episode_ends             (743,) int64
```

样本元数据没有 fps 和语言指令，转换时需要显式传 `--fps 30 --task ...`。

## 2. 安装转换环境

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_umi_lerobot_env.sh
```

脚本会安装 `zarr<3`、`h5py`、`imagecodecs` 和兼容 Python 3.10 的
LeRobot 0.4.4。转换器不要求 GPU。

## 3. 检查并转换数据

### 3.1 先 dry-run

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /home/work/datasets/umi/raw/dataset_curtain_umi_3d.zarr.zip \
  --dry-run \
  --fps 30 \
  --task "close the curtain" \
  --contract nero-ros \
  --episodes 0-2
```

重点确认：

- `episodes/frames/fps/task` 正确。
- `action` 是 7D：`ee.x, ee.y, ee.z, ee.rx, ee.ry, ee.rz, ee.gripper`。
- `observation.state` 存在且同为 7D。
- 图像键是 `observation.images.camera1`。
- `robot_type=nero_ros`。
- 打印的 raw gripper width 范围合理。

### 3.2 转换 zarr

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /home/work/datasets/umi/raw/dataset_curtain_umi_3d.zarr.zip \
  -o /home/work/datasets/lerobot/umi_curtain_nero \
  --repo-id local/umi_curtain_nero \
  --fps 30 \
  --task "close the curtain" \
  --contract nero-ros \
  --gripper-min 0.0 \
  --gripper-max 0.084288 \
  --pose-transform /path/to/T_robot_from_umi.json
```

首轮冒烟可先使用 `--episodes 0-4 --no-videos --image-writer-threads 0`，
避免一次编码全部视频。正式训练通常保留视频，以减少磁盘占用。

### 3.3 转换 HDF5

同一转换器接受 `.h5` 和 `.hdf5`。最小命令只需替换输入：

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /path/to/umi_session.h5 \
  -o /home/work/datasets/lerobot/umi_hdf5_nero \
  --repo-id local/umi_hdf5_nero \
  --fps 30 \
  --task "close the curtain" \
  --contract nero-ros \
  --gripper-min 0.0 \
  --gripper-max 0.084288 \
  --pose-transform /path/to/T_robot_from_umi.json
```

支持的 HDF5 布局：

```text
data/camera0_rgb
data/robot0_eef_pos
data/robot0_eef_rot_axis_angle
data/robot0_gripper_width
meta/episode_ends
```

也包括：

- 单个外层 group：`session/data/...` + `session/meta/...`。
- 扁平单集：上述四个数组直接位于文件根目录；缺少 `episode_ends` 时按一集处理。

不支持每集一个任意 group 的自定义 HDF5。Xense 原始逐流 HDF5/MKV 双臂数据
继续使用 `hdf52lerobot`，输出是 20D rot6d 双臂契约，不是本文的单臂
`nero_ros` 契约。

### 3.4 回读检查

```bash
/home/work/.venvs/umi_lerobot/bin/python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = "/home/work/datasets/lerobot/umi_curtain_nero"
ds = LeRobotDataset(repo_id="local/umi_curtain_nero", root=root)
print("episodes", ds.meta.total_episodes)
print("frames", ds.meta.total_frames)
print("robot_type", ds.meta.robot_type)
print("features", ds.meta.features)
sample = ds[0]
print("action", sample["action"].shape)
print("state", sample["observation.state"].shape)
print("image", sample["observation.images.camera1"].shape)
assert tuple(sample["action"].shape) == (7,)
assert tuple(sample["observation.state"].shape) == (7,)
assert "observation.images.camera1" in sample
PY
```

## 4. 上传到 GPU 服务器

```bash
rsync -avP /home/work/datasets/lerobot/umi_curtain_nero/ \
  <USER>@<SERVER>:/data/lerobot/umi_curtain_nero/
```

服务器上安装与本地 `lerobot` 环境一致的 LeRobot 0.6.1，并确认：

```bash
lerobot-train --help
nvidia-smi
```

转换环境和训练环境不需要相同 LeRobot 版本；LeRobot v3 数据格式兼容。但
policy server 与本地 robot client 必须使用同一套协议版本。当前本地
`/home/work/miniforge3/envs/lerobot` 是 0.6.1，仍提供
`lerobot.async_inference.*`；工作区中的 0.6.2 源码已经改为新的 remote
inference API。升级时必须同时迁移 server/client，不能只升级一边。

## 5. 在服务器微调 SmolVLA

先做短跑确认特征和显存：

```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=local/umi_curtain_nero \
  --dataset.root=/data/lerobot/umi_curtain_nero \
  --policy.repo_id=${HF_USER}/nero_umi_curtain_smolvla \
  --batch_size=8 \
  --steps=200 \
  --save_freq=100 \
  --output_dir=/data/checkpoints/nero_umi_curtain_smoke \
  --job_name=nero_umi_curtain_smoke \
  --policy.device=cuda
```

确认 loss 正常下降、显存有余量后，再增加 batch/steps：

```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=local/umi_curtain_nero \
  --dataset.root=/data/lerobot/umi_curtain_nero \
  --policy.repo_id=${HF_USER}/nero_umi_curtain_smolvla \
  --batch_size=32 \
  --steps=20000 \
  --save_freq=2000 \
  --output_dir=/data/checkpoints/nero_umi_curtain_smolvla \
  --job_name=nero_umi_curtain_smolvla \
  --policy.device=cuda
```

当前数据已经是 `camera1`，不需要为 `smolvla_base` 额外设置
`--rename_map`。如果训练日志报相机特征不匹配，先检查数据集 `info.json`，
不要用错误的 rename 掩盖维度或键名问题。

训练完成后，将下列目录同步回本地：

```text
/data/checkpoints/nero_umi_curtain_smolvla/checkpoints/last/pretrained_model
```

也可只把该 checkpoint 目录保留在服务器，由 policy server 使用；本地
robot client 只引用其路径。

## 6. 本地部署顺序

不要跳过阶段。每一步都先确认上一阶段无故障。

先确认 ROS bridge 已编译、LeRobot NERO 插件已安装：

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_lerobot_nero_plugin.sh

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dual_nero_vla_bridge
source install/setup.bash
```

### 6.1 启动驱动、相机、MoveIt

```bash
cd /home/work/nero_duo_arm/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dual_nero_control dual_nero_drivers.launch.py \
  can_left:=can_left can_right:=can_right \
  speed_percent:=5 auto_enable:=true

ros2 launch dual_nero_cameras all_cameras.launch.py

ros2 launch dual_nero_moveit_config moveit_execution.launch.py \
  dry_run:=true rviz:=false
```

### 6.2 启动 VLA bridge：dry-run

```bash
ros2 launch dual_nero_vla_bridge vla_bridge.launch.py \
  execution_mode:=dry_run \
  allow_real_motion:=false \
  side:=left \
  image_key:=camera1 \
  ik_frame_id:=left_base_link
```

确认 `/tmp/nero_vla_bridge.sock` 存在，且 bridge 能收到 TCP、关节和相机。

### 6.3 启动策略服务器

在 GPU 服务器上：

```bash
python -m lerobot.async_inference.policy_server \
  --host=0.0.0.0 \
  --port=8080 \
  --fps=5 \
  --inference_latency=0.2 \
  --obs_queue_timeout=2
```

server 必须能访问 checkpoint。首次从低速开始，不要直接 30 Hz。

### 6.4 本地 robot client

```bash
env -u PYTHONPATH \
    -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u LD_LIBRARY_PATH \
    -u ROS_DISTRO \
    -u ROS_VERSION \
    -u ROS_PYTHON_VERSION \
    /home/work/miniforge3/envs/lerobot/bin/python \
    -m lerobot.async_inference.robot_client \
  --robot.type=nero_ros \
  --robot.bridge_socket=/tmp/nero_vla_bridge.sock \
  --robot.side=left \
  --robot.camera_key=camera1 \
  --server_address=<SERVER_IP>:8080 \
  --policy_type=smolvla \
  --pretrained_name_or_path=/data/checkpoints/nero_umi_curtain_smolvla/checkpoints/last/pretrained_model \
  --actions_per_chunk=10 \
  --fps=5 \
  --task="close the curtain"
```

图像尺寸、相机视角和任务文本必须与训练数据一致。首轮 `fps=5`、
`actions_per_chunk=10`，确认队列和短轨迹不超时后再提高。

### 6.5 逐步放开运动

1. `execution_mode:=dry_run`：只看动作、不调用 IK、不运动。
2. `execution_mode:=ik_dry_run`：验证 `left_base_link` 坐标系和 IK 解。
3. `execution_mode:=trajectory` + 轨迹桥 `dry_run:=true`：验证短轨迹发布。
4. `execution_mode:=trajectory` + `allow_real_motion:=true` + 轨迹桥
   `dry_run:=false`：最后一轮才允许真机低速运动。

真机阶段必须有现场监护、清空工作区、确认软件急停可用。任何 IK、关节限位、
轨迹超时或反馈过期错误都要先停止并排查，不要自动恢复。

## 7. 必须完成 UMI→NERO 坐标标定

UMI 位姿通常位于该次采集的 SLAM/world frame；NERO bridge 把 action 解释为
`left_base_link` 下的 TCP 位姿。两者不能默认相同。

在训练前测量固定变换 `T_robot_from_umi`，保存为 JSON：

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

转换时传 `--pose-transform /path/to/T_robot_from_umi.json`。上面的单位矩阵
只是格式示例，不表示公开样本可以直接部署。不同采集 session 需要先统一到
同一机器人/桌面坐标系，再合并训练。

## 8. 已知限制

- 公开样本可以验证格式和训练管线，但没有 NERO 标定数据。
- 当前 bridge 不发送真实夹爪命令；`ee.gripper` 参与训练/推理，但夹爪执行
  尚未接入。
- 当前链路是低速短轨迹/latest-wins，不是 15–30 Hz 连续笛卡尔伺服。
- 机器人相机与 UMI 相机不是同一设备；图像分辨率、视角和安装位置不同会
  显著影响策略迁移。
- 5 集冒烟数据不足以训练可用策略。按相似任务经验，至少需要几十集高质量、
  覆盖不同物体位置和初始状态的演示。

## 9. 相关文档

- [阶段 7：UMI 数据转 LeRobot v3](10_stage7_umi_lerobot.md)
- [阶段 9：SmolVLA VLA 部署链路](13_stage9_vla_smolvla.md)
- [阶段 10：VLA 动作到 NERO 运动执行](14_stage10_vla_motion.md)
- [完整使用手册与命令速查](12_usage_and_commands.md)
