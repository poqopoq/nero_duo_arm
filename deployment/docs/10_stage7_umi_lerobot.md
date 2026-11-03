# Stage 7: UMI Data to LeRobot v3

This stage converts **handheld UMI demonstrations** into a LeRobot dataset.
It does not use NERO joint feedback and does not require the robot to move.
Canonical UMI replay buffers are accepted as zarr (`.zarr`/`.zarr.zip`) or
HDF5 (`.h5`/`.hdf5`).

## Two different UMI inputs

Use the converter that matches the recording format:

| Input | Converter | Output contract |
|---|---|---|
| Canonical UMI replay-buffer zarr (`.zarr` or `.zarr.zip`) | `deployment/scripts/umi_to_lerobot.py` | Selectable 7-D EE contract (`umi-sroi` or `nero-ros`) |
| Canonical, nested, or flat UMI HDF5 (`.h5` or `.hdf5`) | `deployment/scripts/umi_to_lerobot.py` | Same 7-D EE contracts as zarr |
| Raw bimanual UMI HDF5/MKV session | Xense `hdf52lerobot` | Per-side 20-D state/action with rot6d |

The canonical replay buffer has this layout:

```text
data/cameraN_rgb                 (T, H, W, 3) uint8
data/robot0_eef_pos              (T, 3) float32, metres
data/robot0_eef_rot_axis_angle   (T, 3) float32, radians
data/robot0_gripper_width        (T, 1) float32, metres
meta/episode_ends                (N,) int64
```

HDF5 uses the same structure with groups and datasets in place of zarr arrays:

```text
data/camera0_rgb                 (T, H, W, 3) uint8
data/robot0_eef_pos              (T, 3) float32
data/robot0_eef_rot_axis_angle   (T, 3) float32
data/robot0_gripper_width        (T, 1) float32
meta/episode_ends                (N,) int64
```

The HDF5 reader also accepts one enclosing group (for example
`session/data/...` and `session/meta/...`) and a flat single-episode file where
the four arrays live at the root. A flat file without `episode_ends` is treated
as one episode. Arbitrary per-episode HDF5 group layouts are not supported.

The RAW Xense path is not this zarr layout. It reads `*_tracker_*.hdf5`,
`*_openv2_{lo}_{hi}.hdf5`, and MKV streams. Use that tool directly when the
source directory contains those files.

## Published UMI/SROI contract

The reference dataset and converter store only the features below:

```text
observation.images.camera   video, H x W x 3
action                      float32[7]
```

Action names and units:

```text
ee.x, ee.y, ee.z            metres
ee.wx, ee.wy, ee.wz         rotation vector / axis-angle, radians
ee.gripper_pos              normalized width: 0.0 closed, 1.0 open
```

Conversion semantics:

- `action[t]` is the absolute pose at `t + 1`.
- The final frame repeats the final pose.
- `observation.state` is omitted. The UMI/SROI training stack derives state
  from the absolute action column, then builds relative 10-D chunks
  `[dx, dy, dz, rot6d(6), gripper]` during training.
- Relative rotation must be computed with SE(3) transforms, not by subtracting
  axis-angle vectors.
- The converted data stays in absolute axis-angle form on disk.

Adding a state column is available with `--include-state`, but it is not the
published UMI/SROI default. Do not enable it unless the downstream policy
expects a state feature.

## NERO single-arm contract

For the local `lerobot_robot_nero` plugin, use `--contract nero-ros`:

```text
observation.state = [ee.x, ee.y, ee.z, ee.rx, ee.ry, ee.rz, ee.gripper]
action            = [ee.x, ee.y, ee.z, ee.rx, ee.ry, ee.rz, ee.gripper]
observation.images.camera1
robot_type        = nero_ros
```

That matches `lerobot_robot_nero.nero.STATE_NAMES`, the default `camera_key` in
`NeroConfig`, and the ROS bridge feature contract. The default `umi-sroi`
contract remains unchanged for compatibility with the published UMI/SROI
training stack.

## Install the isolated environment

LeRobot 0.4.4 is used because it still supports the Python 3.10 environment
available on Ubuntu 22.04 / ROS 2 Humble.

```bash
cd /home/work/nero_duo_arm
./deployment/scripts/install_umi_lerobot_env.sh
```

The environment is created at `/home/work/.venvs/umi_lerobot`.

## Inspect before converting

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /path/to/umi_session.zarr.zip \
  --fps 30 \
  --task "pick up the block" \
  --dry-run
```

The same command accepts `/path/to/umi_session.h5` or `.hdf5`. The dry run
validates the schema and prints the observed raw gripper-width range. It does
not import LeRobot or write a dataset.

## Required coordinate transform

UMI poses are normally expressed in the SLAM/world frame for that recording.
Before combining sessions, apply a fixed `T_robot_from_umi` transform that maps
the UMI frame into the NERO base/table frame. The converter accepts a JSON 4x4
matrix:

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

Translation is in metres. The top-left 3x3 block must be a proper rotation.
If the data is already in the target robot frame, omit `--pose-transform`.

## Gripper normalization

Canonical UMI replay buffers store a calibrated physical width in metres, not
the 0–1 value expected by LeRobot EE policies. Supply the physical closed and
fully-open bounds:

```bash
--gripper-min 0.0 --gripper-max 0.09
```

Prefer these explicit bounds. `--gripper-mode auto` uses the observed min/max
and is recorded as such, but it can be wrong when the demonstration never
fully opens or closes the jaw.

Every conversion writes `meta/gripper_normalization.json` with the formula,
bounds, source, and selected episodes.

## Convert canonical zarr or HDF5

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

For an HDF5 source, only the input path changes:

```bash
/home/work/.venvs/umi_lerobot/bin/python \
  deployment/scripts/umi_to_lerobot.py \
  /path/to/umi_session.h5 \
  -o /home/work/datasets/lerobot/umi_pick \
  --repo-id local/umi_pick \
  --fps 30 \
  --task "pick up the block" \
  --contract nero-ros \
  --gripper-min 0.0 \
  --gripper-max 0.09 \
  --pose-transform /path/to/T_robot_from_umi.json
```

Useful options:

- `--camera camera1_rgb` selects one camera. With `--contract nero-ros` a
  single selected camera is written as `observation.images.camera1`; the
  `umi-sroi` contract writes `observation.images.camera`.
- `--contract nero-ros` selects the local NERO 7-D state/action and `camera1`
  contract. The default `umi-sroi` contract is unchanged.
- `--no-state` overrides a contract that normally writes `observation.state`.
- `--episodes 0,2-4` and `--max-episodes` limit conversion.
- `--task-file tasks.json` supplies per-episode instructions as a JSON object
  mapping episode index to text.
- `--action-offset 1` is the published next-frame target. Keep the default.
- `--no-videos` stores images instead of MP4 files (larger, but avoids video
  encoding).
- `--robot-index 1` selects one arm if the zarr is bimanual.
- `--robot-type` defaults to `so100` for `umi-sroi` and `nero_ros` for
  `nero-ros`.

The converter verifies the finished dataset by loading its first frame with
`LeRobotDataset` unless `--no-verify` is supplied.

## Raw bimanual HDF5/MKV path

For the Xense raw recording layout, use the upstream project rather than this
zarr converter:

```bash
git clone https://github.com/XenseRobotics-AI/xense-dataset-convert.git
cd xense-dataset-convert
python3.12 -m pip install -e .

hdf52lerobot \
  --input /data/umi_raw_session \
  --output /data/umi_raw_session_lerobot \
  --task "pick up the block"
```

That converter writes `observation.state` and `action` with 20 dimensions:

```text
{left,right}_tcp.x/y/z
{left,right}_tcp.r1..r6
{left,right}_gripper.pos
```

It uses rotation-matrix rot6d, normalized gripper `0=closed`, `1=open`,
`action[t] = state[t+1]`, and `robot_type: bi_rdt_gripper`. It is a different
schema from the single-arm 7-D SROI contract above.

## Validation status

- Synthetic canonical-zarr and canonical/flat HDF5 conversion, image mode,
  video mode, action semantics, pose transform, and gripper normalization tests
  pass.
- A real public UMI sample (`ZIMBOT/umi-3d-dataset-curtain`, 743 episodes) was
  converted from both zarr and canonical HDF5 with `--contract nero-ros`; the
  resulting 5-episode smoke datasets reload through `LeRobotDataset` with 7-D
  action/state and `camera1` images.
- The public sample has no NERO calibration. Its poses remain in the UMI
  SLAM/world frame, so it validates the data pipeline but is not directly
  deployable on the real arm without a measured `T_robot_from_umi` transform.
- The NERO-specific question of mapping absolute UMI EE poses to NERO joint
  commands is outside this converter. The output is an EE-action dataset for
  an EE policy or an EE-to-joint training/inference adapter.

## References

- UMI/SROI LeRobot walkthrough:
  <https://huggingface.co/blog/zfff/umi-lerobot>
- Reference SROI converter:
  <https://github.com/AgRoboticsResearch/sroi_dataprocess>
- Relative-EE LeRobot fork:
  <https://github.com/ZhenghaoFei/lerobot-umi-relative-ee/tree/feat/relative-ee>
- Xense raw HDF5/MKV converter:
  <https://github.com/XenseRobotics-AI/xense-dataset-convert>
