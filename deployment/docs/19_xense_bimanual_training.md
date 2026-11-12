# Xense Dual-Arm Dataset to NERO SmolVLA Training

This path uses the public Xense dual-arm UMI dataset:

```text
Xense/xtac-umi-g1-clean-plates-with-a-sponge
```

It is already a LeRobot v3 dataset with 100 episodes, 88,861 frames, 30 FPS,
and three useful `640x480` cameras. Its first 20 state/action dimensions use
the same left-then-right `xyz + rot6d + gripper` layout required by the local
bimanual NERO plugin. The source also contains a 9-D head pose, tracker
confidence, a stereo-right scene stream, and tactile streams; the converter
drops those.

## Pipeline layers

The preparation has three separate layers; they are not interchangeable:

1. **Raw Xense rig -> LeRobot v3.** Xense's official
   [`hdf52lerobot`](https://github.com/XenseRobotics-AI/xense-dataset-convert)
   reads the per-stream tracker, gripper, camera, and MKV files. It synchronizes
   streams, converts quaternion poses to rot6d, normalizes the calibrated
   gripper range, writes `action[t] = state[t+1]`, and re-encodes video. Its
   default `state_layout` is the 20-D left/right TCP-plus-gripper contract.
2. **Public Xense dataset -> NERO contract.** The selected Hub dataset is
   already LeRobot v3 and already contains the first 20 useful dimensions, so
   `hdf52lerobot` must not be run on it. This repository's
   `xense_to_nero_lerobot.py` trims the optional head pose, applies the NERO
   feature names and camera keys, and copies the existing videos unchanged.
3. **SmolVLA training.** This uses LeRobot's normal `lerobot-train` and
   `lerobot/smolvla_base` pipeline. The only Xense-specific training detail is
   the camera `--rename_map` expected by `smolvla_base`.

If starting from raw Xense recordings, use `hdf52lerobot` first, then run the
same NERO adapter only if the output still has head pose or non-NERO feature
names.

## 1. Download the required streams

Only the scene-left and two wrist streams are needed. This avoids downloading
the tactile and stereo-right videos.

```bash
/home/zhiyuan/miniforge3/envs/lerobot/bin/python - <<'PY'
from pathlib import Path
from huggingface_hub import snapshot_download

root = Path.home() / "datasets/xense/xtac-umi-g1-clean-plates-with-a-sponge"
snapshot_download(
    repo_id="Xense/xtac-umi-g1-clean-plates-with-a-sponge",
    repo_type="dataset",
    local_dir=root,
    allow_patterns=[
        "meta/**",
        "data/**",
        "videos/observation.images.head_left/**",
        "videos/observation.images.left_wrist/**",
        "videos/observation.images.right_wrist/**",
    ],
)
print(root)
PY
```

## 2. Convert to the NERO 20-D contract

```bash
cd /home/zhiyuan/nero_duo_arm

/home/zhiyuan/miniforge3/envs/lerobot/bin/python \
  deployment/scripts/xense_to_nero_lerobot.py \
  "$HOME/datasets/xense/xtac-umi-g1-clean-plates-with-a-sponge" \
  -o "$HOME/datasets/lerobot/xense_plates_nero20" \
  --repo-id local/xense_plates_nero20 \
  --overwrite
```

The converter:

- validates that `action[t] == state[t+1]` inside every episode;
- verifies both gripper dimensions and the rot6d representation;
- keeps only state/action dimensions `0:20`;
- writes the plugin's exact action/state names, ending in `.pos`;
- maps `observation.images.head_left` to `observation.images.side`;
- keeps `left_wrist` and `right_wrist`;
- copies the existing H.264 videos without re-encoding;
- writes `meta/xense_nero_conversion.json`.

Expected metadata:

```text
100 episodes
88,861 frames
30 FPS
20-D observation.state
20-D action
side, left_wrist, right_wrist
```

## 3. Train SmolVLA

The SmolVLM processor needs `num2words` in the LeRobot environment:

```bash
/home/zhiyuan/miniforge3/envs/lerobot/bin/python -m pip install num2words
```

Smoke test first:

```bash
STEPS=20 SAVE_FREQ=20 BATCH_SIZE=2 \
  deployment/scripts/train_xense_nero_smolvla.sh
```

Then run the normal fine-tuning job:

```bash
STEPS=20000 SAVE_FREQ=2000 BATCH_SIZE=8 \
  deployment/scripts/train_xense_nero_smolvla.sh
```

The script uses `lerobot/smolvla_base`, the converted local dataset, PyAV video
decoding, and the local LeRobot 0.6 environment. It also handles the
SmolVLA-specific mappings without changing the dataset on disk:

- renames `side`, `left_wrist`, and `right_wrist` to `camera1..3` during training;
- sets `--policy.input_features=null` so the saved checkpoint records all 20
  state dimensions instead of the base model's 6-D state metadata;
- keeps `--policy.push_to_hub=false`, so local `local/...` repo ids never trigger
  an unintended Hub upload.

The smoke run was verified with 20 steps, batch size 2, a saved checkpoint, and
one offline inference returning a finite `(1, 20)` action. The final model is
written under:

```text
~/checkpoints/xense_plates_nero20_smolvla/checkpoints/last/pretrained_model
```

## 4. Deployment boundary

The converter preserves the source TCP coordinates. Before real NERO motion,
the source world/tracker frame must be mapped into `left_base_link` and
`right_base_link` with a calibrated rigid transform. The current dataset is
therefore suitable for training and dry-run rollout, but not yet a calibrated
real-robot policy.

After frame calibration, use the bimanual rollout path in
`deployment/docs/15_stage11_vla_bimanual_eval.md` with
`--robot.action_schema=rot6d20` and the default camera keys `side`,
`left_wrist`, and `right_wrist`. Pass the same camera rename map used during
training to `lerobot-rollout`; the official SmolVLA guide requires it at both
training and inference time.
