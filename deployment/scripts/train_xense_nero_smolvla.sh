#!/usr/bin/env bash
# Train SmolVLA on the converted 20-D Xense/NERO dataset.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
LEROBOT_PYTHON="${LEROBOT_PYTHON:-${HOME}/miniforge3/envs/lerobot/bin/python}"
DATASET_ROOT="${1:-${HOME}/datasets/lerobot/xense_plates_nero20}"
OUTPUT_DIR="${2:-${HOME}/checkpoints/xense_plates_nero20_smolvla}"
STEPS="${STEPS:-20000}"
BATCH_SIZE="${BATCH_SIZE:-8}"
SAVE_FREQ="${SAVE_FREQ:-2000}"
NUM_WORKERS="${NUM_WORKERS:-4}"
JOB_NAME="${JOB_NAME:-xense_plates_nero20_smolvla}"
DATASET_REPO_ID="${DATASET_REPO_ID:-local/xense_plates_nero20}"
POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
POLICY_REPO_ID="${POLICY_REPO_ID:-local/xense_plates_nero20_smolvla}"
RENAME_MAP="${RENAME_MAP:-{\"observation.images.side\":\"observation.images.camera1\",\"observation.images.left_wrist\":\"observation.images.camera2\",\"observation.images.right_wrist\":\"observation.images.camera3\"}}"

# The first two positional arguments select the dataset and output directory.
# Any remaining arguments are forwarded verbatim to lerobot_train.
if (( $# > 0 )); then shift; fi
if (( $# > 0 )); then shift; fi

if [[ ! -x "${LEROBOT_PYTHON}" ]]; then
  echo "LeRobot Python not found: ${LEROBOT_PYTHON}" >&2
  exit 1
fi
if [[ ! -f "${DATASET_ROOT}/meta/info.json" ]]; then
  echo "Converted dataset not found: ${DATASET_ROOT}" >&2
  echo "Run deployment/scripts/xense_to_nero_lerobot.py first." >&2
  exit 1
fi

mkdir -p "$(dirname -- "${OUTPUT_DIR}")"
export PYTHONUNBUFFERED=1
export TOKENIZERS_PARALLELISM=false

exec "${LEROBOT_PYTHON}" -m lerobot.scripts.lerobot_train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_ROOT}" \
  --dataset.video_backend=pyav \
  --rename_map="${RENAME_MAP}" \
  --policy.repo_id="${POLICY_REPO_ID}" \
  --policy.input_features=null \
  --policy.push_to_hub=false \
  --policy.device="${POLICY_DEVICE}" \
  --batch_size="${BATCH_SIZE}" \
  --num_workers="${NUM_WORKERS}" \
  --steps="${STEPS}" \
  --save_freq="${SAVE_FREQ}" \
  --save_checkpoint=true \
  --output_dir="${OUTPUT_DIR}" \
  --job_name="${JOB_NAME}" \
  "$@"
