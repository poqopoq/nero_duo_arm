#!/usr/bin/env bash
# Create an isolated Python environment for UMI -> LeRobot conversion.
#
# ROS 2 Humble uses Python 3.10.  LeRobot 0.4.4 supports Python 3.10 and is
# the last release that does, so keep it isolated from the ROS environment.
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV="${NERO_UMI_LEROBOT_VENV:-${HOME}/.venvs/umi_lerobot}"
PYTHON="${NERO_UMI_PYTHON:-python3}"
TORCH_INDEX="${NERO_UMI_TORCH_INDEX:-https://download.pytorch.org/whl/cpu}"

if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  echo "error: ${PYTHON} is not available" >&2
  exit 1
fi

PY_VERSION="$("${PYTHON}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if ! "${PYTHON}" -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)'; then
  echo "error: Python >= 3.10 is required (found ${PY_VERSION})" >&2
  exit 1
fi

if [ -x "${VENV}/bin/python" ] && "${VENV}/bin/python" -c 'import pip' >/dev/null 2>&1; then
  echo "[1/4] Reusing virtual environment: ${VENV}"
else
  echo "[1/4] Creating virtual environment: ${VENV}"
  if ! "${PYTHON}" -m venv --clear "${VENV}" 2>/dev/null; then
    echo "      python3-venv is unavailable; using user-space virtualenv"
    "${PYTHON}" -m pip install --user --upgrade virtualenv
    "${PYTHON}" -m virtualenv --clear "${VENV}"
  fi
fi

PY="${VENV}/bin/python"
PIP="${VENV}/bin/pip"

echo "[2/4] Upgrading pip"
"${PY}" -m pip install --upgrade pip wheel

echo "[3/4] Installing CPU PyTorch"
"${PIP}" install --index-url "${TORCH_INDEX}" \
  "torch>=2.2,<2.11" "torchvision>=0.21,<0.26"

echo "[4/4] Installing LeRobot 0.4.4 and UMI zarr dependencies"
"${PIP}" install \
  "lerobot==0.4.4" \
  "zarr<3" \
  "imagecodecs" \
  "pytest"

echo
echo "Verifying imports:"
"${PY}" - <<'PY'
import av
import imagecodecs
import lerobot
import pyarrow
import torch
import zarr

print(f"  lerobot={lerobot.__version__}")
print(f"  torch={torch.__version__}")
print(f"  zarr={zarr.__version__}")
print(f"  pyarrow={pyarrow.__version__}")
print(f"  av={av.__version__}")
print(f"  imagecodecs={imagecodecs.__version__}")
if not getattr(imagecodecs, "JPEGXL", False):
    print("  warning: imagecodecs has no JPEG XL support")
PY

echo
echo "Environment ready: ${VENV}"
echo
echo "Inspect a UMI dataset:"
echo "  ${VENV}/bin/python ${ROOT_DIR}/deployment/scripts/umi_to_lerobot.py \\"
echo "    /path/to/dataset.zarr.zip --dry-run --fps 30 --task 'pick up the block'"
echo
echo "Convert it:"
echo "  ${VENV}/bin/python ${ROOT_DIR}/deployment/scripts/umi_to_lerobot.py \\"
echo "    /path/to/dataset.zarr.zip -o ~/datasets/lerobot/umi_pick \\"
echo "    --repo-id local/umi_pick --fps 30 --task 'pick up the block' \\"
echo "    --gripper-min 0.0 --gripper-max 0.09 \\"
echo "    --pose-transform /path/to/T_robot_from_umi.json"
