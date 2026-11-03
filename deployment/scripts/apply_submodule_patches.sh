#!/usr/bin/env bash
# Apply deployment-specific patches to pinned third-party submodules.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${NERO_DUO_ROOT:-$(cd -- "${SCRIPT_DIR}/../.." && pwd)}"
AGX_DIR="${NERO_DUO_AGX_ARM_DIR:-${ROOT_DIR}/ros2_ws/src/agx_arm_ros}"
PATCH_FILE="${ROOT_DIR}/deployment/patches/agx_arm_ctrl_recovery_services.patch"

if [ ! -d "${AGX_DIR}/.git" ] && ! git -C "${AGX_DIR}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "agx_arm_ros submodule not found at ${AGX_DIR}" >&2
  exit 1
fi
if [ ! -f "${PATCH_FILE}" ]; then
  echo "Patch not found: ${PATCH_FILE}" >&2
  exit 1
fi

if git -C "${AGX_DIR}" apply --reverse --check "${PATCH_FILE}" >/dev/null 2>&1; then
  echo "agx_arm_ros recovery-services patch already applied."
elif git -C "${AGX_DIR}" apply --check "${PATCH_FILE}" >/dev/null 2>&1; then
  git -C "${AGX_DIR}" apply "${PATCH_FILE}"
  echo "Applied agx_arm_ros recovery-services patch."
else
  echo "Cannot apply ${PATCH_FILE}; check the agx_arm_ros submodule revision." >&2
  echo "Expected a clean checkout at commit 286633e unless the patch is already present." >&2
  exit 1
fi
