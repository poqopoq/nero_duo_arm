#!/usr/bin/env python3
"""Declare inference-only PI05 config fields used by NERO joint16 checkpoints.

The RoboTwin joint16 checkpoint was trained with a PI05 configuration that
stores a few training-only keys in ``config.json``.  The pinned LeRobot 0.6.2
runtime used by the policy server does not declare those keys, and draccus
rejects the checkpoint before any weights are loaded.  The checkpoint has
``use_feature_adapter=False`` and no adapter/memory tensors, so declaring the
keys as inert values is sufficient for inference.

Usage:
    python deployment/scripts/patch_lerobot_pi05_checkpoint_compat.py \
        --lerobot-root /home/zhiyuan/lerobot

    python ... --check   # exit 1 when the compatibility fields are missing
"""

from __future__ import annotations

import argparse
import py_compile
import shutil
import sys
from pathlib import Path

SENTINEL = "NERO joint16 checkpoint compatibility"

DROP_ANCHOR = (
    "    n_action_steps: int = 50  # Number of action steps to execute\n"
)
DROP_INSERT = (
    DROP_ANCHOR
    + "\n    # Training-only field emitted by the NERO checkpoint; ignored at inference.\n"
    + "    drop_n_last_frames: int | None = None\n"
)

RELATIVE_ANCHOR = "    use_relative_actions: bool = False\n"
RELATIVE_INSERT = (
    RELATIVE_ANCHOR
    + '    relative_action_type: str = "joint"\n'
)

ADAPTER_ANCHOR = (
    "    train_expert_only: bool = False  # Freeze entire VLM, train only action expert and projections\n"
)
ADAPTER_INSERT = (
    ADAPTER_ANCHOR
    + f"""
    # {SENTINEL} fields. These are training-only keys from the checkpoint
    # config; they are inert during inference.
    action_loss_mask_indices: list[int] = field(default_factory=list)
    action_loss_weight_key: str | None = None

    use_feature_adapter: bool = False
    feature_adapter_camera_keys: list[str] = field(
        default_factory=lambda: ["observation.images.front"]
    )
    feature_adapter_variant: str = "legacy"
    feature_adapter_bottleneck_dim: int = 256
    feature_adapter_num_heads: int = 8
    feature_adapter_num_blocks: int = 2
    feature_adapter_ffn_expansion: int = 4
    feature_adapter_num_experts: int = 4
    feature_adapter_pose_enabled: bool = False
    feature_adapter_pose_dim: int = 9
    feature_adapter_checkpoint: str | None = None
    feature_adapter_flow_loss_weight: float = 0.05
    feature_adapter_velocity_loss_weight: float = 1.0
    feature_adapter_global_feature_loss_weight: float = 0.05
    feature_adapter_canonical_identity_loss_weight: float = 0.2
    feature_adapter_canonical_velocity_loss_weight: float = 1.0
    feature_adapter_residual_loss_weight: float = 0.01
"""
)

POST_INIT_ANCHOR = (
    '        if self.memory_temporal_attention_every < 1:\n'
    '            raise ValueError("memory_temporal_attention_every must be at least 1")\n'
)
POST_INIT_INSERT = (
    POST_INIT_ANCHOR
    + f"""
        # {SENTINEL}: only the disabled path is implemented by this runtime.
        if self.use_feature_adapter:
            raise ValueError(
                "This LeRobot runtime does not implement use_feature_adapter=True"
            )
        if self.relative_action_type not in ("joint", "ee_so3", "ee_local_se3"):
            raise ValueError(
                f"Invalid relative_action_type: {{self.relative_action_type}}"
            )
"""
)


def _insert_once(text: str, anchor: str, replacement: str, label: str) -> str:
    if anchor not in text:
        raise RuntimeError(f"cannot find anchor for {label}")
    return text.replace(anchor, replacement, 1)


def patch_text(text: str) -> tuple[str, list[str]]:
    """Return patched file text plus the names of added field groups."""
    added: list[str] = []
    if "drop_n_last_frames" not in text:
        text = _insert_once(text, DROP_ANCHOR, DROP_INSERT, "drop_n_last_frames")
        added.append("drop_n_last_frames")
    if "relative_action_type" not in text:
        text = _insert_once(
            text, RELATIVE_ANCHOR, RELATIVE_INSERT, "relative_action_type")
        added.append("relative_action_type")
    if "use_feature_adapter" not in text:
        text = _insert_once(
            text, ADAPTER_ANCHOR, ADAPTER_INSERT, "feature-adapter fields")
        added.append("feature-adapter fields")
    if "does not implement use_feature_adapter=True" not in text:
        text = _insert_once(
            text, POST_INIT_ANCHOR, POST_INIT_INSERT, "PI05 __post_init__ guard")
        added.append("__post_init__ guard")
    return text, added


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lerobot-root",
        type=Path,
        default=Path("/home/zhiyuan/lerobot"),
        help="LeRobot checkout containing src/lerobot (default: %(default)s)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="only verify the compatibility fields are present",
    )
    args = parser.parse_args()

    config_path = (
        args.lerobot_root
        / "src/lerobot/policies/pi05/configuration_pi05.py"
    )
    if not config_path.is_file():
        print(f"PI05 configuration not found: {config_path}", file=sys.stderr)
        return 2

    original = config_path.read_text()
    patched, added = patch_text(original)
    if args.check:
        if added:
            print(
                "PI05 checkpoint compatibility is incomplete; missing: "
                + ", ".join(added),
                file=sys.stderr,
            )
            return 1
        print(f"PI05 checkpoint compatibility already present: {config_path}")
        return 0

    if not added:
        print(f"PI05 checkpoint compatibility already present: {config_path}")
        return 0

    backup = config_path.with_suffix(config_path.suffix + ".bak-joint16")
    if not backup.exists():
        shutil.copy2(config_path, backup)
    config_path.write_text(patched)
    py_compile.compile(str(config_path), doraise=True)
    print(f"Patched {config_path}")
    print("Added: " + ", ".join(added))
    print(f"Backup: {backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
