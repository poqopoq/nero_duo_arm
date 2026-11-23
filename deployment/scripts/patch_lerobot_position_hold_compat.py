#!/usr/bin/env python3
"""Allow native-named robot actions to use LeRobot's local position hold.

LeRobot 0.6.2 assumes every retained position action ends in ``.pos``.  The
RoboTwin joint16 checkpoint instead uses its native names, including
``right_nero_joint_1`` and ``right_gripper_width``.  The NERO plugin declares
``position_hold_action_keys`` explicitly; this script teaches ``ThreadSafeRobot``
to honor that declaration while preserving the legacy ``.pos`` fallback.

Usage:
    python deployment/scripts/patch_lerobot_position_hold_compat.py \
        --lerobot-root /home/work/lerobot

    python ... --check   # exit 1 when the compatibility patch is missing
"""

from __future__ import annotations

import argparse
import py_compile
import shutil
import sys
from pathlib import Path

OBS_ANCHOR = """            self._observed_positions = {
                key: float(observation[key])
                for key in self.action_features
                if key.endswith(".pos") and key in observation
            }
"""
OBS_REPLACEMENT = """            self._observed_positions = {
                key: float(observation[key])
                for key in self._position_hold_keys()
                if key in observation
            }
"""

HELPER_ANCHOR = """        if not all(math.isfinite(value) for value in positions.values()):
            raise RuntimeError("Position hold requires finite position targets for every actuator")
        return positions
"""
HELPER_REPLACEMENT = HELPER_ANCHOR + """
    def _position_hold_keys(self) -> tuple[str, ...]:
        \"\"\"Return the action fields that can be retained as position targets.\"\"\"
        declared = getattr(self._robot, "position_hold_action_keys", None)
        if declared is None:
            return tuple(key for key in self.action_features if key.endswith(".pos"))
        if isinstance(declared, str):
            raise ValueError("position_hold_action_keys must be a sequence of action field names")
        try:
            keys = tuple(declared)
        except TypeError as exc:
            raise ValueError("position_hold_action_keys must be iterable") from exc
        if (
            not keys
            or any(not isinstance(key, str) or not key for key in keys)
            or len(keys) != len(set(keys))
        ):
            raise ValueError("position_hold_action_keys must contain unique non-empty strings")
        return keys

    @property
    def position_hold_keys(self) -> tuple[str, ...]:
        \"\"\"Public position-field contract used for startup pose capture.\"\"\"
        return self._position_hold_keys()
"""

PROPERTY_ANCHOR = """        return keys

    def configure_position_hold(self) -> None:
"""
PROPERTY_REPLACEMENT = """        return keys

    @property
    def position_hold_keys(self) -> tuple[str, ...]:
        \"\"\"Public position-field contract used for startup pose capture.\"\"\"
        return self._position_hold_keys()

    def configure_position_hold(self) -> None:
"""

CONFIGURE_ANCHOR = """        if (
            not self._robot.supports_position_hold
            or not self.action_features
            or any(not key.endswith(".pos") for key in self.action_features)
        ):
            raise ValueError(f"{self.robot_type} has no supported local position-hold contract")
"""
CONFIGURE_REPLACEMENT = """        if not self._robot.supports_position_hold or not self.action_features:
            raise ValueError(f"{self.robot_type} has no supported local position-hold contract")
        hold_keys = self._position_hold_keys()
        if set(hold_keys) != set(self.action_features):
            raise ValueError(f"{self.robot_type} has no supported local position-hold contract")
"""

CONTEXT_ANCHOR = """        initial_obs = wrapper.get_observation()
        initial_position = {key: value for key, value in initial_obs.items() if key.endswith(".pos")}
        logger.info("Captured initial robot position (%d keys)", len(initial_position))
"""
CONTEXT_REPLACEMENT = """        initial_obs = wrapper.get_observation()
        position_keys = wrapper.position_hold_keys
        initial_position = {
            key: float(initial_obs[key])
            for key in position_keys
            if key in initial_obs
        }
        logger.info("Captured initial robot position (%d keys)", len(initial_position))
        if len(initial_position) != len(position_keys):
            missing = [key for key in position_keys if key not in initial_obs]
            logger.warning(
                "Initial position is incomplete; reset will require all %d position keys. Missing: %s",
                len(position_keys),
                ", ".join(missing),
            )
"""


def _replace_once(text: str, anchor: str, replacement: str, label: str) -> str:
    if anchor not in text:
        raise RuntimeError(f"cannot find anchor for {label}")
    return text.replace(anchor, replacement, 1)


def patch_text(text: str) -> tuple[str, list[str]]:
    """Return patched text plus the names of missing compatibility pieces."""
    added: list[str] = []
    if "for key in self._position_hold_keys()" not in text:
        text = _replace_once(text, OBS_ANCHOR, OBS_REPLACEMENT, "observation hold keys")
        added.append("observation hold keys")
    if "def _position_hold_keys(self)" not in text:
        text = _replace_once(text, HELPER_ANCHOR, HELPER_REPLACEMENT, "hold-key resolver")
        added.append("hold-key resolver")
    if "def position_hold_keys(self)" not in text:
        text = _replace_once(text, PROPERTY_ANCHOR, PROPERTY_REPLACEMENT, "public hold-key property")
        added.append("public hold-key property")
    if "hold_keys = self._position_hold_keys()" not in text:
        text = _replace_once(text, CONFIGURE_ANCHOR, CONFIGURE_REPLACEMENT, "position-hold validation")
        added.append("position-hold validation")
    return text, added


def patch_context_text(text: str) -> tuple[str, list[str]]:
    """Patch startup capture to use the same declared position-hold keys."""
    if "position_keys = wrapper.position_hold_keys" in text:
        return text, []
    text = _replace_once(text, CONTEXT_ANCHOR, CONTEXT_REPLACEMENT, "initial-position capture")
    return text, ["initial-position capture"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lerobot-root",
        type=Path,
        default=Path("/home/work/lerobot"),
        help="LeRobot checkout containing src/lerobot (default: %(default)s)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="only verify the compatibility patch is present",
    )
    args = parser.parse_args()

    wrapper_path = args.lerobot_root / "src/lerobot/rollout/robot_wrapper.py"
    context_path = args.lerobot_root / "src/lerobot/rollout/context.py"
    if not wrapper_path.is_file() or not context_path.is_file():
        print(f"robot wrapper/context not found under: {args.lerobot_root}", file=sys.stderr)
        return 2

    wrapper_original = wrapper_path.read_text()
    wrapper_patched, wrapper_added = patch_text(wrapper_original)
    context_original = context_path.read_text()
    context_patched, context_added = patch_context_text(context_original)
    added = wrapper_added + context_added
    if args.check:
        if added:
            print(
                "position-hold compatibility is incomplete; missing: " + ", ".join(added),
                file=sys.stderr,
            )
            return 1
        print(f"position-hold compatibility already present: {args.lerobot_root}")
        return 0

    if not added:
        print(f"position-hold compatibility already present: {args.lerobot_root}")
        return 0

    if wrapper_added:
        backup = wrapper_path.with_suffix(wrapper_path.suffix + ".bak-joint16")
        if not backup.exists():
            shutil.copy2(wrapper_path, backup)
        wrapper_path.write_text(wrapper_patched)
        py_compile.compile(str(wrapper_path), doraise=True)
        print(f"Patched {wrapper_path}")
        print(f"Backup: {backup}")
    if context_added:
        context_backup = context_path.with_suffix(context_path.suffix + ".bak-joint16")
        if not context_backup.exists():
            shutil.copy2(context_path, context_backup)
        context_path.write_text(context_patched)
        py_compile.compile(str(context_path), doraise=True)
        print(f"Patched {context_path}")
        print(f"Backup: {context_backup}")
    print("Added: " + ", ".join(added))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
