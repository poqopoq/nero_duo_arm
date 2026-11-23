#!/usr/bin/env python3
"""Replay a RoboTwin joint16 HDF5 episode through the bimanual NERO bridge.

RoboTwin stores its 16-D joint vector as::

    left_arm[7], left_gripper, right_arm[7], right_gripper

The native NERO policy/bridge contract is::

    right_arm[7], left_arm[7], right_gripper_width_m, left_gripper_width_m

This script performs that conversion, converts RoboTwin's normalized gripper
values to metres, resamples the 30 Hz expert to the bridge command rate, and
optionally streams it to the real robot.  It is a dry-run by default; motion
requires both ``--execute`` and a bridge already running in ``stream`` mode
with real motion enabled.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import h5py
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
PLUGIN_ROOT = REPO_ROOT / "lerobot_robot_nero"
if str(PLUGIN_ROOT) not in sys.path:
    sys.path.insert(0, str(PLUGIN_ROOT))

from lerobot_robot_nero.bridge_client import BridgeClient, BridgeError  # noqa: E402


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 8))
HDF5_NAMES = tuple(
    [
        *(f"left_joint{index}" for index in range(1, 8)),
        "left_gripper_normalized",
        *(f"right_joint{index}" for index in range(1, 8)),
        "right_gripper_normalized",
    ]
)
BUS_NAMES = tuple(
    [
        *(f"right_joint{index}" for index in range(1, 8)),
        *(f"left_joint{index}" for index in range(1, 8)),
        "right_gripper_width_m",
        "left_gripper_width_m",
    ]
)

# These are the bridge defaults in bimanual_bridge.py.  Keep the replay checks
# identical to the component that will actually filter the command.
DEFAULT_JOINT_MIN = np.asarray(
    [
        -2.70526,
        -1.760,
        -2.75,
        -1.01,
        -2.75,
        -0.7330383,
        -1.5707963,
    ],
    dtype=np.float64,
)
DEFAULT_JOINT_MAX = np.asarray(
    [
        2.70526,
        1.74,
        2.75,
        2.14,
        2.75,
        0.8726646,
        1.5707963,
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class Episode:
    path: Path
    action_group: str
    fps: float
    timestamps: np.ndarray
    raw: np.ndarray
    bus: np.ndarray
    right_arm: np.ndarray
    left_arm: np.ndarray
    right_gripper_normalized: np.ndarray
    left_gripper_normalized: np.ndarray


@dataclass(frozen=True)
class ReplayCommands:
    times_s: np.ndarray
    actions: np.ndarray
    source_times_s: np.ndarray
    source_frame_indices: np.ndarray
    command_rate_hz: float
    speed: float


def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return str(value)


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(payload, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )


def resolve_hdf5_path(path: Path, episode: int) -> Path:
    path = path.expanduser().resolve()
    if path.is_file():
        return path
    candidate = path / "data" / f"episode{episode}.hdf5"
    if candidate.is_file():
        return candidate
    candidate = path / f"episode{episode}.hdf5"
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(
        f"cannot find episode{episode}.hdf5 under {path} (checked data/ and root)"
    )


def _read_fps(handle: h5py.File, fallback: float) -> float:
    if "timing/sample_hz" in handle:
        value = np.asarray(handle["timing/sample_hz"][()]).reshape(-1)
        if value.size and math.isfinite(float(value[0])) and float(value[0]) > 0:
            return float(value[0])
    return float(fallback)


def _require_component(handle: h5py.File, group: str, name: str) -> np.ndarray:
    key = f"{group}/{name}"
    if key not in handle:
        raise KeyError(f"missing HDF5 dataset {key!r}")
    return np.asarray(handle[key][:], dtype=np.float64)


def _verify_vector_matches_components(
    vector: np.ndarray,
    left_arm: np.ndarray,
    left_gripper: np.ndarray,
    right_arm: np.ndarray,
    right_gripper: np.ndarray,
) -> None:
    expected = np.concatenate(
        [left_arm, left_gripper[:, None], right_arm, right_gripper[:, None]],
        axis=1,
    )
    if vector.shape != expected.shape:
        raise ValueError(
            f"HDF5 vector shape {vector.shape} does not match component shape "
            f"{expected.shape}"
        )
    error = float(np.max(np.abs(vector - expected))) if vector.size else 0.0
    if error > 1e-6:
        raise ValueError(
            "HDF5 joint vector order is not left7+left_gripper+right7+right_gripper; "
            f"component reconstruction differs by {error:.6g}"
        )


def _to_width_m(
    values: np.ndarray,
    *,
    gripper_format: str,
    open_width_m: float,
    label: str,
) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError(f"{label} contains non-finite values")
    if gripper_format == "normalized":
        if values.min() < -1e-6 or values.max() > 1.0 + 1e-6:
            raise ValueError(
                f"{label} is not normalized to [0, 1]: "
                f"{values.min():.6f}..{values.max():.6f}. "
                "Use --gripper-format metres only for a metre-unit source."
            )
        return np.clip(values, 0.0, 1.0) * open_width_m
    if gripper_format == "metres":
        if values.min() < -1e-6 or values.max() > open_width_m + 1e-6:
            raise ValueError(
                f"{label} is outside 0..{open_width_m:.4f} m: "
                f"{values.min():.6f}..{values.max():.6f}"
            )
        return values
    raise ValueError(f"unsupported gripper format {gripper_format!r}")


def load_episode(
    path: Path,
    *,
    action_group: str = "joint_action",
    gripper_format: str = "normalized",
    gripper_open_width_m: float = 0.09,
    fallback_fps: float = 30.0,
) -> Episode:
    if gripper_open_width_m <= 0:
        raise ValueError("gripper open width must be positive")
    with h5py.File(path, "r") as handle:
        vector_key = f"{action_group}/vector"
        if vector_key not in handle:
            raise KeyError(f"missing HDF5 dataset {vector_key!r}")
        raw = np.asarray(handle[vector_key][:], dtype=np.float64)
        left_arm = _require_component(handle, action_group, "left_arm")
        left_gripper = _require_component(handle, action_group, "left_gripper")
        right_arm = _require_component(handle, action_group, "right_arm")
        right_gripper = _require_component(handle, action_group, "right_gripper")
        _verify_vector_matches_components(
            raw, left_arm, left_gripper, right_arm, right_gripper
        )
        fps = _read_fps(handle, fallback_fps)

    if raw.ndim != 2 or raw.shape[1] != 16:
        raise ValueError(f"{path} {action_group}/vector must be N x 16, got {raw.shape}")
    if raw.shape[0] == 0:
        raise ValueError(f"{path} contains no frames")
    if not np.isfinite(raw).all():
        raise ValueError(f"{path} contains non-finite joint values")

    right_width = _to_width_m(
        right_gripper,
        gripper_format=gripper_format,
        open_width_m=gripper_open_width_m,
        label=f"{path}:{action_group}/right_gripper",
    )
    left_width = _to_width_m(
        left_gripper,
        gripper_format=gripper_format,
        open_width_m=gripper_open_width_m,
        label=f"{path}:{action_group}/left_gripper",
    )
    bus = np.concatenate(
        [right_arm, left_arm, right_width[:, None], left_width[:, None]], axis=1
    )
    timestamps = np.arange(raw.shape[0], dtype=np.float64) / fps
    return Episode(
        path=path,
        action_group=action_group,
        fps=fps,
        timestamps=timestamps,
        raw=raw,
        bus=bus,
        right_arm=right_arm,
        left_arm=left_arm,
        right_gripper_normalized=right_gripper,
        left_gripper_normalized=left_gripper,
    )


def validate_trajectory(
    episode: Episode,
    *,
    joint_min: np.ndarray = DEFAULT_JOINT_MIN,
    joint_max: np.ndarray = DEFAULT_JOINT_MAX,
    max_joint_step_rad: float = 0.02,
    max_gripper_step_m: float = 0.02,
) -> dict[str, Any]:
    joints = episode.bus[:, :14]
    widths = episode.bus[:, 14:16]
    limits = np.concatenate([joint_max, joint_max])
    lower = np.concatenate([joint_min, joint_min])
    outside = (joints < lower) | (joints > limits)
    joint_steps = np.abs(np.diff(joints, axis=0)) if len(joints) > 1 else np.empty((0, 14))
    width_steps = np.abs(np.diff(widths, axis=0)) if len(widths) > 1 else np.empty((0, 2))
    max_delta = float(np.max(joint_steps)) if joint_steps.size else 0.0
    max_width_delta = float(np.max(width_steps)) if width_steps.size else 0.0
    return {
        "frames": int(episode.bus.shape[0]),
        "duration_s": float((episode.bus.shape[0] - 1) / episode.fps),
        "fps": float(episode.fps),
        "joint_min": joints.min(axis=0),
        "joint_max": joints.max(axis=0),
        "right_gripper_width_m": [float(widths[:, 0].min()), float(widths[:, 0].max())],
        "left_gripper_width_m": [float(widths[:, 1].min()), float(widths[:, 1].max())],
        "limit_violation_count": int(outside.sum()),
        "limit_violation_frames": np.flatnonzero(outside.any(axis=1)).tolist(),
        "max_frame_joint_delta_rad": max_delta,
        "max_frame_gripper_delta_m": max_width_delta,
        "frames_over_joint_step_rad": int(
            np.count_nonzero(joint_steps.max(axis=1) > max_joint_step_rad)
        ) if joint_steps.size else 0,
        "frames_over_gripper_step_m": int(
            np.count_nonzero(width_steps.max(axis=1) > max_gripper_step_m)
        ) if width_steps.size else 0,
        "max_joint_step_rad": float(max_joint_step_rad),
        "max_gripper_step_m": float(max_gripper_step_m),
    }


def suggested_speed(
    episode: Episode,
    *,
    command_rate_hz: float,
    max_joint_step_rad: float,
    max_gripper_step_m: float,
    safety: float = 0.9,
) -> float:
    joint_delta = (
        float(np.max(np.abs(np.diff(episode.bus[:, :14], axis=0))))
        if episode.bus.shape[0] > 1 else 0.0
    )
    gripper_delta = (
        float(np.max(np.abs(np.diff(episode.bus[:, 14:16], axis=0))))
        if episode.bus.shape[0] > 1 else 0.0
    )
    limits = []
    if joint_delta > 0:
        limits.append(
            safety * max_joint_step_rad * command_rate_hz / (episode.fps * joint_delta)
        )
    if gripper_delta > 0:
        limits.append(
            safety * max_gripper_step_m * command_rate_hz / (episode.fps * gripper_delta)
        )
    return min(limits) if limits else 1.0


def build_replay_commands(
    episode: Episode,
    *,
    speed: float,
    command_rate_hz: float,
    start_frame: int = 0,
    end_frame: int | None = None,
) -> ReplayCommands:
    if speed <= 0:
        raise ValueError("speed must be positive")
    if command_rate_hz <= 0:
        raise ValueError("command rate must be positive")
    end = episode.bus.shape[0] if end_frame is None else int(end_frame)
    if not 0 <= start_frame < end <= episode.bus.shape[0]:
        raise ValueError(
            f"invalid frame range [{start_frame}, {end}) for {episode.bus.shape[0]} frames"
        )
    source = episode.bus[start_frame:end]
    source_times = np.arange(source.shape[0], dtype=np.float64) / episode.fps / speed
    duration = source_times[-1] if source_times.size > 1 else 0.0
    command_count = max(1, int(math.floor(duration * command_rate_hz + 1e-9)) + 1)
    replay_times = np.arange(command_count, dtype=np.float64) / command_rate_hz
    actions = np.empty((command_count, source.shape[1]), dtype=np.float64)
    for index in range(source.shape[1]):
        actions[:, index] = np.interp(
            replay_times, source_times, source[:, index], left=source[0, index],
            right=source[-1, index]
        )
    return ReplayCommands(
        times_s=replay_times,
        actions=actions,
        source_times_s=source_times,
        source_frame_indices=(
            start_frame
            + np.clip(
                np.rint(replay_times * episode.fps * speed).astype(np.int64),
                0,
                source.shape[0] - 1,
            )
        ),
        command_rate_hz=float(command_rate_hz),
        speed=float(speed),
    )


def _image_suffix(key: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", key)
    return f"{safe}.jpg"


def _save_observation(
    record_dir: Path,
    frame_index: int,
    header: dict[str, Any],
    payload: bytes,
) -> dict[str, Any]:
    image_paths = []
    for image in header.get("images") or []:
        offset = int(image["offset"])
        size = int(image["size"])
        if offset < 0 or size < 0 or offset + size > len(payload):
            raise ValueError(f"invalid image payload for {image.get('key')!r}")
        path = record_dir / "images" / f"{frame_index:06d}_{_image_suffix(str(image['key']))}"
        path.write_bytes(payload[offset : offset + size])
        image_paths.append(str(path))
    return {
        "state": header.get("state"),
        "timestamps": header.get("timestamps"),
        "images": image_paths,
    }


def _save_reference_images(
    hdf5_path: Path,
    camera_keys: tuple[str, ...],
    source_frame_index: int,
    record_dir: Path,
    command_index: int,
) -> list[str]:
    camera_sources = {
        "base_0_rgb": "head_camera",
        "left_wrist_0_rgb": "left_camera",
        "right_wrist_0_rgb": "right_camera",
    }
    saved = []
    with h5py.File(hdf5_path, "r") as handle:
        for camera_key in camera_keys:
            source_key = camera_sources.get(camera_key)
            dataset_key = f"observation/{source_key}/rgb"
            if dataset_key not in handle:
                continue
            frame = int(np.clip(source_frame_index, 0, len(handle[dataset_key]) - 1))
            path = (
                record_dir
                / "reference_images"
                / f"{command_index:06d}_ref_{_image_suffix(camera_key)}"
            )
            path.write_bytes(bytes(handle[dataset_key][frame]))
            saved.append(str(path))
    return saved


def _require_stream_bridge(
    status: dict[str, Any],
    *,
    require_gripper: bool,
) -> None:
    problems = []
    if not status.get("bimanual"):
        problems.append("not a bimanual bridge")
    if status.get("schema") != "joint16":
        problems.append(f"schema={status.get('schema')!r}, expected 'joint16'")
    if status.get("execution_mode") != "stream":
        problems.append(
            f"execution_mode={status.get('execution_mode')!r}, expected 'stream'"
        )
    if not status.get("real_motion_enabled"):
        problems.append("real_motion_enabled is false")
    if status.get("motion_fault"):
        problems.append(f"bridge fault: {status['motion_fault']}")
    if not status.get("state_ready"):
        problems.append(f"state not ready: {status.get('state_error')}")
    if require_gripper:
        if not status.get("gripper_enabled"):
            problems.append("gripper_enabled is false")
        if not status.get("allow_gripper_motion"):
            problems.append("allow_gripper_motion is false")
        if not status.get("gripper_feedback_ready"):
            problems.append("gripper feedback is not ready")
    if problems:
        raise RuntimeError("bridge preflight failed: " + "; ".join(problems))


def _observation_state(client: BridgeClient) -> tuple[list[float], dict[str, Any], bytes]:
    header, payload = client.get_observation()
    if header.get("schema") != "joint16":
        raise RuntimeError(f"bridge observation schema is {header.get('schema')!r}")
    state = header.get("state")
    if not isinstance(state, list) or len(state) != 16:
        raise RuntimeError(f"bridge observation state is invalid: {state!r}")
    values = np.asarray(state, dtype=np.float64)
    if not np.isfinite(values).all():
        raise RuntimeError("bridge observation state contains non-finite values")
    return state, header, payload


def _initial_error(current: np.ndarray, reference: np.ndarray) -> dict[str, float]:
    delta = np.abs(current - reference)
    return {
        "max_rad_or_m": float(delta.max()),
        "right_arm_max_rad": float(delta[:7].max()),
        "left_arm_max_rad": float(delta[7:14].max()),
        "right_gripper_m": float(delta[14]),
        "left_gripper_m": float(delta[15]),
    }


def execute_replay(
    commands: ReplayCommands,
    *,
    socket_path: Path,
    output_dir: Path,
    require_gripper: bool,
    max_initial_joint_error_rad: float,
    start_hold_s: float,
    settle_s: float,
    observe_every: int,
    abort_on_clamped: bool,
    stop_on_exit: bool,
    reference_hdf5_path: Path | None = None,
    reference_camera_keys: tuple[str, ...] = (),
) -> dict[str, Any]:
    client = BridgeClient(str(socket_path), timeout_s=5.0)
    (output_dir / "images").mkdir(parents=True, exist_ok=True)
    (output_dir / "reference_images").mkdir(parents=True, exist_ok=True)
    records_path = output_dir / "replay.jsonl"
    observations = 0
    clamped_count = 0
    max_applied_error = 0.0
    max_applied_error_by_dim = np.zeros(16, dtype=np.float64)
    initial_error: dict[str, float] | None = None
    last_observation: dict[str, Any] | None = None
    records_written = 0
    motion_started = False

    try:
        client.connect()
        status = client.ping()
        _require_stream_bridge(status, require_gripper=require_gripper)
        state, header, payload = _observation_state(client)
        initial_error = _initial_error(
            np.asarray(state, dtype=np.float64), commands.actions[0]
        )
        if initial_error["max_rad_or_m"] > max_initial_joint_error_rad:
            raise RuntimeError(
                "initial robot pose is too far from the expert first frame: "
                f"max error {initial_error['max_rad_or_m']:.6f} > "
                f"{max_initial_joint_error_rad:.6f}. Move the arms to training home "
                "first, or adjust --max-initial-joint-error-rad."
            )
        last_observation = _save_observation(output_dir, 0, header, payload)
        observations += 1

        with records_path.open("w", encoding="utf-8") as records:
            start_mono = time.monotonic()
            command_count = commands.actions.shape[0]
            first_command = commands.actions[0].astype(float).tolist()
            hold_until = time.monotonic() + max(0.0, start_hold_s)
            while time.monotonic() < hold_until:
                hold_response = client.send_action(first_command)
                motion_started = True
                if not hold_response.get("ok", False) or hold_response.get("accepted") is False:
                    raise RuntimeError(
                        "bridge rejected the initial hold: "
                        f"{hold_response.get('error', 'unknown error')}"
                    )
                time.sleep(1.0 / 50.0)
            start_mono = time.monotonic()
            period = 1.0 / commands.command_rate_hz
            next_tick = start_mono
            for index in range(command_count):
                command = commands.actions[index].astype(float).tolist()
                header_sent = client.send_action(command)
                motion_started = True
                if not header_sent.get("ok", False):
                    raise RuntimeError(
                        f"bridge rejected frame {index}: {header_sent.get('error')}"
                    )
                if header_sent.get("accepted") is False:
                    raise RuntimeError(
                        f"bridge did not accept frame {index}: "
                        f"{header_sent.get('error', 'unknown error')}"
                    )
                if header_sent.get("clamped"):
                    clamped_count += 1
                    if abort_on_clamped:
                        raise RuntimeError(
                            f"bridge clamped frame {index}; reduce --speed or "
                            "--auto-speed before replaying on the real robot"
                        )
                applied = np.asarray(header_sent.get("action", command), dtype=np.float64)
                if applied.shape != (16,) or not np.isfinite(applied).all():
                    raise RuntimeError(f"bridge returned invalid applied action: {applied!r}")
                error = np.abs(applied - commands.actions[index])
                max_applied_error = max(max_applied_error, float(error.max()))
                max_applied_error_by_dim = np.maximum(max_applied_error_by_dim, error)
                record = {
                    "index": index,
                    "wall_s": time.monotonic() - start_mono,
                    "source_time_s": float(commands.times_s[index]),
                    "command": command,
                    "applied": applied.tolist(),
                    "clamped": bool(header_sent.get("clamped")),
                    "bridge": header_sent,
                }
                records.write(json.dumps(record, default=_json_default) + "\n")
                records_written += 1

                if observe_every > 0 and (index + 1) % observe_every == 0:
                    state, obs_header, obs_payload = _observation_state(client)
                    last_observation = _save_observation(
                        output_dir, index + 1, obs_header, obs_payload
                    )
                    last_observation["actual_state"] = state
                    if reference_hdf5_path is not None and reference_camera_keys:
                        last_observation["reference_images"] = _save_reference_images(
                            reference_hdf5_path,
                            reference_camera_keys,
                            int(commands.source_frame_indices[index]),
                            output_dir,
                            index + 1,
                        )
                    records.write(
                        json.dumps(
                            {
                                "index": index + 1,
                                "wall_s": time.monotonic() - start_mono,
                                "observation": last_observation,
                            },
                            default=_json_default,
                        )
                        + "\n"
                    )
                    observations += 1

                next_tick += period
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                else:
                    next_tick = now

            final = commands.actions[-1].astype(float).tolist()
            hold_deadline = time.monotonic() + max(0.0, settle_s)
            while time.monotonic() < hold_deadline:
                client.send_action(final)
                time.sleep(1.0 / 50.0)

        summary = {
            "ok": True,
            "frames_sent": records_written,
            "observations": observations,
            "clamped_count": clamped_count,
            "max_applied_error": max_applied_error,
            "max_applied_error_by_dim": max_applied_error_by_dim,
            "initial_error": initial_error,
            "last_observation": last_observation,
        }
        return summary
    finally:
        if stop_on_exit and motion_started:
            try:
                client.stop()
            except (BridgeError, OSError):
                pass
        client.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "episode",
        type=Path,
        help="HDF5 file, dataset directory, or RoboTwin task output directory",
    )
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument(
        "--action-group", choices=("joint_action", "joint_state"), default="joint_action"
    )
    parser.add_argument(
        "--gripper-format",
        choices=("normalized", "metres"),
        default="normalized",
        help="RoboTwin HDF5 uses normalized [0,1] gripper values",
    )
    parser.add_argument("--gripper-open-width-m", type=float, default=0.09)
    parser.add_argument("--fps", type=float, default=30.0, help="Fallback source FPS")
    parser.add_argument("--command-rate-hz", type=float, default=50.0)
    parser.add_argument("--speed", type=float, default=0.25, help="Replay speed factor")
    parser.add_argument(
        "--auto-speed",
        action="store_true",
        help="Reduce --speed automatically so bridge step limits are respected",
    )
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.02)
    parser.add_argument("--max-gripper-step-m", type=float, default=0.02)
    parser.add_argument(
        "--right-joint2-max-rad",
        type=float,
        default=None,
        help=(
            "Clamp the replayed right joint2 target to this maximum. Use to "
            "keep a real grasp from descending through the table."
        ),
    )
    parser.add_argument(
        "--right-joint2-offset-rad",
        type=float,
        default=0.0,
        help=(
            "Add this offset to the replayed right joint2 target after the "
            "optional maximum clamp. A negative value raises the arm."
        ),
    )
    parser.add_argument(
        "--right-joint4-offset-rad",
        type=float,
        default=0.0,
        help=(
            "Add this offset to the replayed right joint4 target. A negative "
            "value raises the TCP in the low placing configuration."
        ),
    )
    parser.add_argument(
        "--right-joint4-offset-ramp-s",
        type=float,
        default=0.0,
        help=(
            "Ramp the right joint4 offset in over this many seconds. Use a "
            "positive value to keep the initial command at the training home."
        ),
    )
    parser.add_argument("--socket", type=Path, default=Path("/tmp/nero_bimanual_vla_bridge.sock"))
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Connect to the bridge and move the robot; omitted means dry-run",
    )
    parser.add_argument(
        "--require-gripper",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Require bridge gripper motion and feedback for execution",
    )
    parser.add_argument("--max-initial-joint-error-rad", type=float, default=0.15)
    parser.add_argument("--start-hold-s", type=float, default=1.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--observe-every", type=int, default=15)
    parser.add_argument(
        "--abort-on-clamped",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    hdf5_path = resolve_hdf5_path(args.episode, args.episode_index)
    episode = load_episode(
        hdf5_path,
        action_group=args.action_group,
        gripper_format=args.gripper_format,
        gripper_open_width_m=args.gripper_open_width_m,
        fallback_fps=args.fps,
    )
    metrics = validate_trajectory(
        episode,
        max_joint_step_rad=args.max_joint_step_rad,
        max_gripper_step_m=args.max_gripper_step_m,
    )
    safe_speed = suggested_speed(
        episode,
        command_rate_hz=args.command_rate_hz,
        max_joint_step_rad=args.max_joint_step_rad,
        max_gripper_step_m=args.max_gripper_step_m,
    )
    speed = min(args.speed, safe_speed) if args.auto_speed else args.speed
    commands = build_replay_commands(
        episode,
        speed=speed,
        command_rate_hz=args.command_rate_hz,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
    )
    right_joint2_clamped_commands = 0
    right_joint2_source_max_rad = float(np.max(commands.actions[:, 1]))
    if args.right_joint2_max_rad is not None:
        before = commands.actions[:, 1].copy()
        commands.actions[:, 1] = np.minimum(
            commands.actions[:, 1], args.right_joint2_max_rad)
        right_joint2_clamped_commands = int(
            np.count_nonzero(before != commands.actions[:, 1]))
    if args.right_joint2_offset_rad:
        commands.actions[:, 1] = (
            commands.actions[:, 1] + args.right_joint2_offset_rad)
    if args.right_joint4_offset_rad:
        if args.right_joint4_offset_ramp_s > 0:
            blend = np.clip(
                commands.times_s / args.right_joint4_offset_ramp_s, 0.0, 1.0)
        else:
            blend = np.ones_like(commands.times_s)
        commands.actions[:, 3] = (
            commands.actions[:, 3] + args.right_joint4_offset_rad * blend)

    output_dir = args.output_dir or (
        Path("/Data/zhiyuan/outputs/nero_real_replay")
        / datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    )
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "images").mkdir()
    (output_dir / "reference_images").mkdir()

    command_steps = (
        np.abs(np.diff(commands.actions[:, :14], axis=0))
        if commands.actions.shape[0] > 1 else np.empty((0, 14))
    )
    command_width_steps = (
        np.abs(np.diff(commands.actions[:, 14:16], axis=0))
        if commands.actions.shape[0] > 1 else np.empty((0, 2))
    )
    replay_metrics = {
        "max_command_joint_step_rad": float(command_steps.max()) if command_steps.size else 0.0,
        "max_command_gripper_step_m": (
            float(command_width_steps.max()) if command_width_steps.size else 0.0
        ),
        "commands_over_joint_step": int(
            np.count_nonzero(command_steps.max(axis=1) > args.max_joint_step_rad)
        ) if command_steps.size else 0,
        "commands_over_gripper_step": int(
            np.count_nonzero(command_width_steps.max(axis=1) > args.max_gripper_step_m)
        ) if command_width_steps.size else 0,
    }

    summary: dict[str, Any] = {
        "mode": "execute" if args.execute else "dry_run",
        "hdf5": str(hdf5_path),
        "action_group": args.action_group,
        "gripper_format": args.gripper_format,
        "gripper_open_width_m": args.gripper_open_width_m,
        "source_storage_order": HDF5_NAMES,
        "bridge_order": BUS_NAMES,
        "metrics": metrics,
        "replay_metrics": replay_metrics,
        "suggested_speed": safe_speed,
        "selected_speed": speed,
        "command_rate_hz": args.command_rate_hz,
        "commands": int(commands.actions.shape[0]),
        "right_joint2_max_rad": args.right_joint2_max_rad,
        "right_joint2_source_max_rad": right_joint2_source_max_rad,
        "right_joint2_clamped_commands": right_joint2_clamped_commands,
        "right_joint2_offset_rad": args.right_joint2_offset_rad,
        "right_joint4_offset_rad": args.right_joint4_offset_rad,
        "right_joint4_offset_ramp_s": args.right_joint4_offset_ramp_s,
        "command_duration_s": float(
            commands.times_s[-1] if len(commands.times_s) > 1 else 0.0
        ),
    }
    sim_video = hdf5_path.parents[1] / "video" / f"episode{args.episode_index}.mp4"
    if sim_video.is_file():
        summary["sim_video"] = str(sim_video)
    np.save(output_dir / "source_actions_left_right.npy", episode.raw)
    np.save(output_dir / "source_actions_bus.npy", episode.bus)
    np.save(output_dir / "replay_commands.npy", commands.actions)

    print(
        f"episode: {hdf5_path}\n"
        f"frames={metrics['frames']} fps={metrics['fps']:.3f} "
        f"source_duration={metrics['duration_s']:.3f}s\n"
        f"command_rate={args.command_rate_hz:.1f}Hz speed={speed:.3f} "
        f"commands={commands.actions.shape[0]} duration={summary['command_duration_s']:.3f}s\n"
        f"max_frame_delta={metrics['max_frame_joint_delta_rad']:.5f}rad "
        f"limit_violations={metrics['limit_violation_count']} "
        f"frames_over_step={metrics['frames_over_joint_step_rad']}\n"
        + (
            f"right_joint2_clamp_max={args.right_joint2_max_rad:.4f}rad "
            f"clamped_commands={right_joint2_clamped_commands}\n"
            if args.right_joint2_max_rad is not None else ""
        )
        + (
            f"right_joint2_offset={args.right_joint2_offset_rad:+.4f}rad\n"
            if args.right_joint2_offset_rad else ""
        )
        + (
            f"right_joint4_offset={args.right_joint4_offset_rad:+.4f}rad\n"
            if args.right_joint4_offset_rad else ""
        )
        + (
            f"right_joint4_offset_ramp={args.right_joint4_offset_ramp_s:.2f}s\n"
            if args.right_joint4_offset_rad and
            args.right_joint4_offset_ramp_s > 0 else ""
        )
        + f"suggested_speed={safe_speed:.3f} output={output_dir}"
    )

    if args.execute:
        if metrics["limit_violation_count"]:
            raise RuntimeError(
                "refusing real replay: expert joints exceed the bridge joint limits; "
                f"frames={metrics['limit_violation_frames'][:20]}"
            )
        summary["replay"] = execute_replay(
            commands,
            socket_path=args.socket.expanduser(),
            output_dir=output_dir,
            require_gripper=args.require_gripper,
            max_initial_joint_error_rad=args.max_initial_joint_error_rad,
            start_hold_s=args.start_hold_s,
            settle_s=args.settle_s,
            observe_every=max(0, args.observe_every),
            abort_on_clamped=args.abort_on_clamped,
            stop_on_exit=args.stop_on_exit,
            reference_hdf5_path=hdf5_path,
            reference_camera_keys=("base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"),
        )
    _write_json(output_dir / "summary.json", summary)
    print(json.dumps(summary, indent=2, default=_json_default))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
