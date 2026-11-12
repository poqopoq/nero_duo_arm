#!/usr/bin/env python3
"""Convert a Xense LeRobot v3 dual-arm UMI dataset to NERO's 20-D contract.

The source dataset stores two 10-D TCP blocks followed by a 9-D head pose::

    left_tcp.xyz, left_tcp.r1..r6, left_gripper,
    right_tcp.xyz, right_tcp.r1..r6, right_gripper,
    head.xyz, head.r1..r6

NERO's ``lerobot_robot_nero`` bimanual plugin consumes only the first 20
dimensions.  This converter keeps those dimensions, renames the feature names
to the plugin's ``*.pos`` names, drops tactile/head data, and reuses the
already encoded videos instead of decoding and re-encoding 88k frames.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


SOURCE_NAMES = (
    "left_tcp.x", "left_tcp.y", "left_tcp.z",
    "left_tcp.r1", "left_tcp.r2", "left_tcp.r3",
    "left_tcp.r4", "left_tcp.r5", "left_tcp.r6",
    "left_gripper.pos",
    "right_tcp.x", "right_tcp.y", "right_tcp.z",
    "right_tcp.r1", "right_tcp.r2", "right_tcp.r3",
    "right_tcp.r4", "right_tcp.r5", "right_tcp.r6",
    "right_gripper.pos",
)

NERO_NAMES = (
    "left_tcp.x.pos", "left_tcp.y.pos", "left_tcp.z.pos",
    "left_tcp.r1.pos", "left_tcp.r2.pos", "left_tcp.r3.pos",
    "left_tcp.r4.pos", "left_tcp.r5.pos", "left_tcp.r6.pos",
    "left_gripper.pos",
    "right_tcp.x.pos", "right_tcp.y.pos", "right_tcp.z.pos",
    "right_tcp.r1.pos", "right_tcp.r2.pos", "right_tcp.r3.pos",
    "right_tcp.r4.pos", "right_tcp.r5.pos", "right_tcp.r6.pos",
    "right_gripper.pos",
)

DEFAULT_SOURCE_REPO = "Xense/xtac-umi-g1-clean-plates-with-a-sponge"
DEFAULT_CAMERA_MAP = {
    "observation.images.head_left": "observation.images.side",
    "observation.images.left_wrist": "observation.images.left_wrist",
    "observation.images.right_wrist": "observation.images.right_wrist",
}
NUMERIC_META = ("timestamp", "frame_index", "episode_index", "index", "task_index")


class XenseError(RuntimeError):
    """A user-facing Xense source or conversion error."""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a Xense LeRobot v3 two-arm UMI dataset into the 20-D "
            "NERO rot6d contract. The source videos are copied unchanged."),
    )
    parser.add_argument("source", help="Xense LeRobot v3 dataset root")
    parser.add_argument(
        "-o", "--output",
        help="Output LeRobot dataset root (required unless --dry-run)",
    )
    parser.add_argument(
        "--repo-id",
        default="local/xense_plates_nero20",
        help="Output LeRobot repo id",
    )
    parser.add_argument(
        "--robot-type",
        default="xtac_umi_g1",
        help="robot_type written to the output metadata",
    )
    parser.add_argument(
        "--scene-camera",
        default="observation.images.head_left",
        help="Source camera used as NERO's side/scene camera",
    )
    parser.add_argument(
        "--scene-key",
        default="observation.images.side",
        help="Output scene-camera feature key",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing output dataset",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate the source and print the output contract only",
    )
    parser.add_argument(
        "--no-verify",
        action="store_true",
        help="Skip loading the finished dataset with LeRobotDataset",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return _run(args)
    except XenseError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    except ModuleNotFoundError as exc:
        print(
            f"[ERROR] missing Python dependency: {exc.name}. Use the LeRobot "
            "0.6 environment, not the ROS Python environment.",
            file=sys.stderr,
        )
        return 2


def _run(args: argparse.Namespace) -> int:
    source = Path(args.source).expanduser().resolve()
    info = load_source_info(source)
    camera_map = camera_map_for(args)
    validate_source_info(info, camera_map)
    data_files = sorted((source / "data").glob("*/*.parquet"))
    if not data_files:
        raise XenseError(f"no data parquet files under {source / 'data'}")

    episode_data: dict[int, dict[str, np.ndarray]] = {}
    total_frames = 0
    for parquet_path in data_files:
        table = pq.read_table(parquet_path)
        state, action = _state_and_action(table)
        episodes = np.asarray(table["episode_index"], dtype=np.int64)
        _validate_arrays(state, action, episodes)
        for episode_index in np.unique(episodes):
            mask = episodes == episode_index
            episode_data[int(episode_index)] = {
                "action": action[mask, :20],
                "observation.state": state[mask, :20],
                "timestamp": _scalar_column(table, "timestamp")[mask],
                "frame_index": _scalar_column(table, "frame_index")[mask],
                "episode_index": episodes[mask],
                "index": _scalar_column(table, "index")[mask],
                "task_index": _scalar_column(table, "task_index")[mask],
            }
        total_frames += len(table)

    if total_frames != int(info["total_frames"]):
        raise XenseError(
            f"data has {total_frames} frames but info.json declares "
            f"{info['total_frames']}")

    output_features = build_output_features(info, camera_map)
    print("[XENSE]")
    print(f"source: {source}")
    print(
        f"episodes={info['total_episodes']} frames={info['total_frames']} "
        f"fps={info['fps']} robot_type={info['robot_type']}")
    print("state/action: 29-D -> 20-D (head pose and tracker confidence dropped)")
    print("cameras:")
    for source_key, target_key in camera_map.items():
        print(f"  {source_key} -> {target_key}")
    print("[OUTPUT FEATURES]")
    print(json.dumps(output_features, indent=2))

    if args.dry_run:
        print("[DRY-RUN] source and data contract are valid; nothing written.")
        return 0

    if not args.output:
        raise XenseError("--output is required unless --dry-run is used")
    output = Path(args.output).expanduser().resolve()
    if output == source or source in output.parents or output in source.parents:
        raise XenseError("source and output datasets must not contain each other")
    if output.exists() and not args.overwrite:
        raise XenseError(f"output already exists: {output} (pass --overwrite)")

    staging = output.parent / f".{output.name}.tmp"
    if staging.exists():
        shutil.rmtree(staging)
    staging.parent.mkdir(parents=True, exist_ok=True)

    try:
        _write_dataset(
            source=source,
            output=staging,
            info=info,
            camera_map=camera_map,
            output_features=output_features,
            episode_data=episode_data,
            repo_id=args.repo_id,
            robot_type=args.robot_type,
        )
        if output.exists():
            shutil.rmtree(output)
        staging.rename(output)
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise

    if not args.no_verify:
        verify_output(output, args.repo_id, camera_map.values())

    print(f"[DONE] wrote {output}")
    return 0


def camera_map_for(args: argparse.Namespace) -> dict[str, str]:
    return {
        args.scene_camera: args.scene_key,
        "observation.images.left_wrist": "observation.images.left_wrist",
        "observation.images.right_wrist": "observation.images.right_wrist",
    }


def load_source_info(source: Path) -> dict[str, Any]:
    path = source / "meta" / "info.json"
    if not path.is_file():
        raise XenseError(f"not a LeRobot dataset root (missing {path})")
    try:
        info = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise XenseError(f"cannot read {path}: {exc}") from exc
    if not isinstance(info, dict):
        raise XenseError(f"{path} must contain a JSON object")
    return info


def validate_source_info(
    info: Mapping[str, Any], camera_map: Mapping[str, str]
) -> None:
    version = str(info.get("codebase_version", ""))
    if not version.startswith("v3."):
        raise XenseError(
            f"expected LeRobot v3 metadata, got codebase_version={version!r}")
    features = info.get("features")
    if not isinstance(features, dict):
        raise XenseError("info.json has no features object")
    for key in ("action", "observation.state"):
        feature = features.get(key)
        if not isinstance(feature, dict):
            raise XenseError(f"missing feature {key!r}")
        if list(feature.get("shape") or []) != [29]:
            raise XenseError(
                f"{key} shape is {feature.get('shape')}; expected [29]")
        names = list(feature.get("names") or [])
        if names and tuple(names[:20]) != SOURCE_NAMES:
            raise XenseError(
                f"{key} does not use the expected left/right TCP ordering")
    if int(info.get("fps", 0)) != 30:
        raise XenseError(f"expected 30 FPS, got {info.get('fps')!r}")
    for source_key in camera_map:
        feature = features.get(source_key)
        if not isinstance(feature, dict):
            raise XenseError(f"missing camera feature {source_key!r}")
        if feature.get("dtype") != "video":
            raise XenseError(
                f"{source_key} has dtype {feature.get('dtype')}; expected video")
        if list(feature.get("shape") or []) != [480, 640, 3]:
            raise XenseError(
                f"{source_key} shape is {feature.get('shape')}; expected "
                "[480, 640, 3]")


def build_output_features(
    info: Mapping[str, Any], camera_map: Mapping[str, str]
) -> dict[str, Any]:
    source_features = info["features"]
    vector = {
        "dtype": "float32",
        "shape": [20],
        "names": list(NERO_NAMES),
    }
    features: dict[str, Any] = {
        "action": dict(vector),
        "observation.state": dict(vector),
    }
    for source_key, target_key in camera_map.items():
        features[target_key] = dict(source_features[source_key])
    for key in NUMERIC_META:
        feature = source_features.get(key)
        if isinstance(feature, dict):
            features[key] = dict(feature)
    return features


def _state_and_action(table: pa.Table) -> tuple[np.ndarray, np.ndarray]:
    try:
        state = np.asarray(
            table["observation.state"].to_pylist(), dtype=np.float32)
        action = np.asarray(table["action"].to_pylist(), dtype=np.float32)
    except (KeyError, ValueError) as exc:
        raise XenseError(f"cannot read state/action columns: {exc}") from exc
    return state, action


def _scalar_column(table: pa.Table, key: str) -> np.ndarray:
    try:
        return np.asarray(table[key], dtype=np.float64)
    except KeyError as exc:
        raise XenseError(f"missing required column {key!r}") from exc


def _validate_arrays(
    state: np.ndarray, action: np.ndarray, episodes: np.ndarray
) -> None:
    if state.ndim != 2 or state.shape[1] < 20:
        raise XenseError(f"state shape is {state.shape}; expected (T, >=20)")
    if action.shape != state.shape:
        raise XenseError(
            f"action shape {action.shape} does not match state shape {state.shape}")
    if not np.isfinite(state).all() or not np.isfinite(action).all():
        raise XenseError("state/action contains non-finite values")
    state = state[:, :20]
    action = action[:, :20]
    if np.any(state[:, 9] < -1e-6) or np.any(state[:, 9] > 1 + 1e-6):
        raise XenseError("left gripper is not normalized to [0, 1]")
    if np.any(state[:, 19] < -1e-6) or np.any(state[:, 19] > 1 + 1e-6):
        raise XenseError("right gripper is not normalized to [0, 1]")
    for offset in (0, 10):
        _validate_rot6d(state[:, offset + 3:offset + 9])
    for episode_index in np.unique(episodes):
        mask = episodes == episode_index
        ep_action = action[mask]
        ep_state = state[mask]
        if len(ep_action) > 1 and not np.allclose(
            ep_action[:-1], ep_state[1:], atol=1e-6, rtol=0
        ):
            raise XenseError(
                f"episode {episode_index}: action[t] != state[t+1]")


def _validate_rot6d(rot6d: np.ndarray) -> None:
    column_a = rot6d[:, :3]
    column_b = rot6d[:, 3:]
    norm_a = np.linalg.norm(column_a, axis=1)
    norm_b = np.linalg.norm(column_b, axis=1)
    dot = np.abs(np.sum(column_a * column_b, axis=1))
    if not (
        np.allclose(norm_a, 1.0, atol=1e-4)
        and np.allclose(norm_b, 1.0, atol=1e-4)
        and np.all(dot < 1e-4)
    ):
        raise XenseError(
            "rot6d is not the column-major first-two-column representation")


def _write_dataset(
    *,
    source: Path,
    output: Path,
    info: Mapping[str, Any],
    camera_map: Mapping[str, str],
    output_features: Mapping[str, Any],
    episode_data: Mapping[int, Mapping[str, np.ndarray]],
    repo_id: str,
    robot_type: str,
) -> None:
    output.mkdir(parents=True)
    _write_data(source, output)
    _write_info(info, output, output_features, robot_type)
    _write_tasks(source, output)
    _write_episodes(source, output, camera_map)
    _write_stats(source, output, info, camera_map, episode_data)
    _copy_videos(source, output, camera_map)
    _write_manifest(source, output, camera_map, repo_id)


def _write_data(source: Path, output: Path) -> None:
    for source_path in sorted((source / "data").glob("*/*.parquet")):
        table = pq.read_table(source_path)
        state, action = _state_and_action(table)
        episodes = np.asarray(table["episode_index"], dtype=np.int64)
        columns = {
            "observation.state": _list_array(state[:, :20]),
            "action": _list_array(action[:, :20]),
        }
        for key in NUMERIC_META:
            columns[key] = table[key]
        target = output / source_path.relative_to(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        out_table = pa.table(columns)
        _write_one_row_group_per_episode(out_table, target, episodes)


def _list_array(values: np.ndarray) -> pa.Array:
    return pa.array(values.tolist(), type=pa.list_(pa.float32()))


def _write_one_row_group_per_episode(
    table: pa.Table, path: Path, episodes: np.ndarray
) -> None:
    starts = np.concatenate(([0], np.nonzero(np.diff(episodes))[0] + 1))
    stops = np.append(starts[1:], len(episodes))
    writer = pq.ParquetWriter(
        str(path), table.schema, compression="snappy", use_dictionary=True)
    try:
        for start, stop in zip(starts, stops, strict=True):
            writer.write_table(
                table.slice(int(start), int(stop - start)),
                row_group_size=int(stop - start) or None,
            )
    finally:
        writer.close()


def _write_info(
    source_info: Mapping[str, Any],
    output: Path,
    features: Mapping[str, Any],
    robot_type: str,
) -> None:
    info = dict(source_info)
    info["features"] = dict(features)
    info["robot_type"] = robot_type
    (output / "meta").mkdir(parents=True, exist_ok=True)
    (output / "meta" / "info.json").write_text(
        json.dumps(info, indent=2) + "\n", encoding="utf-8")


def _write_tasks(source: Path, output: Path) -> None:
    tasks = source / "meta" / "tasks.parquet"
    if not tasks.is_file():
        raise XenseError(f"missing {tasks}")
    shutil.copy2(tasks, output / "meta" / "tasks.parquet")


def _write_episodes(
    source: Path, output: Path, camera_map: Mapping[str, str]
) -> None:
    source_episodes = sorted((source / "meta" / "episodes").glob("*/*.parquet"))
    if not source_episodes:
        raise XenseError(f"no episode metadata under {source / 'meta' / 'episodes'}")
    target_dir = output / "meta" / "episodes"
    for source_path in source_episodes:
        table = pq.read_table(source_path)
        rename = _episode_column_map(table.column_names, camera_map)
        target = target_dir / source_path.relative_to(source / "meta" / "episodes")
        target.parent.mkdir(parents=True, exist_ok=True)
        if rename:
            table = table.select(list(rename))
            table = table.rename_columns(list(rename.values()))
            pq.write_table(table, target)


def _episode_column_map(
    columns: Sequence[str], camera_map: Mapping[str, str]
) -> dict[str, str]:
    keep = [
        "episode_index", "tasks", "length", "data/chunk_index",
        "data/file_index", "dataset_from_index", "dataset_to_index",
        "meta/episodes/chunk_index", "meta/episodes/file_index",
    ]
    result: dict[str, str] = {key: key for key in keep if key in columns}
    for source_key, target_key in camera_map.items():
        for suffix in ("chunk_index", "file_index", "from_timestamp", "to_timestamp"):
            source_column = f"videos/{source_key}/{suffix}"
            if source_column not in columns:
                raise XenseError(f"missing episode metadata column {source_column}")
            result[source_column] = f"videos/{target_key}/{suffix}"
    return result


def _write_stats(
    source: Path,
    output: Path,
    info: Mapping[str, Any],
    camera_map: Mapping[str, str],
    episode_data: Mapping[int, Mapping[str, np.ndarray]],
) -> None:
    from lerobot.datasets.compute_stats import (
        aggregate_stats,
        compute_episode_stats,
    )
    from lerobot.datasets.io_utils import write_stats

    numeric_features = {
        key: info["features"][key]
        for key in ("action", "observation.state", *NUMERIC_META)
        if key in info["features"]
    }
    per_episode = [
        compute_episode_stats(dict(data), numeric_features)
        for _, data in sorted(episode_data.items())
    ]
    stats = aggregate_stats(per_episode)

    source_stats_path = source / "meta" / "stats.json"
    if not source_stats_path.is_file():
        raise XenseError(f"missing {source_stats_path}")
    source_stats = json.loads(source_stats_path.read_text(encoding="utf-8"))
    for source_key, target_key in camera_map.items():
        if source_key not in source_stats:
            raise XenseError(f"missing image stats for {source_key}")
        stats[target_key] = source_stats[source_key]
    write_stats(stats, output)


def _copy_videos(
    source: Path, output: Path, camera_map: Mapping[str, str]
) -> None:
    for source_key, target_key in camera_map.items():
        source_dir = source / "videos" / source_key
        if not source_dir.is_dir():
            raise XenseError(f"missing video directory {source_dir}")
        target_dir = output / "videos" / target_key
        target_dir.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(source_dir, target_dir)


def _write_manifest(
    source: Path,
    output: Path,
    camera_map: Mapping[str, str],
    repo_id: str,
) -> None:
    manifest = {
        "converter": "xense_to_nero_lerobot",
        "format_version": 1,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "source": str(source),
        "repo_id": repo_id,
        "source_dim": 29,
        "output_dim": 20,
        "feature_names": list(NERO_NAMES),
        "dropped_dimensions": {
            "head_pose": "indices 20:29",
            "tracker_confidence": "observation.tracker_confidence",
        },
        "cameras": dict(camera_map),
        "action_convention": (
            "absolute next-frame TCP target; gripper 0=closed, 1=open"),
        "pose_transform": None,
        "note": (
            "Source TCP poses are unchanged. Map them into left_base_link and "
            "right_base_link before deployment on NERO."),
    }
    (output / "meta" / "xense_nero_conversion.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


def verify_output(
    root: Path, repo_id: str, camera_keys: Sequence[str]
) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(repo_id=repo_id, root=str(root))
    expected = list(NERO_NAMES)
    if list(dataset.meta.features["action"].get("names") or []) != expected:
        raise XenseError("output action feature names do not match NERO")
    if list(dataset.meta.features["observation.state"].get("names") or []) != expected:
        raise XenseError("output state feature names do not match NERO")
    for key in camera_keys:
        if key not in dataset.meta.features:
            raise XenseError(f"output is missing camera {key}")

    for key in ("action", "observation.state"):
        if key not in dataset.meta.stats:
            raise XenseError(f"output is missing statistics for {key}")
        for stat_name in (
            "min", "max", "mean", "std", "q01", "q10", "q50", "q90", "q99"
        ):
            shape = np.asarray(dataset.meta.stats[key][stat_name]).shape
            if shape != (20,):
                raise XenseError(
                    f"{key} {stat_name} stats shape is {shape}; expected (20,)")

    for index in sorted({0, len(dataset) // 2, len(dataset) - 1}):
        sample = dataset[index]
        if tuple(sample["action"].shape) != (20,):
            raise XenseError(
                f"frame {index} action shape is {tuple(sample['action'].shape)}")
        if tuple(sample["observation.state"].shape) != (20,):
            raise XenseError(
                f"frame {index} state shape is "
                f"{tuple(sample['observation.state'].shape)}")
        for key in camera_keys:
            if key not in sample:
                raise XenseError(f"frame {index} is missing {key}")
            if tuple(sample[key].shape[-2:]) != (480, 640):
                raise XenseError(
                    f"frame {index} camera {key} shape is "
                    f"{tuple(sample[key].shape)}")
    print(
        f"[VERIFY] {dataset.meta.total_episodes} episodes, "
        f"{dataset.meta.total_frames} frames, 20-D state/action, 3 cameras")


if __name__ == "__main__":
    raise SystemExit(main())
