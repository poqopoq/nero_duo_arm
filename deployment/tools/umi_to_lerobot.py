#!/usr/bin/env python3
"""Convert a single-arm UMI zarr/HDF5 replay buffer into LeRobot v3.

UMI is a handheld data-collection system.  Its replay buffer contains camera
images, end-effector position, end-effector axis-angle rotation, and gripper
width -- not NERO joint feedback.  The default output follows the published
UMI/SROI EE contract: a 7-D absolute ``action`` (xyz + axis-angle + normalized
gripper), no ``observation.state``, and ``action[t]`` equal to the pose at
``t + 1``.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
import time
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from tools.umi_dataset import (  # noqa: E402
        ACTION_NAMES,
        NERO_ACTION_NAMES,
        UmiError,
        build_lerobot_features,
        load_pose_transform,
        load_task_map,
        observed_gripper_range,
        open_umi,
        parse_resize,
        select_episode_indices,
        summarize,
    )
else:
    from .umi_dataset import (
        ACTION_NAMES,
        NERO_ACTION_NAMES,
        UmiError,
        build_lerobot_features,
        load_pose_transform,
        load_task_map,
        observed_gripper_range,
        open_umi,
        parse_resize,
        select_episode_indices,
        summarize,
    )


CONTRACTS = {
    "umi-sroi": {
        "action_names": ACTION_NAMES,
        "robot_type": "so100",
        "image_key": "camera",
        "include_state": False,
    },
    "nero-ros": {
        "action_names": NERO_ACTION_NAMES,
        "robot_type": "nero_ros",
        "image_key": "camera1",
        "include_state": True,
    },
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a UMI zarr or HDF5 replay buffer (camera + eef pose + "
            "gripper width) to a LeRobot v3 dataset."),
    )
    parser.add_argument(
        "source",
        help="UMI zarr directory/.zarr.zip, or canonical/flat .h5/.hdf5 file",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output LeRobot dataset root (required unless --dry-run)",
    )
    parser.add_argument(
        "--repo-id",
        default="local/umi_dataset",
        help="LeRobot repo id, default: local/umi_dataset",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=None,
        help="Dataset frame rate. Required when the UMI metadata has no fps.",
    )
    parser.add_argument(
        "--task",
        default=None,
        help="Language instruction for every episode. Required unless the "
             "UMI metadata or --task-file provides one.",
    )
    parser.add_argument(
        "--task-file",
        default=None,
        help="JSON object mapping episode index to language instruction.",
    )
    parser.add_argument(
        "--episodes",
        default=None,
        help="Episode selection, for example: 0,2-4. Default: all episodes.",
    )
    parser.add_argument(
        "--max-episodes",
        type=int,
        default=None,
        help="Convert at most this many selected episodes (smoke tests).",
    )
    parser.add_argument(
        "--action-offset",
        type=int,
        default=1,
        help="Action at t is state at t+offset. Default: 1 (next-step target). "
             "Use 0 for UMI's recorded state-as-action convention.",
    )
    parser.add_argument(
        "--robot-index",
        type=int,
        default=None,
        help="UMI robotN to convert. Required only when the source contains "
             "multiple robots.",
    )
    state_group = parser.add_mutually_exclusive_group()
    state_group.add_argument(
        "--include-state",
        dest="include_state",
        action="store_true",
        default=None,
        help="Also write observation.state. The published UMI/SROI contract "
             "omits it; leave this off unless a downstream policy requires it.",
    )
    state_group.add_argument(
        "--no-state",
        dest="include_state",
        action="store_false",
        help="Omit observation.state even when the selected contract normally "
             "writes it.",
    )
    parser.add_argument(
        "--pose-transform",
        default=None,
        help="JSON 4x4 target-from-source transform applied to every eef pose. "
             "Use this to map UMI SLAM/world coordinates into the robot-base "
             "or table frame.",
    )
    parser.add_argument(
        "--gripper-mode",
        choices=("explicit", "auto"),
        default="explicit",
        help="Gripper width normalization. 'explicit' requires --gripper-min "
             "and --gripper-max. 'auto' uses the observed min/max and records "
             "that choice in metadata. Default: explicit.",
    )
    parser.add_argument(
        "--gripper-min",
        type=float,
        default=None,
        help="Raw UMI gripper width corresponding to closed, in metres.",
    )
    parser.add_argument(
        "--gripper-max",
        type=float,
        default=None,
        help="Raw UMI gripper width corresponding to fully open, in metres.",
    )
    parser.add_argument(
        "--robot-type",
        default=None,
        help="LeRobot robot_type metadata. Default depends on --contract.",
    )
    parser.add_argument(
        "--contract",
        choices=tuple(CONTRACTS),
        default="umi-sroi",
        help="Output feature contract. 'umi-sroi' keeps the published 7-D "
             "action and camera feature; 'nero-ros' uses ee.rx/ry/rz + "
             "ee.gripper, camera1, observation.state, and robot_type nero_ros.",
    )
    parser.add_argument(
        "--image-key",
        default=None,
        help="Feature suffix for a single selected camera. Defaults to "
             "'camera' for umi-sroi and 'camera1' for nero-ros.",
    )
    parser.add_argument(
        "--no-videos",
        action="store_true",
        help="Store per-frame images instead of MP4 video features. Larger "
             "dataset, but avoids video encoding.",
    )
    parser.add_argument(
        "--vcodec",
        choices=("h264", "hevc", "libsvtav1", "auto"),
        default="libsvtav1",
        help="Video codec when videos are enabled. Default: libsvtav1 "
             "(matches the published UMI/SROI dataset).",
    )
    parser.add_argument(
        "--resize",
        default=None,
        help="Resize all cameras to WIDTHxHEIGHT, for example 224x224.",
    )
    parser.add_argument(
        "--camera",
        action="append",
        default=None,
        help="Source camera dataset key to include (repeatable). Default: each "
             "cameraN_rgb, excluding _mirror_crop.",
    )
    parser.add_argument(
        "--include-mirror",
        action="store_true",
        help="Also include cameraN_rgb_mirror_crop arrays.",
    )
    parser.add_argument(
        "--image-writer-threads",
        type=int,
        default=2,
        help="LeRobot async image-writer threads; 0 writes synchronously. "
             "Default: 2.",
    )
    parser.add_argument(
        "--parallel-encoding",
        action="store_true",
        help="Encode episode videos in parallel processes. Default: sequential.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Move an existing output directory to a timestamped .bak path "
             "before conversion.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Inspect and validate the UMI source without importing LeRobot "
             "or writing a dataset.",
    )
    parser.add_argument(
        "--no-verify",
        action="store_true",
        help="Skip loading the finished dataset back with LeRobotDataset.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return _run(args)
    except UmiError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    except ModuleNotFoundError as exc:
        print(
            f"[ERROR] missing Python dependency: {exc.name}. "
            "Run deployment/scripts/install_umi_lerobot_env.sh.",
            file=sys.stderr,
        )
        return 2


def _run(args: argparse.Namespace) -> int:
    resize = parse_resize(args.resize)
    task_map = load_task_map(args.task_file)
    pose_transform = load_pose_transform(args.pose_transform)
    action_names, robot_type, include_state, image_key = _resolve_contract(args)

    with open_umi(
        args.source,
        fps=args.fps,
        task=args.task,
        cameras=args.camera,
        include_mirror=args.include_mirror,
        robot_index=args.robot_index,
    ) as umi:
        info = umi.info
        selected = select_episode_indices(
            info.episode_count, args.episodes, args.max_episodes)
        observed_gripper = observed_gripper_range(umi, selected)

        print("[UMI]")
        camera_features = _camera_features(info, image_key)
        print(summarize(
            info,
            resize,
            action_names=action_names,
            camera_features=camera_features,
        ))
        print(
            f"output contract: {args.contract} "
            f"(state={include_state}, image_key={image_key}, "
            f"robot_type={robot_type})")
        print(f"selected episodes: {selected}")
        print(f"action offset: {args.action_offset}")
        print(
            "observed gripper width: "
            f"[{observed_gripper[0]:.6f}, {observed_gripper[1]:.6f}] m")
        if pose_transform is not None:
            print(f"pose transform: {args.pose_transform}")

        features = build_lerobot_features(
            info,
            use_videos=not args.no_videos,
            resize=resize,
            include_state=include_state,
            state_names=action_names,
            camera_features=camera_features,
        )
        print("[LEROBOT] features:")
        print(json.dumps(features, indent=2))

        if args.dry_run:
            if args.gripper_mode == "explicit" and (
                args.gripper_min is None or args.gripper_max is None
            ):
                print(
                    "[HINT] pass --gripper-min 0.0 --gripper-max "
                    f"{observed_gripper[1]:.6f} if the jaw was fully open in "
                    "these demonstrations; otherwise use the physical "
                    "calibration bounds.")
            print("[DRY-RUN] source is valid; no dataset written.")
            return 0

        if not args.output:
            raise UmiError("--output is required unless --dry-run is used")
        if info.fps is None:
            raise UmiError(
                "UMI metadata has no fps; pass --fps (GoPro/UMI output is "
                "usually 30 Hz)")
        if args.action_offset < 0:
            raise UmiError("--action-offset must be >= 0")

        gripper_range = _resolve_gripper_range(args, observed_gripper)

        tasks = _episode_tasks(info.task, task_map, selected)
        output = Path(args.output).expanduser().resolve()
        _prepare_output(output, args.overwrite)

        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise UmiError(
                "LeRobot is not installed in this Python environment. Run "
                "deployment/scripts/install_umi_lerobot_env.sh and use its "
                "python interpreter."
            ) from exc

        create_kwargs = {
            "repo_id": args.repo_id,
            "fps": int(info.fps),
            "features": features,
            "root": str(output),
            "robot_type": robot_type,
            "use_videos": not args.no_videos,
        }
        if not args.no_videos:
            create_kwargs["vcodec"] = args.vcodec
        if args.image_writer_threads > 0:
            create_kwargs["image_writer_threads"] = args.image_writer_threads

        dataset = LeRobotDataset.create(**create_kwargs)
        try:
            for episode_index in selected:
                _convert_episode(
                    dataset,
                    umi,
                    episode_index,
                    tasks[episode_index],
                    resize,
                    args.action_offset,
                    gripper_range,
                    pose_transform,
                    include_state=include_state,
                    camera_features=camera_features,
                    parallel_encoding=args.parallel_encoding,
                )
        except Exception:
            try:
                dataset.finalize()
            except Exception:
                pass
            raise
        finally:
            pass

        dataset.finalize()
        _write_metadata(
            output,
            args,
            info,
            selected,
            tasks,
            resize,
            gripper_range,
            observed_gripper,
            pose_transform,
            action_names=action_names,
            robot_type=robot_type,
            include_state=include_state,
            camera_features=camera_features,
        )
        print(f"[OK] wrote LeRobot dataset: {output}")

        if not args.no_verify:
            _verify(
                output,
                args.repo_id,
                action_names=action_names,
                camera_features=camera_features,
                include_state=include_state,
            )
        return 0


def _resolve_contract(args: argparse.Namespace):
    defaults = CONTRACTS[args.contract]
    include_state = (
        defaults["include_state"]
        if args.include_state is None
        else bool(args.include_state)
    )
    image_key = args.image_key or defaults["image_key"]
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", image_key):
        raise UmiError(
            "--image-key must start with a letter and contain only letters, "
            "digits, and underscores")
    robot_type = args.robot_type or defaults["robot_type"]
    return (
        tuple(defaults["action_names"]),
        str(robot_type),
        include_state,
        image_key,
    )


def _camera_features(info, image_key: str) -> dict[str, str]:
    if len(info.cameras) == 1:
        return {
            info.cameras[0].key: f"observation.images.{image_key}",
        }
    if image_key != CONTRACTS["umi-sroi"]["image_key"]:
        raise UmiError(
            "--image-key is only supported with one selected camera; "
            f"found {len(info.cameras)}. Select one with --camera.")
    return {camera.key: camera.feature for camera in info.cameras}


def _resolve_gripper_range(
    args: argparse.Namespace,
    observed: tuple[float, float],
) -> tuple[float, float]:
    if args.gripper_mode == "auto":
        if args.gripper_min is not None or args.gripper_max is not None:
            raise UmiError(
                "--gripper-min/--gripper-max cannot be combined with "
                "--gripper-mode auto")
        chosen = observed
        print(
            "[WARN] using observed gripper min/max; if the jaw did not reach "
            "both endpoints, pass --gripper-mode explicit with calibrated "
            "bounds")
    else:
        if args.gripper_min is None or args.gripper_max is None:
            raise UmiError(
                "gripper normalization is required for the 7-D EE contract. "
                "Pass --gripper-min and --gripper-max in metres, or opt into "
                "--gripper-mode auto. Run --dry-run to inspect the observed "
                "range.")
        chosen = (float(args.gripper_min), float(args.gripper_max))
    minimum, maximum = chosen
    if not np.isfinite(minimum) or not np.isfinite(maximum):
        raise UmiError("gripper bounds must be finite")
    if maximum <= minimum:
        raise UmiError(
            f"gripper max must be greater than min (min={minimum}, "
            f"max={maximum})")
    return minimum, maximum


def _episode_tasks(
    default_task: str | None,
    task_map: dict[int, str],
    selected: list[int],
) -> dict[int, str]:
    tasks: dict[int, str] = {}
    missing = []
    for episode_index in selected:
        task = task_map.get(episode_index, default_task)
        if task is None or not task.strip():
            missing.append(episode_index)
        else:
            tasks[episode_index] = task.strip()
    if missing:
        raise UmiError(
            "no language instruction for episode(s) "
            f"{missing}; pass --task or add them to --task-file")
    return tasks


def _prepare_output(output: Path, overwrite: bool) -> None:
    if not output.exists():
        return
    if not overwrite:
        raise UmiError(
            f"output already exists: {output}. Pass --overwrite to move it "
            "to a timestamped backup.")
    backup = output.with_name(
        f"{output.name}.bak-{time.strftime('%Y%m%d-%H%M%S')}")
    counter = 1
    while backup.exists():
        backup = output.with_name(
            f"{output.name}.bak-{time.strftime('%Y%m%d-%H%M%S')}-{counter}")
        counter += 1
    shutil.move(str(output), str(backup))
    print(f"[WARN] moved existing output to {backup}")


def _convert_episode(
    dataset,
    umi,
    episode_index: int,
    task: str,
    resize: tuple[int, int] | None,
    action_offset: int,
    gripper_range: tuple[float, float],
    pose_transform: np.ndarray | None,
    include_state: bool,
    camera_features: dict[str, str],
    parallel_encoding: bool,
) -> None:
    start, end = umi.info.episode_ranges[episode_index]
    expected = end - start
    written = 0
    print(
        f"[EPISODE {episode_index}] {expected} frames, "
        f"{expected / float(umi.info.fps):.2f}s, task={task!r}")
    for frame in umi.iter_episode(
        episode_index,
        action_offset=action_offset,
        resize=resize,
        gripper_range=gripper_range,
        pose_transform=pose_transform,
    ):
        payload = {
            "action": np.asarray(frame.action, dtype=np.float32),
            "task": task,
        }
        if include_state:
            payload["observation.state"] = np.asarray(
                frame.state, dtype=np.float32)
        for camera in umi.info.cameras:
            payload[camera_features[camera.key]] = frame.images[camera.key]
        dataset.add_frame(payload)
        written += 1
        if written % 500 == 0:
            print(f"[EPISODE {episode_index}] {written}/{expected}")
    if written != expected:
        raise UmiError(
            f"episode {episode_index} wrote {written} frames; expected {expected}")
    dataset.save_episode(parallel_encoding=parallel_encoding)


def _write_metadata(
    output: Path,
    args: argparse.Namespace,
    info,
    selected: list[int],
    tasks: dict[int, str],
    resize: tuple[int, int] | None,
    gripper_range: tuple[float, float],
    observed_gripper: tuple[float, float],
    pose_transform: np.ndarray | None,
    action_names: tuple[str, ...],
    robot_type: str,
    include_state: bool,
    camera_features: dict[str, str],
) -> None:
    frame_count = sum(
        info.episode_ranges[index][1] - info.episode_ranges[index][0]
        for index in selected
    )
    gripper_manifest = {
        "schema_version": 1,
        "feature": "action",
        "index": len(action_names) - 1,
        "name": action_names[-1],
        "raw_units": "m",
        "value_convention": {"closed": 0.0, "open": 1.0},
        "normalization": {
            "formula": "clip((raw_width_m - min_width_m) / "
                       "(max_width_m - min_width_m), 0, 1)",
            "min_width_m": gripper_range[0],
            "max_width_m": gripper_range[1],
            "observed_min_width_m": observed_gripper[0],
            "observed_max_width_m": observed_gripper[1],
            "source": (
                "observed_min_max"
                if args.gripper_mode == "auto"
                else "user_provided_calibration"
            ),
        },
        "run_scope": {
            "episode_indices": selected,
            "frame_count": frame_count,
        },
    }
    meta_dir = output / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    (meta_dir / "gripper_normalization.json").write_text(
        json.dumps(gripper_manifest, indent=2) + "\n", encoding="utf-8")

    payload = {
        "converter": "umi_to_lerobot",
        "format_version": 2,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "source": str(info.source),
        "repo_id": args.repo_id,
        "fps": info.fps,
        "robot_type": robot_type,
        "schema": {
            "action_names": list(action_names),
            "action_convention": (
                "absolute end-effector pose at t+action_offset; final frame "
                "repeats the final pose"),
            "observation_state_written": bool(include_state),
            "gripper_normalized": True,
            "gripper_convention": {"closed": 0.0, "open": 1.0},
        },
        "action_offset": args.action_offset,
        "use_videos": not args.no_videos,
        "vcodec": None if args.no_videos else args.vcodec,
        "resize": list(resize) if resize else None,
        "source_state_names": list(info.state_names),
        "robot_index": info.robots[0].index,
        "pose_transform": {
            "path": (
                str(Path(args.pose_transform).expanduser().resolve())
                if args.pose_transform else None
            ),
            "matrix": pose_transform.tolist() if pose_transform is not None else None,
            "convention": "target_from_source",
        },
        "gripper_normalization": {
            "manifest": "meta/gripper_normalization.json",
            "mode": args.gripper_mode,
            "min_width_m": gripper_range[0],
            "max_width_m": gripper_range[1],
        },
        "cameras": [
            {
                "zarr_key": camera.key,
                "feature": camera_features[camera.key],
                "source_shape": list(camera.shape),
            }
            for camera in info.cameras
        ],
        "episodes": [
            {
                "index": episode_index,
                "start": info.episode_ranges[episode_index][0],
                "end": info.episode_ranges[episode_index][1],
                "frames": info.episode_ranges[episode_index][1]
                          - info.episode_ranges[episode_index][0],
                "task": tasks[episode_index],
            }
            for episode_index in selected
        ],
    }
    (output / "umi_conversion.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _verify(
    output: Path,
    repo_id: str,
    action_names: tuple[str, ...],
    camera_features: dict[str, str],
    include_state: bool,
) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(
        repo_id=repo_id,
        root=str(output),
        video_backend="pyav",
    )
    print("[VERIFY]")
    print(
        f"total_episodes={dataset.meta.total_episodes} "
        f"total_frames={dataset.meta.total_frames} "
        f"fps={dataset.meta.fps}")
    features = dataset.meta.features
    if "action" not in features:
        raise UmiError("converted dataset has no action feature")
    actual_names = features["action"].get("names")
    if actual_names is not None and tuple(actual_names) != tuple(action_names):
        raise UmiError(
            f"converted action names are {actual_names}; "
            f"expected {list(action_names)}")
    if include_state and "observation.state" not in features:
        raise UmiError("converted dataset has no observation.state feature")
    if not include_state and "observation.state" in features:
        raise UmiError("converted dataset unexpectedly contains observation.state")
    for feature in camera_features.values():
        if feature not in features:
            raise UmiError(f"converted dataset is missing {feature}")
    if len(dataset) == 0:
        raise UmiError("converted dataset is empty")
    sample = dataset[0]
    for key in sorted(sample):
        value = sample[key]
        shape = tuple(value.shape) if hasattr(value, "shape") else type(value).__name__
        print(f"  {key}: {shape}")
    print("[OK] LeRobotDataset loaded the first frame")


if __name__ == "__main__":
    raise SystemExit(main())
