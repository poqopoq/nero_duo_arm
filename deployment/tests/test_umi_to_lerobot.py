from __future__ import annotations

import json
import sys
import zipfile
from pathlib import Path

import numpy as np
import pytest

DEPLOYMENT_DIR = Path(__file__).resolve().parents[1]
if str(DEPLOYMENT_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOYMENT_DIR))

from tools.umi_dataset import (  # noqa: E402
    ACTION_NAMES,
    NERO_ACTION_NAMES,
    CameraSpec,
    RobotSpec,
    UmiInfo,
    apply_pose_transform,
    build_lerobot_features,
    camera_feature_name,
    load_pose_transform,
    normalize_gripper_width,
    normalize_image,
    observed_gripper_range,
    open_umi,
    parse_episode_spec,
    parse_resize,
    select_episode_indices,
    shift_state,
)
from tools.umi_to_lerobot import main  # noqa: E402


def _write_zip(source: Path, destination: Path) -> None:
    with zipfile.ZipFile(destination, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source.rglob("*")):
            if path.is_file():
                archive.write(path, path.relative_to(source).as_posix())


def make_umi_zarr(
    root_path: Path,
    episode_lengths: tuple[int, ...] = (6, 5),
    cameras: int = 2,
    robots: int = 1,
    height: int = 24,
    width: int = 32,
    compressor=None,
) -> Path:
    zarr = pytest.importorskip("zarr")
    rng = np.random.default_rng(7)
    total = sum(episode_lengths)
    root = zarr.open_group(str(root_path), mode="w")
    data = root.create_group("data")
    meta = root.create_group("meta")

    ends = np.cumsum(np.asarray(episode_lengths, dtype=np.int64))
    meta.create_dataset("episode_ends", data=ends)
    for robot in range(robots):
        data.create_dataset(
            f"robot{robot}_eef_pos",
            data=rng.normal(size=(total, 3)).astype(np.float32),
        )
        data.create_dataset(
            f"robot{robot}_eef_rot_axis_angle",
            data=rng.normal(size=(total, 3)).astype(np.float32),
        )
        data.create_dataset(
            f"robot{robot}_gripper_width",
            data=rng.uniform(0.0, 0.08, size=(total, 1)).astype(np.float32),
        )
    for camera in range(cameras):
        kwargs = {}
        if compressor is not None:
            kwargs["compressor"] = compressor
        data.create_dataset(
            f"camera{camera}_rgb",
            data=rng.integers(
                0, 255, size=(total, height, width, 3), dtype=np.uint8),
            chunks=(1, height, width, 3),
            **kwargs,
        )
    root.attrs["fps"] = 30
    meta.attrs["task"] = "pick up the block"
    return root_path


def _write_hdf5_data(
    data,
    episode_lengths: tuple[int, ...],
    cameras: int,
    height: int,
    width: int,
) -> None:
    rng = np.random.default_rng(11)
    total = sum(episode_lengths)
    for robot in range(1):
        data.create_dataset(
            f"robot{robot}_eef_pos",
            data=rng.normal(size=(total, 3)).astype(np.float32),
        )
        data.create_dataset(
            f"robot{robot}_eef_rot_axis_angle",
            data=rng.normal(size=(total, 3)).astype(np.float32),
        )
        data.create_dataset(
            f"robot{robot}_gripper_width",
            data=rng.uniform(0.0, 0.08, size=(total, 1)).astype(np.float32),
        )
    for camera in range(cameras):
        data.create_dataset(
            f"camera{camera}_rgb",
            data=rng.integers(
                0, 255, size=(total, height, width, 3), dtype=np.uint8),
        )


def make_umi_hdf5(
    path: Path,
    episode_lengths: tuple[int, ...] = (5, 4),
    cameras: int = 1,
    height: int = 24,
    width: int = 32,
    layout: str = "canonical",
    nested: bool = False,
) -> Path:
    h5py = pytest.importorskip("h5py")
    with h5py.File(path, "w") as file:
        if layout == "flat":
            if len(episode_lengths) != 1:
                raise ValueError("flat HDF5 represents exactly one episode")
            file.attrs["fps"] = 30
            file.attrs["task"] = "pick up the block"
            _write_hdf5_data(file, episode_lengths, cameras, height, width)
            return path

        parent = file.create_group("session") if nested else file
        parent.attrs["fps"] = 30
        data = parent.create_group("data")
        meta = parent.create_group("meta")
        meta.create_dataset(
            "episode_ends",
            data=np.cumsum(np.asarray(episode_lengths, dtype=np.int64)),
        )
        meta.attrs["task"] = "pick up the block"
        _write_hdf5_data(data, episode_lengths, cameras, height, width)
    return path


def test_shift_state_holds_last_frame() -> None:
    state = np.arange(12, dtype=np.float32).reshape(4, 3)
    assert np.array_equal(shift_state(state, 0), state)
    expected = np.stack([state[1], state[2], state[3], state[3]])
    assert np.array_equal(shift_state(state, 1), expected)
    assert np.array_equal(
        shift_state(state, 99), np.repeat(state[-1:], 4, axis=0))


def test_episode_selection() -> None:
    assert parse_episode_spec(None) is None
    assert parse_episode_spec("0,2-4,4") == [0, 2, 3, 4]
    assert select_episode_indices(6, "1-3", max_episodes=2) == [1, 2]
    with pytest.raises(Exception):
        parse_episode_spec("4-2")
    with pytest.raises(Exception):
        select_episode_indices(2, "5")


def test_camera_feature_name_and_resize() -> None:
    assert camera_feature_name("camera0_rgb") == "observation.images.camera0_rgb"
    assert camera_feature_name("camera-1 rgb") == "observation.images.camera_1_rgb"
    assert parse_resize("224x224") == (224, 224)
    with pytest.raises(Exception):
        parse_resize("225x224")


def test_normalize_gripper_width_clips_to_unit_range() -> None:
    raw = np.array([0.0, 0.045, 0.09, 0.12], dtype=np.float32)
    normalized = normalize_gripper_width(raw, 0.0, 0.09)
    np.testing.assert_allclose(normalized, [0.0, 0.5, 1.0, 1.0], atol=1e-6)
    with pytest.raises(Exception):
        normalize_gripper_width(raw, 0.1, 0.1)


def test_load_and_apply_pose_transform(tmp_path: Path) -> None:
    angle = np.pi / 2.0
    rotation = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [1.0, 2.0, 3.0]
    transform_path = tmp_path / "transform.json"
    transform_path.write_text(json.dumps({"matrix": matrix.tolist()}))
    loaded = load_pose_transform(transform_path)
    assert loaded is not None
    positions = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
    rotations = np.zeros((2, 3), dtype=np.float32)
    transformed_pos, transformed_rot = apply_pose_transform(
        positions, rotations, loaded)
    np.testing.assert_allclose(
        transformed_pos, [[1.0, 3.0, 3.0], [1.0, 2.0, 3.0]], atol=1e-5)
    np.testing.assert_allclose(transformed_rot, [[0.0, 0.0, angle]] * 2, atol=1e-5)


def test_normalize_image_converts_channels_and_float() -> None:
    gray = np.full((4, 5, 1), 128, dtype=np.uint8)
    assert normalize_image(gray).shape == (4, 5, 3)

    rgba = np.full((4, 5, 4), 128, dtype=np.uint8)
    assert normalize_image(rgba).shape == (4, 5, 3)

    floating = np.full((4, 5, 3), 0.5, dtype=np.float32)
    converted = normalize_image(floating)
    assert converted.dtype == np.uint8
    assert int(converted[0, 0, 0]) == 128

    resized = normalize_image(gray, resize=(8, 6))
    assert resized.shape == (6, 8, 3)


def test_build_features_for_umi_schema() -> None:
    info = UmiInfo(
        source=Path("/tmp/umi"),
        fps=30,
        total_frames=10,
        episode_ends=(10,),
        cameras=(CameraSpec("camera0_rgb", "observation.images.camera", (24, 32, 3)),),
        robots=(RobotSpec(
            index=0,
            eef_pos_key="robot0_eef_pos",
            eef_rot_key="robot0_eef_rot_axis_angle",
            gripper_key="robot0_gripper_width",
            state_names=(
                "robot0_eef_x", "robot0_eef_y", "robot0_eef_z",
                "robot0_eef_rx", "robot0_eef_ry", "robot0_eef_rz",
                "robot0_gripper_width",
            ),
        ),),
        task="pick",
        attrs={},
    )
    features = build_lerobot_features(info, use_videos=True)
    assert "observation.state" not in features
    assert features["action"]["shape"] == (7,)
    assert features["action"]["names"] == list(ACTION_NAMES)
    assert features["observation.images.camera"]["dtype"] == "video"
    assert features["observation.images.camera"]["shape"] == (24, 32, 3)

    images = build_lerobot_features(info, use_videos=False, resize=(48, 36))
    assert images["observation.images.camera"]["dtype"] == "image"
    assert images["observation.images.camera"]["shape"] == (36, 48, 3)

    with_state = build_lerobot_features(info, include_state=True)
    assert with_state["observation.state"]["names"] == list(ACTION_NAMES)


def test_open_umi_reads_episodes(tmp_path: Path) -> None:
    source = make_umi_zarr(tmp_path / "dataset.zarr", episode_lengths=(6, 4))
    with open_umi(source) as dataset:
        assert dataset.info.episode_count == 2
        assert dataset.info.total_frames == 10
        assert dataset.info.fps == 30
        assert dataset.info.task == "pick up the block"
        assert len(dataset.info.robots) == 1
        assert len(dataset.info.cameras) == 2
        assert dataset.info.state_dim == 7
        assert observed_gripper_range(dataset, [0, 1])[1] > 0.0
        frames = list(dataset.iter_episode(0, action_offset=1))
        assert len(frames) == 6
        assert frames[0].state.shape == (7,)
        assert frames[0].images["camera0_rgb"].shape == (24, 32, 3)
        assert np.array_equal(frames[0].action, frames[1].state)
        assert np.array_equal(frames[-1].action, frames[-1].state)

        normalized = list(dataset.iter_episode(
            0, action_offset=1, gripper_range=(0.0, 0.08)))
        normalized_gripper = np.array([frame.state[-1] for frame in normalized])
        assert np.all(normalized_gripper >= 0.0)
        assert np.all(normalized_gripper <= 1.0)


def test_open_umi_requires_robot_selection_when_bimanual(tmp_path: Path) -> None:
    source = make_umi_zarr(
        tmp_path / "bimanual.zarr", episode_lengths=(3,), robots=2)
    with pytest.raises(Exception, match="multiple robots"):
        open_umi(source)
    with open_umi(source, robot_index=1) as dataset:
        assert dataset.info.robots[0].index == 1


def test_single_camera_uses_published_feature_name(tmp_path: Path) -> None:
    source = make_umi_zarr(
        tmp_path / "one_camera.zarr", episode_lengths=(3,), cameras=1)
    with open_umi(source) as dataset:
        assert dataset.info.cameras[0].feature == "observation.images.camera"


def test_open_umi_reads_zarr_zip(tmp_path: Path) -> None:
    source = make_umi_zarr(tmp_path / "dataset.zarr", episode_lengths=(3,))
    archive = tmp_path / "dataset.zarr.zip"
    _write_zip(source, archive)
    with open_umi(archive) as dataset:
        assert dataset.info.total_frames == 3
        assert len(list(dataset.iter_episode(0))) == 3


@pytest.mark.parametrize("nested", [False, True])
def test_open_umi_reads_canonical_hdf5(tmp_path: Path, nested: bool) -> None:
    source = make_umi_hdf5(
        tmp_path / "dataset.hdf5",
        episode_lengths=(5, 4),
        cameras=2,
        nested=nested,
    )
    with open_umi(source) as dataset:
        assert dataset.info.episode_count == 2
        assert dataset.info.total_frames == 9
        assert dataset.info.fps == 30
        assert dataset.info.task == "pick up the block"
        assert len(dataset.info.cameras) == 2
        frames = list(dataset.iter_episode(1, action_offset=1))
        assert len(frames) == 4
        assert frames[0].state.shape == (7,)
        assert frames[0].images["camera0_rgb"].shape == (24, 32, 3)
        assert np.array_equal(frames[0].action, frames[1].state)


def test_open_umi_reads_flat_single_episode_hdf5(tmp_path: Path) -> None:
    source = make_umi_hdf5(
        tmp_path / "flat.h5",
        episode_lengths=(6,),
        layout="flat",
    )
    with open_umi(source) as dataset:
        assert dataset.info.episode_count == 1
        assert dataset.info.total_frames == 6
        assert dataset.info.episode_ends == (6,)
        assert dataset.info.fps == 30
        assert dataset.info.task == "pick up the block"
        assert len(list(dataset.iter_episode(0))) == 6


def test_cli_hdf5_dry_run_nero_contract(tmp_path: Path, capsys) -> None:
    source = make_umi_hdf5(tmp_path / "dataset.h5", episode_lengths=(4,))
    code = main([str(source), "--dry-run", "--contract", "nero-ros"])
    assert code == 0
    output = capsys.readouterr().out
    assert "output contract: nero-ros" in output
    assert "robot_type=nero_ros" in output
    assert "-> observation.images.camera1" in output
    assert '"observation.images.camera1"' in output
    assert '"observation.state"' in output
    assert "ee.rx" in output
    assert "ee.gripper" in output


def test_convert_hdf5_with_nero_contract(tmp_path: Path) -> None:
    pytest.importorskip("lerobot")
    source = make_umi_hdf5(
        tmp_path / "dataset.h5",
        episode_lengths=(5,),
    )
    output = tmp_path / "lerobot" / "umi_hdf5_nero"
    code = main([
        str(source),
        "-o", str(output),
        "--repo-id", "local/umi_hdf5_nero",
        "--contract", "nero-ros",
        "--gripper-min", "0.0",
        "--gripper-max", "0.09",
        "--no-videos",
        "--image-writer-threads", "0",
    ])
    assert code == 0

    provenance = json.loads((output / "umi_conversion.json").read_text())
    assert provenance["robot_type"] == "nero_ros"
    assert provenance["schema"]["action_names"] == list(NERO_ACTION_NAMES)
    assert provenance["schema"]["observation_state_written"] is True
    assert provenance["cameras"][0]["feature"] == "observation.images.camera1"

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(
        repo_id="local/umi_hdf5_nero", root=str(output), video_backend="pyav")
    assert dataset.meta.robot_type == "nero_ros"
    assert dataset.meta.features["action"]["names"] == list(NERO_ACTION_NAMES)
    assert dataset.meta.features["observation.state"]["names"] == list(
        NERO_ACTION_NAMES)
    sample = dataset[0]
    assert tuple(sample["action"].shape) == (7,)
    assert tuple(sample["observation.state"].shape) == (7,)
    assert tuple(sample["observation.images.camera1"].shape) == (3, 24, 32)


def test_cli_dry_run(tmp_path: Path, capsys) -> None:
    source = make_umi_zarr(tmp_path / "dataset.zarr", episode_lengths=(4,))
    code = main([str(source), "--dry-run"])
    assert code == 0
    output = capsys.readouterr().out
    assert "fps: 30" in output
    assert "pick up the block" in output
    assert "DRY-RUN" in output


def test_convert_without_videos(tmp_path: Path) -> None:
    pytest.importorskip("lerobot")
    source = make_umi_zarr(
        tmp_path / "dataset.zarr",
        episode_lengths=(5, 4),
        cameras=2,
        height=24,
        width=32,
    )
    output = tmp_path / "lerobot" / "umi_test"
    code = main([
        str(source),
        "-o", str(output),
        "--repo-id", "local/umi_test",
        "--gripper-min", "0.0",
        "--gripper-max", "0.09",
        "--no-videos",
        "--image-writer-threads", "0",
    ])
    assert code == 0
    assert (output / "meta" / "info.json").is_file()
    assert (output / "umi_conversion.json").is_file()

    provenance = json.loads((output / "umi_conversion.json").read_text())
    assert provenance["repo_id"] == "local/umi_test"
    assert provenance["fps"] == 30
    assert len(provenance["episodes"]) == 2
    assert provenance["schema"]["action_names"] == list(ACTION_NAMES)
    assert provenance["schema"]["observation_state_written"] is False
    assert (output / "meta" / "gripper_normalization.json").is_file()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(
        repo_id="local/umi_test", root=str(output), video_backend="pyav")
    assert dataset.meta.total_episodes == 2
    assert dataset.meta.total_frames == 9
    assert "observation.state" not in dataset.meta.features
    assert dataset.meta.features["action"]["names"] == list(ACTION_NAMES)
    sample = dataset[0]
    assert "observation.state" not in sample
    assert tuple(sample["action"].shape) == (7,)
    assert tuple(sample["observation.images.camera0_rgb"].shape) == (3, 24, 32)

    zarr = pytest.importorskip("zarr")
    root = zarr.open(str(source), mode="r")
    position = np.asarray(root["data"]["robot0_eef_pos"][:], dtype=np.float32)
    rotation = np.asarray(
        root["data"]["robot0_eef_rot_axis_angle"][:], dtype=np.float32)
    gripper = np.asarray(
        root["data"]["robot0_gripper_width"][:], dtype=np.float32).reshape(-1)
    expected_next = np.array([
        *position[1],
        *rotation[1],
        np.clip((gripper[1] - 0.0) / 0.09, 0.0, 1.0),
    ])
    np.testing.assert_allclose(
        sample["action"].numpy(), expected_next, atol=1e-6)
    expected_last = np.array([
        *position[4],
        *rotation[4],
        np.clip((gripper[4] - 0.0) / 0.09, 0.0, 1.0),
    ])
    np.testing.assert_allclose(
        dataset[4]["action"].numpy(), expected_last, atol=1e-6)


def test_convert_with_videos(tmp_path: Path) -> None:
    pytest.importorskip("lerobot")
    pytest.importorskip("av")
    source = make_umi_zarr(
        tmp_path / "dataset.zarr",
        episode_lengths=(5, 5),
        cameras=1,
        height=24,
        width=32,
    )
    output = tmp_path / "lerobot" / "umi_video"
    code = main([
        str(source),
        "-o", str(output),
        "--repo-id", "local/umi_video",
        "--gripper-min", "0.0",
        "--gripper-max", "0.09",
        "--vcodec", "h264",
        "--image-writer-threads", "0",
    ])
    assert code == 0
    assert any((output / "videos").rglob("*.mp4"))

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(
        repo_id="local/umi_video", root=str(output), video_backend="pyav")
    sample = dataset[0]
    assert "observation.state" not in sample
    assert tuple(sample["observation.images.camera"].shape) == (3, 24, 32)
