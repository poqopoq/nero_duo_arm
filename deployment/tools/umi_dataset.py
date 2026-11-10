"""Read UMI zarr/HDF5 datasets and map them to the EE LeRobot v3 contract.

The canonical UMI replay buffer is a zarr group with this layout::

    data/camera0_rgb                 (T, H, W, 3) uint8
    data/robot0_eef_pos              (T, 3) float32
    data/robot0_eef_rot_axis_angle   (T, 3) float32
    data/robot0_gripper_width        (T, 1) float32
    meta/episode_ends                (N,) int64

The training contract is the one used by the UMI/SROI LeRobot pipeline: one
absolute 7-D action named ``ee.x .. ee.gripper_pos`` and no on-disk
``observation.state``.  ``action[t]`` is the pose at ``t + 1`` and the final
frame repeats the final pose.

The same canonical layout is accepted from zarr or HDF5.  HDF5 additionally
accepts one enclosing group and a flat single-episode file.  This module
deliberately does not import ``lerobot`` at import time.  The reader can
therefore be exercised on its own, and the converter imports LeRobot only after
the source has been validated.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator, Mapping, Sequence

import numpy as np

CAMERA_RE = re.compile(r"^camera(?P<index>\d+)_rgb(?P<mirror>_mirror_crop)?$")
ROBOT_POS_RE = re.compile(r"^robot(?P<index>\d+)_eef_pos$")

ACTION_NAMES = (
    "ee.x",
    "ee.y",
    "ee.z",
    "ee.wx",
    "ee.wy",
    "ee.wz",
    "ee.gripper_pos",
)

NERO_ACTION_NAMES = (
    "ee.x.pos",
    "ee.y.pos",
    "ee.z.pos",
    "ee.rx.pos",
    "ee.ry.pos",
    "ee.rz.pos",
    "ee.gripper.pos",
)


class UmiError(RuntimeError):
    """A user-facing UMI source or conversion error."""


@dataclass(frozen=True)
class CameraSpec:
    key: str
    feature: str
    shape: tuple[int, int, int]


@dataclass(frozen=True)
class RobotSpec:
    index: int
    eef_pos_key: str
    eef_rot_key: str
    gripper_key: str | None
    state_names: tuple[str, ...]


@dataclass(frozen=True)
class UmiInfo:
    source: Path
    fps: int | None
    total_frames: int
    episode_ends: tuple[int, ...]
    cameras: tuple[CameraSpec, ...]
    robots: tuple[RobotSpec, ...]
    task: str | None
    attrs: Mapping[str, Any]

    @property
    def episode_count(self) -> int:
        return len(self.episode_ends)

    @property
    def episode_ranges(self) -> tuple[tuple[int, int], ...]:
        start = 0
        ranges = []
        for end in self.episode_ends:
            ranges.append((start, end))
            start = end
        return tuple(ranges)

    @property
    def state_names(self) -> tuple[str, ...]:
        return tuple(name for robot in self.robots for name in robot.state_names)

    @property
    def state_dim(self) -> int:
        return len(self.state_names)


@dataclass(frozen=True)
class EpisodeFrame:
    frame_index: int
    timestamp: float
    state: np.ndarray
    action: np.ndarray
    images: Mapping[str, np.ndarray]


class UmiDataset:
    """An opened UMI zarr/HDF5 source plus validated schema metadata."""

    def __init__(self, root, info: UmiInfo, store=None):
        self._root = root
        self.info = info
        self._store = store

    @property
    def root(self):
        return self._root

    def array(self, key: str):
        return self._root["data"][key]

    def close(self) -> None:
        if self._store is not None:
            self._store.close()
            self._store = None

    def __enter__(self) -> "UmiDataset":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def iter_episode(
        self,
        episode_index: int,
        action_offset: int = 1,
        resize: tuple[int, int] | None = None,
        gripper_range: tuple[float, float] | None = None,
        pose_transform: np.ndarray | None = None,
    ) -> Iterator[EpisodeFrame]:
        """Yield fixed-rate frames for one UMI episode.

        ``resize`` is ``(width, height)`` to match OpenCV.  Action at time t
        is the state at ``t + action_offset``; the last state is held at the
        end of the episode.
        """
        if action_offset < 0:
            raise UmiError("action_offset must be >= 0")
        if not 0 <= episode_index < self.info.episode_count:
            raise UmiError(
                f"episode {episode_index} is out of range "
                f"(0..{self.info.episode_count - 1})")
        if self.info.fps is None:
            raise UmiError("UMI fps is unknown; pass --fps")

        start, end = self.info.episode_ranges[episode_index]
        states = self._episode_state(
            start,
            end,
            gripper_range=gripper_range,
            pose_transform=pose_transform,
        )
        actions = shift_state(states, action_offset)
        image_arrays = {
            camera.key: self.array(camera.key)
            for camera in self.info.cameras
        }

        for local_index in range(end - start):
            images = {
                camera.key: normalize_image(
                    image_arrays[camera.key][start + local_index],
                    expected_hw=(camera.shape[0], camera.shape[1]),
                    resize=resize,
                )
                for camera in self.info.cameras
            }
            yield EpisodeFrame(
                frame_index=local_index,
                timestamp=local_index / float(self.info.fps),
                state=states[local_index].copy(),
                action=actions[local_index].copy(),
                images=images,
            )

    def _episode_state(
        self,
        start: int,
        end: int,
        gripper_range: tuple[float, float] | None = None,
        pose_transform: np.ndarray | None = None,
    ) -> np.ndarray:
        parts: list[np.ndarray] = []
        for robot in self.info.robots:
            position = np.asarray(
                self.array(robot.eef_pos_key)[start:end], dtype=np.float32)
            rotation = np.asarray(
                self.array(robot.eef_rot_key)[start:end], dtype=np.float32)
            if position.shape != (end - start, 3):
                raise UmiError(
                    f"{robot.eef_pos_key} has shape {position.shape}; "
                    f"expected {(end - start, 3)}")
            if rotation.shape != (end - start, 3):
                raise UmiError(
                    f"{robot.eef_rot_key} has shape {rotation.shape}; "
                    f"expected {(end - start, 3)}")
            position, rotation = apply_pose_transform(
                position, rotation, pose_transform)
            parts.extend((position, rotation))
            if robot.gripper_key is not None:
                width = np.asarray(
                    self.array(robot.gripper_key)[start:end], dtype=np.float32)
                width = width.reshape(end - start, -1)
                if width.shape[1] != 1:
                    raise UmiError(
                        f"{robot.gripper_key} has {width.shape[1]} columns; "
                        "expected one width column")
                if gripper_range is not None:
                    width = normalize_gripper_width(
                        width, gripper_range[0], gripper_range[1])
                parts.append(width)
        state = np.concatenate(parts, axis=1).astype(np.float32, copy=False)
        if state.shape[1] != self.info.state_dim:
            raise UmiError(
                f"state dimension {state.shape[1]} does not match "
                f"schema {self.info.state_dim}")
        if not np.all(np.isfinite(state)):
            raise UmiError(
                "episode contains non-finite eef/gripper values; "
                "clean the UMI SLAM output before conversion")
        return state


def shift_state(state: np.ndarray, offset: int) -> np.ndarray:
    """Shift a state matrix forward in time, holding the final state."""
    if offset < 0:
        raise UmiError("action offset must be >= 0")
    if len(state) == 0:
        return state.copy()
    if offset == 0:
        return state.copy()
    offset = min(offset, len(state))
    tail = np.repeat(state[-1:], offset, axis=0)
    return np.concatenate([state[offset:], tail], axis=0)


def normalize_gripper_width(
    width: np.ndarray,
    minimum: float,
    maximum: float,
) -> np.ndarray:
    """Map a calibrated UMI width in metres to the 0=closed, 1=open range."""
    minimum = float(minimum)
    maximum = float(maximum)
    if not np.isfinite(minimum) or not np.isfinite(maximum):
        raise UmiError("gripper normalization bounds must be finite")
    if maximum <= minimum:
        raise UmiError(
            f"gripper max must be greater than min (min={minimum}, "
            f"max={maximum})")
    normalized = (np.asarray(width, dtype=np.float32) - minimum) / (
        maximum - minimum)
    return np.clip(normalized, 0.0, 1.0).astype(np.float32, copy=False)


def observed_gripper_range(
    dataset: "UmiDataset",
    episode_indices: Sequence[int],
) -> tuple[float, float]:
    """Return the observed raw widths over the selected episodes.

    This is useful for inspection and explicit ``--gripper-mode auto`` runs,
    but it is not a substitute for the physical jaw calibration when the
    demonstrations never reach both endpoints.
    """
    robot = dataset.info.robots[0]
    if robot.gripper_key is None:
        raise UmiError("UMI source has no gripper width array")
    values = dataset.array(robot.gripper_key)
    lows: list[float] = []
    highs: list[float] = []
    for episode_index in episode_indices:
        start, end = dataset.info.episode_ranges[episode_index]
        chunk = np.asarray(values[start:end], dtype=np.float32).reshape(-1)
        if chunk.size == 0:
            continue
        if not np.all(np.isfinite(chunk)):
            raise UmiError(
                f"{robot.gripper_key} episode {episode_index} contains "
                "non-finite widths")
        lows.append(float(chunk.min()))
        highs.append(float(chunk.max()))
    if not lows:
        raise UmiError("selected episodes contain no gripper widths")
    minimum = min(lows)
    maximum = max(highs)
    return minimum, maximum


def load_pose_transform(path: str | Path | None) -> np.ndarray | None:
    """Load a 4x4 target-from-source transform from JSON.

    Accepted JSON forms are a 4x4 nested list, a flat 16-value list, or an
    object with a ``matrix`` or ``pose_transform`` key containing either.
    """
    if path is None:
        return None
    source = Path(path).expanduser()
    try:
        raw = json.loads(source.read_text(encoding="utf-8"))
    except OSError as exc:
        raise UmiError(f"cannot read pose transform {source}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise UmiError(f"pose transform {source} is not valid JSON: {exc}") from exc
    if isinstance(raw, dict):
        for key in ("matrix", "pose_transform", "transform"):
            if key in raw:
                raw = raw[key]
                break
    try:
        matrix = np.asarray(raw, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise UmiError("pose transform must contain only numbers") from exc
    if matrix.shape == (16,):
        matrix = matrix.reshape(4, 4)
    if matrix.shape != (4, 4):
        raise UmiError(
            f"pose transform has shape {matrix.shape}; expected 4x4 or 16 values")
    if not np.all(np.isfinite(matrix)):
        raise UmiError("pose transform contains non-finite values")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6):
        raise UmiError("pose transform bottom row must be [0, 0, 0, 1]")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5):
        raise UmiError("pose transform rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise UmiError("pose transform rotation determinant must be +1")
    return matrix.astype(np.float32)


def apply_pose_transform(
    position: np.ndarray,
    rotation: np.ndarray,
    transform: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply ``target_from_source`` to a batch of position/axis-angle poses."""
    if transform is None:
        return position, rotation
    if position.shape[-1] != 3 or rotation.shape[-1] != 3:
        raise UmiError("pose transform expects position and rotation with 3 columns")
    matrix_rotation = transform[:3, :3]
    translation = transform[:3, 3]
    transformed_position = position @ matrix_rotation.T + translation
    source_matrix = _rotvec_to_matrix(rotation)
    transformed_matrix = matrix_rotation @ source_matrix
    transformed_rotation = _matrix_to_rotvec(transformed_matrix)
    return (
        np.asarray(transformed_position, dtype=np.float32),
        np.asarray(transformed_rotation, dtype=np.float32),
    )


def _skew(vector: np.ndarray) -> np.ndarray:
    """Return the batch of skew-symmetric matrices for 3-D vectors."""
    x, y, z = np.moveaxis(vector, -1, 0)
    zeros = np.zeros_like(x)
    return np.stack([
        np.stack([zeros, -z, y], axis=-1),
        np.stack([z, zeros, -x], axis=-1),
        np.stack([-y, x, zeros], axis=-1),
    ], axis=-2)


def _rotvec_to_matrix(rotation: np.ndarray) -> np.ndarray:
    """Rodrigues conversion from axis-angle vectors to rotation matrices."""
    rotation = np.asarray(rotation, dtype=np.float64)
    theta = np.linalg.norm(rotation, axis=-1)
    skew = _skew(rotation)
    skew_squared = skew @ skew
    identity = np.broadcast_to(np.eye(3), rotation.shape[:-1] + (3, 3))
    safe_theta = np.where(theta > 1e-12, theta, 1.0)
    sine = np.sin(theta) / safe_theta
    cosine = (1.0 - np.cos(theta)) / (safe_theta ** 2)
    matrix = identity + sine[..., None, None] * skew + cosine[..., None, None] * skew_squared
    small = theta <= 1e-12
    if np.any(small):
        matrix[small] = identity[small] + skew[small]
    return matrix


def _matrix_to_rotvec(matrix: np.ndarray) -> np.ndarray:
    """Rotation-matrix to axis-angle conversion without SciPy."""
    matrix = np.asarray(matrix, dtype=np.float64)
    batch_shape = matrix.shape[:-2]
    flat = matrix.reshape(-1, 3, 3)
    quaternions = np.empty((flat.shape[0], 4), dtype=np.float64)
    for index, rotation in enumerate(flat):
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = np.sqrt(trace + 1.0) * 2.0
            quaternions[index] = (
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            )
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            scale = np.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternions[index] = (
                (rotation[2, 1] - rotation[1, 2]) / scale,
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
            )
        elif rotation[1, 1] > rotation[2, 2]:
            scale = np.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternions[index] = (
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
            )
        else:
            scale = np.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternions[index] = (
                (rotation[1, 0] - rotation[0, 1]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
            )
    quaternions /= np.linalg.norm(quaternions, axis=-1, keepdims=True)
    quaternions[quaternions[:, 0] < 0.0] *= -1.0
    vector = quaternions[:, 1:]
    scalar = quaternions[:, 0]
    norm = np.linalg.norm(vector, axis=-1)
    angle = 2.0 * np.arctan2(norm, scalar)
    safe_norm = np.where(norm > 1e-12, norm, 1.0)
    rotvec = vector * (angle / safe_norm)[:, None]
    return rotvec.reshape(batch_shape + (3,))


def camera_feature_name(key: str) -> str:
    suffix = re.sub(r"[^A-Za-z0-9_]+", "_", key).strip("_")
    if not suffix:
        raise UmiError(f"camera key {key!r} cannot be converted to a feature name")
    return f"observation.images.{suffix}"


def parse_episode_spec(spec: str | None) -> list[int] | None:
    if spec is None or not spec.strip():
        return None
    indices: list[int] = []
    for raw_part in spec.split(","):
        part = raw_part.strip()
        if not part:
            continue
        if "-" in part:
            left, right = part.split("-", 1)
            try:
                start = int(left)
                end = int(right)
            except ValueError as exc:
                raise UmiError(f"invalid episode range {part!r}") from exc
            if end < start:
                raise UmiError(f"episode range {part!r} is descending")
            indices.extend(range(start, end + 1))
        else:
            try:
                indices.append(int(part))
            except ValueError as exc:
                raise UmiError(f"invalid episode index {part!r}") from exc
    if not indices:
        raise UmiError("episode selection is empty")
    return list(dict.fromkeys(indices))


def select_episode_indices(
    episode_count: int,
    spec: str | None = None,
    max_episodes: int | None = None,
) -> list[int]:
    indices = parse_episode_spec(spec)
    if indices is None:
        indices = list(range(episode_count))
    for index in indices:
        if not 0 <= index < episode_count:
            raise UmiError(
                f"episode {index} is out of range (0..{episode_count - 1})")
    if max_episodes is not None:
        if max_episodes < 1:
            raise UmiError("max_episodes must be >= 1")
        indices = indices[:max_episodes]
    return indices


def load_task_map(path: str | Path | None) -> dict[int, str]:
    if path is None:
        return {}
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except OSError as exc:
        raise UmiError(f"cannot read task file {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise UmiError(f"task file {path} is not valid JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise UmiError("task file must contain a JSON object of episode -> task")
    tasks: dict[int, str] = {}
    for key, value in raw.items():
        try:
            index = int(key)
        except (TypeError, ValueError) as exc:
            raise UmiError(f"task file episode key {key!r} is not an integer") from exc
        if not isinstance(value, str) or not value.strip():
            raise UmiError(f"task for episode {index} must be a non-empty string")
        tasks[index] = value.strip()
    return tasks


def parse_resize(value: str | None) -> tuple[int, int] | None:
    if value is None:
        return None
    match = re.fullmatch(r"\s*(\d+)\s*[xX]\s*(\d+)\s*", value)
    if match is None:
        raise UmiError("--resize must look like WIDTHxHEIGHT, for example 224x224")
    width, height = int(match.group(1)), int(match.group(2))
    if width < 2 or height < 2:
        raise UmiError("--resize dimensions must be at least 2 pixels")
    if width % 2 or height % 2:
        raise UmiError(
            "--resize dimensions must be even for yuv420p video output")
    return width, height


def build_lerobot_features(
    info: UmiInfo,
    use_videos: bool = True,
    resize: tuple[int, int] | None = None,
    include_state: bool = False,
    state_names: Sequence[str] = ACTION_NAMES,
    camera_features: Mapping[str, str] | None = None,
) -> dict[str, dict]:
    """Build the EE LeRobot v3 feature dictionary for a UMI dataset.

    ``observation.state`` is intentionally omitted by default.  The SROI/UMI
    training pipeline derives its state from the absolute action column and
    later builds relative 10-D chunks in the training processor.
    """
    _validate_ee_contract(info)
    names = list(state_names)
    features: dict[str, dict] = {
        "action": {
            "dtype": "float32",
            "shape": (len(names),),
            "names": names,
        },
    }
    if include_state:
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (len(names),),
            "names": names,
        }
    for camera in info.cameras:
        height, width = camera.shape[0], camera.shape[1]
        if resize is not None:
            width, height = resize
        feature = (
            camera_features[camera.key]
            if camera_features is not None
            else camera.feature
        )
        features[feature] = {
            "dtype": "video" if use_videos else "image",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        }
    return features


def _validate_ee_contract(info: UmiInfo) -> None:
    if len(info.robots) != 1:
        raise UmiError(
            "the EE LeRobot contract requires exactly one UMI robot; "
            f"found {len(info.robots)}")
    if info.robots[0].gripper_key is None:
        raise UmiError(
            "the EE LeRobot contract requires a gripper width array")
    if info.state_dim != len(ACTION_NAMES):
        raise UmiError(
            f"UMI state dimension is {info.state_dim}; expected "
            f"{len(ACTION_NAMES)} (xyz + axis-angle + gripper)")


def normalize_image(
    image: np.ndarray,
    expected_hw: tuple[int, int] | None = None,
    resize: tuple[int, int] | None = None,
) -> np.ndarray:
    """Return a contiguous RGB uint8 image, optionally resized."""
    array = np.asarray(image)
    if array.ndim != 3:
        raise UmiError(f"image frame has shape {array.shape}; expected HxWxC")
    if expected_hw is not None and tuple(array.shape[:2]) != tuple(expected_hw):
        raise UmiError(
            f"image frame has HxW {array.shape[:2]}; expected {expected_hw}")
    if array.shape[2] == 1:
        array = np.repeat(array, 3, axis=2)
    elif array.shape[2] == 4:
        array = array[:, :, :3]
    elif array.shape[2] != 3:
        raise UmiError(
            f"image frame has {array.shape[2]} channels; expected 1, 3, or 4")

    if array.dtype != np.uint8:
        if np.issubdtype(array.dtype, np.floating):
            if not np.isfinite(array).all():
                raise UmiError("image frame contains non-finite values")
            minimum = float(array.min()) if array.size else 0.0
            maximum = float(array.max()) if array.size else 0.0
            if minimum < 0.0 or maximum > 1.0:
                raise UmiError(
                    f"floating image values span [{minimum}, {maximum}]; "
                    "expected [0, 1]")
            array = np.rint(array * 255.0).astype(np.uint8)
        else:
            raise UmiError(
                f"image dtype {array.dtype} is unsupported; expected uint8")

    if resize is not None:
        import cv2

        array = cv2.resize(array, resize, interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(array)


def open_umi(
    source: str | Path,
    fps: int | None = None,
    task: str | None = None,
    cameras: Sequence[str] | None = None,
    include_mirror: bool = False,
    robot_index: int | None = None,
) -> UmiDataset:
    """Open and validate a UMI zarr or HDF5 replay buffer."""
    source_path = Path(source).expanduser().resolve()
    if not source_path.exists():
        raise UmiError(f"UMI source does not exist: {source_path}")

    if source_path.suffix.lower() in (".h5", ".hdf5"):
        root, store = _open_hdf5_root(source_path)
    else:
        root, store = _open_root(source_path)
    try:
        if "data" not in root or "meta" not in root:
            raise UmiError(
                f"{source_path} is not a UMI replay buffer: "
                "missing data/ or meta/ group")
        data = root["data"]
        meta = root["meta"]
        data_keys = sorted(data.keys())
        if not data_keys:
            raise UmiError(f"{source_path} has an empty data/ group")

        episode_ends = _read_episode_ends(meta)
        total_frames = int(episode_ends[-1])
        if total_frames <= 0:
            raise UmiError("UMI dataset has no frames")
        _validate_time_lengths(data, data_keys, total_frames)
        robots = _discover_robots(data, data_keys, total_frames)
        if robot_index is not None:
            matched = [robot for robot in robots if robot.index == robot_index]
            if not matched:
                raise UmiError(
                    f"robot index {robot_index} is not present; available: "
                    f"{[robot.index for robot in robots]}")
            robots = matched
        elif len(robots) > 1:
            raise UmiError(
                "UMI source contains multiple robots; pass --robot-index. "
                f"Available: {[robot.index for robot in robots]}. For raw "
                "bimanual HDF5/MKV data use Xense's hdf52lerobot instead.")
        camera_specs = _discover_cameras(
            data, data_keys, total_frames, cameras, include_mirror)
        attrs = _merged_attrs(root, meta)
        resolved_fps = _resolve_fps(attrs, meta, fps)
        resolved_task = _resolve_task(attrs, meta, task)

        info = UmiInfo(
            source=source_path,
            fps=resolved_fps,
            total_frames=total_frames,
            episode_ends=tuple(int(x) for x in episode_ends),
            cameras=tuple(camera_specs),
            robots=tuple(robots),
            task=resolved_task,
            attrs=attrs,
        )
        return UmiDataset(root, info, store)
    except Exception:
        if store is not None:
            store.close()
        raise


def _open_root(source: Path):
    try:
        import zarr
    except ImportError as exc:  # pragma: no cover - dependency error path
        raise UmiError(
            "UMI conversion requires zarr. Run "
            "deployment/scripts/install_umi_lerobot_env.sh first."
        ) from exc

    from .umi_codecs import register_umi_codecs

    register_umi_codecs()
    store = None
    if source.is_dir():
        root = zarr.open(str(source), mode="r")
    else:
        try:
            from zarr.storage import ZipStore
        except ImportError:  # pragma: no cover - zarr v2 fallback
            ZipStore = zarr.ZipStore
        try:
            store = ZipStore(str(source), mode="r")
        except Exception as exc:
            raise UmiError(f"cannot open UMI archive {source}: {exc}") from exc
        root = zarr.open(store, mode="r")

    if "data" not in root:
        groups = [
            name for name in root.group_keys()
            if "data" in root[name] and "meta" in root[name]
        ]
        if len(groups) == 1:
            root = root[groups[0]]
    return root, store


class _NumpyArray:
    """Minimal array view used to synthesize ``meta/episode_ends``."""

    def __init__(self, values: np.ndarray):
        self._values = np.asarray(values)
        self.shape = self._values.shape
        self.dtype = self._values.dtype

    def __getitem__(self, item):
        return self._values[item]


class _Hdf5DataView:
    """Expose root-level HDF5 datasets as the canonical ``data`` group."""

    def __init__(self, file, keys: Sequence[str]):
        self._file = file
        self._keys = tuple(keys)

    def keys(self):
        return self._keys

    def __contains__(self, key: str) -> bool:
        return key in self._keys

    def __getitem__(self, key: str):
        return self._file[key]


class _Hdf5MetaView:
    """Provide episode metadata for a flat, single-episode HDF5 file."""

    def __init__(self, file, total_frames: int):
        self._file = file
        self._total_frames = int(total_frames)

    @property
    def attrs(self):
        return self._file.attrs

    def keys(self):
        return ("episode_ends",)

    def __contains__(self, key: str) -> bool:
        return key == "episode_ends"

    def __getitem__(self, key: str):
        if key != "episode_ends":
            raise KeyError(key)
        if "episode_ends" in self._file:
            return self._file["episode_ends"]
        return _NumpyArray(
            np.asarray([self._total_frames], dtype=np.int64))


class _Hdf5RootView:
    """Map canonical and flat HDF5 layouts onto the zarr root interface."""

    def __init__(self, file, data, meta, attrs_source=None):
        self._file = file
        self._data = data
        self._meta = meta
        self._attrs_source = attrs_source

    @property
    def attrs(self):
        attrs = dict(self._file.attrs)
        if self._attrs_source is not None:
            attrs.update(dict(self._attrs_source.attrs))
        return attrs

    def __contains__(self, key: str) -> bool:
        return key in ("data", "meta")

    def __getitem__(self, key: str):
        if key == "data":
            return self._data
        if key == "meta":
            return self._meta
        raise KeyError(key)

    def group_keys(self):
        return ("data", "meta")

    def close(self) -> None:
        self._file.close()


def _open_hdf5_root(source: Path):
    try:
        import h5py
    except ImportError as exc:  # pragma: no cover - dependency error path
        raise UmiError(
            "HDF5 conversion requires h5py. Run "
            "deployment/scripts/install_umi_lerobot_env.sh first."
        ) from exc

    try:
        file = h5py.File(source, mode="r")
    except OSError as exc:
        raise UmiError(f"cannot open HDF5 source {source}: {exc}") from exc

    try:
        if "data" in file:
            data = file["data"]
            meta = file["meta"] if "meta" in file else _synth_meta(file, data)
            view = _Hdf5RootView(file, data, meta)
            return view, view

        nested = [
            name for name in file.keys()
            if isinstance(file[name], h5py.Group)
            and "data" in file[name]
        ]
        if len(nested) == 1:
            group = file[nested[0]]
            data = group["data"]
            meta = group["meta"] if "meta" in group else _synth_meta(file, data)
            view = _Hdf5RootView(file, data, meta, attrs_source=group)
            return view, view

        keys = _flat_umi_keys(file)
        if not keys:
            raise UmiError(
                f"{source} is not a supported UMI HDF5 file. Expected "
                "data/robot0_eef_pos + data/camera0_rgb + meta/episode_ends, "
                "or those arrays at the root. For Xense per-stream HDF5/MKV "
                "recordings use Xense's hdf52lerobot.")
        data = _Hdf5DataView(file, keys)
        meta = file["meta"] if "meta" in file else _synth_meta(file, data)
        view = _Hdf5RootView(file, data, meta)
        return view, view
    except Exception:
        file.close()
        raise


def _flat_umi_keys(file) -> list[str]:
    """Return root-level datasets that can form a flat UMI episode."""

    ignored = {"episode_ends", "fps", "frame_rate", "task"}
    keys = []
    for key in file.keys():
        node = file[key]
        if key in ignored or not hasattr(node, "shape"):
            continue
        keys.append(key)
    return sorted(keys)


def _synth_meta(file, data):
    for key in data.keys():
        shape = getattr(data[key], "shape", ())
        if len(shape) > 0:
            return _Hdf5MetaView(file, int(shape[0]))
    raise UmiError("HDF5 source has no non-empty data arrays")


def _read_episode_ends(meta) -> np.ndarray:
    if "episode_ends" not in meta:
        raise UmiError(
            "UMI dataset is missing meta/episode_ends; "
            "cannot split demonstrations into episodes")
    ends = np.asarray(meta["episode_ends"][:], dtype=np.int64).reshape(-1)
    if ends.size == 0:
        raise UmiError("meta/episode_ends is empty")
    if np.any(ends <= 0) or np.any(np.diff(ends) <= 0):
        raise UmiError(
            f"meta/episode_ends must be strictly increasing, got {ends.tolist()}")
    return ends


def _validate_time_lengths(data, keys: Sequence[str], total_frames: int) -> None:
    mismatches = []
    for key in keys:
        shape = getattr(data[key], "shape", ())
        if len(shape) == 0:
            continue
        if int(shape[0]) != total_frames:
            mismatches.append(f"{key}={shape[0]}")
    if mismatches:
        raise UmiError(
            "data arrays do not share the episode_ends length "
            f"{total_frames}: {', '.join(mismatches)}")


def _discover_robots(data, keys: Sequence[str], total_frames: int) -> list[RobotSpec]:
    positions = {
        int(match.group("index")): key
        for key in keys
        if (match := ROBOT_POS_RE.match(key))
    }
    if not positions and "eef_pos" in keys:
        positions[0] = "eef_pos"
    if not positions:
        eef_keys = [key for key in keys if "eef_pos" in key]
        raise UmiError(
            "no UMI end-effector position array found; expected "
            "data/robot0_eef_pos or data/eef_pos. "
            f"Available eef keys: {eef_keys or 'none'}")

    robots: list[RobotSpec] = []
    for index in sorted(positions):
        position_key = positions[index]
        rotation_key = _first_existing(
            keys,
            f"robot{index}_eef_rot_axis_angle",
            "eef_rot_axis_angle" if index == 0 else None,
        )
        if rotation_key is None:
            raise UmiError(
                f"missing axis-angle rotation array for robot{index}; "
                f"expected data/robot{index}_eef_rot_axis_angle")
        gripper_key = _first_existing(
            keys,
            f"robot{index}_gripper_width",
            "gripper_width" if index == 0 else None,
        )
        _validate_xyz_array(data[position_key], position_key, total_frames)
        _validate_xyz_array(data[rotation_key], rotation_key, total_frames)
        if gripper_key is not None:
            gripper_shape = tuple(int(x) for x in data[gripper_key].shape)
            if gripper_shape[0] != total_frames or len(gripper_shape) not in (1, 2):
                raise UmiError(
                    f"{gripper_key} has shape {gripper_shape}; "
                    "expected (T,) or (T,1)")
            if len(gripper_shape) == 2 and gripper_shape[1] != 1:
                raise UmiError(
                    f"{gripper_key} has shape {gripper_shape}; "
                    "expected one width column")
        names = [
            f"robot{index}_eef_{axis}"
            for axis in ("x", "y", "z", "rx", "ry", "rz")
        ]
        if gripper_key is not None:
            names.append(f"robot{index}_gripper_width")
        robots.append(RobotSpec(
            index=index,
            eef_pos_key=position_key,
            eef_rot_key=rotation_key,
            gripper_key=gripper_key,
            state_names=tuple(names),
        ))
    return robots


def _discover_cameras(
    data,
    keys: Sequence[str],
    total_frames: int,
    requested: Sequence[str] | None,
    include_mirror: bool,
) -> list[CameraSpec]:
    if requested:
        selected = list(dict.fromkeys(requested))
        missing = [key for key in selected if key not in keys]
        if missing:
            raise UmiError(f"requested cameras are missing: {missing}")
    else:
        selected = []
        for key in keys:
            match = CAMERA_RE.match(key)
            if match is None:
                continue
            if match.group("mirror") and not include_mirror:
                continue
            selected.append(key)

    if not selected:
        visual_keys = [
            key for key in keys
            if len(getattr(data[key], "shape", ())) == 4
            and int(data[key].shape[-1]) in (1, 3, 4)
        ]
        raise UmiError(
            "no UMI camera array found; expected data/camera0_rgb. "
            f"Visual candidates: {visual_keys or 'none'}")

    selected.sort(key=lambda key: (
        int(CAMERA_RE.match(key).group("index")) if CAMERA_RE.match(key) else 10**9,
        key.endswith("_mirror_crop"),
        key,
    ))

    cameras: list[CameraSpec] = []
    features: set[str] = set()
    for key in selected:
        shape = tuple(int(x) for x in data[key].shape)
        if len(shape) != 4 or shape[0] != total_frames:
            raise UmiError(
                f"{key} has shape {shape}; expected (T, H, W, C)")
        if shape[-1] not in (1, 3, 4):
            raise UmiError(
                f"{key} has {shape[-1]} channels; expected 1, 3, or 4")
        if shape[1] < 2 or shape[2] < 2:
            raise UmiError(f"{key} has invalid image size {shape[1:3]}")
        feature = (
            "observation.images.camera"
            if len(selected) == 1
            else camera_feature_name(key)
        )
        if feature in features:
            raise UmiError(f"duplicate LeRobot camera feature {feature}")
        features.add(feature)
        cameras.append(CameraSpec(
            key=key,
            feature=feature,
            shape=(shape[1], shape[2], 3),
        ))
    return cameras


def _validate_xyz_array(array, key: str, total_frames: int) -> None:
    shape = tuple(int(x) for x in array.shape)
    if shape != (total_frames, 3):
        raise UmiError(f"{key} has shape {shape}; expected {(total_frames, 3)}")


def _first_existing(keys: Sequence[str], *candidates: str | None) -> str | None:
    for candidate in candidates:
        if candidate is not None and candidate in keys:
            return candidate
    return None


def _merged_attrs(root, meta) -> dict[str, Any]:
    attrs: dict[str, Any] = {}
    for node in (root, meta):
        try:
            attrs.update(dict(node.attrs))
        except Exception:
            pass
    return {str(key): _jsonable(value) for key, value in attrs.items()}


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (list, tuple, dict, str, int, float, bool)) or value is None:
        return value
    return str(value)


def _resolve_fps(attrs: Mapping[str, Any], meta, requested: int | None) -> int | None:
    if requested is not None:
        if requested <= 0:
            raise UmiError("--fps must be a positive integer")
        return int(requested)
    for key in ("fps", "frame_rate", "video_fps"):
        if key in attrs:
            try:
                value = int(round(float(attrs[key])))
            except (TypeError, ValueError):
                continue
            if value > 0:
                return value
    for key in ("fps", "frame_rate"):
        if key in meta:
            try:
                array = np.asarray(meta[key][:]).reshape(-1)
                if array.size:
                    value = int(round(float(array[0])))
                    if value > 0:
                        return value
            except Exception:
                pass
    return None


def _resolve_task(
    attrs: Mapping[str, Any], meta, requested: str | None) -> str | None:
    if requested is not None:
        task = requested.strip()
        if not task:
            raise UmiError("--task must not be empty")
        return task
    for key in ("task", "language_instruction", "instruction", "text"):
        value = attrs.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    for key in ("task", "language_instruction"):
        if key in meta:
            try:
                array = np.asarray(meta[key][:]).reshape(-1)
                if array.size:
                    value = str(array[0]).strip()
                    if value:
                        return value
            except Exception:
                pass
    return None


def summarize(
    info: UmiInfo,
    resize: tuple[int, int] | None = None,
    action_names: Sequence[str] = ACTION_NAMES,
    camera_features: Mapping[str, str] | None = None,
) -> str:
    lines = [
        f"source: {info.source}",
        f"episodes: {info.episode_count}",
        f"frames: {info.total_frames}",
        f"fps: {info.fps if info.fps is not None else '(not set; pass --fps)'}",
        f"task: {info.task if info.task else '(not set; pass --task)'}",
        f"state_dim: {info.state_dim}",
        f"action: {list(action_names)}",
    ]
    for robot in info.robots:
        gripper = robot.gripper_key or "no gripper"
        lines.append(
            f"robot{robot.index}: {robot.eef_pos_key}, "
            f"{robot.eef_rot_key}, {gripper}")
    for camera in info.cameras:
        height, width = camera.shape[0], camera.shape[1]
        if resize is not None:
            width, height = resize
        feature = (
            camera_features.get(camera.key, camera.feature)
            if camera_features is not None
            else camera.feature
        )
        lines.append(
            f"camera {camera.key}: source {camera.shape[1]}x{camera.shape[0]}, "
            f"output {width}x{height} -> {feature}")
    lines.append("episode ranges: " + ", ".join(
        f"{index}:{start}-{end}"
        for index, (start, end) in enumerate(info.episode_ranges)))
    return "\n".join(lines)
