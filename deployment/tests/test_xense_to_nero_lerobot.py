from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pyarrow as pa
import pytest

DEPLOYMENT_DIR = Path(__file__).resolve().parents[1]
if str(DEPLOYMENT_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOYMENT_DIR))

from tools.xense_to_nero_lerobot import (  # noqa: E402
    NERO_NAMES,
    SOURCE_NAMES,
    XenseError,
    _episode_column_map,
    _state_and_action,
    _validate_arrays,
    build_output_features,
    validate_source_info,
)


def _source_info() -> dict:
    vector = {
        "dtype": "float32",
        "shape": [29],
        "names": [*SOURCE_NAMES, *(f"head.{name}" for name in (
            "x", "y", "z", "r1", "r2", "r3", "r4", "r5", "r6"))],
    }
    camera = {
        "dtype": "video",
        "shape": [480, 640, 3],
        "names": ["height", "width", "channels"],
    }
    return {
        "codebase_version": "v3.0",
        "robot_type": "xtac_umi_g1",
        "fps": 30,
        "features": {
            "action": vector,
            "observation.state": vector,
            "observation.images.head_left": camera,
            "observation.images.left_wrist": camera,
            "observation.images.right_wrist": camera,
            "timestamp": {"dtype": "float32", "shape": [1], "names": None},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None},
            "index": {"dtype": "int64", "shape": [1], "names": None},
            "task_index": {"dtype": "int64", "shape": [1], "names": None},
        },
    }


def _state_and_action_arrays(frames: int = 5) -> tuple[np.ndarray, np.ndarray]:
    state = np.zeros((frames, 29), dtype=np.float32)
    for offset in (0, 10):
        state[:, offset + 3] = 1.0
        state[:, offset + 4] = 0.0
        state[:, offset + 5] = 0.0
        state[:, offset + 6] = 0.0
        state[:, offset + 7] = 1.0
        state[:, offset + 8] = 0.0
        state[:, offset + 9] = 0.5
    action = np.concatenate([state[1:], state[-1:]], axis=0)
    return state, action


def test_source_info_and_output_features() -> None:
    info = _source_info()
    camera_map = {
        "observation.images.head_left": "observation.images.side",
        "observation.images.left_wrist": "observation.images.left_wrist",
        "observation.images.right_wrist": "observation.images.right_wrist",
    }
    validate_source_info(info, camera_map)
    features = build_output_features(info, camera_map)
    assert features["action"]["shape"] == [20]
    assert features["action"]["names"] == list(NERO_NAMES)
    assert features["observation.state"]["names"] == list(NERO_NAMES)
    assert "observation.images.side" in features
    assert "observation.images.head_left" not in features


def test_episode_columns_are_renamed_and_filtered() -> None:
    source_columns = [
        "episode_index",
        "tasks",
        "length",
        "data/chunk_index",
        "data/file_index",
        "dataset_from_index",
        "dataset_to_index",
        "videos/observation.images.head_left/chunk_index",
        "videos/observation.images.head_left/file_index",
        "videos/observation.images.head_left/from_timestamp",
        "videos/observation.images.head_left/to_timestamp",
        "videos/observation.images.left_wrist/chunk_index",
        "videos/observation.images.left_wrist/file_index",
        "videos/observation.images.left_wrist/from_timestamp",
        "videos/observation.images.left_wrist/to_timestamp",
        "videos/observation.images.right_wrist/chunk_index",
        "videos/observation.images.right_wrist/file_index",
        "videos/observation.images.right_wrist/from_timestamp",
        "videos/observation.images.right_wrist/to_timestamp",
        "videos/observation.images.right_tactile_left/chunk_index",
        "meta/episodes/chunk_index",
        "meta/episodes/file_index",
    ]
    camera_map = {
        "observation.images.head_left": "observation.images.side",
        "observation.images.left_wrist": "observation.images.left_wrist",
        "observation.images.right_wrist": "observation.images.right_wrist",
    }
    mapping = _episode_column_map(source_columns, camera_map)
    assert "videos/observation.images.side/chunk_index" in mapping.values()
    assert all("tactile" not in column for column in mapping)


def test_validate_arrays_rejects_broken_action_continuity() -> None:
    state, action = _state_and_action_arrays()
    episodes = np.zeros(len(state), dtype=np.int64)
    _validate_arrays(state, action, episodes)
    action[0, 0] += 0.1
    with pytest.raises(XenseError, match="action\\[t\\]"):
        _validate_arrays(state, action, episodes)


def test_state_and_action_read_from_list_columns() -> None:
    state, action = _state_and_action_arrays()
    table = pa.table({
        "observation.state": pa.array(state.tolist(), type=pa.list_(pa.float32())),
        "action": pa.array(action.tolist(), type=pa.list_(pa.float32())),
    })
    loaded_state, loaded_action = _state_and_action(table)
    np.testing.assert_allclose(loaded_state, state)
    np.testing.assert_allclose(loaded_action, action)


def test_source_info_rejects_wrong_dimension() -> None:
    info = _source_info()
    info["features"]["action"]["shape"] = [20]
    with pytest.raises(XenseError, match="expected \\[29\\]"):
        validate_source_info(info, {
            "observation.images.head_left": "observation.images.side",
            "observation.images.left_wrist": "observation.images.left_wrist",
            "observation.images.right_wrist": "observation.images.right_wrist",
        })
