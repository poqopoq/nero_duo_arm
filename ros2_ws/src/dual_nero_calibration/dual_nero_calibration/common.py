from __future__ import annotations

import time
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def load_yaml(path: str | Path) -> dict:
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def transform_from_xyz_rpy(xyz: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', list(rpy)).as_matrix()
    transform[:3, 3] = list(xyz)
    return transform


def transform_from_msg(msg) -> np.ndarray:
    transform = np.eye(4)
    q = msg.rotation
    transform[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    transform[:3, 3] = [msg.translation.x, msg.translation.y, msg.translation.z]
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    rotation = transform[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ transform[:3, 3]
    return result


def average_transforms(transforms: list[np.ndarray]) -> np.ndarray:
    translations = np.array([t[:3, 3] for t in transforms])
    rotations = Rotation.from_matrix([t[:3, :3] for t in transforms])
    mean_rotation = rotations.mean().as_matrix()
    result = np.eye(4)
    result[:3, :3] = mean_rotation
    result[:3, 3] = translations.mean(axis=0)
    return result


def transform_residual(transforms: list[np.ndarray], reference: np.ndarray) -> tuple[float, float]:
    inv_ref = invert_transform(reference)
    trans_errors = []
    rot_errors = []
    for transform in transforms:
        delta = inv_ref @ transform
        trans_errors.append(float(np.linalg.norm(delta[:3, 3])))
        rot_errors.append(float(np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec())))
    return (
        float(np.sqrt(np.mean(np.square(trans_errors)))),
        float(np.degrees(np.sqrt(np.mean(np.square(rot_errors))))),
    )


def write_extrinsic_yaml(path: str | Path, name: str, parent_frame: str,
                         child_frame: str, transform: np.ndarray,
                         samples: int, translation_rms: float,
                         rotation_rms_deg: float) -> None:
    rotation = Rotation.from_matrix(transform[:3, :3])
    quat = rotation.as_quat()
    rpy = rotation.as_euler('xyz')
    data = {
        'extrinsic': {
            'name': name,
            'parent_frame': parent_frame,
            'child_frame': child_frame,
            'translation': [float(v) for v in transform[:3, 3]],
            'quaternion_xyzw': [float(v) for v in quat],
            'rpy_rad': [float(v) for v in rpy],
            'samples': int(samples),
            'translation_rms_m': float(translation_rms),
            'rotation_rms_deg': float(rotation_rms_deg),
            'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        }
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
