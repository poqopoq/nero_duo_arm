"""Shared on-disk layout for calibration capture sessions."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import yaml


def make_session_dir(root, camera: str, prefix: str) -> Path:
    directory = Path(root) / camera / f'{prefix}_{time.strftime("%Y%m%d_%H%M%S")}'
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def save_sample(directory, index: int, *, camera_matrix, distortion,
                image_size, object_points, image_points, T_cam_board,
                T_base_tool, T_camera_link_optical, reprojection_rms_px,
                stamp_sec: int, stamp_nanosec: int) -> Path:
    path = Path(directory) / f'sample_{index:04d}.npz'
    np.savez(
        path,
        camera_matrix=camera_matrix,
        distortion=distortion,
        image_size=np.asarray(image_size, dtype=np.int32),
        object_points=object_points,
        image_points=image_points,
        T_cam_board=T_cam_board,
        T_base_tool=T_base_tool,
        T_camera_link_optical=T_camera_link_optical,
        reprojection_rms_px=reprojection_rms_px,
        stamp_sec=int(stamp_sec),
        stamp_nanosec=int(stamp_nanosec),
    )
    return path


def write_metadata(directory, *, camera: str, camera_cfg: dict,
                   board_cfg: dict, samples: list[dict],
                   capture_mode: str) -> Path:
    data = {
        'camera': camera,
        'mode': camera_cfg['mode'],
        'capture_mode': capture_mode,
        'image_topic': camera_cfg['image_topic'],
        'info_topic': camera_cfg['info_topic'],
        'camera_link_frame': camera_cfg['camera_link_frame'],
        'optical_frame': camera_cfg['optical_frame'],
        'parent_frame': camera_cfg['parent_frame'],
        'base_frame': camera_cfg.get('base_frame'),
        'tool_frame': camera_cfg.get('tool_frame'),
        'board': board_cfg,
        'samples': samples,
    }
    path = Path(directory) / 'metadata.yaml'
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
    return path
