#!/usr/bin/env python3
"""Solve fixed-camera or eye-in-hand camera extrinsics from ChArUco samples."""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from .common import (
    average_transforms,
    invert_transform,
    load_yaml,
    transform_from_xyz_rpy,
    transform_residual,
    write_extrinsic_yaml,
)


def load_samples(session: Path):
    files = sorted(session.glob('sample_*.npz'))
    if not files:
        raise RuntimeError(f'No sample_*.npz files in {session}')
    return [np.load(path) for path in files]


def reprojection_rms(samples) -> float:
    errors = []
    for sample in samples:
        ok, rvec, tvec = cv2.solvePnP(
            sample['object_points'], sample['image_points'],
            sample['camera_matrix'], sample['distortion'],
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        projected, _ = cv2.projectPoints(
            sample['object_points'], rvec, tvec,
            sample['camera_matrix'], sample['distortion'])
        residual = projected.reshape(-1, 2) - sample['image_points']
        errors.append(float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))
    if not errors:
        return float('nan')
    return float(np.sqrt(np.mean(np.square(errors))))


def solve_fixed(samples, camera_cfg):
    pose = camera_cfg['board_pose_in_world']
    T_world_board = transform_from_xyz_rpy(pose['xyz'], pose['rpy'])
    camera_transforms = []
    T_link_optical = None
    for sample in samples:
        T_cam_board = sample['T_cam_board']
        T_world_cam = T_world_board @ invert_transform(T_cam_board)
        camera_transforms.append(T_world_cam)
        T_link_optical = sample['T_camera_link_optical']
    T_world_cam = average_transforms(camera_transforms)
    T_world_cam_link = T_world_cam @ invert_transform(T_link_optical)
    trans_rms, rot_rms = transform_residual(camera_transforms, T_world_cam)
    return camera_cfg['parent_frame'], camera_cfg['camera_link_frame'], T_world_cam_link, trans_rms, rot_rms


def solve_eye_in_hand(samples, camera_cfg):
    R_gripper2base = [s['T_base_tool'][:3, :3] for s in samples]
    t_gripper2base = [s['T_base_tool'][:3, 3] for s in samples]
    R_target2cam = [s['T_cam_board'][:3, :3] for s in samples]
    t_target2cam = [s['T_cam_board'][:3, 3] for s in samples]
    if len(samples) < 3:
        raise RuntimeError('Eye-in-hand calibration needs at least 3 samples')

    R_tool_cam, t_tool_cam = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_PARK)
    # PARK returns the tool->camera transform for the input convention used here.
    T_tool_cam_optical = np.eye(4)
    T_tool_cam_optical[:3, :3] = R_tool_cam
    T_tool_cam_optical[:3, 3] = t_tool_cam.reshape(3)

    T_link_optical = average_transforms([s['T_camera_link_optical'] for s in samples])
    T_tool_cam_link = T_tool_cam_optical @ invert_transform(T_link_optical)

    board_poses = []
    for sample in samples:
        T_base_cam = sample['T_base_tool'] @ T_tool_cam_optical
        board_poses.append(T_base_cam @ sample['T_cam_board'])
    mean_board = average_transforms(board_poses)
    trans_rms, rot_rms = transform_residual(board_poses, mean_board)
    return camera_cfg['parent_frame'], camera_cfg['camera_link_frame'], T_tool_cam_link, trans_rms, rot_rms


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', required=True,
                        choices=('scene', 'left_wrist', 'right_wrist'))
    parser.add_argument('--session', required=True,
                        help='Directory containing sample_*.npz and metadata.yaml')
    parser.add_argument('--output', default=None)
    args = parser.parse_args()

    share = Path(get_package_share_directory('dual_nero_calibration'))
    targets = load_yaml(share / 'config' / 'calibration_targets.yaml')['cameras']
    camera_cfg = targets[args.camera]
    session = Path(args.session).expanduser()
    samples = load_samples(session)

    if camera_cfg['mode'] == 'eye_in_hand':
        parent, child, transform, trans_rms, rot_rms = solve_eye_in_hand(
            samples, camera_cfg)
    elif camera_cfg['mode'] == 'fixed':
        parent, child, transform, trans_rms, rot_rms = solve_fixed(
            samples, camera_cfg)
    else:
        raise RuntimeError(f"Unsupported mode: {camera_cfg['mode']}")

    output = Path(args.output or
                  Path('/home/a/workspace/dual_nero_deployment/calibration_results') /
                  f'{args.camera}_extrinsic.yaml')
    write_extrinsic_yaml(
        output, args.camera, parent, child, transform,
        len(samples), trans_rms, rot_rms)

    q = Rotation.from_matrix(transform[:3, :3]).as_quat()
    print(f'OUTPUT {output}')
    print(f'PARENT {parent}')
    print(f'CHILD {child}')
    print('TRANSLATION', [round(float(v), 9) for v in transform[:3, 3]])
    print('QUATERNION_XYZW', [round(float(v), 9) for v in q])
    print('TRANSLATION_RMS_M', round(trans_rms, 9))
    print('ROTATION_RMS_DEG', round(rot_rms, 6))
    print('REPROJECTION_RMS_PX', round(reprojection_rms(samples), 6))


if __name__ == '__main__':
    main()
