#!/usr/bin/env python3
"""Solve fixed-camera or eye-in-hand camera extrinsics from ChArUco samples."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from .common import (
    average_transforms,
    default_deployment_dir,
    invert_transform,
    load_yaml,
    transform_from_xyz_rpy,
    transform_residual,
    write_extrinsic_yaml,
)
from .robust_extrinsic import ransac_eye_in_hand


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


def max_rotation_spread_deg(rotations) -> float:
    """Return the largest pairwise rotation difference in a sample set."""
    matrices = [np.asarray(rotation, dtype=np.float64) for rotation in rotations]
    if len(matrices) < 2:
        return 0.0
    spread = 0.0
    for index, first in enumerate(matrices):
        for second in matrices[index + 1:]:
            delta = first.T @ second
            angle = np.linalg.norm(Rotation.from_matrix(delta).as_rotvec())
            spread = max(spread, float(np.degrees(angle)))
    return spread


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


def solve_eye_in_hand(samples, camera_cfg, method: str = 'PARK'):
    R_gripper2base = [s['T_base_tool'][:3, :3] for s in samples]
    t_gripper2base = [s['T_base_tool'][:3, 3] for s in samples]
    R_target2cam = [s['T_cam_board'][:3, :3] for s in samples]
    t_target2cam = [s['T_cam_board'][:3, 3] for s in samples]
    if len(samples) < 3:
        raise RuntimeError('Eye-in-hand calibration needs at least 3 samples')

    method_id = getattr(cv2, f'CALIB_HAND_EYE_{method}')
    R_tool_cam, t_tool_cam = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam,
        method=method_id)
    # OpenCV calls the result ``cam2gripper``: it maps camera points into the
    # gripper frame.  That is ``T_tool_camera``, not the inverse.
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


def format_report(samples, tool_camera: np.ndarray) -> str:
    """Per-sample residual table plus same-pose repeatability groups.

    The per-sample table shows where the inconsistency lives: when the
    translation residual tracks ``angle * board_distance`` the error is a
    pose-dependent *rotation* error of the kinematic chain, not a scale or
    corner-detection problem.
    """
    board_poses = [
        sample['T_base_tool'] @ tool_camera @ sample['T_cam_board']
        for sample in samples
    ]
    mean_board = average_transforms(board_poses)
    inv_mean = invert_transform(mean_board)
    lines = [
        f"{'#':>3} {'|t| mm':>8} {'rot deg':>8} {'dist m':>7} "
        f"{'angle*d mm':>11} {'ratio':>6}"
    ]
    ratios = []
    for index, (pose, sample) in enumerate(zip(board_poses, samples)):
        angle, translation = _delta_pose(inv_mean @ pose, radians=False)
        distance = float(np.linalg.norm(sample['T_cam_board'][:3, 3]))
        predicted = np.radians(angle) * distance * 1000.0
        ratio = translation / predicted if predicted > 1e-9 else float('nan')
        ratios.append(ratio)
        lines.append(
            f'{index:3d} {translation:8.2f} {angle:8.3f} '
            f'{distance:7.3f} {predicted:11.2f} {ratio:6.2f}')
    finite = [value for value in ratios if np.isfinite(value)]
    if finite:
        lines.append(
            'ratio = translation residual / (rotation residual x board '
            'distance); near 1 means a pure orientation error: '
            f'median {float(np.median(finite)):.2f}')
    lines.extend(_pose_groups(samples))
    return '\n'.join(lines)


def _delta_pose(delta: np.ndarray, radians: bool = False):
    angle = np.arccos(np.clip((np.trace(delta[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))
    if not radians:
        angle = np.degrees(angle)
    return float(angle), float(np.linalg.norm(delta[:3, 3])) * 1000.0


def _pose_groups(samples, translation_mm: float = 2.0,
                 rotation_deg: float = 0.3) -> list[str]:
    """Group samples that report the same tool pose.

    Recording the same arm pose several times, each approached from a
    different direction, separates robot positioning error from camera-mount
    flex: a tool-pose difference means the arm is the problem, while a
    ``T_cam_board`` difference with matching tool poses points at the mount.
    """
    groups: list[list[int]] = []
    for index, sample in enumerate(samples):
        pose = sample['T_base_tool']
        for group in groups:
            reference = samples[group[0]]['T_base_tool']
            angle, distance = _delta_pose(invert_transform(reference) @ pose)
            if distance <= translation_mm and angle <= rotation_deg:
                group.append(index)
                break
        else:
            groups.append([index])
    repeated = [group for group in groups if len(group) > 1]
    if not repeated:
        return ['', 'same-pose repeatability: no repeated tool pose found '
                    '(record the same pose 2-3 times to run this check)']
    lines = ['', 'same-pose repeatability:']
    for group in repeated:
        tool = [samples[i]['T_base_tool'] for i in group]
        board = [samples[i]['T_cam_board'] for i in group]
        tool_trans, tool_rot = transform_residual(tool, average_transforms(tool))
        board_trans, board_rot = transform_residual(
            board, average_transforms(board))
        lines.append(
            f'  samples {group}: T_base_tool {tool_trans:.3f} mm / '
            f'{tool_rot:.3f} deg, T_cam_board {board_trans:.3f} mm / '
            f'{board_rot:.3f} deg')
    return lines


def _format_vector(values) -> str:
    return '[' + ', '.join(f'{float(value):+.9f}' for value in values) + ']'


def _compare_sessions(samples, other_session: Path, camera_cfg: dict,
                      transform: np.ndarray, args) -> dict:
    """Measure how well two independent sessions agree on the extrinsic.

    A hand-eye solve on a rigid setup is a fixed physical quantity, so two
    sessions should agree far better than their own per-session residual.
    Disagreement of the same order as that residual means the pose error is
    systematic per session and the extrinsic is not trustworthy.
    """
    other = load_samples(other_session)
    if camera_cfg['mode'] == 'eye_in_hand':
        _, _, other_transform, other_trans, other_rot = solve_eye_in_hand(
            other, camera_cfg, method=args.method)
    else:
        _, _, other_transform, other_trans, other_rot = solve_fixed(
            other, camera_cfg)
    delta = invert_transform(transform) @ other_transform
    translation_mm = float(np.linalg.norm(delta[:3, 3])) * 1000.0
    rotation_deg = float(np.degrees(np.linalg.norm(
        Rotation.from_matrix(delta[:3, :3]).as_rotvec())))
    _, _, _, combined_trans, combined_rot = solve_eye_in_hand(
        samples + other, camera_cfg, method=args.method)
    return {
        'session': str(other_session),
        'translation_mm': translation_mm,
        'rotation_deg': rotation_deg,
        'other_translation_rms_mm': other_trans * 1000.0,
        'other_rotation_rms_deg': other_rot,
        'combined_translation_rms_mm': combined_trans * 1000.0,
        'combined_rotation_rms_deg': combined_rot,
        'ok': (translation_mm <= args.max_agreement_mm
               and rotation_deg <= args.max_agreement_deg),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', required=True,
                        choices=('scene', 'left_wrist', 'right_wrist'))
    parser.add_argument('--session', required=True,
                        help='Directory containing sample_*.npz and metadata.yaml')
    parser.add_argument('--output', default=None)
    parser.add_argument('--method', default='PARK',
                        choices=('PARK', 'TSAI', 'HORAUD', 'ANDREFF', 'DANIILIDIS'),
                        help='OpenCV hand-eye method (eye-in-hand only)')
    parser.add_argument('--min-samples', type=int, default=6,
                        help='Warn when fewer samples were captured')
    parser.add_argument('--max-translation-rms-mm', type=float, default=3.0)
    parser.add_argument('--max-rotation-rms-deg', type=float, default=0.5)
    parser.add_argument('--max-reprojection-rms-px', type=float, default=1.0)
    parser.add_argument('--max-agreement-mm', type=float, default=3.0)
    parser.add_argument('--max-agreement-deg', type=float, default=0.3)
    parser.add_argument('--robust', action='store_true',
                        help='Fit the largest self-consistent subset instead '
                             'of trusting every frame (use with extra samples)')
    parser.add_argument('--robust-min-samples', type=int, default=8)
    parser.add_argument('--robust-iterations', type=int, default=400)
    parser.add_argument('--robust-inlier-mm', type=float, default=10.0)
    parser.add_argument('--robust-inlier-deg', type=float, default=1.0)
    parser.add_argument('--robust-seed', type=int, default=0)
    parser.add_argument('--min-inlier-fraction', type=float, default=0.6,
                        help='Reject a robust result supported by fewer than '
                             'this share of the captured frames')
    parser.add_argument('--min-inlier-spread-deg', type=float, default=30.0,
                        help='Reject a robust result whose inliers cover less '
                             'rotation than this')
    parser.add_argument('--strict', action='store_true',
                        help='Return a non-zero exit code when a threshold fails')
    parser.add_argument('--report', action='store_true',
                        help='Print per-sample residuals and same-pose '
                             'repeatability to diagnose a failing calibration')
    parser.add_argument('--compare', default=None,
                        help='Second session directory. Reports how far two '
                             'independent sessions disagree, which is the '
                             'real repeatability check')
    args = parser.parse_args()

    share = Path(get_package_share_directory('dual_nero_calibration'))
    targets = load_yaml(share / 'config' / 'calibration_targets.yaml')['cameras']
    camera_cfg = targets[args.camera]
    session = Path(args.session).expanduser()
    samples = load_samples(session)

    if camera_cfg['mode'] == 'eye_in_hand':
        parent, child, transform, trans_rms, rot_rms = solve_eye_in_hand(
            samples, camera_cfg, method=args.method)
    elif camera_cfg['mode'] == 'fixed':
        parent, child, transform, trans_rms, rot_rms = solve_fixed(
            samples, camera_cfg)
    else:
        raise RuntimeError(f"Unsupported mode: {camera_cfg['mode']}")

    robust = None
    if args.robust:
        if camera_cfg['mode'] != 'eye_in_hand':
            raise RuntimeError('--robust is only implemented for eye-in-hand')
        try:
            robust = ransac_eye_in_hand(
                samples, camera_cfg, minimum=args.robust_min_samples,
                iterations=args.robust_iterations,
                max_mm=args.robust_inlier_mm, max_deg=args.robust_inlier_deg,
                seed=args.robust_seed, method=args.method)
        except RuntimeError as error:
            print(f'ERROR {error}')
            print('CALIBRATION FAIL')
            print('NOTE no extrinsic was written from this session')
            return 2
        link_optical = average_transforms(
            [sample['T_camera_link_optical'] for sample in samples])
        transform = robust.transform @ invert_transform(link_optical)
        trans_rms = robust.translation_rms_m
        rot_rms = robust.rotation_rms_deg

    explicit_output = args.output is not None
    output = Path(
        args.output or
        default_deployment_dir('calibration_results') /
        f'{args.camera}_extrinsic.yaml'
    )

    q = Rotation.from_matrix(transform[:3, :3]).as_quat()
    reprojection = reprojection_rms(samples)
    if camera_cfg['mode'] == 'eye_in_hand':
        spread_samples = (
            [samples[index] for index in robust.inliers]
            if robust is not None else samples)
        spread = max_rotation_spread_deg(
            [sample['T_base_tool'][:3, :3] for sample in spread_samples])
    else:
        spread = max_rotation_spread_deg(
            [sample['T_cam_board'][:3, :3] for sample in samples])

    checks = {
        'sample_count': len(samples) >= args.min_samples,
        'rotation_spread': spread >= 10.0,
        'translation_rms': trans_rms * 1000.0 <= args.max_translation_rms_mm,
        'rotation_rms': rot_rms <= args.max_rotation_rms_deg,
        'reprojection_rms': (
            np.isfinite(reprojection)
            and reprojection <= args.max_reprojection_rms_px),
    }
    agreement = None
    if robust is not None:
        checks['inlier_fraction'] = (
            robust.inlier_fraction >= args.min_inlier_fraction)
        checks['inlier_spread'] = (
            robust.spread_deg >= args.min_inlier_spread_deg)
    if args.compare:
        agreement = _compare_sessions(
            samples, Path(args.compare).expanduser(), camera_cfg, transform,
            args)
        checks['session_agreement'] = agreement['ok']
    passed = all(checks.values())
    # A failed solve must never overwrite a published extrinsic in place.
    redirected = not passed and not explicit_output
    if redirected:
        # Tag the warn file with the session so a failed re-run cannot
        # clobber the result of an earlier session.
        output = output.with_name(
            f'{output.stem}.{session.name}.warn{output.suffix}')
    failed = [name for name, ok in checks.items() if not ok]
    write_extrinsic_yaml(
        output, args.camera, parent, child, transform,
        len(samples), trans_rms, rot_rms,
        status='PASS' if passed else 'WARN', failed_checks=failed)

    print(f'OUTPUT {output}')
    print(f'PARENT {parent}')
    print(f'CHILD {child}')
    print('TRANSLATION', _format_vector(transform[:3, 3]))
    print('QUATERNION_XYZW', _format_vector(q))
    print('SAMPLES', len(samples))
    print('ROTATION_SPREAD_DEG', round(spread, 6))
    print('TRANSLATION_RMS_M', round(trans_rms, 9))
    print('ROTATION_RMS_DEG', round(rot_rms, 6))
    print('REPROJECTION_RMS_PX', round(reprojection, 6))
    for name, ok in checks.items():
        print(f'CHECK {name} {"PASS" if ok else "FAIL"}')
    if robust is not None:
        print(f'INLIERS {len(robust.inliers)}/{robust.total} '
              f'({robust.inlier_fraction * 100:.0f}%)')
        print('INLIER_FRAMES',
              ' '.join(str(index) for index in robust.inliers))
        print('INLIER_SPREAD_DEG', round(robust.spread_deg, 6))
        print('COMPETING_FRAMES', robust.competing)
    if agreement is not None:
        print(f'COMPARE {agreement["session"]}')
        print('COMPARE_OTHER_RMS_MM', round(
            agreement['other_translation_rms_mm'], 6))
        print('COMPARE_OTHER_RMS_DEG', round(
            agreement['other_rotation_rms_deg'], 6))
        print('COMPARE_TRANSLATION_DIFF_MM', round(
            agreement['translation_mm'], 6))
        print('COMPARE_ROTATION_DIFF_DEG', round(
            agreement['rotation_deg'], 6))
        print('COMPARE_COMBINED_RMS_MM', round(
            agreement['combined_translation_rms_mm'], 6))
        print('COMPARE_COMBINED_RMS_DEG', round(
            agreement['combined_rotation_rms_deg'], 6))
        print('TEST session_agreement',
              'PASS' if agreement['ok'] else 'FAIL')
    print('CALIBRATION', 'PASS' if passed else 'WARN')
    if not passed:
        print('FAILED_CHECKS', ' '.join(failed))
        if redirected:
            print('NOTE result written next to, not over, the published '
                  'extrinsic; fix the checks before publishing')
    if args.report:
        if camera_cfg['mode'] == 'eye_in_hand':
            link_optical = average_transforms(
                [sample['T_camera_link_optical'] for sample in samples])
            print(format_report(samples, transform @ link_optical))
        else:
            print('report is only available for eye-in-hand cameras')
    if args.strict and not passed:
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
