"""RANSAC-style consensus hand-eye solve.

Recording more frames and keeping "the ones with the smallest residual" biases
the answer towards whatever pose cluster happens to dominate: the fit then
looks good while the extrinsic is wrong.  This module instead looks for the
largest set of frames that agree on *one* rigid extrinsic and reports how much
of the session that set covers, plus how much pose diversity it contains.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from .common import average_transforms, invert_transform, transform_residual
from .frame_selection import pose_rotation_spread_deg


@dataclass
class RobustResult:
    transform: np.ndarray
    inliers: list[int]
    translation_rms_m: float
    rotation_rms_deg: float
    spread_deg: float
    trials: int
    competing: int = 0
    history: list[float] = field(default_factory=list)
    total: int = 0

    @property
    def inlier_fraction(self) -> float:
        return 0.0 if not self.total else len(self.inliers) / self.total


def solve_optical(samples, camera_cfg: dict, method: str = 'PARK') -> np.ndarray:
    """``T_tool_camera`` in the optical frame, which is what residuals need."""
    # Imported here: solve_extrinsics imports this module for its CLI flag.
    from .solve_extrinsics import solve_eye_in_hand

    _, _, transform_link, _, _ = solve_eye_in_hand(
        samples, camera_cfg, method=method)
    link_optical = average_transforms(
        [sample['T_camera_link_optical'] for sample in samples])
    return transform_link @ link_optical


def board_pose(samples, tool_camera: np.ndarray) -> np.ndarray:
    """Mean board pose predicted by one tool-to-camera transform."""
    return average_transforms([
        sample['T_base_tool'] @ tool_camera @ sample['T_cam_board']
        for sample in samples
    ])


def board_residuals(samples, tool_camera: np.ndarray,
                    reference: np.ndarray | None = None) -> np.ndarray:
    """Per-frame ``(translation_mm, rotation_deg)`` against the mean board.

    Returns an ``(n, 2)`` array.  Pass ``reference`` to score frames against a
    board pose estimated from the RANSAC subset.  Re-estimating the mean from
    all frames (the default) lets outliers drag the reference and can hide an
    otherwise obvious consistent subset.
    """
    if reference is None:
        reference = board_pose(samples, tool_camera)
    inverse = invert_transform(reference)
    rows = []
    for sample in samples:
        pose = (sample['T_base_tool'] @ tool_camera
                @ sample['T_cam_board'])
        delta = inverse @ pose
        angle = np.degrees(np.linalg.norm(
            Rotation.from_matrix(delta[:3, :3]).as_rotvec()))
        rows.append([float(np.linalg.norm(delta[:3, 3])) * 1000.0, float(angle)])
    return np.asarray(rows)


def _inlier_mask(residuals: np.ndarray, max_mm: float,
                 max_deg: float) -> np.ndarray:
    return (residuals[:, 0] <= max_mm) & (residuals[:, 1] <= max_deg)


def ransac_eye_in_hand(samples, camera_cfg: dict, *, minimum: int = 8,
                       iterations: int = 300, max_mm: float = 10.0,
                       max_deg: float = 1.0, refine_rounds: int = 5,
                       seed: int = 0, method: str = 'PARK') -> RobustResult:
    """Find the largest self-consistent subset of frames.

    ``minimum`` frames are drawn per trial; a trial is scored by how many
    frames of the whole session agree with it.  The winning model is then
    refined on its inliers.
    """
    total = len(samples)
    if total < minimum:
        raise RuntimeError(
            f'robust solve needs at least {minimum} samples, got {total}')
    rng = np.random.default_rng(seed)
    best_mask = None
    best_score = (-1.0, 0.0)
    for _ in range(int(iterations)):
        picked = rng.choice(total, size=minimum, replace=False)
        try:
            transform = solve_optical(
                [samples[int(index)] for index in picked],
                camera_cfg, method=method)
        except Exception:  # noqa: BLE001 - degenerate draws are expected
            continue
        picked_samples = [samples[int(index)] for index in picked]
        reference = board_pose(picked_samples, transform)
        residuals = board_residuals(samples, transform, reference)
        mask = _inlier_mask(residuals, max_mm, max_deg)
        count = int(mask.sum())
        if count < minimum:
            continue
        score = (float(count), -float(residuals[mask, 1].mean())
                 if count else -np.inf)
        if score > best_score:
            best_score = score
            best_mask = mask

    if best_mask is None:
        residuals = board_residuals(samples, solve_optical(samples, camera_cfg))
        best_mask = _inlier_mask(residuals, max_mm, max_deg)
    if int(best_mask.sum()) < minimum:
        raise RuntimeError(
            f'no consistent subset: fewer than {minimum} frames agree within '
            f'{max_mm:.1f} mm / {max_deg:.2f} deg. The session is internally '
            'inconsistent; fix the mechanics instead of filtering frames')

    mask = best_mask
    history = []
    inlier_samples = [sample for sample, keep in zip(samples, mask) if keep]
    transform = solve_optical(inlier_samples, camera_cfg, method=method)
    for _ in range(int(refine_rounds)):
        reference = board_pose(inlier_samples, transform)
        residuals = board_residuals(samples, transform, reference)
        updated = _inlier_mask(residuals, max_mm, max_deg)
        if updated.sum() < minimum:
            break
        history.append(float(updated.sum()))
        if np.array_equal(updated, mask):
            break
        mask = updated
        inlier_samples = [
            sample for sample, keep in zip(samples, mask) if keep]
        transform = solve_optical(inlier_samples, camera_cfg, method=method)

    inliers = [index for index, keep in enumerate(mask) if keep]
    subset = [samples[index] for index in inliers]
    residuals = board_residuals(subset, transform, board_pose(subset, transform))
    translation_rms = float(np.sqrt(np.mean(residuals[:, 0] ** 2))) / 1000.0
    rotation_rms = float(np.sqrt(np.mean(residuals[:, 1] ** 2)))
    result = RobustResult(
        transform=transform,
        inliers=inliers,
        translation_rms_m=translation_rms,
        rotation_rms_deg=rotation_rms,
        spread_deg=pose_rotation_spread_deg(
            [sample['T_base_tool'] for sample in subset]),
        trials=int(iterations),
        history=history,
        total=total)
    result.competing = _competing_models(
        samples, camera_cfg, mask, max_mm, max_deg, minimum, method)
    return result


def _competing_models(samples, camera_cfg, mask, max_mm, max_deg, minimum,
                      method) -> int:
    """Count other transforms that still explain a large slice of the data."""
    remainder = [index for index, keep in enumerate(mask) if not keep]
    if len(remainder) < minimum:
        return 0
    try:
        other = solve_optical([samples[index] for index in remainder],
                              camera_cfg, method=method)
    except Exception:  # noqa: BLE001
        return 0
    residuals = board_residuals([samples[index] for index in remainder], other)
    return int(_inlier_mask(residuals, max_mm, max_deg).sum())
