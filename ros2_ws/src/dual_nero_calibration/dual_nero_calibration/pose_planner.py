"""Pick calibration arm poses that keep the chessboard in view.

The planner works purely in joint space: it samples offsets around the pose
where the board was first seen, rejects the ones that would push the board
out of the image or into a joint limit, and then keeps the most diverse
orientations.  Everything here is pure numpy so it can be tested without ROS.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .frame_selection import select_diverse_indices


@dataclass(frozen=True)
class Candidate:
    joints: np.ndarray
    T_base_tool: np.ndarray
    view_score: float


@dataclass(frozen=True)
class ViewLimits:
    margin_px: float = 45.0
    min_range_m: float = 0.18
    max_range_m: float = 0.95
    max_tilt_deg: float = 55.0
    min_gap_px: float = 8.0
    max_tilt_between_poses_deg: float = 75.0


def board_corners_in_base(T_base_board: np.ndarray,
                          object_points: np.ndarray) -> np.ndarray:
    points = np.asarray(object_points, dtype=np.float64)
    return points @ T_base_board[:3, :3].T + T_base_board[:3, 3]


def project(points_base: np.ndarray, T_base_camera: np.ndarray,
            camera_matrix: np.ndarray) -> np.ndarray:
    camera = points_base @ T_base_camera[:3, :3].T + T_base_camera[:3, 3]
    depth = camera[:, 2]
    normalized = camera[:, :2] / np.where(
        np.abs(depth)[:, None] < 1e-9, 1e-9, depth[:, None])
    pixels = normalized @ camera_matrix[:2, :2].T + camera_matrix[:2, 2]
    return np.column_stack([pixels, depth])


def min_gap_px(pixels: np.ndarray, pattern_size) -> float:
    grid = pixels.reshape(int(pattern_size[1]), int(pattern_size[0]), -1)
    horizontal = np.linalg.norm(np.diff(grid, axis=1)[..., :2], axis=-1)
    vertical = np.linalg.norm(np.diff(grid, axis=0)[..., :2], axis=-1)
    gaps = np.concatenate([horizontal.ravel(), vertical.ravel()])
    return float(gaps.min()) if gaps.size else 0.0


def view_ok(T_base_camera: np.ndarray, corners_base: np.ndarray,
            camera_matrix: np.ndarray, image_size, pattern_size,
            limits: ViewLimits = ViewLimits()) -> tuple[bool, str, float]:
    width, height = float(image_size[0]), float(image_size[1])
    pixels = project(corners_base, T_base_camera, camera_matrix)
    depth = pixels[:, 2]
    if depth.min() < limits.min_range_m:
        return False, 'too close', 0.0
    if depth.max() > limits.max_range_m:
        return False, 'too far', 0.0
    u = pixels[:, 0]
    v = pixels[:, 1]
    margin = limits.margin_px
    if (u.min() < margin or u.max() > width - margin
            or v.min() < margin or v.max() > height - margin):
        return False, 'outside image', 0.0
    gap = min_gap_px(pixels, pattern_size)
    if gap < limits.min_gap_px:
        return False, f'corners too close ({gap:.1f} px)', 0.0
    normal_base = corners_base_to_normal(corners_base)
    normal_camera = T_base_camera[:3, :3].T @ normal_base
    tilt = np.degrees(np.arccos(np.clip(abs(normal_camera[2]), -1.0, 1.0)))
    if tilt > limits.max_tilt_deg:
        return False, f'tilted {tilt:.0f} deg', 0.0
    centre = pixels[:, :2].mean(axis=0)
    offset = np.hypot(centre[0] - width / 2.0, centre[1] - height / 2.0)
    score = 1.0 - 0.5 * offset / np.hypot(width, height) - 0.5 * tilt / limits.max_tilt_deg
    return True, 'ok', float(score)


def corners_base_to_normal(corners_base: np.ndarray) -> np.ndarray:
    points = np.asarray(corners_base, dtype=np.float64)
    centre = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - centre, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    return normal / np.linalg.norm(normal)


def generate_candidates(*, chain, q_seed: np.ndarray, T_tool_camera: np.ndarray,
                        T_base_board: np.ndarray, object_points: np.ndarray,
                        camera_matrix: np.ndarray, image_size, pattern_size,
                        count: int, seed: int = 0, max_step_deg: float = 22.0,
                        limit_margin_deg: float = 4.0, attempts: int = 6000,
                        limits: ViewLimits = ViewLimits()) -> list[Candidate]:
    """Sample joint offsets and keep the ones that still see the board."""
    rng = np.random.default_rng(seed)
    q_seed = np.asarray(q_seed, dtype=np.float64).reshape(-1)
    lower, upper = chain.limits()
    margin = np.radians(limit_margin_deg)
    max_step = np.radians(max_step_deg)
    corners = board_corners_in_base(T_base_board, object_points)
    seed_rotation = (chain.forward(q_seed)[:3, :3])
    kept: list[Candidate] = []
    for _ in range(attempts):
        if len(kept) >= count * 6:
            break
        delta = rng.uniform(-max_step, max_step, size=len(q_seed))
        # Clip into the usable range instead of rejecting: next to a joint
        # limit most uniform samples would otherwise be thrown away.
        joints = np.clip(q_seed + delta, lower + margin, upper - margin)
        pose = chain.forward(joints)
        # Reject poses that flip the arm into a very different orientation,
        # which is where unreachable or self-colliding configurations live.
        delta_rotation = seed_rotation.T @ pose[:3, :3]
        angle = np.degrees(np.linalg.norm(Rotation.from_matrix(delta_rotation).as_rotvec()))
        if angle > limits.max_tilt_between_poses_deg:
            continue
        camera = pose @ T_tool_camera
        ok, _, score = view_ok(camera, corners, camera_matrix, image_size,
                               pattern_size, limits)
        if ok:
            kept.append(Candidate(joints=joints, T_base_tool=pose,
                                  view_score=score))
    return select_candidates(kept, count)


def select_candidates(candidates: list[Candidate], count: int) -> list[Candidate]:
    """Farthest-point selection on tool orientation and position."""
    if not candidates:
        return []
    rotations = np.array([c.T_base_tool[:3, :3] for c in candidates])
    translations = np.array([c.T_base_tool[:3, 3] for c in candidates])
    qualities = -np.array([c.view_score for c in candidates])
    indices = select_diverse_indices(rotations, translations, qualities,
                                     count=count)
    return [candidates[index] for index in indices]
