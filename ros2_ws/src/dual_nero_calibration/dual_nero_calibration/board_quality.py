"""Quality checks for detected chessboard corners."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


class DetectionError(RuntimeError):
    """Raised when a chessboard detection fails an acceptance check."""


@dataclass(frozen=True)
class ChessboardDetection:
    object_points: np.ndarray
    image_points: np.ndarray
    T_cam_board: np.ndarray
    reprojection_rms_px: float
    min_corner_gap_px: float


def min_adjacent_gap_px(image_points, pattern_size) -> float:
    """Return the smallest distance between horizontally/vertically adjacent corners.

    OpenCV returns chessboard corners in row-major order, matching the object
    point ordering used by the capture node.
    """
    points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    cols, rows = int(pattern_size[0]), int(pattern_size[1])
    if points.shape[0] != cols * rows:
        raise ValueError(
            f'expected {cols * rows} corners for {cols}x{rows}, '
            f'got {points.shape[0]}')
    if cols < 2 and rows < 2:
        return float('inf')

    grid = points.reshape(rows, cols, 2)
    gaps = []
    if cols > 1:
        gaps.append(np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel())
    if rows > 1:
        gaps.append(np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel())
    return float(np.min(np.concatenate(gaps)))


def reprojection_rms(object_points, image_points, camera_matrix,
                     distortion) -> float:
    """Fit a board pose with solvePnP and return its reprojection RMS in pixels."""
    object_points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    image_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=np.float64)

    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError('solvePnP failed')
    try:
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points, image_points, camera_matrix, distortion, rvec, tvec)
    except cv2.error:
        pass

    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion)
    residual = projected.reshape(-1, 2) - image_points
    return float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))


def detect_chessboard(gray, pattern_size, object_points, camera_matrix,
                      distortion, min_corner_gap_px: float = 2.0,
                      max_reprojection_rms_px: float = 1.0) -> ChessboardDetection:
    """Detect a chessboard and reject collapsed or badly fitted corners."""
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
    needs_subpix = False
    if not ok:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(
            gray, pattern_size, flags=flags)
        needs_subpix = ok
    if not ok or corners is None:
        raise DetectionError('chessboard not found')

    # SB already returns sub-pixel corners.  The classic detector does not,
    # so refine only that fallback path.  Derive the window from the observed
    # spacing; a fixed 11x11 window can snap onto a neighbouring corner.
    if needs_subpix:
        raw_gap = min_adjacent_gap_px(corners, pattern_size)
        half_window = int(np.clip(raw_gap / 3.0, 2, 5))
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    50, 0.001)
        corners = cv2.cornerSubPix(
            gray, corners, (half_window, half_window), (-1, -1), criteria)

    image_points = corners.reshape(-1, 2)
    object_points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    gap = min_adjacent_gap_px(image_points, pattern_size)
    if not np.isfinite(gap) or gap < min_corner_gap_px:
        raise DetectionError(
            f'adjacent corners too close ({gap:.2f} px < '
            f'{min_corner_gap_px:.2f} px)')

    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise DetectionError('solvePnP failed')
    try:
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points, image_points, camera_matrix, distortion, rvec, tvec)
    except cv2.error:
        pass

    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion)
    residual = projected.reshape(-1, 2) - image_points
    rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
    if not np.isfinite(rms) or rms > max_reprojection_rms_px:
        raise DetectionError(
            f'reprojection RMS {rms:.3f} px > '
            f'{max_reprojection_rms_px:.3f} px')

    T_cam_board = np.eye(4)
    T_cam_board[:3, :3] = Rotation.from_rotvec(rvec.reshape(3)).as_matrix()
    T_cam_board[:3, 3] = tvec.reshape(3)
    return ChessboardDetection(
        object_points=object_points,
        image_points=image_points,
        T_cam_board=T_cam_board,
        reprojection_rms_px=rms,
        min_corner_gap_px=gap,
    )
