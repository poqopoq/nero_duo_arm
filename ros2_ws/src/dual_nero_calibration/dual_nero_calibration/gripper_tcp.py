"""Pure geometry helpers for pivot-based gripper TCP calibration."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from .common import invert_transform, transform_from_xyz_rpy


# Fixed transform used by dual_nero_description for link7 -> tool0.
LINK7_TO_TOOL0_XYZ = (0.031, 0.0, -0.0235)
LINK7_TO_TOOL0_RPY = (-1.5708, 0.0, -1.5708)

# Orientation of the CTAG2F90-D pinch frame relative to link7.  A pivot
# calibration solves translation only; keep the already deployed orientation.
DEFAULT_TCP_RPY = (-1.5708, 0.0, -1.5708)


@dataclass(frozen=True)
class PivotResult:
    tcp_xyz_link7: np.ndarray
    pivot_xyz_base: np.ndarray
    sample_count: int
    rms_m: float
    max_error_m: float
    rotation_spread_deg: float
    condition_number: float


def as_transform(rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def rotation_spread_deg(transforms: Iterable[np.ndarray]) -> float:
    matrices = [np.asarray(transform)[:3, :3] for transform in transforms]
    spread = 0.0
    for index, first in enumerate(matrices):
        for second in matrices[index + 1:]:
            delta = first.T @ second
            angle = np.linalg.norm(Rotation.from_matrix(delta).as_rotvec())
            spread = max(spread, float(np.degrees(angle)))
    return spread


def solve_pivot(transforms: Sequence[np.ndarray]) -> PivotResult:
    """Solve the fixed point touched by a moving tool frame.

    Each input is ``T_base_link7``.  The solved ``tcp_xyz_link7`` is the
    translation of the physical tool point expressed in the link7 frame.
    At least three poses with non-parallel rotation axes are required.
    """
    if len(transforms) < 3:
        raise ValueError('pivot calibration needs at least 3 poses')

    rows = []
    targets = []
    for transform in transforms:
        rotation = np.asarray(transform, dtype=np.float64)[:3, :3]
        translation = np.asarray(transform, dtype=np.float64)[:3, 3]
        rows.append(np.hstack([rotation, -np.eye(3)]))
        targets.append(-translation)
    matrix = np.vstack(rows)
    vector = np.concatenate(targets)

    solution, _, rank, singular_values = np.linalg.lstsq(
        matrix, vector, rcond=None)
    if rank < 6:
        raise ValueError(
            'pivot calibration is degenerate; rotate the tool around at least '
            'two non-parallel axes')

    tcp_xyz = solution[:3]
    pivot_xyz = solution[3:]
    errors = []
    for transform in transforms:
        predicted = (
            np.asarray(transform, dtype=np.float64)[:3, :3] @ tcp_xyz
            + np.asarray(transform, dtype=np.float64)[:3, 3]
        )
        errors.append(float(np.linalg.norm(predicted - pivot_xyz)))
    errors_array = np.asarray(errors, dtype=np.float64)

    return PivotResult(
        tcp_xyz_link7=tcp_xyz,
        pivot_xyz_base=pivot_xyz,
        sample_count=len(transforms),
        rms_m=float(np.sqrt(np.mean(np.square(errors_array)))),
        max_error_m=float(np.max(errors_array)),
        rotation_spread_deg=rotation_spread_deg(transforms),
        condition_number=float(singular_values[0] / singular_values[-1]),
    )


def driver_offset(
        tcp_xyz_link7: Sequence[float],
        rpy_link7: Sequence[float] = DEFAULT_TCP_RPY) -> np.ndarray:
    """Build ``T_link7_tcp`` using the measured translation and fixed RPY."""
    rotation = Rotation.from_euler('xyz', list(rpy_link7)).as_matrix()
    return as_transform(rotation, tcp_xyz_link7)


def urdf_tcp_origin(tcp_xyz_link7: Sequence[float],
                    rpy_link7: Sequence[float] = DEFAULT_TCP_RPY) -> np.ndarray:
    """Convert a link7 TCP pose to the tool0->tcp origin used by the URDF."""
    link7_tool0 = transform_from_xyz_rpy(
        LINK7_TO_TOOL0_XYZ, LINK7_TO_TOOL0_RPY)
    link7_tcp = driver_offset(tcp_xyz_link7, rpy_link7)
    return invert_transform(link7_tool0) @ link7_tcp


def fit_opening_map(measurements: Sequence[tuple[float, float]],
                    position_min: int = 0,
                    position_max: int = 1000) -> dict[str, float]:
    """Fit the linear raw-position to finger-opening map from caliper data.

    ``measurements`` contains ``(raw_position, measured_opening_mm)`` pairs.
    The deployed gripper node uses the same two-point linear model, so at
    least two well-separated positions are sufficient; more points expose
    linkage non-linearity through ``rms_mm``.
    """
    if len(measurements) < 2:
        raise ValueError('opening calibration needs at least 2 measurements')
    if position_max <= position_min:
        raise ValueError('position_max must be greater than position_min')

    positions = np.asarray([item[0] for item in measurements], dtype=np.float64)
    openings = np.asarray([item[1] for item in measurements], dtype=np.float64)
    if np.ptp(positions) < 100.0:
        raise ValueError('measurement positions must span at least 100 counts')

    slope, intercept = np.polyfit(positions, openings, 1)
    predicted = slope * positions + intercept
    rms = float(np.sqrt(np.mean(np.square(predicted - openings))))
    return {
        'slope_mm_per_count': float(slope),
        'intercept_mm': float(intercept),
        'opening_min_mm': float(slope * position_min + intercept),
        'opening_max_mm': float(slope * position_max + intercept),
        'rms_mm': rms,
        'max_error_mm': float(np.max(np.abs(predicted - openings))),
    }
