"""Select a diverse, high-quality subset of calibration frames."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def select_diverse_indices(rotations, translations, qualities, count: int,
                           translation_weight: float = 500.0) -> list[int]:
    """Pick ``count`` frames with good quality and diverse robot/board poses.

    The first frame is the one with the lowest reprojection RMS.  Each next
    frame maximises its minimum combined pose distance to the selected set:
    rotation error in degrees plus ``translation_weight`` times translation
    error in metres.  The default weights make 2 cm roughly equivalent to
    10 degrees.
    """
    rotations = np.asarray(rotations, dtype=np.float64)
    translations = np.asarray(translations, dtype=np.float64).reshape(-1, 3)
    qualities = np.asarray(qualities, dtype=np.float64).reshape(-1)
    if not (len(rotations) == len(translations) == len(qualities)):
        raise ValueError('rotations, translations and qualities must align')
    if len(rotations) == 0:
        return []
    count = int(count)
    if count <= 0:
        return []

    rotation_objects = Rotation.from_matrix(rotations.reshape(-1, 3, 3))
    selected = [int(np.argmin(qualities))]
    remaining = set(range(len(rotations))) - set(selected)

    while remaining and len(selected) < count:
        best_index = None
        best_distance = -1.0
        for index in remaining:
            distance = min(
                _pose_distance(rotation_objects, translations, index, chosen,
                               translation_weight)
                for chosen in selected)
            if (distance > best_distance or
                    (distance == best_distance and
                     (best_index is None or qualities[index] < qualities[best_index]))):
                best_index = index
                best_distance = distance
        selected.append(int(best_index))
        remaining.remove(best_index)
    return selected


def _pose_distance(rotations: Rotation, translations: np.ndarray,
                   first: int, second: int, translation_weight: float) -> float:
    relative = rotations[first].inv() * rotations[second]
    rotation_deg = float(np.degrees(np.linalg.norm(relative.as_rotvec())))
    translation_m = float(np.linalg.norm(translations[first] - translations[second]))
    return rotation_deg + translation_weight * translation_m


def pose_rotation_spread_deg(poses) -> float:
    """Largest pairwise rotation between recorded tool poses."""
    matrices = [np.asarray(pose, dtype=np.float64)[:3, :3] for pose in poses]
    spread = 0.0
    for first in range(len(matrices)):
        for second in range(first + 1, len(matrices)):
            delta = matrices[first].T @ matrices[second]
            spread = max(spread, float(np.degrees(
                np.linalg.norm(Rotation.from_matrix(delta).as_rotvec()))))
    return spread


def nearest_pose(pose, existing, min_rotation_deg: float,
                 min_translation_mm: float):
    """Find the closest recorded pose if the new one is too similar.

    Returns ``(index, translation_m, rotation_deg)`` for the closest pose that
    is inside both thresholds, or ``None`` when the new pose is distinct
    enough (or nothing has been recorded yet).
    """
    pose = np.asarray(pose, dtype=np.float64)
    closest = None
    for index, candidate in enumerate(existing):
        candidate = np.asarray(candidate, dtype=np.float64)
        if candidate.shape != (4, 4):
            continue
        delta = np.linalg.inv(candidate) @ pose
        translation = float(np.linalg.norm(delta[:3, 3]))
        rotation = float(np.degrees(np.linalg.norm(
            Rotation.from_matrix(delta[:3, :3]).as_rotvec())))
        if (translation > min_translation_mm / 1000.0
                or rotation > min_rotation_deg):
            continue
        score = (rotation / max(min_rotation_deg, 1e-6)
                 + translation * 1000.0 / max(min_translation_mm, 1e-6))
        if closest is None or score < closest[0]:
            closest = (score, index, translation, rotation)
    if closest is None:
        return None
    return closest[1], closest[2], closest[3]
