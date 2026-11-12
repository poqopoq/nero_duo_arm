"""URDF forward kinematics for the two NERO arms.

The calibration planner needs to predict where the wrist camera will look
before the arm moves.  ``robot_state_publisher`` uses the same URDF, so the
chain built here matches the TF tree as long as the description was launched
with the same xacro arguments.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class ArmChain:
    """Serial chain from ``world`` to one ``<side>_tool0`` frame."""

    side: str
    segments: tuple
    joints: tuple

    @property
    def joint_names(self) -> list[str]:
        return [joint.name for joint in self.joints]

    def limits(self) -> tuple[np.ndarray, np.ndarray]:
        lower = np.array([joint.limit.lower for joint in self.joints])
        upper = np.array([joint.limit.upper for joint in self.joints])
        return lower, upper

    def forward(self, positions: Sequence[float]) -> np.ndarray:
        values = list(np.asarray(positions, dtype=np.float64).reshape(-1))
        if len(values) != len(self.joints):
            raise ValueError(
                f'{self.side} chain needs {len(self.joints)} joint values, '
                f'got {len(values)}')
        frame = np.eye(4)
        index = 0
        for segment, uses_joint in self.segments:
            angle = values[index] if uses_joint else 0.0
            if uses_joint:
                index += 1
            frame = frame @ _segment_transform(segment, angle)
        return frame


def _segment_transform(segment, angle: float) -> np.ndarray:
    import PyKDL

    pose = segment.pose(float(angle))
    matrix = np.eye(4)
    for row in range(3):
        for column in range(3):
            matrix[row, column] = pose.M[row, column]
        matrix[row, 3] = pose.p[row]
    return matrix


def urdf_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory('dual_nero_description'))
    return share / 'urdf' / 'two_nero_tabletop.urdf.xacro'


def load_arm_chain(side: str, path: Path | None = None,
                   xacro_args: dict | None = None) -> ArmChain:
    """Build the ``world`` -> ``<side>_tool0`` chain from the URDF."""
    import PyKDL
    import xacro
    from urdf_parser_py.urdf import URDF

    if side not in ('left', 'right'):
        raise ValueError("side must be 'left' or 'right'")
    path = Path(path) if path is not None else urdf_path()
    document = xacro.process_file(str(path), mappings=xacro_args or {})
    robot = URDF.from_xml_string(document.toxml())

    by_child = {joint.child: joint for joint in robot.joints}
    ordered = []
    link = f'{side}_tool0'
    while link != 'world':
        if link not in by_child:
            raise RuntimeError(f'{link} is not connected to world in {path}')
        joint = by_child[link]
        ordered.append(joint)
        link = joint.parent
    ordered.reverse()

    segments = []
    joints = []
    for joint in ordered:
        # URDF puts the joint origin before the rotation, KDL puts its joint
        # rotation before the segment tip, so split origin and joint apart.
        segments.append((PyKDL.Segment(
            f'{joint.name}_origin', PyKDL.Joint(PyKDL.Joint.Fixed),
            _kdl_frame(joint.origin)), False))
        if joint.type == 'fixed':
            continue
        axis = getattr(joint, 'axis', None) or [0.0, 0.0, 1.0]
        kdl_joint = PyKDL.Joint(
            joint.name, PyKDL.Vector(0.0, 0.0, 0.0),
            PyKDL.Vector(*axis), PyKDL.Joint.RotAxis)
        segments.append((PyKDL.Segment(joint.name, kdl_joint), True))
        joints.append(joint)
    return ArmChain(side=side, segments=tuple(segments), joints=tuple(joints))


def _kdl_frame(origin) -> 'PyKDL.Frame':
    import PyKDL

    xyz = list(getattr(origin, 'xyz', None) or [0.0, 0.0, 0.0])
    rpy = list(getattr(origin, 'rpy', None) or [0.0, 0.0, 0.0])
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))
