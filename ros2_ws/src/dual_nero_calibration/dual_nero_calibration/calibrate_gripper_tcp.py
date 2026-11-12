#!/usr/bin/env python3
"""Interactive pivot calibration for the physical gripper TCP.

The script does not move the robot.  Move the arm manually or with MoveIt so
the chosen physical reference point stays fixed against a pivot, then press
Enter to record each ``world -> link7`` transform.  A least-squares pivot
solve returns the TCP translation in the link7 frame.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from .common import (
    default_deployment_dir,
    transform_from_msg,
)
from .gripper_tcp import (
    DEFAULT_TCP_RPY,
    solve_pivot,
    urdf_tcp_origin,
)


class GripperTcpCalibration(Node):
    def __init__(self, base_frame: str, link_frame: str) -> None:
        super().__init__('gripper_tcp_calibration')
        self.base_frame = base_frame
        self.link_frame = link_frame
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def wait_for_transform(self, timeout_sec: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.buffer.can_transform(
                    self.base_frame, self.link_frame, rclpy.time.Time()):
                return
        raise RuntimeError(
            f'timed out waiting for TF {self.base_frame} -> {self.link_frame}')

    def capture(self) -> np.ndarray:
        self.wait_for_transform(timeout_sec=2.0)
        # Refresh the TF buffer after the operator moves the arm.  Without this
        # short spin, the latest lookup can return the pose from before motion.
        deadline = time.monotonic() + 0.25
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        message = self.buffer.lookup_transform(
            self.base_frame, self.link_frame, Time(),
            timeout=Duration(seconds=1.0))
        return transform_from_msg(message.transform)


def _relative_rotation_deg(first: np.ndarray, second: np.ndarray) -> float:
    delta = first[:3, :3].T @ second[:3, :3]
    return float(np.degrees(
        np.linalg.norm(Rotation.from_matrix(delta).as_rotvec())))


def _nearest_rotation_deg(candidate: np.ndarray, transforms: list[np.ndarray]) -> float:
    if not transforms:
        return 180.0
    return min(_relative_rotation_deg(candidate, item) for item in transforms)


def _write_result(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(payload, stream, sort_keys=False)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Fit the physical gripper TCP with a fixed-point pivot.')
    parser.add_argument('--side', required=True, choices=('left', 'right'))
    parser.add_argument('--base-frame', default='world')
    parser.add_argument('--link-frame', default=None,
                        help='Default: <side>_link7')
    parser.add_argument('--samples', type=int, default=12)
    parser.add_argument('--min-new-rotation-deg', type=float, default=5.0,
                        help='Reject poses too similar to every existing pose')
    parser.add_argument('--min-rotation-spread-deg', type=float, default=15.0)
    parser.add_argument('--max-rms-mm', type=float, default=2.0)
    parser.add_argument('--max-condition', type=float, default=1.0e5)
    parser.add_argument('--rpy', nargs=3, type=float,
                        default=list(DEFAULT_TCP_RPY),
                        metavar=('ROLL', 'PITCH', 'YAW'),
                        help='Fixed TCP orientation relative to link7 in radians')
    parser.add_argument('--output', default=None)
    parser.add_argument('--strict', action='store_true')
    args = parser.parse_args()

    link_frame = args.link_frame or f'{args.side}_link7'
    rclpy.init()
    node = GripperTcpCalibration(args.base_frame, link_frame)
    transforms: list[np.ndarray] = []
    try:
        node.wait_for_transform()
        print(f'base={args.base_frame} link={link_frame}')
        print('Move the arm so the physical TCP point stays fixed on a pivot.')
        print('Keep changing orientation; pure translation does not identify TCP.')
        print(f'Capture {args.samples} poses. Press Enter to capture, q to finish.')
        while len(transforms) < args.samples:
            answer = input(
                f'[{len(transforms) + 1}/{args.samples}] Enter=capture, q=finish: ')
            if answer.strip().lower() == 'q':
                break
            transform = node.capture()
            nearest = _nearest_rotation_deg(transform, transforms)
            if nearest < args.min_new_rotation_deg:
                print(
                    f'rejected: only {nearest:.1f} deg change from the nearest '
                    f'pose (need {args.min_new_rotation_deg:.1f})')
                continue
            transforms.append(transform)
            print(
                f'captured {len(transforms)}: '
                f't=({transform[0, 3]:+.4f}, {transform[1, 3]:+.4f}, '
                f'{transform[2, 3]:+.4f}) m, new_rotation={nearest:.1f} deg')
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(transforms) < 3:
        print(f'need at least 3 poses, got {len(transforms)}', file=sys.stderr)
        return 1

    try:
        result = solve_pivot(transforms)
    except ValueError as exc:
        print(f'pivot solve failed: {exc}', file=sys.stderr)
        return 1

    urdf_origin = urdf_tcp_origin(result.tcp_xyz_link7, args.rpy)
    urdf_rpy = Rotation.from_matrix(
        urdf_origin[:3, :3]).as_euler('xyz')
    checks = {
        'sample_count': result.sample_count >= 3,
        'rotation_spread': (
            result.rotation_spread_deg >= args.min_rotation_spread_deg),
        'pivot_rms': result.rms_m * 1000.0 <= args.max_rms_mm,
        'condition': result.condition_number <= args.max_condition,
    }
    passed = all(checks.values())

    offset_values = [
        *result.tcp_xyz_link7,
        *[float(value) for value in args.rpy],
    ]
    urdf_args = {
        f'{args.side}_tcp_x': float(urdf_origin[0, 3]),
        f'{args.side}_tcp_y': float(urdf_origin[1, 3]),
        f'{args.side}_tcp_z': float(urdf_origin[2, 3]),
    }
    payload = {
        'gripper_tcp': {
            'side': args.side,
            'base_frame': args.base_frame,
            'link_frame': link_frame,
            'samples': result.sample_count,
            'tcp_xyz_link7_m': [float(value) for value in result.tcp_xyz_link7],
            'tcp_rpy_link7_rad': [float(value) for value in args.rpy],
            'pivot_xyz_base_m': [float(value) for value in result.pivot_xyz_base],
            'driver_tcp_offset': [float(value) for value in offset_values],
            'urdf_args': urdf_args,
            'urdf_tcp_rpy_rad': [float(value) for value in urdf_rpy],
            'pivot_rms_m': result.rms_m,
            'pivot_max_error_m': result.max_error_m,
            'rotation_spread_deg': result.rotation_spread_deg,
            'condition_number': result.condition_number,
            'checks': checks,
            'status': 'PASS' if passed else 'WARN',
        }
    }
    output = Path(
        args.output or
        default_deployment_dir('calibration_results') /
        f'gripper_tcp_{args.side}.yaml')
    _write_result(output, payload)

    print(f'OUTPUT {output}')
    print('TCP_XYZ_LINK7_M', [round(float(value), 9)
                             for value in result.tcp_xyz_link7])
    print('DRIVER_TCP_OFFSET', [round(float(value), 9)
                               for value in offset_values])
    print('URDF_ARGS', {key: round(value, 9) for key, value in urdf_args.items()})
    print('PIVOT_RMS_MM', round(result.rms_m * 1000.0, 6))
    print('PIVOT_MAX_ERROR_MM', round(result.max_error_m * 1000.0, 6))
    print('ROTATION_SPREAD_DEG', round(result.rotation_spread_deg, 6))
    print('CONDITION', round(result.condition_number, 3))
    for name, ok in checks.items():
        print(f'CHECK {name} {"PASS" if ok else "FAIL"}')
    print('CALIBRATION', 'PASS' if passed else 'WARN')
    if args.strict and not passed:
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
