#!/usr/bin/env python3
"""Fit the gripper raw-position to finger-opening map from caliper readings."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import yaml

from .common import default_deployment_dir
from .gripper_tcp import fit_opening_map


def _measurement(value: str) -> tuple[float, float]:
    try:
        position, opening = value.split('=', 1)
        return float(position), float(opening)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'measurement must be RAW_POSITION=OPENING_MM') from exc


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Fit a linear gripper opening map from measured widths.')
    parser.add_argument('--side', required=True, choices=('left', 'right'))
    parser.add_argument(
        '--measurement', action='append', type=_measurement, required=True,
        metavar='POSITION=MM',
        help='Raw /position feedback and caliper opening, repeat 2+ times')
    parser.add_argument('--position-min', type=int, default=0)
    parser.add_argument('--position-max', type=int, default=1000)
    parser.add_argument('--output', default=None)
    args = parser.parse_args()

    try:
        result = fit_opening_map(
            args.measurement, args.position_min, args.position_max)
    except ValueError as exc:
        print(f'opening calibration failed: {exc}', file=sys.stderr)
        return 1

    payload = {
        'gripper_opening': {
            'side': args.side,
            'measurements': [
                {'position': position, 'opening_mm': opening}
                for position, opening in args.measurement
            ],
            'model': 'opening_mm = slope * raw_position + intercept',
            'position_min': args.position_min,
            'position_max': args.position_max,
            **result,
            'node_parameters': {
                'position_min': args.position_min,
                'position_max': args.position_max,
                'opening_min_mm': result['opening_min_mm'],
                'opening_max_mm': result['opening_max_mm'],
            },
            'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        }
    }
    output = Path(
        args.output or
        default_deployment_dir('calibration_results') /
        f'gripper_opening_{args.side}.yaml')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(payload, stream, sort_keys=False)

    print(f'OUTPUT {output}')
    print(f'SLOPE_MM_PER_COUNT {result["slope_mm_per_count"]:.9f}')
    print(f'INTERCEPT_MM {result["intercept_mm"]:.9f}')
    print(f'OPENING_MIN_MM {result["opening_min_mm"]:.6f}')
    print(f'OPENING_MAX_MM {result["opening_max_mm"]:.6f}')
    print(f'FIT_RMS_MM {result["rms_mm"]:.6f}')
    print(f'FIT_MAX_ERROR_MM {result["max_error_mm"]:.6f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
