#!/usr/bin/python3
"""Bring NERO joint4 back inside the deployment envelope after power-up.

The driver reports joint4 up to ~2.198 rad while the deployment cap is 2.14.
IK then either fails or returns a solution the VLA bridge rejects.  This script
performs one guarded, low-speed joint4 correction and leaves the motors enabled
so gravity cannot pull the arm back to the mechanical stop.

It refuses to move if:

* joint feedback is missing or stale;
* any non-target joint is outside the deployment limits;
* joint4 is more than --max-correction above the cap;
* the requested target is not strictly below the cap.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool

SIDES = ('left', 'right')
# Keep in sync with dual_nero_vla_bridge/launch/bimanual_vla_bridge.launch.py.
JOINT_MIN = [-2.70526, -1.760, -2.75, -1.01, -2.75, -0.7330383, -1.5707963]
JOINT_MAX = [2.70526, 1.74, 2.75, 2.14, 2.75, 0.8726646, 1.5707963]
JOINT4_INDEX = 3
FEEDBACK_TIMEOUT_SEC = 1.0


class JointLimitRecover(Node):
    def __init__(self, feedback_timeout: float = 5.0) -> None:
        super().__init__('nero_joint_limit_recover')
        self.feedback_timeout = feedback_timeout
        self._feedback: dict[str, JointState] = {}
        self._feedback_mono: dict[str, float] = {}
        self._joint_command_publishers = {}
        self._gates = {}
        self._enablers = {}
        self._stops = {}
        for side in SIDES:
            self.create_subscription(
                JointState,
                f'/{side}/feedback/joint_states',
                lambda msg, side=side: self._store_feedback(side, msg),
                10,
            )
            self._joint_command_publishers[side] = self.create_publisher(
                JointState, f'/{side}/control/joint_states', 10)
            self._gates[side] = self.create_client(
                SetBool, f'/{side}/control_enable')
            self._enablers[side] = self.create_client(
                SetBool, f'/{side}/enable_agx_arm')
            self._stops[side] = self.create_client(
                Empty, f'/{side}/emergency_stop')

    def _store_feedback(self, side: str, msg: JointState) -> None:
        self._feedback[side] = msg
        self._feedback_mono[side] = time.monotonic()

    def spin_once(self, timeout: float = 0.05) -> None:
        rclpy.spin_once(self, timeout_sec=timeout)

    def read_joints(self, side: str) -> Optional[list[float]]:
        self.spin_once()
        msg = self._feedback.get(side)
        mono = self._feedback_mono.get(side)
        if msg is None or mono is None:
            return None
        if time.monotonic() - mono > FEEDBACK_TIMEOUT_SEC:
            return None
        values = dict(zip(msg.name, [float(value) for value in msg.position]))
        joints = [values.get(f'joint{index}') for index in range(1, 8)]
        if any(value is None or not math.isfinite(value) for value in joints):
            return None
        return [float(value) for value in joints]

    def call_setbool(
        self, client, value: bool, timeout: float = 30.0
    ) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        request = SetBool.Request()
        request.data = bool(value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return bool(
            future.done() and future.result() is not None and future.result().success)

    def emergency_stop(self, side: str) -> None:
        client = self._stops[side]
        if client.service_is_ready():
            client.call_async(Empty.Request())
            self.spin_once(0.3)

    def publish_target(
        self, side: str, start: list[float], target: float, rate_hz: float = 20.0
    ) -> None:
        message = JointState()
        message.name = [f'joint{index}' for index in range(1, 8)]
        message.position = list(start)
        message.position[JOINT4_INDEX] = target
        period = 1.0 / rate_hz
        for _ in range(30):
            message.header.stamp = self.get_clock().now().to_msg()
            self._joint_command_publishers[side].publish(message)
            self.spin_once(period)

    def wait_settled(
        self, side: str, start: list[float], target: float,
        tolerance: float, timeout: float, max_drift: float,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            joints = self.read_joints(side)
            if joints is None:
                return False, 'feedback became stale'
            error = joints[JOINT4_INDEX] - target
            drift = max(
                abs(value - previous)
                for index, (value, previous) in enumerate(zip(joints, start))
                if index != JOINT4_INDEX
            )
            if drift > max_drift:
                return False, f'non-target joint drift {drift:.4f} rad exceeds {max_drift:.4f}'
            if abs(error) <= tolerance:
                return True, f'joint4={joints[JOINT4_INDEX]:.5f}'
        return False, 'settle timeout'


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Move joint4 inside the NERO deployment limits.')
    parser.add_argument('--side', choices=('both', *SIDES), default='both')
    parser.add_argument('--target', type=float, default=2.128)
    parser.add_argument('--cap-margin', type=float, default=0.005,
                        help='Treat joint4 above max-margin as needing recovery.')
    parser.add_argument('--max-correction', type=float, default=0.09)
    parser.add_argument('--settle-tolerance', type=float, default=0.012)
    parser.add_argument('--max-other-joint-drift', type=float, default=0.03)
    parser.add_argument('--timeout', type=float, default=25.0)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--disable', action='store_true',
                        help='Disable motors after recovery (allows gravity sag).')
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    sides = SIDES if args.side == 'both' else (args.side,)
    cap = JOINT_MAX[JOINT4_INDEX]
    if args.target >= cap - 1e-4:
        print(f'REFUSED: target {args.target:.5f} must be below cap {cap:.5f}')
        return 2
    if args.max_correction <= 0.0:
        print('REFUSED: --max-correction must be positive')
        return 2

    rclpy.init()
    node = JointLimitRecover()
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            node.spin_once(0.1)
            if all(node.read_joints(side) is not None for side in sides):
                break
        else:
            print('REFUSED: joint feedback unavailable or stale')
            return 2

        current = {side: node.read_joints(side) for side in sides}
        needs: list[str] = []
        for side in sides:
            joints = current[side]
            assert joints is not None
            for index, value in enumerate(joints):
                if index == JOINT4_INDEX:
                    continue
                if value < JOINT_MIN[index] - 0.01 or value > JOINT_MAX[index] + 0.01:
                    print(
                        f'REFUSED: {side} joint{index + 1}={value:.5f} outside '
                        f'[{JOINT_MIN[index]:.5f}, {JOINT_MAX[index]:.5f}]')
                    return 2
            value = joints[JOINT4_INDEX]
            if value > cap + args.max_correction:
                print(
                    f'REFUSED: {side} joint4={value:.5f} is more than '
                    f'{args.max_correction:.3f} rad above cap {cap:.5f}')
                return 2
            if value > cap - args.cap_margin:
                if value < args.target:
                    print(
                        f'REFUSED: {side} joint4={value:.5f} is already below '
                        f'target {args.target:.5f}; refusing to increase it')
                    return 2
                needs.append(side)
                print(
                    f'{side}: joint4={value:.5f} -> {args.target:.5f} '
                    f'(correction {args.target - value:+.5f} rad)')
            else:
                print(f'{side}: joint4={value:.5f} already inside cap, no move')

        if not needs:
            print('RESULT: ALREADY_INSIDE_LIMITS')
            return 0
        if args.dry_run:
            print('RESULT: DRY_RUN')
            return 0

        for side in needs:
            if not node.call_setbool(node._enablers[side], True):
                print(f'FAILED: {side} enable_agx_arm')
                return 3
        print('motors enabled')

        for side in needs:
            if not node.call_setbool(node._gates[side], True, timeout=5.0):
                print(f'FAILED: {side} control_enable open')
                for failed_side in needs:
                    node.call_setbool(node._gates[failed_side], False, timeout=5.0)
                return 3

        failure = None
        for side in needs:
            node.publish_target(side, current[side], args.target)
        for side in needs:
            # A correction smaller than the settle tolerance must still move at
            # least halfway; otherwise the first sample would count as settled.
            correction = abs(args.target - current[side][JOINT4_INDEX])
            tolerance = min(
                args.settle_tolerance, max(0.002, correction * 0.5))
            ok, detail = node.wait_settled(
                side, current[side], args.target, tolerance,
                args.timeout, args.max_other_joint_drift)
            if not ok:
                failure = f'{side}: {detail}'
                break

        for side in needs:
            node.call_setbool(node._gates[side], False, timeout=5.0)

        if failure is not None:
            for side in needs:
                node.emergency_stop(side)
            if args.disable:
                for side in needs:
                    node.call_setbool(node._enablers[side], False)
            print(f'FAILED: {failure}')
            return 3

        if args.disable:
            for side in needs:
                node.call_setbool(node._enablers[side], False)
            print('RESULT: MOVED_AND_DISABLED')
        else:
            print('RESULT: MOVED_KEEP_ENABLED')

        for side in sides:
            joints = node.read_joints(side)
            if joints is not None:
                print(
                    f'{side}: final joint4={joints[JOINT4_INDEX]:.5f} '
                    f'all={[round(value, 4) for value in joints]}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
