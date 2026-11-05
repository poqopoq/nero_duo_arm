#!/usr/bin/env python3
"""Run the CTAG2F90-D size/angle/speed acceptance sweep.

The test uses only the existing ROS 2 interfaces:

* ``/<side>_gripper/position_cmd`` for the 0..990 jaw command;
* ``motion_speed_percent`` for the vendor speed register;
* ``/<side>_gripper/enable`` and ``/clear_fault`` for the safety state;
* ``position``, ``width``, ``state`` and ``status`` for verification.

The default sweep is 10 positions x 10 speeds for each requested gripper.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from datetime import datetime
import math
from pathlib import Path
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32
from std_srvs.srv import SetBool, Trigger

from dual_nero_control.ctag_gripper import (
    joint_from_position,
    opening_from_position,
)


SIDES = ('left', 'right')
# Keep the default matrix inside the mechanically reachable command range.
# The measured grippers contact their hard stops around 914 raw counts, so
# 990 remains available for an explicit contact/end-stop test but is not a
# useful open-loop position target for the repeatability matrix.
DEFAULT_POSITIONS = '0,100,200,300,400,500,600,700,800,900'
DEFAULT_SPEEDS = '10,20,30,40,50,60,70,80,90,100'
# Conservative estimate from the measured vendor speed-register response.
# At speed=10 the gripper moves roughly 6 raw-position counts per second.
RAW_COUNTS_PER_SECOND_AT_FULL_SPEED = 0.6
MOTION_TIMEOUT_MARGIN_SEC = 5.0
GOAL_TIMEOUT_EXTRA_SEC = 2.0
MOTION_RAMP_MS = 200


@dataclass
class SideFeedback:
    status: dict[str, str] = field(default_factory=dict)
    status_time: float = 0.0
    position: int | None = None
    position_time: float = 0.0
    width_m: float | None = None
    width_time: float = 0.0
    joint_rad: float | None = None
    joint_time: float = 0.0


def _parse_ints(value: str, name: str, minimum: int, maximum: int) -> list[int]:
    try:
        values = [int(item.strip()) for item in value.split(',') if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f'{name} must be comma-separated integers') from exc
    if len(values) != 10:
        raise argparse.ArgumentTypeError(f'{name} must contain exactly 10 values')
    if len(set(values)) != len(values):
        raise argparse.ArgumentTypeError(f'{name} must not contain duplicates')
    if any(item < minimum or item > maximum for item in values):
        raise argparse.ArgumentTypeError(
            f'{name} values must be within {minimum}..{maximum}')
    return values


def _positions_arg(value: str) -> list[int]:
    return _parse_ints(value, '--positions', 0, 990)


def _speeds_arg(value: str) -> list[int]:
    return _parse_ints(value, '--speeds', 1, 100)


def _alternating_order(values: list[int]) -> list[int]:
    """Visit both ends first, then ping-pong inward to reduce long travel."""
    if len(values) < 2:
        return list(values)
    ordered = [values[0], values[-1]]
    low = 1
    high = len(values) - 2
    while low <= high:
        ordered.append(values[low])
        low += 1
        if low <= high:
            ordered.append(values[high])
            high -= 1
    return ordered


class GripperSweep(Node):
    def __init__(self, sides: list[str], timeout_sec: float) -> None:
        super().__init__('gripper_acceptance_sweep')
        self.sides = sides
        self.timeout_sec = timeout_sec
        self.feedback = {side: SideFeedback() for side in sides}
        self.command_publishers = {}
        self.enable_clients = {}
        self.clear_fault_clients = {}
        self.parameter_clients = {}
        self.rows: list[dict[str, str]] = []

        for side in sides:
            topic_prefix = f'/{side}_gripper'
            self.command_publishers[side] = self.create_publisher(
                Int32, f'{topic_prefix}/position_cmd', 10)
            self.create_subscription(
                Int32,
                f'{topic_prefix}/position',
                lambda msg, s=side: self._on_position(s, msg),
                10)
            self.create_subscription(
                Float32,
                f'{topic_prefix}/width',
                lambda msg, s=side: self._on_width(s, msg),
                10)
            self.create_subscription(
                JointState,
                f'{topic_prefix}/state',
                lambda msg, s=side: self._on_state(s, msg),
                10)
            self.create_subscription(
                DiagnosticArray,
                f'{topic_prefix}/status',
                lambda msg, s=side: self._on_status(s, msg),
                10)
            self.enable_clients[side] = self.create_client(
                SetBool, f'{topic_prefix}/enable')
            self.clear_fault_clients[side] = self.create_client(
                Trigger, f'{topic_prefix}/clear_fault')
            self.parameter_clients[side] = self.create_client(
                SetParameters,
                f'/{side}_gripper_node/set_parameters')

    def _on_position(self, side: str, msg: Int32) -> None:
        self.feedback[side].position = int(msg.data)
        self.feedback[side].position_time = time.monotonic()

    def _on_width(self, side: str, msg: Float32) -> None:
        self.feedback[side].width_m = float(msg.data)
        self.feedback[side].width_time = time.monotonic()

    def _on_state(self, side: str, msg: JointState) -> None:
        if msg.position:
            self.feedback[side].joint_rad = float(msg.position[0])
            self.feedback[side].joint_time = time.monotonic()

    def _on_status(self, side: str, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        values = {item.key: item.value for item in msg.status[0].values}
        self.feedback[side].status = values
        self.feedback[side].status_time = time.monotonic()

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return bool(predicate())

    def _call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=3.0):
            return None, 'service unavailable'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None, 'service call timed out'
        try:
            return future.result(), ''
        except Exception as exc:  # pragma: no cover - ROS transport failure
            return None, str(exc)

    def _status(self, side: str, key: str) -> str:
        return self.feedback[side].status.get(key, '')

    def _feedback_is_fresh(self, side: str, timeout: float = 2.0) -> bool:
        now = time.monotonic()
        item = self.feedback[side]
        return all(
            stamp > 0.0 and now - stamp <= timeout
            for stamp in (
                item.status_time,
                item.position_time,
                item.joint_time,
            )
        )

    def _set_motion_parameters(
            self,
            side: str,
            speed: int,
            goal_timeout_sec: float) -> bool:
        request = SetParameters.Request()
        request.parameters = [
            Parameter('motion_speed_percent', value=speed).to_parameter_msg(),
            Parameter(
                'motion_ramp_ms', value=MOTION_RAMP_MS
            ).to_parameter_msg(),
            Parameter(
                'goal_timeout_sec', value=float(goal_timeout_sec)
            ).to_parameter_msg(),
        ]
        response, error = self._call(self.parameter_clients[side], request)
        if error:
            print(f'{side}: cannot set motion parameters: {error}')
            return False
        if response is None or not response.results:
            print(f'{side}: empty motion-parameter response')
            return False
        for result in response.results:
            if not result.successful:
                print(
                    f'{side}: motion parameters rejected: {result.reason}')
                return False
        return True

    def _motion_timeout(self, distance: int, speed: int) -> float:
        travel_sec = distance / max(
            0.1, RAW_COUNTS_PER_SECOND_AT_FULL_SPEED * speed)
        return max(
            self.timeout_sec,
            travel_sec + MOTION_TIMEOUT_MARGIN_SEC,
        )

    def _set_speed(self, side: str, speed: int) -> bool:
        return self._set_motion_parameters(
            side, speed, max(30.0, self.timeout_sec))

    def _set_enabled(self, side: str, enabled: bool) -> bool:
        request = SetBool.Request()
        request.data = enabled
        response, error = self._call(self.enable_clients[side], request)
        if error:
            print(f'{side}: enable call failed: {error}')
            return False
        if response is None or not response.success:
            message = response.message if response is not None else 'no response'
            print(f'{side}: {"enable" if enabled else "disable"} failed: {message}')
            return False
        return True

    def _clear_fault(self, side: str) -> bool:
        response, error = self._call(
            self.clear_fault_clients[side], Trigger.Request())
        if error:
            print(f'{side}: clear-fault failed: {error}')
            return False
        if response is None or not response.success:
            message = response.message if response is not None else 'no response'
            print(f'{side}: clear-fault failed: {message}')
            return False
        return True

    def prepare_side(self, side: str, speed: int) -> bool:
        if not self.wait_until(
                lambda: self._feedback_is_fresh(side), timeout=5.0):
            print(f'{side}: no fresh position/width/state/status feedback')
            return False
        if not self._set_motion_parameters(
                side, speed, max(30.0, self.timeout_sec)):
            return False
        if self._status(side, 'enabled') == 'true':
            if not self._set_enabled(side, False):
                return False
            self.wait_until(
                lambda: self._status(side, 'enabled') == 'false', timeout=3.0)
        if not self._clear_fault(side):
            return False
        if not self.wait_until(
                lambda: self._status(side, 'state') == 'ready', timeout=3.0):
            print(
                f'{side}: not ready after clear-fault '
                f'(state={self._status(side, "state")})')
            return False
        if not self._set_enabled(side, True):
            return False
        enabled = self.wait_until(
            lambda: (
                self._status(side, 'enabled') == 'true'
                and self._status(side, 'enable_register') == '1'
                and self._status(side, 'feedback_fresh') == 'true'
                and self._status(side, 'state') != 'fault'
            ),
            timeout=5.0,
        )
        if not enabled:
            print(
                f'{side}: motor did not become ready: '
                f'state={self._status(side, "state")}, '
                f'enable_register={self._status(side, "enable_register")}, '
                f'controller_enabled={self._status(side, "controller_enabled")}, '
                f'fault={self._status(side, "fault_reason") or "none"}')
            return False
        return True

    def _wait_for_completion(
            self,
            side: str,
            target: int,
            timeout_sec: float) -> tuple[bool, float, str]:
        started = time.monotonic()
        command_seen = False
        deadline = started + timeout_sec + GOAL_TIMEOUT_EXTRA_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            command = self._status(side, 'command_position')
            if command and int(command) == target:
                command_seen = True
            fault = self._status(side, 'fault_reason')
            last_error = self._status(side, 'last_error')
            if fault or last_error:
                return False, time.monotonic() - started, fault or last_error
            if command_seen and self._status(side, 'motion_active') == 'false':
                return True, time.monotonic() - started, ''
        if command_seen:
            return False, time.monotonic() - started, 'completion timed out'
        return False, time.monotonic() - started, 'command was not accepted'

    def run_case(self, side: str, speed: int, target: int) -> tuple[bool, str]:
        item = self.feedback[side]
        baseline = item.position
        target_width_mm = opening_from_position(target)
        target_joint_rad = joint_from_position(target)
        distance = abs(target - baseline) if baseline is not None else 990
        motion_timeout = self._motion_timeout(distance, speed)
        parameter_ok = self._set_motion_parameters(
            side,
            speed,
            motion_timeout + GOAL_TIMEOUT_EXTRA_SEC)
        if parameter_ok:
            message = Int32()
            message.data = target
            self.command_publishers[side].publish(message)
            completed, duration, error = self._wait_for_completion(
                side, target, motion_timeout)
        else:
            completed = False
            duration = 0.0
            error = 'motion parameter update failed'
        self.spin_for(0.15)

        position = item.position
        width_mm = item.width_m * 1000.0 if item.width_m is not None else None
        joint_rad = item.joint_rad
        status = item.status
        fresh = self._feedback_is_fresh(side)
        controller_target = status.get('controller_target_position', '')
        position_error = (
            abs(position - target) if position is not None else math.inf)
        stalled = status.get('motion_stalled') == 'true'
        controller_closed = (
            target == 990
            and stalled
            and baseline is not None
            and position is not None
            and position - baseline >= max(2 * self.position_tolerance, 10)
            and 'closed on object' in status.get('motion_result', ''))
        reached = position_error <= self.position_tolerance
        healthy = (
            fresh
            and status.get('enabled') == 'true'
            and status.get('enable_register') == '1'
            and not status.get('fault_reason')
            and not status.get('last_error'))
        map_ok = True
        if width_mm is not None:
            map_ok = map_ok and abs(width_mm - target_width_mm) <= 2.0
        if joint_rad is not None:
            map_ok = map_ok and abs(joint_rad - target_joint_rad) <= 0.025
        passed = (
            completed
            and healthy
            and (reached or controller_closed)
            and (map_ok or controller_closed))

        reason = error
        if not completed:
            reason = error or 'motion did not complete'
        elif not healthy:
            reason = (
                'status is faulted, disabled, stale, or hardware enable '
                'register is not 1')
        elif not reached and not controller_closed:
            reason = f'position error {position_error} exceeds {self.position_tolerance}'
        elif not map_ok and not controller_closed:
            reason = 'width/angle mapping does not match position feedback'
        else:
            reason = 'closed at mechanical stop' if controller_closed else 'ok'

        self.rows.append({
            'side': side,
            'speed_percent': str(speed),
            'target_position': str(target),
            'target_width_mm': f'{target_width_mm:.3f}',
            'target_angle_rad': f'{target_joint_rad:.6f}',
            'position': '' if position is None else str(position),
            'width_mm': '' if width_mm is None else f'{width_mm:.3f}',
            'angle_rad': '' if joint_rad is None else f'{joint_rad:.6f}',
            'controller_target': controller_target,
            'duration_sec': f'{duration:.3f}',
            'result': 'PASS' if passed else 'FAIL',
            'reason': reason,
        })
        return passed, reason

    def run_side(
            self,
            side: str,
            positions: list[int],
            speeds: list[int],
            continue_on_failure: bool = False) -> bool:
        print(f'\n=== {side} gripper: {len(positions)} positions x '
              f'{len(speeds)} speeds ===')
        ordered_positions = _alternating_order(positions)
        if not self.prepare_side(side, speeds[0]):
            self._set_enabled(side, False)
            return False
        all_passed = True
        try:
            for speed in speeds:
                if not self._set_speed(side, speed):
                    all_passed = False
                    break
                print(f'-- speed {speed}%')
                for target in ordered_positions:
                    passed, reason = self.run_case(side, speed, target)
                    row = self.rows[-1]
                    print(
                        f'  pos={target:4d} width={float(row["target_width_mm"]):6.2f}mm '
                        f'angle={float(row["target_angle_rad"]): .5f}rad '
                        f'actual={row["position"] or "-":>4} '
                        f'{row["duration_sec"]:>6}s '
                        f'{"PASS" if passed else "FAIL"}: {reason}')
                    if not passed:
                        all_passed = False
                        if not continue_on_failure:
                            print(f'Stopping {side} sweep after first failure.')
                            return False
        finally:
            self._set_enabled(side, False)
        return all_passed

    def write_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            'side', 'speed_percent', 'target_position', 'target_width_mm',
            'target_angle_rad', 'position', 'width_mm', 'angle_rad',
            'controller_target', 'duration_sec', 'result', 'reason',
        ]
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='CTAG2F90-D 10-position x 10-speed acceptance sweep')
    parser.add_argument(
        '--sides', choices=('left', 'right', 'both'), default='both',
        help='grippers to test sequentially (default: both)')
    parser.add_argument(
        '--positions', type=_positions_arg, default=_positions_arg(DEFAULT_POSITIONS),
        help=f'10 raw positions, 0=open, 990=closed (default: {DEFAULT_POSITIONS})')
    parser.add_argument(
        '--speeds', type=_speeds_arg, default=_speeds_arg(DEFAULT_SPEEDS),
        help=f'10 speed percentages, 1..100 (default: {DEFAULT_SPEEDS})')
    parser.add_argument(
        '--position-tolerance', type=int, default=25,
        help='allowed raw-position error (default: 25)')
    parser.add_argument(
        '--timeout-sec', type=float, default=10.0,
        help='wait time for each motion after the command (default: 10)')
    parser.add_argument(
        '--csv', type=Path,
        help='optional CSV output path')
    parser.add_argument(
        '--continue-on-failure', action='store_true',
        help='record failures and continue the full matrix instead of '
             'stopping that gripper at its first failed case')
    args = parser.parse_args()
    if args.timeout_sec <= 0.0:
        parser.error('--timeout-sec must be greater than zero')
    if args.position_tolerance < 0:
        parser.error('--position-tolerance must be non-negative')

    print('WARNING: the gripper will move. Keep hands clear and the e-stop ready.')
    print('The sweep stops that gripper after its first failed case.')

    rclpy.init()
    node = None
    try:
        sides = list(SIDES) if args.sides == 'both' else [args.sides]
        node = GripperSweep(sides, args.timeout_sec)
        node.position_tolerance = args.position_tolerance
        all_passed = True
        for side in sides:
            all_passed = node.run_side(
                side,
                args.positions,
                args.speeds,
                continue_on_failure=args.continue_on_failure,
            ) and all_passed
        csv_path = args.csv
        if csv_path is None:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = Path.cwd() / f'gripper_sweep_{stamp}.csv'
        node.write_csv(csv_path)
        passed = sum(row['result'] == 'PASS' for row in node.rows)
        failed = len(node.rows) - passed
        print(f'\nRESULT: {passed} passed, {failed} failed, CSV={csv_path}')
        return 0 if all_passed and failed == 0 else 1
    except KeyboardInterrupt:
        print('\nInterrupted; disabling grippers.')
        return 130
    finally:
        if node is not None:
            for side in node.sides:
                try:
                    node._set_enabled(side, False)
                except Exception:
                    pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
