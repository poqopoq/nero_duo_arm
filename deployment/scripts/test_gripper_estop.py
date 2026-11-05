#!/usr/bin/env python3
"""Hardware acceptance for the gripper motion emergency-stop path."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Int32
from std_srvs.srv import Empty, SetBool, Trigger


SIDES = ('left', 'right')
DEFAULT_CSV = (
    Path(__file__).resolve().parents[1]
    / 'acceptance'
    / f'gripper_estop_{datetime.now():%Y%m%d_%H%M%S}.csv'
)


class GripperEstopTest(Node):
    def __init__(self, sides: list[str], timeout_sec: float) -> None:
        super().__init__('gripper_estop_acceptance')
        self.sides = sides
        self.timeout_sec = timeout_sec
        self.status = {side: {} for side in sides}
        self.position = {side: None for side in sides}
        self.position_publishers = {}
        self.enable_clients = {}
        self.estop_clients = {}
        self.clear_fault_clients = {}
        self.parameter_clients = {}
        self.rows: list[dict[str, str]] = []

        for side in sides:
            prefix = f'/{side}_gripper'
            self.position_publishers[side] = self.create_publisher(
                Int32, f'{prefix}/position_cmd', 10)
            self.create_subscription(
                Int32,
                f'{prefix}/position',
                lambda msg, s=side: self._on_position(s, msg),
                10)
            self.create_subscription(
                DiagnosticArray,
                f'{prefix}/status',
                lambda msg, s=side: self._on_status(s, msg),
                10)
            self.enable_clients[side] = self.create_client(
                SetBool, f'{prefix}/enable')
            self.estop_clients[side] = self.create_client(
                Empty, f'{prefix}/emergency_stop')
            self.clear_fault_clients[side] = self.create_client(
                Trigger, f'{prefix}/clear_fault')
            self.parameter_clients[side] = self.create_client(
                SetParameters, f'/{side}_gripper_node/set_parameters')

    def _on_position(self, side: str, msg: Int32) -> None:
        self.position[side] = int(msg.data)

    def _on_status(self, side: str, msg: DiagnosticArray) -> None:
        if msg.status:
            self.status[side] = {
                item.key: item.value for item in msg.status[0].values}

    def _status(self, side: str, key: str) -> str:
        return self.status[side].get(key, '')

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return bool(predicate())

    def call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=3.0):
            return None, 'service unavailable'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None, 'service call timed out'
        try:
            return future.result(), ''
        except Exception as exc:  # pragma: no cover - transport failure
            return None, str(exc)

    def call_many(self, calls: dict[str, tuple[object, object]],
                  timeout: float = 5.0) -> dict[str, tuple[object, str]]:
        results: dict[str, tuple[object, str]] = {}
        futures = {}
        for side, (client, request) in calls.items():
            if not client.wait_for_service(timeout_sec=3.0):
                results[side] = (None, 'service unavailable')
                continue
            futures[side] = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(future.done() for future in futures.values()):
                break
            rclpy.spin_once(self, timeout_sec=0.01)
        for side, future in futures.items():
            if not future.done():
                results[side] = (None, 'service call timed out')
                continue
            try:
                results[side] = (future.result(), '')
            except Exception as exc:  # pragma: no cover - transport failure
                results[side] = (None, str(exc))
        for side in self.sides:
            results.setdefault(side, (None, 'service was not called'))
        return results

    def fresh(self, side: str, timeout: float = 2.0) -> bool:
        if self._status(side, 'can_connected') != 'true':
            return False
        if self._status(side, 'feedback_fresh') != 'true':
            return False
        stamp = self._status(side, 'feedback_age_sec')
        if not stamp:
            return False
        try:
            return float(stamp) <= timeout
        except ValueError:
            return False

    def set_motion_parameters(self, side: str, speed: int) -> bool:
        request = SetParameters.Request()
        request.parameters = [
            Parameter('motion_speed_percent', value=speed).to_parameter_msg(),
            Parameter('motion_ramp_ms', value=200).to_parameter_msg(),
            Parameter('goal_timeout_sec', value=30.0).to_parameter_msg(),
        ]
        response, error = self.call(
            self.parameter_clients[side], request, timeout=5.0)
        if error or response is None:
            print(f'{side}: parameter call failed: {error or "no response"}')
            return False
        if not response.results or any(
                not result.successful for result in response.results):
            print(f'{side}: motion parameters rejected')
            return False
        return True

    def set_enabled(self, side: str, enabled: bool) -> tuple[bool, str]:
        request = SetBool.Request()
        request.data = enabled
        response, error = self.call(
            self.enable_clients[side], request, timeout=5.0)
        if error or response is None:
            return False, error or 'no response'
        return bool(response.success), response.message

    def clear_fault(self, side: str) -> tuple[bool, str]:
        response, error = self.call(
            self.clear_fault_clients[side], Trigger.Request(), timeout=5.0)
        if error or response is None:
            return False, error or 'no response'
        return bool(response.success), response.message

    def prepare_side(self, side: str, speed: int) -> tuple[bool, str]:
        if not self.wait_until(lambda: self.fresh(side), timeout=5.0):
            return False, 'no fresh CAN/status/position feedback'
        if not self.set_motion_parameters(side, speed):
            return False, 'failed to set motion parameters'
        if self._status(side, 'enabled') == 'true':
            ok, message = self.set_enabled(side, False)
            if not ok:
                return False, f'disable failed: {message}'
            if not self.wait_until(
                    lambda: self._status(side, 'enabled') == 'false',
                    timeout=3.0):
                return False, 'disable was not confirmed'
        ok, message = self.clear_fault(side)
        if not ok:
            return False, f'clear_fault failed: {message}'
        if not self.wait_until(
                lambda: self._status(side, 'state') == 'ready', timeout=3.0):
            return False, (
                f'not ready after clear_fault: {self._status(side, "state")}')
        ok, message = self.set_enabled(side, True)
        if not ok:
            return False, f'enable failed: {message}'
        if not self.wait_until(
                lambda: (
                    self._status(side, 'enabled') == 'true'
                    and self._status(side, 'enable_register') == '1'
                    and self._status(side, 'state') != 'fault'
                    and self.fresh(side)
                ),
                timeout=5.0,
        ):
            return False, (
                f'motor did not enable: state={self._status(side, "state")}, '
                f'fault={self._status(side, "fault_reason") or "none"}')
        if self.position[side] is None:
            return False, 'position feedback is missing'
        return True, ''

    def target_for(self, position: int) -> int:
        # Use the longer available travel so the stop can be triggered after
        # a confirmed displacement without approaching either hard stop.
        return 900 if position <= 495 else 0

    def wait_for_displacement(
            self, side: str, start: int, trigger: int) -> tuple[bool, str]:
        deadline = time.monotonic() + self.timeout_sec
        started = time.monotonic()
        active_seen = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            position = self.position[side]
            if position is not None and abs(position - start) >= trigger:
                return True, ''
            if self._status(side, 'motion_active') == 'true':
                active_seen = True
            elif active_seen:
                return False, (
                    f'motion ended before trigger: state={self._status(side, "state")}, '
                    f'fault={self._status(side, "fault_reason") or "none"}')
            elif time.monotonic() - started > 1.0:
                return False, 'motion did not become active'
        return False, 'timed out waiting for displacement'

    def record_position(self, side: str, duration: float) -> tuple[int, int]:
        reference = self.position[side]
        if reference is None:
            raise RuntimeError(f'{side}: missing position feedback')
        max_travel = 0
        self.spin_for(0.1)
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            current = self.position[side]
            if current is not None:
                max_travel = max(max_travel, abs(current - reference))
        final = self.position[side]
        return max_travel, reference if final is None else final

    def recover_side(self, side: str, start: int) -> tuple[bool, str]:
        ok, message = self.set_enabled(side, True)
        if not ok:
            return False, f'enable after clear_fault failed: {message}'
        message = Int32()
        message.data = start
        self.position_publishers[side].publish(message)
        reached = self.wait_until(
            lambda: (
                self._status(side, 'motion_active') == 'false'
                and self.position[side] is not None
                and abs(self.position[side] - start) <= 25
            ),
            timeout=10.0,
        )
        if not reached:
            return False, (
                f'recovery move did not reach {start}: '
                f'position={self.position[side]}, '
                f'fault={self._status(side, "fault_reason") or "none"}')
        ok, message = self.set_enabled(side, False)
        if not ok:
            return False, f'final disable failed: {message}'
        self.wait_until(
            lambda: self._status(side, 'enabled') == 'false', timeout=3.0)
        return True, ''

    def run_case(self, case: str, active_sides: list[str], speed: int,
                 trigger: int, post_window: float,
                 max_stop_travel: int) -> bool:
        print(f'\n=== {case}: speed={speed}% sides={active_sides} ===')
        starts: dict[str, int] = {}
        targets: dict[str, int] = {}
        for side in active_sides:
            ok, reason = self.prepare_side(side, speed)
            if not ok:
                print(f'{side}: prepare failed: {reason}')
                self.set_enabled(side, False)
                self._row(case, active_sides, speed, {}, {}, 0.0, 0,
                          False, f'prepare failed: {reason}')
                return False
            starts[side] = self.position[side]
            targets[side] = self.target_for(starts[side])
            print(
                f'{side}: start={starts[side]} target={targets[side]} '
                f'current_register={self._status(side, "max_current_register")}')

        for side in active_sides:
            message = Int32()
            message.data = targets[side]
            self.position_publishers[side].publish(message)

        for side in active_sides:
            ok, reason = self.wait_for_displacement(
                side, starts[side], trigger)
            if not ok:
                print(f'{side}: motion trigger failed: {reason}')
                for other in active_sides:
                    self.set_enabled(other, False)
                self._row(case, active_sides, speed, {}, targets, 0.0, 0,
                          False, f'{side}: {reason}')
                return False

        trigger_positions = {
            side: self.position[side] for side in active_sides}
        calls = {
            side: (self.estop_clients[side], Empty.Request())
            for side in active_sides
        }
        requested_at = time.monotonic()
        responses = self.call_many(calls, timeout=5.0)
        stopped_at = time.monotonic()
        latency = stopped_at - requested_at

        reasons: list[str] = []
        latched_ok: dict[str, bool] = {}
        for side in active_sides:
            response, error = responses[side]
            if error:
                reasons.append(f'{side}: e-stop call failed: {error}')
                continue
            if response is None:
                reasons.append(f'{side}: e-stop returned no response')
                continue
            latched_ok[side] = self.wait_until(
                    lambda s=side: (
                        self._status(s, 'state') == 'fault'
                        and self._status(s, 'enabled') == 'false'
                        and self._status(s, 'estop_latched') == 'true'
                    ),
                    timeout=2.0,
            )
            if not latched_ok[side]:
                reasons.append(
                    f'{side}: e-stop state not confirmed: '
                    f'state={self._status(side, "state")}, '
                    f'enabled={self._status(side, "enabled")}, '
                    f'latched={self._status(side, "estop_latched")}')

        travels: dict[str, int] = {}
        finals: dict[str, int] = {}
        for side in active_sides:
            travel, final = self.record_position(side, post_window)
            travels[side] = travel
            finals[side] = final
            if travel > max_stop_travel:
                reasons.append(
                    f'{side}: post-stop travel {travel} > {max_stop_travel}')
            if self._status(side, 'motion_active') == 'true':
                reasons.append(f'{side}: motion_active remained true')

        enable_rejections: dict[str, bool] = {}
        for side in active_sides:
            ok, message = self.set_enabled(side, True)
            enable_rejections[side] = not ok
            if ok:
                reasons.append(
                    f'{side}: enable unexpectedly succeeded after e-stop')
            else:
                print(f'{side}: e-stop latch rejected enable: {message}')

        clear_ok: dict[str, bool] = {}
        for side in active_sides:
            ok, message = self.clear_fault(side)
            clear_ok[side] = ok
            if not ok:
                reasons.append(f'{side}: clear_fault failed: {message}')
            else:
                print(f'{side}: clear_fault: {message}')

        recovery_ok: dict[str, bool] = {}
        for side in active_sides:
            if not clear_ok.get(side, False):
                recovery_ok[side] = False
                continue
            ok, message = self.recover_side(side, starts[side])
            recovery_ok[side] = ok
            if not ok:
                reasons.append(f'{side}: {message}')

        passed = not reasons
        for side in active_sides:
            print(
                f'{side}: trigger={trigger_positions[side]} '
                f'stop_pos={finals[side]} post_travel={travels[side]} '
                f'latched={latched_ok.get(side, False)} '
                f'recovery={"PASS" if recovery_ok.get(side) else "FAIL"}')
        print(
            f'result={"PASS" if passed else "FAIL"}; '
            f'latency={latency:.3f}s; '
            f'reason={"; ".join(reasons) if reasons else "ok"}')
        self._row(case, active_sides, speed, starts, targets, latency,
                  max(travels.values(), default=0), passed,
                  '; '.join(reasons) if reasons else 'ok',
                  trigger_positions=trigger_positions, finals=finals,
                  enable_rejections=enable_rejections,
                  clear_ok=clear_ok, recovery_ok=recovery_ok)
        return passed

    def _row(self, case: str, active_sides: list[str], speed: int,
             starts: dict[str, int], targets: dict[str, int],
             latency: float, post_travel: int, passed: bool, reason: str,
             trigger_positions: dict[str, int] | None = None,
             finals: dict[str, int] | None = None,
             enable_rejections: dict[str, bool] | None = None,
             clear_ok: dict[str, bool] | None = None,
             recovery_ok: dict[str, bool] | None = None) -> None:
        trigger_positions = trigger_positions or {}
        finals = finals or {}
        enable_rejections = enable_rejections or {}
        clear_ok = clear_ok or {}
        recovery_ok = recovery_ok or {}
        for side in active_sides:
            self.rows.append({
                'case': case,
                'side': side,
                'speed_percent': str(speed),
                'start_position': str(starts.get(side, '')),
                'target_position': str(targets.get(side, '')),
                'trigger_position': str(trigger_positions.get(side, '')),
                'position_at_stop': str(finals.get(side, '')),
                'post_stop_travel': str(post_travel),
                'stop_latency_sec': f'{latency:.3f}',
                'enable_rejected': str(enable_rejections.get(side, '')),
                'clear_fault_ok': str(clear_ok.get(side, '')),
                'recovery_ok': str(recovery_ok.get(side, '')),
                'result': 'PASS' if passed else 'FAIL',
                'reason': reason,
            })

    def write_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        fields = [
            'case', 'side', 'speed_percent', 'start_position',
            'target_position', 'trigger_position', 'position_at_stop',
            'post_stop_travel', 'stop_latency_sec', 'enable_rejected',
            'clear_fault_ok', 'recovery_ok', 'result', 'reason',
        ]
        with path.open('w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--sides', choices=('left', 'right', 'both'), default='both')
    parser.add_argument('--speed', type=int, default=10)
    parser.add_argument('--trigger-travel', type=int, default=40)
    parser.add_argument('--post-stop-window', type=float, default=1.5)
    parser.add_argument('--max-stop-travel', type=int, default=20)
    parser.add_argument('--timeout-sec', type=float, default=10.0)
    parser.add_argument('--csv', type=Path, default=DEFAULT_CSV)
    args = parser.parse_args()
    if not 1 <= args.speed <= 100:
        parser.error('--speed must be 1..100')
    if args.trigger_travel <= 0:
        parser.error('--trigger-travel must be positive')
    if args.post_stop_window <= 0:
        parser.error('--post-stop-window must be positive')
    if args.max_stop_travel < 0:
        parser.error('--max-stop-travel must be non-negative')

    sides = list(SIDES) if args.sides == 'both' else [args.sides]
    rclpy.init()
    node = GripperEstopTest(sides, args.timeout_sec)
    try:
        ordered: list[tuple[str, list[str]]] = []
        if args.sides == 'both':
            ordered.extend([('left', ['left']), ('right', ['right'])])
            ordered.append(('both', list(SIDES)))
        else:
            ordered.append((args.sides, [args.sides]))

        all_passed = True
        for case, active_sides in ordered:
            passed = node.run_case(
                case,
                active_sides,
                args.speed,
                args.trigger_travel,
                args.post_stop_window,
                args.max_stop_travel,
            )
            all_passed = all_passed and passed
        node.write_csv(args.csv)
        print(
            f'\nRESULT: {"PASS" if all_passed else "FAIL"}, '
            f'CSV={args.csv}')
        return 0 if all_passed else 1
    finally:
        for side in sides:
            try:
                node.set_enabled(side, False)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
