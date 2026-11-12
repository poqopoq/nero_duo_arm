#!/usr/bin/python3
"""Recover a NERO arm whose power-off pose drifted outside joint limits.

The physical stop pose can legitimately be outside the software envelope: with
the grippers installed, joint4 settles near 2.20 rad (cap 2.14) and joint7 can
pass 1.57 rad.  MoveIt rejects that state as an invalid start state, so the
normal Park/planning path cannot recover it.  This script talks directly to the
driver first:

1. close the control gate;
2. clear latched joint faults and reset the motion controller;
3. enable the motors and verify that all seven brakes released;
4. move joint7 first, then any other slightly out-of-limit joint, and finally
   joint4 to the safe support-pose targets (this ordering clears the wrist
   collision before moving joint4);
5. close the gate and keep the motors enabled so gravity cannot pull the arm
   back to the mechanical stops.

It refuses to move when feedback is stale, the required correction is too
large, or a configured target is outside the deployment limits.  Motion is
sent as a smooth sequence of small joint steps instead of changing the driver
speed at runtime, so a driver launched at 100% can still be recovered safely.
``--watch`` keeps this process alive, performs one startup check, and repeats
the check after every motor-disabled -> motor-enabled transition.

The physical stop pose is intentionally not written into ``support_pose.yaml``:
that file is the in-limit MoveIt Park target, not the power-off resting
position.
"""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import math
import os
import sys
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, Trigger

SIDES = ('left', 'right')
# Keep in sync with dual_nero_vla_bridge/launch/bimanual_vla_bridge.launch.py.
JOINT_MIN = [-2.70526, -1.760, -2.75, -1.01, -2.75, -0.7330383, -1.5707963]
JOINT_MAX = [2.70526, 1.74, 2.75, 2.14, 2.75, 0.8726646, 1.5707963]
JOINT4_INDEX = 3
JOINT7_INDEX = 6
FEEDBACK_TIMEOUT_SEC = 1.0


def plan_corrections(
    joints: list[float],
    targets: dict[int, float],
    max_corrections: dict[int, float],
    cap_margin: float,
    max_other_correction: float,
) -> dict[int, float]:
    """Return the joint targets needed to get back inside the envelope.

    ``targets`` contains the preferred recovery targets for joint4/joint7.
    Other joints are only trimmed to the nearest boundary.  A correction that
    exceeds its guard limit is an error, not an invitation to move farther.
    """
    corrections: dict[int, float] = {}
    for index, value in enumerate(joints):
        low, high = JOINT_MIN[index], JOINT_MAX[index]
        if index in targets:
            target = targets[index]
            if value > high - cap_margin:
                new_value = target
                limit = max_corrections[index]
            elif value < low:
                new_value = target
                limit = max_corrections[index]
            else:
                continue
        else:
            if value > high:
                new_value = high - cap_margin
            elif value < low:
                new_value = low + cap_margin
            else:
                continue
            limit = max_other_correction

        correction = new_value - value
        if abs(correction) > limit + 1e-9:
            raise ValueError(
                f'joint{index + 1}={value:.5f} needs {correction:+.5f} rad '
                f'but the guard is {limit:.5f} rad')
        if abs(correction) > 1e-6:
            corrections[index] = new_value
    return corrections


def inspect_plan(
    node,
    sides: tuple[str, ...],
    targets: dict[int, float],
    max_corrections: dict[int, float],
    cap_margin: float,
    max_other_correction: float,
    label: str,
) -> tuple[dict[str, list[float]], dict[str, dict[int, float]]]:
    """Read fresh feedback and build the guarded recovery plan."""
    current: dict[str, list[float]] = {}
    needs: dict[str, dict[int, float]] = {}
    for side in sides:
        joints = node.read_joints(side)
        if joints is None:
            raise RuntimeError(f'{side}: joint feedback unavailable or stale')
        print(f'{side}: {label}={[round(value, 5) for value in joints]}')
        side_targets = plan_corrections(
            joints, targets, max_corrections,
            cap_margin, max_other_correction)
        for index, target in sorted(side_targets.items()):
            print(
                f'{side}: joint{index + 1}={joints[index]:.5f} -> '
                f'{target:.5f} '
                f'(correction {target - joints[index]:+.5f} rad)')
        if side_targets:
            needs[side] = side_targets
        else:
            print(f'{side}: already inside the deployment envelope')
        current[side] = joints
    return current, needs


class JointLimitRecover(Node):
    def __init__(self, feedback_timeout: float = 5.0) -> None:
        super().__init__('nero_joint_limit_recover')
        self.feedback_timeout = feedback_timeout
        self._feedback: dict[str, JointState] = {}
        self._feedback_mono: dict[str, float] = {}
        self._motor_status: dict[str, dict[str, dict[str, str]]] = {}
        self._arm_status: dict[str, dict[str, str]] = {}
        self._joint_command_publishers = {}
        self._gates = {}
        self._enablers = {}
        self._stops = {}
        self._clear_errors = {}
        self._reset_motion = {}
        for side in SIDES:
            self.create_subscription(
                JointState,
                f'/{side}/feedback/joint_states',
                lambda msg, side=side: self._store_feedback(side, msg),
                10,
            )
            self.create_subscription(
                DiagnosticArray,
                f'/{side}/feedback/motor_status',
                lambda msg, side=side: self._store_motor_status(side, msg),
                10,
            )
            self.create_subscription(
                DiagnosticArray,
                f'/{side}/feedback/arm_status',
                lambda msg, side=side: self._store_arm_status(side, msg),
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
            self._clear_errors[side] = self.create_client(
                Trigger, f'/{side}/clear_joint_error')
            self._reset_motion[side] = self.create_client(
                Trigger, f'/{side}/reset_motion')

    def _store_feedback(self, side: str, msg: JointState) -> None:
        self._feedback[side] = msg
        self._feedback_mono[side] = time.monotonic()

    def _store_motor_status(self, side: str, msg: DiagnosticArray) -> None:
        self._motor_status[side] = {
            status.name: {item.key: item.value for item in status.values}
            for status in msg.status
        }

    def _store_arm_status(self, side: str, msg: DiagnosticArray) -> None:
        if msg.status:
            self._arm_status[side] = {
                item.key: item.value for item in msg.status[0].values}

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

    def call_trigger(self, client, timeout: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return bool(
            future.done() and future.result() is not None and future.result().success)

    def all_motors_enabled(self, side: str) -> Optional[bool]:
        statuses = self._motor_status.get(side)
        if not statuses:
            return None
        states = []
        for index in range(1, 8):
            entry = statuses.get(f'joint{index}')
            if entry is None or entry.get('enabled') not in ('true', 'false'):
                return None
            states.append(entry['enabled'] == 'true')
        return all(states)

    def wait_for_motor_state(self, side: str, enabled: bool, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            state = self.all_motors_enabled(side)
            if state is not None and state == enabled:
                return True
        return False

    def close_gate(self, side: str) -> bool:
        return self.call_setbool(self._gates[side], False, timeout=5.0)

    def clear_faults_and_reset(self, side: str) -> None:
        """Clear a latched angle-limit/brake fault before asking for enable."""
        self.close_gate(side)
        if self.call_trigger(self._clear_errors[side]):
            print(f'{side}: joint faults cleared')
        else:
            print(f'{side}: WARNING clear_joint_error unavailable or failed')
        if self.call_trigger(self._reset_motion[side]):
            print(f'{side}: motion controller reset')
        else:
            print(f'{side}: WARNING reset_motion unavailable or failed')
        self.spin_once(0.1)

    def enable_with_fault_clear(self, side: str, attempts: int = 3) -> bool:
        """Enable a side, clearing latched faults and verifying brake release."""
        if self.wait_for_motor_state(side, True, 0.5):
            print(f'{side}: motors already enabled')
            return True
        for attempt in range(1, attempts + 1):
            self.clear_faults_and_reset(side)
            ok = self.call_setbool(
                self._enablers[side], True, timeout=12.0)
            if ok and self.wait_for_motor_state(side, True, 3.0):
                print(f'{side}: motors enabled on attempt {attempt}')
                return True
            print(
                f'{side}: enable attempt {attempt}/{attempts} failed; '
                'retrying after fault reset')
            time.sleep(0.5)
        return False

    def emergency_stop(self, side: str) -> None:
        client = self._stops[side]
        if client.service_is_ready():
            client.call_async(Empty.Request())
            self.spin_once(0.3)

    def publish_ramp(
        self, side: str, start: list[float], targets: dict[int, float],
        max_step_rad: float, rate_hz: float,
    ) -> None:
        """Move with many small targets instead of changing driver speed.

        The driver's position-velocity mode smooths ``move_j`` targets.  Keeping
        every command step small bounds the effective recovery speed even when
        the driver was explicitly started at 100%.
        """
        largest = max(
            abs(target - start[index]) for index, target in targets.items())
        # Smoothstep has a peak slope of 1.5, so include it in the step budget.
        steps = max(1, int(math.ceil(1.5 * largest / max_step_rad)))
        period = 1.0 / rate_hz
        print(
            f'{side}: ramping {largest:.5f} rad in {steps} steps '
            f'at {rate_hz:.0f} Hz (max step {max_step_rad:.5f} rad)')
        message = JointState()
        message.name = [f'joint{index}' for index in range(1, 8)]
        for step in range(steps + 1):
            ratio = step / steps
            blend = ratio * ratio * (3.0 - 2.0 * ratio)
            message.position = [
                start[index] + (targets.get(index, start[index]) - start[index]) * blend
                for index in range(7)
            ]
            message.header.stamp = self.get_clock().now().to_msg()
            self._joint_command_publishers[side].publish(message)
            if step % 10 == 0:
                enabled = self.all_motors_enabled(side)
                if enabled is False:
                    raise RuntimeError('motors were disabled during recovery')
            self.spin_once(period)
        for _ in range(3):
            message.header.stamp = self.get_clock().now().to_msg()
            self._joint_command_publishers[side].publish(message)
            self.spin_once(period)

    def wait_settled(
        self, side: str, start: list[float], targets: dict[int, float],
        tolerances: dict[int, float], timeout: float, max_drift: float,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            joints = self.read_joints(side)
            if joints is None:
                return False, 'feedback became stale'
            drift = max(
                abs(value - previous)
                for index, (value, previous) in enumerate(zip(joints, start))
                if index not in targets
            )
            if drift > max_drift:
                return False, f'non-target joint drift {drift:.4f} rad exceeds {max_drift:.4f}'
            errors = {
                index: abs(joints[index] - target)
                for index, target in targets.items()
            }
            if all(errors[index] <= tolerances[index] for index in targets):
                detail = ', '.join(
                    f'joint{index + 1}={joints[index]:.5f}'
                    for index in sorted(targets)
                )
                return True, detail
        return False, 'settle timeout'


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Keep joint4/joint7 inside the NERO deployment limits.')
    parser.add_argument('--side', choices=('both', *SIDES), default='both')
    parser.add_argument('--target', type=float, default=2.115,
                        help='Target for joint4 (default: 2.115 rad).')
    parser.add_argument('--joint7-target', type=float, default=1.545,
                        help='Target for joint7 (default: 1.545 rad).')
    parser.add_argument('--cap-margin', type=float, default=0.005,
                        help='Treat joint4/joint7 above max-margin as needing recovery.')
    parser.add_argument('--max-correction', type=float, default=0.20,
                        help='Maximum joint4 correction in rad (default: 0.20).')
    parser.add_argument('--joint7-max-correction', type=float, default=0.30,
                        help='Maximum joint7 correction in rad (default: 0.30).')
    parser.add_argument(
        '--max-other-correction', type=float, default=0.12,
        help='Maximum boundary trim for joints other than joint4/joint7 '
             '(default: 0.12 rad).')
    parser.add_argument('--enable-attempts', type=int, default=3,
                        help='Enable attempts after clearing latched faults.')
    parser.add_argument(
        '--max-step-rad', type=float, default=0.0015,
        help='Largest joint step in each recovery command (default: 0.0015 rad).')
    parser.add_argument(
        '--ramp-rate-hz', type=float, default=50.0,
        help='Recovery command rate in Hz (default: 50).')
    parser.add_argument(
        '--settle-tolerance', type=float, default=0.012,
        help='Final recovery target tolerance in rad.')
    parser.add_argument('--max-other-joint-drift', type=float, default=0.03)
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Per-phase settle timeout in seconds.')
    parser.add_argument('--feedback-wait-sec', type=float, default=12.0,
                        help='How long to wait for fresh joint feedback.')
    parser.add_argument('--lock-timeout', type=float, default=180.0,
                        help='Maximum wait for another recovery process.')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument(
        '--ensure-enabled', action='store_true',
        help='Enable the requested side even when it is already inside limits.')
    parser.add_argument('--disable', action='store_true',
                        help='Disable motors after recovery (allows gravity sag).')
    parser.add_argument(
        '--watch', action='store_true',
        help='Stay resident: recover once at startup and after every '
             'motor-disabled -> motor-enabled transition.')
    parser.add_argument(
        '--startup-delay', type=float, default=0.0,
        help='Additional delay before the first recovery check in watch mode.')
    parser.add_argument(
        '--enable-settle-sec', type=float, default=0.8,
        help='Wait for brake release and feedback after an enable transition.')
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> Optional[str]:
    targets = {JOINT4_INDEX: args.target, JOINT7_INDEX: args.joint7_target}
    for index, target in targets.items():
        low, high = JOINT_MIN[index], JOINT_MAX[index]
        if not low + 1e-4 <= target <= high - 1e-4:
            return (
                f'joint{index + 1} target {target:.5f} must be inside '
                f'[{low:.5f}, {high:.5f}]')
    if (args.max_correction <= 0.0 or
            args.joint7_max_correction <= 0.0 or
            args.max_other_correction <= 0.0 or
            args.enable_attempts < 1 or
            args.max_step_rad <= 0.0 or
            args.ramp_rate_hz <= 0.0 or
            args.settle_tolerance <= 0.0 or
            args.timeout <= 0.0 or
            args.feedback_wait_sec <= 0.0 or
            args.lock_timeout <= 0.0 or
            args.startup_delay < 0.0 or
            args.enable_settle_sec < 0.0):
        return 'correction, motion, timeout and rate guards must be positive'
    if args.watch and (args.dry_run or args.disable):
        return '--watch cannot be combined with --dry-run or --disable'
    return None


@contextlib.contextmanager
def _recovery_lock(timeout: float):
    """Serialize manual and resident recovery runs across processes."""
    lock_path = os.environ.get(
        'NERO_JOINT_LIMIT_LOCK', '/tmp/nero_duo_joint_limit_recovery.lock')
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    deadline = time.monotonic() + timeout
    try:
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f'another recovery process held {lock_path} for '
                        f'more than {timeout:.0f} s')
                time.sleep(0.1)
        yield
    finally:
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)


def _spin_for(node: JointLimitRecover, duration: float) -> None:
    deadline = time.monotonic() + max(0.0, duration)
    while rclpy.ok() and time.monotonic() < deadline:
        node.spin_once(min(0.1, max(0.0, deadline - time.monotonic())))


def _run_recovery(
    node: JointLimitRecover, args: argparse.Namespace,
    sides: tuple[str, ...], reason: str,
) -> int:
    """Run one guarded recovery check and return the process exit code."""
    print(f'CHECK: {reason}; sides={",".join(sides)}')
    targets = {JOINT4_INDEX: args.target, JOINT7_INDEX: args.joint7_target}
    max_corrections = {
        JOINT4_INDEX: args.max_correction,
        JOINT7_INDEX: args.joint7_max_correction,
    }

    deadline = time.monotonic() + args.feedback_wait_sec
    while time.monotonic() < deadline:
        node.spin_once(0.1)
        if all(node.read_joints(side) is not None for side in sides):
            break
    else:
        print('REFUSED: joint feedback unavailable or stale')
        return 2

    if not args.dry_run:
        for side in sides:
            node.close_gate(side)

    try:
        current, needs = inspect_plan(
            node, sides, targets, max_corrections,
            args.cap_margin, args.max_other_correction, 'current')
    except (RuntimeError, ValueError) as exc:
        print(f'REFUSED: {exc}')
        return 2

    if args.dry_run:
        if not needs:
            print('RESULT: ALREADY_INSIDE_LIMITS')
        else:
            print('RESULT: DRY_RUN')
        return 0

    enable_sides = tuple(needs) if needs else (
        sides if args.ensure_enabled else ())
    if not enable_sides:
        print('RESULT: ALREADY_INSIDE_LIMITS')
        return 0

    failure = None
    for side in enable_sides:
        if not node.enable_with_fault_clear(side, args.enable_attempts):
            failure = f'{side}: failed to enable all motors'
            break
    if failure is not None:
        for side in sides:
            node.emergency_stop(side)
            node.close_gate(side)
        print(f'FAILED: {failure}')
        print('MANUAL_SUPPORT_REQUIRED: check brake release / joint faults')
        return 3

    if not needs:
        _spin_for(node, 1.0)
        try:
            current, needs = inspect_plan(
                node, sides, targets, max_corrections,
                args.cap_margin, args.max_other_correction,
                'post-enable current')
        except (RuntimeError, ValueError) as exc:
            for side in sides:
                node.emergency_stop(side)
                node.close_gate(side)
            print(f'FAILED: post-enable check: {exc}')
            return 3
        if not needs:
            for side in sides:
                node.close_gate(side)
            print('RESULT: ENABLED_INSIDE_LIMITS')
            return 0

    active_sides = tuple(needs)
    for side in active_sides:
        if not node.call_setbool(node._gates[side], True, timeout=5.0):
            failure = f'{side}: control gate did not open'
            break
    if failure is not None:
        for side in sides:
            node.emergency_stop(side)
            node.close_gate(side)
        print(f'FAILED: {failure}')
        return 3

    # Brake release can let a gravity-loaded joint sag. Re-read feedback after
    # the gate opens and replan from that live pose.
    _spin_for(node, 1.0)
    try:
        current, needs = inspect_plan(
            node, active_sides, targets, max_corrections,
            args.cap_margin, args.max_other_correction,
            'post-enable current')
    except (RuntimeError, ValueError) as exc:
        for side in active_sides:
            node.emergency_stop(side)
            node.close_gate(side)
        print(f'FAILED: post-enable replan: {exc}')
        print('MANUAL_SUPPORT_REQUIRED: check brake release / joint faults')
        return 3
    if not needs:
        for side in active_sides:
            node.close_gate(side)
        print('RESULT: ALREADY_INSIDE_LIMITS_AFTER_ENABLE')
        return 0

    pending = {index for targets_for_side in needs.values()
               for index in targets_for_side}
    order = [JOINT7_INDEX]
    order.extend(sorted(index for index in pending
                        if index not in (JOINT4_INDEX, JOINT7_INDEX)))
    order.append(JOINT4_INDEX)

    failure = None
    for index in order:
        phase = {
            side: {index: side_targets[index]}
            for side, side_targets in needs.items() if index in side_targets
        }
        if not phase:
            continue
        print(f'PHASE joint{index + 1}: ' + ', '.join(sorted(phase)))
        for side in phase:
            try:
                node.publish_ramp(
                    side, current[side], phase[side],
                    args.max_step_rad, args.ramp_rate_hz)
            except Exception as exc:  # noqa: BLE001
                failure = f'{side}: recovery ramp aborted: {exc}'
                break
        if failure is not None:
            break
        for side, side_targets in phase.items():
            correction = abs(side_targets[index] - current[side][index])
            tolerance = min(
                args.settle_tolerance, max(0.002, correction * 0.5))
            ok, detail = node.wait_settled(
                side, current[side], side_targets,
                {index: tolerance}, args.timeout,
                args.max_other_joint_drift)
            if not ok:
                failure = f'{side}: {detail}'
                break
        if failure is not None:
            break
        for side in phase:
            latest = node.read_joints(side)
            if latest is None:
                failure = f'{side}: feedback became stale after phase'
                break
            current[side] = latest
        if failure is not None:
            break

    for side in needs:
        node.close_gate(side)

    if failure is not None:
        for side in needs:
            node.emergency_stop(side)
        if args.disable:
            for side in needs:
                node.call_setbool(node._enablers[side], False)
        print(f'FAILED: {failure}')
        print('MANUAL_SUPPORT_REQUIRED: recovery stopped inside the guarded path')
        return 3

    if args.disable:
        for side in needs:
            node.call_setbool(node._enablers[side], False)
    print(
        'RESULT: MOVED_AND_DISABLED' if args.disable
        else 'RESULT: MOVED_KEEP_ENABLED')
    for side in sides:
        joints = node.read_joints(side)
        if joints is not None:
            print(f'{side}: final all={[round(value, 4) for value in joints]}')
        limit_flags = node._arm_status.get(side, {}).get('joint_angle_limit')
        if limit_flags is not None:
            print(f'{side}: hardware joint_angle_limit={limit_flags}')
    return 0


def _wait_until_ready(
    node: JointLimitRecover, sides: tuple[str, ...], timeout: float,
) -> bool:
    deadline = time.monotonic() + timeout
    last_warning = 0.0
    while rclpy.ok():
        node.spin_once(0.1)
        feedback = all(node.read_joints(side) is not None for side in sides)
        status = all(node.all_motors_enabled(side) is not None for side in sides)
        if feedback and status:
            return True
        now = time.monotonic()
        if now >= deadline and now - last_warning > 5.0:
            print(
                'WATCH: waiting for fresh joint feedback and motor status '
                f'(feedback={feedback}, motor_status={status})')
            last_warning = now
    return False


def _watch(
    node: JointLimitRecover, args: argparse.Namespace, sides: tuple[str, ...],
) -> int:
    print('WATCH: resident joint-limit guard armed')
    if not _wait_until_ready(node, sides, args.feedback_wait_sec):
        if not rclpy.ok():
            return 0
        print('WATCH: feedback not ready yet; continuing to wait')
        if not _wait_until_ready(node, sides, float('inf')):
            return 0

    _spin_for(node, args.startup_delay)
    try:
        with _recovery_lock(args.lock_timeout):
            _run_recovery(node, args, sides, 'startup')
    except Exception as exc:  # noqa: BLE001
        print(f'WATCH: startup recovery aborted: {exc}')

    previous = {
        side: node.all_motors_enabled(side) for side in sides
    }
    print(f'WATCH: tracking enable transitions; state={previous}')
    while rclpy.ok():
        node.spin_once(0.1)
        for side in sides:
            enabled = node.all_motors_enabled(side)
            if enabled is None:
                continue
            before = previous.get(side)
            if before is False and enabled is True:
                print(f'WATCH: {side} motors disabled -> enabled')
                previous[side] = True
                _spin_for(node, args.enable_settle_sec)
                try:
                    with _recovery_lock(args.lock_timeout):
                        _run_recovery(node, args, (side,), 'enable transition')
                except Exception as exc:  # noqa: BLE001
                    print(f'WATCH: {side} transition recovery aborted: {exc}')
                previous[side] = node.all_motors_enabled(side)
            else:
                previous[side] = enabled
    return 0


def main() -> int:
    args = _parse_args()
    error = _validate_args(args)
    if error is not None:
        print(f'REFUSED: {error}')
        return 2
    sides = SIDES if args.side == 'both' else (args.side,)

    rclpy.init()
    node = JointLimitRecover()
    try:
        if args.watch:
            return _watch(node, args, sides)
        with _recovery_lock(args.lock_timeout):
            return _run_recovery(node, args, sides, 'manual')
    except TimeoutError as exc:
        print(f'REFUSED: {exc}')
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
