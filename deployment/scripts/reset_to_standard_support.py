#!/usr/bin/env python3
"""Plan and execute a guarded return to the stored NERO support pose."""
import json
import os
import sys
import time
from pathlib import Path

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool

SIDES = ('left', 'right')
TOLERANCE = 0.03
MOVE_SCALE = 0.05
POST_DISABLE_SETTLE_SEC = 5.0
POST_DISABLE_STABILITY_SEC = 5.0
POST_DISABLE_DRIFT_TOLERANCE = 0.01
ROOT = Path(os.environ.get('NERO_DUO_ROOT', Path(__file__).resolve().parents[2]))
SUPPORT_FILE = ROOT / 'deployment' / 'config' / 'support_pose.yaml'
NAMES = [f'{side}_joint{i}' for side in SIDES for i in range(1, 8)]


class SupportReset(Node):
    def __init__(self, target, disable_after_reset=False, plan_only=False):
        super().__init__('reset_to_standard_support')
        self.target = target
        self.disable_after_reset = disable_after_reset
        self.plan_only = plan_only
        self.feedback = {side: None for side in SIDES}
        self.arm_status = {side: None for side in SIDES}
        self.motor_status = {side: None for side in SIDES}
        self.bridge_state = None
        self.enable_clients = {}
        self.gate_clients = {}
        self.stop_clients = {}
        for side in SIDES:
            self.create_subscription(
                JointState, f'/{side}/feedback/joint_states',
                lambda msg, s=side: self._feedback_cb(s, msg), 10)
            self.create_subscription(
                DiagnosticArray, f'/{side}/feedback/arm_status',
                lambda msg, s=side: self._arm_status_cb(s, msg), 10)
            self.create_subscription(
                DiagnosticArray, f'/{side}/feedback/motor_status',
                lambda msg, s=side: self._motor_status_cb(s, msg), 10)
            self.enable_clients[side] = self.create_client(
                SetBool, f'/{side}/enable_agx_arm')
            self.gate_clients[side] = self.create_client(
                SetBool, f'/{side}/control_enable')
            self.stop_clients[side] = self.create_client(
                Empty, f'/{side}/emergency_stop')
        self.create_subscription(JointState, '/joint_states', self._bridge_cb, 10)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.motors_enabled = False
        self.finalized = False

    def _feedback_cb(self, side, msg):
        values = dict(zip(msg.name, msg.position))
        if all(f'joint{i}' in values for i in range(1, 8)):
            self.feedback[side] = [float(values[f'joint{i}']) for i in range(1, 8)]

    def _bridge_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in NAMES):
            self.bridge_state = [float(values[name]) for name in NAMES]

    def _arm_status_cb(self, side, msg):
        if msg.status:
            self.arm_status[side] = {
                item.key: item.value for item in msg.status[0].values}

    def _motor_status_cb(self, side, msg):
        self.motor_status[side] = {
            status.name: {item.key: item.value for item in status.values}
            for status in msg.status
        }

    def spin_once(self, timeout=0.05):
        rclpy.spin_once(self, timeout_sec=timeout)

    def wait_ready(self, timeout=45.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            self.spin_once(0.1)
            services_ready = all(
                client.service_is_ready()
                for group in (self.enable_clients, self.gate_clients, self.stop_clients)
                for client in group.values())
            if (services_ready and self.move_client.server_is_ready() and
                    all(value is not None for value in self.feedback.values()) and
                    self.bridge_state is not None):
                return True
        return False

    def current_pose(self):
        return {
            side: [self.target[f'{side}_joint{i}'] for i in range(1, 8)]
            for side in SIDES
        }

    def live_pose(self):
        return {side: list(self.feedback[side]) for side in SIDES}

    def max_error(self, positions):
        return max(
            abs(actual - desired)
            for side in SIDES
            for actual, desired in zip(positions[side], self.current_pose()[side]))

    def motor_enabled_states(self):
        states = {}
        for side in SIDES:
            statuses = self.motor_status[side]
            if not statuses:
                return None
            for index in range(1, 8):
                entry = statuses.get(f'joint{index}')
                if entry is None or entry.get('enabled') not in ('true', 'false'):
                    return None
                states[f'{side}_joint{index}'] = entry['enabled'] == 'true'
        return states

    def hardware_limit_flags(self):
        flags = {}
        for side in SIDES:
            values = self.arm_status[side]
            if not values:
                return None
            try:
                joint_flags = json.loads(values['joint_angle_limit'])
            except (KeyError, TypeError, ValueError):
                return None
            if len(joint_flags) != 7:
                return None
            flags[side] = [bool(flag) for flag in joint_flags]
        return flags

    def wait_for_hardware_status(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            self.spin_once(0.05)
            if (self.motor_enabled_states() is not None and
                    self.hardware_limit_flags() is not None):
                return True
        return False

    def call_setbool(self, side, client, value, timeout=40.0):
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            return False
        return bool(future.result().success)

    def call_stops(self):
        futures = {}
        for side in SIDES:
            if self.stop_clients[side].service_is_ready():
                futures[side] = self.stop_clients[side].call_async(Empty.Request())
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and rclpy.ok():
            self.spin_once(0.02)
            if all(future.done() for future in futures.values()):
                break
        return {
            side: future.done() and future.result() is not None
            for side, future in futures.items()
        }

    def close_gates(self):
        for side in SIDES:
            self.call_setbool(side, self.gate_clients[side], False)

    def disable_both(self):
        results = {}
        for side in SIDES:
            results[side] = self.call_setbool(
                side, self.enable_clients[side], False)
        return results

    def make_goal(self, plan_only):
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'both_arms'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = MOVE_SCALE
        goal.request.max_acceleration_scaling_factor = MOVE_SCALE
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        for name in NAMES:
            constraint = JointConstraint()
            constraint.joint_name = name
            side, index = name.split('_joint')
            constraint.position = float(self.target[name])
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = False
        goal.planning_options.look_around = False
        return goal

    def run_goal(self, plan_only, timeout):
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            return None
        send_future = self.move_client.send_goal_async(self.make_goal(plan_only))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done() or result_future.result() is None:
            return None
        return result_future.result().result

    def execute(self):
        if not self.wait_ready():
            print('PREFLIGHT_READY_TIMEOUT')
            return 2

        start = self.live_pose()
        print('LIVE_START', {
            side: [round(value, 6) for value in start[side]] for side in SIDES})
        print('MAX_START_ERROR', round(self.max_error(start), 6))

        plan = self.run_goal(True, 35.0)
        if plan is None or plan.error_code.val != 1:
            code = None if plan is None else plan.error_code.val
            print('PLAN_FAILED', code)
            return 3
        points = plan.planned_trajectory.joint_trajectory.points
        duration = 0.0
        if points:
            last = points[-1].time_from_start
            duration = float(last.sec) + float(last.nanosec) * 1e-9
        print('PLAN_OK', {'points': len(points), 'duration_sec': round(duration, 3)})
        if self.plan_only:
            print('PLAN_ONLY', True)
            print('AT_SUPPORT', False)
            return 0

        after_plan = self.live_pose()
        plan_drift = max(
            abs(a - b)
            for side in SIDES
            for a, b in zip(start[side], after_plan[side]))
        print('PRE_ENABLE_DRIFT', round(plan_drift, 6))
        if plan_drift > 0.02:
            print('POSE_CHANGED_BEFORE_ENABLE')
            return 4

        for side in SIDES:
            if not self.call_setbool(side, self.enable_clients[side], True):
                print(f'{side.upper()}_ENABLE_FAILED')
                self.call_stops()
                self.close_gates()
                self.disable_both()
                return 5
        self.motors_enabled = True
        print('MOTORS_ENABLED', True)

        result = self.run_goal(False, 70.0)
        if result is None:
            print('EXECUTION_NO_RESULT')
            self.call_stops()
            return 6
        print('MOVEIT_ERROR_CODE', result.error_code.val)
        if result.error_code.val != 1:
            self.call_stops()
            print('MANUAL_SUPPORT_REQUIRED: motors remain enabled')
            return 7

        settle_end = time.monotonic() + 1.5
        while time.monotonic() < settle_end and rclpy.ok():
            self.spin_once(0.05)
        final = self.live_pose()
        error = self.max_error(final)
        print('FINAL_POSE', {
            side: [round(value, 6) for value in final[side]] for side in SIDES})
        print('MAX_SUPPORT_ERROR', round(error, 6))

        if self.disable_after_reset:
            status_ready = self.wait_for_hardware_status()
            limit_flags = self.hardware_limit_flags()
            print('PRE_DISABLE_STATUS_READY', status_ready)
            print('PRE_DISABLE_ARM_STATUS', self.arm_status)
            print('PRE_DISABLE_LIMIT_FLAGS', limit_flags)
            if not status_ready:
                print('PRE_DISABLE_STATUS_TIMEOUT')
            elif any(any(flags) for flags in limit_flags.values()):
                print('HARDWARE_JOINT_LIMIT_EXCEEDED')
            if not status_ready or any(
                    any(flags) for flags in limit_flags.values()):
                self.call_stops()
                self.close_gates()
                print('MANUAL_SUPPORT_REQUIRED: motors remain enabled')
                self.finalized = True
                return 11

        stops = self.call_stops()
        self.close_gates()
        if error <= TOLERANCE:
            print('EMERGENCY_STOP', stops)
            if not all(stops.values()):
                print('EMERGENCY_STOP_FAILED')
                print('MANUAL_SUPPORT_REQUIRED: motors remain enabled')
                self.finalized = True
                return 10
            if not self.disable_after_reset:
                self.finalized = True
                print('MOTORS_REMAIN_ENABLED', True)
                print('AT_SUPPORT', True)
                return 0
            disabled = self.disable_both()
            print('DISABLED', disabled)
            self.motors_enabled = False
            self.finalized = True
            if not all(disabled.values()):
                print('DISABLE_FAILED')
                return 10
            settle_end = time.monotonic() + POST_DISABLE_SETTLE_SEC
            while time.monotonic() < settle_end and rclpy.ok():
                self.spin_once(0.05)
            settled = self.live_pose()
            settled_error = self.max_error(settled)
            print('POST_DISABLE_SETTLE_POSE', {
                side: [round(value, 6) for value in settled[side]]
                for side in SIDES})
            print('POST_DISABLE_SETTLE_ERROR', round(settled_error, 6))

            stability_end = time.monotonic() + POST_DISABLE_STABILITY_SEC
            while time.monotonic() < stability_end and rclpy.ok():
                self.spin_once(0.05)
            post_disable = self.live_pose()
            post_error = self.max_error(post_disable)
            post_drift = max(
                abs(after - before)
                for side in SIDES
                for after, before in zip(post_disable[side], settled[side]))
            print('POST_DISABLE_POSE', {
                side: [round(value, 6) for value in post_disable[side]]
                for side in SIDES})
            print('POST_DISABLE_MAX_ERROR', round(post_error, 6))
            print('POST_DISABLE_DRIFT', round(post_drift, 6))

            status_ready = self.wait_for_hardware_status()
            motor_states = self.motor_enabled_states()
            limit_flags = self.hardware_limit_flags()
            print('POST_DISABLE_STATUS_READY', status_ready)
            print('MOTOR_ENABLED_STATES', motor_states)
            print('HARDWARE_JOINT_LIMIT_FLAGS', limit_flags)

            if not status_ready:
                print('POST_DISABLE_STATUS_TIMEOUT')
                return 9
            if any(motor_states.values()):
                print('MOTOR_STILL_ENABLED')
                return 9
            if any(any(flags) for flags in limit_flags.values()):
                print('HARDWARE_JOINT_LIMIT_EXCEEDED')
                return 11
            if post_error > TOLERANCE:
                print('POST_DISABLE_ERROR_EXCEEDED', True)
                return 9
            if post_drift > POST_DISABLE_DRIFT_TOLERANCE:
                print('POST_DISABLE_UNSTABLE', True)
                return 9
            print('AT_SUPPORT', True)
            return 0

        print('AT_SUPPORT', False)
        print('EMERGENCY_STOP', stops)
        print('MANUAL_SUPPORT_REQUIRED: motors remain enabled')
        self.finalized = True
        return 8

    def fail_safe(self):
        if self.motors_enabled and not self.finalized:
            print('FAIL_SAFE_EMERGENCY_STOP', self.call_stops())
            self.close_gates()


data = yaml.safe_load(SUPPORT_FILE.read_text())['support_pose']
target = {
    f'{side}_joint{i}': float(data[f'{side}_joint{i}'])
    for side in SIDES for i in range(1, 8)
}

arguments = sys.argv[1:]
plan_only_mode = '--plan-only' in arguments
if plan_only_mode and '--disable' in arguments:
    print('--plan-only and --disable are mutually exclusive')
    raise SystemExit(2)
rclpy.init()
node = SupportReset(
    target,
    disable_after_reset='--disable' in arguments,
    plan_only=plan_only_mode,
)
exit_code = 1
try:
    exit_code = node.execute()
except KeyboardInterrupt:
    node.fail_safe()
    exit_code = 130
except Exception as exc:
    print('RESET_EXCEPTION', repr(exc))
    node.fail_safe()
    exit_code = 9
finally:
    node.fail_safe()
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(exit_code)
