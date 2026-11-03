#!/usr/bin/env python3
"""Cancel a small real two-arm trajectory and verify a controlled stop."""
import os
import time
from pathlib import Path

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

SIDES = ('left', 'right')
NAMES = [f'{side}_joint{i}' for side in SIDES for i in range(1, 8)]
DELTAS = {'left': 0.04, 'right': -0.04}
SUPPORT_TOLERANCE = 0.03
MOTION_THRESHOLD = 0.005
MOTION_TIMEOUT = 3.0
TRAJECTORY_SEC = 4
POST_CANCEL_SEC = 2.0
POST_CANCEL_LATE_SEC = 0.5
STANDSTILL_TOLERANCE = 0.01
ROOT = Path(os.environ.get('NERO_DUO_ROOT', Path(__file__).resolve().parents[2]))
SUPPORT_FILE = ROOT / 'deployment' / 'config' / 'support_pose.yaml'


def trajectory_point(seconds, positions):
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0] * len(positions)
    point.time_from_start = Duration(sec=seconds, nanosec=0)
    return point


class TrajectoryCancelTest(Node):
    def __init__(self):
        super().__init__('trajectory_cancel_real_test')
        self.feedback = {side: None for side in SIDES}
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            '/dual_arm_controller/follow_joint_trajectory')
        self.stop_clients = {
            side: self.create_client(Empty, f'/{side}/emergency_stop')
            for side in SIDES
        }
        self.gate_clients = {
            side: self.create_client(SetBool, f'/{side}/control_enable')
            for side in SIDES
        }
        self.enable_clients = {
            side: self.create_client(SetBool, f'/{side}/enable_agx_arm')
            for side in SIDES
        }
        for side in SIDES:
            self.create_subscription(
                JointState, f'/{side}/feedback/joint_states',
                lambda msg, s=side: self._feedback_cb(s, msg), 10)
        self.motion_started = False
        self.finalized = False

    def _feedback_cb(self, side, msg):
        values = dict(zip(msg.name, msg.position))
        if all(f'joint{i}' in values for i in range(1, 8)):
            self.feedback[side] = [float(values[f'joint{i}']) for i in range(1, 8)]

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout=45.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            services_ready = all(
                client.service_is_ready()
                for group in (self.stop_clients, self.gate_clients,
                              self.enable_clients)
                for client in group.values())
            if (services_ready and self.action_client.server_is_ready() and
                    all(value is not None for value in self.feedback.values())):
                return True
        return False

    def call_setbool(self, client, value, timeout=40.0):
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return (future.done() and future.result() is not None and
                bool(future.result().success))

    def close_gates(self):
        return {
            side: self.call_setbool(self.gate_clients[side], False, timeout=5.0)
            for side in SIDES
        }

    def call_stops(self):
        futures = {
            side: self.stop_clients[side].call_async(Empty.Request())
            for side in SIDES
        }
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if all(future.done() for future in futures.values()):
                break
        return {
            side: future.done() and future.result() is not None
            for side, future in futures.items()
        }

    def disable_both(self):
        return {
            side: self.call_setbool(self.enable_clients[side], False, timeout=40.0)
            for side in SIDES
        }

    def current_pose(self):
        return {side: list(self.feedback[side]) for side in SIDES}

    def make_goal(self, start, target):
        positions = []
        for side in SIDES:
            positions.extend(start[side])
        target_positions = []
        for side in SIDES:
            target_positions.extend(target[side])
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = NAMES
        goal.trajectory.points = [
            trajectory_point(0, positions),
            trajectory_point(TRAJECTORY_SEC, target_positions),
        ]
        return goal

    def fail_safe(self):
        if self.motion_started and not self.finalized:
            print('FAIL_SAFE_EMERGENCY_STOP', self.call_stops())
            print('FAIL_SAFE_CLOSE_GATES', self.close_gates())

    def execute(self):
        if not self.wait_ready():
            print('PREFLIGHT_READY_TIMEOUT')
            return 2

        support_data = yaml.safe_load(SUPPORT_FILE.read_text())['support_pose']
        start = self.current_pose()
        support_error = max(
            abs(start[side][index] - support_data[f'{side}_joint{index + 1}'])
            for side in SIDES for index in range(7))
        print('START_POSE', {
            side: [round(value, 6) for value in start[side]] for side in SIDES})
        print('MAX_SUPPORT_ERROR', round(support_error, 6))
        if support_error > SUPPORT_TOLERANCE:
            print('NOT_AT_SUPPORT')
            return 3

        target = {side: list(start[side]) for side in SIDES}
        for side in SIDES:
            target[side][6] += DELTAS[side]
        print('TEST_TARGET', {
            side: [round(value, 6) for value in target[side]] for side in SIDES})
        print('TRAJECTORY_SEC', TRAJECTORY_SEC)

        enabled = {
            side: self.call_setbool(self.enable_clients[side], True)
            for side in SIDES
        }
        print('MOTORS_ENABLED', enabled)
        if not all(enabled.values()):
            print('ENABLE_FAILED', self.call_stops(), self.close_gates())
            print('DISABLED', self.disable_both())
            self.finalized = True
            return 4
        self.motion_started = True
        self.spin_for(0.5)

        preceding = self.current_pose()
        pre_goal_drift = max(
            abs(preceding[side][index] - start[side][index])
            for side in SIDES for index in range(7))
        print('PRE_GOAL_DRIFT', round(pre_goal_drift, 6))
        if pre_goal_drift > 0.02:
            print('POSE_CHANGED_BEFORE_GOAL')
            return 5

        send_future = self.action_client.send_goal_async(self.make_goal(start, target))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print('GOAL_REJECTED')
            return 6

        moved = {side: False for side in SIDES}
        deadline = time.monotonic() + MOTION_TIMEOUT
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            for side in SIDES:
                if abs(self.feedback[side][6] - start[side][6]) >= MOTION_THRESHOLD:
                    moved[side] = True
            if all(moved.values()):
                break
        before_cancel = self.current_pose()
        motion_before_cancel = {
            side: abs(before_cancel[side][6] - start[side][6]) for side in SIDES}
        print('MOTION_BEFORE_CANCEL', {
            side: round(value, 6) for side, value in motion_before_cancel.items()})

        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        cancel_response = cancel_future.result() if cancel_future.done() else None
        cancel_accepted = bool(
            cancel_response is not None and cancel_response.goals_canceling)
        print('CANCEL_ACCEPTED', cancel_accepted)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        if not result_future.done() or result_future.result() is None:
            print('NO_RESULT')
            return 7
        wrapped_result = result_future.result()
        print('GOAL_STATUS', wrapped_result.status)
        print('ERROR_CODE', wrapped_result.result.error_code)

        samples = {side: [] for side in SIDES}
        cancel_time = time.monotonic()
        while rclpy.ok() and time.monotonic() - cancel_time < POST_CANCEL_SEC:
            rclpy.spin_once(self, timeout_sec=0.01)
            elapsed = time.monotonic() - cancel_time
            for side in SIDES:
                samples[side].append((elapsed, self.feedback[side][6]))

        late_drift = {}
        for side in SIDES:
            late = [value for elapsed, value in samples[side]
                    if elapsed >= POST_CANCEL_LATE_SEC]
            late_drift[side] = max(late) - min(late) if late else float('inf')
        print('POST_CANCEL_LATE_DRIFT', {
            side: round(value, 6) for side, value in late_drift.items()})

        ok = (
            all(moved.values()) and cancel_accepted and
            wrapped_result.status == 5 and
            wrapped_result.result.error_code == -4 and
            all(value <= STANDSTILL_TOLERANCE for value in late_drift.values()))
        print('CANCEL_TEST_PASS', ok)
        self.finalized = True
        return 0 if ok else 8


rclpy.init()
node = TrajectoryCancelTest()
exit_code = 1
try:
    exit_code = node.execute()
except KeyboardInterrupt:
    node.fail_safe()
    exit_code = 130
except Exception as exc:
    print('CANCEL_TEST_EXCEPTION', repr(exc))
    node.fail_safe()
    exit_code = 9
finally:
    node.fail_safe()
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(exit_code)
