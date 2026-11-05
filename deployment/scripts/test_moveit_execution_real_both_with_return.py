#!/usr/bin/env python3
import os
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

SIDES = ('left', 'right')
NAMES = [f'{side}_joint{i}' for side in SIDES for i in range(1, 8)]
DELTAS = {'left': 0.03, 'right': -0.03}
POSITION_TOLERANCE = 0.03
J4_MAX = 2.14
ROOT = Path(os.environ.get('NERO_DUO_ROOT', Path(__file__).resolve().parents[2]))
SUPPORT_FILE = ROOT / 'deployment' / 'config' / 'support_pose.yaml'


class DualReturnTest(Node):
    def __init__(self):
        super().__init__('moveit_real_both_return_test')
        self.current = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.stop_clients = {
            side: self.create_client(Empty, f'/{side}/emergency_stop')
            for side in SIDES
        }
        self.disable_clients = {
            side: self.create_client(SetBool, f'/{side}/enable_agx_arm')
            for side in SIDES
        }

    def _cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in NAMES):
            self.current = {name: float(values[name]) for name in NAMES}

    def wait_current(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current is not None:
                return True
        return False

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def call_empty(self, client):
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.done() and future.result() is not None

    def set_enabled(self, side, enabled):
        client = self.disable_clients[side]
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        req = SetBool.Request()
        req.data = enabled
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.done() and future.result() is not None

    def execute_joint_goal(self, positions, label):
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'both_arms'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.05
        goal.request.max_acceleration_scaling_factor = 0.05
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        for name, value in zip(NAMES, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False

        send = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=5.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print(f'{label}_REJECTED')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=40.0)
        result = result_future.result()
        if result is None:
            print(f'{label}_NO_RESULT')
            return False
        code = result.result.error_code.val
        print(f'{label}_ERROR_CODE {code}')
        return code == 1


rclpy.init()
node = DualReturnTest()
if not node.wait_current():
    print('JOINT_STATE_TIMEOUT')
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(2)
if not node.move_client.wait_for_server(timeout_sec=10.0):
    print('MOVE_ACTION_UNAVAILABLE')
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(3)

support = [node.current[name] for name in NAMES]
for name in ('left_joint4', 'right_joint4'):
    index = NAMES.index(name)
    if support[index] > J4_MAX:
        print(
            f'{name} feedback {support[index]:.9f} exceeds {J4_MAX:.2f}; '
            'clamping the support target')
        support[index] = J4_MAX
target = list(support)
for side in SIDES:
    idx = NAMES.index(f'{side}_joint7')
    target[idx] += DELTAS[side]

print('SUPPORT_POSE', [round(x, 6) for x in support])
print('TEST_TARGET', [round(x, 6) for x in target])
SUPPORT_FILE.parent.mkdir(parents=True, exist_ok=True)
with SUPPORT_FILE.open('w', encoding='utf-8') as f:
    f.write('# Captured before the real dual-arm execution test;'
            ' joint4 <= 2.14 (deployment cap; native 2.146755)\n')
    f.write('support_pose:\n')
    for name, value in zip(NAMES, support):
        f.write(f'  {name}: {value:.9f}\n')

main_ok = node.execute_joint_goal(target, 'MAIN')
node.spin_for(0.5)
return_ok = node.execute_joint_goal(support, 'RETURN')
node.spin_for(1.0)

final = [node.current[name] for name in NAMES]
max_error = max(abs(a - b) for a, b in zip(final, support))
print('FINAL_POSE', [round(x, 6) for x in final])
print('MAX_SUPPORT_ERROR', round(max_error, 6))

# Only stop/disable after the robot is verified back at support.
if return_ok and max_error <= POSITION_TOLERANCE:
    print('AT_SUPPORT', True)
    for side in SIDES:
        print(f'{side.upper()}_EMERGENCY_STOP', node.call_empty(node.stop_clients[side]))
    for side in SIDES:
        print(f'{side.upper()}_DISABLE', node.set_enabled(side, False))
    exit_code = 0 if main_ok else 4
else:
    print('AT_SUPPORT', False)
    print('MANUAL_SUPPORT_REQUIRED: emergency stop only; motors remain enabled')
    for side in SIDES:
        print(f'{side.upper()}_EMERGENCY_STOP', node.call_empty(node.stop_clients[side]))
    exit_code = 5

node.destroy_node()
rclpy.shutdown()
raise SystemExit(exit_code)
