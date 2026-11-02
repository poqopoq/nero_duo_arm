#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

NAMES = [f'left_joint{i}' for i in range(1, 8)] + [
    f'right_joint{i}' for i in range(1, 8)]


class RealExecutionTest(Node):
    def __init__(self):
        super().__init__('moveit_real_left_execution_test')
        self.current = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.disable_clients = {
            side: self.create_client(SetBool, f'/{side}/enable_agx_arm')
            for side in ('left', 'right')
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

    def disable(self, side):
        client = self.disable_clients[side]
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        req = SetBool.Request()
        req.data = False
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.done() and future.result() is not None


rclpy.init()
node = RealExecutionTest()
if not node.wait_current():
    print('JOINT_STATE_TIMEOUT')
    raise SystemExit(2)
if not node.move_client.wait_for_server(timeout_sec=10.0):
    print('MOVE_ACTION_UNAVAILABLE')
    raise SystemExit(3)

start = [node.current[f'left_joint{i}'] for i in range(1, 8)]
target = list(start)
target[6] += 0.05
print('LEFT_START', [round(x, 6) for x in start])
print('LEFT_TARGET', [round(x, 6) for x in target])

goal = MoveGroup.Goal()
goal.request = MotionPlanRequest()
goal.request.group_name = 'left_arm'
goal.request.num_planning_attempts = 5
goal.request.allowed_planning_time = 5.0
goal.request.max_velocity_scaling_factor = 0.05
goal.request.max_acceleration_scaling_factor = 0.05
goal.request.start_state.is_diff = True
constraints = Constraints()
for idx, value in enumerate(target, start=1):
    jc = JointConstraint()
    jc.joint_name = f'left_joint{idx}'
    jc.position = value
    jc.tolerance_above = 0.01
    jc.tolerance_below = 0.01
    jc.weight = 1.0
    constraints.joint_constraints.append(jc)
goal.request.goal_constraints.append(constraints)
goal.planning_options.plan_only = False
goal.planning_options.replan = False
goal.planning_options.look_around = False

send = node.move_client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send, timeout_sec=5.0)
handle = send.result()
if handle is None or not handle.accepted:
    print('GOAL_REJECTED')
    raise SystemExit(4)
result_future = handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
result = result_future.result()
if result is None:
    print('NO_RESULT')
    raise SystemExit(5)

# Let feedback settle before reading the final position.
time.sleep(1.0)
rclpy.spin_once(node, timeout_sec=0.2)
end = [node.current[f'left_joint{i}'] for i in range(1, 8)]
print(f'REAL_EXECUTION error_code={result.result.error_code.val}')
print('LEFT_END', [round(x, 6) for x in end])
print('JOINT7_DELTA', round(end[6] - start[6], 6))
print('LEFT_DISABLE', node.disable('left'))
print('RIGHT_DISABLE', node.disable('right'))
node.destroy_node()
rclpy.shutdown()
