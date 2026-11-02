#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

LEFT_TARGET = [0.08, -0.12, 0.02, 0.18, 0.0, 0.08, 0.0]
RIGHT_TARGET = [-0.08, -0.12, -0.02, 0.18, 0.0, 0.08, 0.0]

parser = argparse.ArgumentParser()
parser.add_argument('--group', choices=('left_arm', 'both_arms'), default='left_arm')
args = parser.parse_args()

rclpy.init()
node = Node('moveit_execution_dry_run_test')
client = ActionClient(node, MoveGroup, '/move_action')
if not client.wait_for_server(timeout_sec=10.0):
    print('MOVE_ACTION_UNAVAILABLE')
    raise SystemExit(2)

goal = MoveGroup.Goal()
goal.request = MotionPlanRequest()
goal.request.group_name = args.group
goal.request.num_planning_attempts = 5
goal.request.allowed_planning_time = 5.0
goal.request.max_velocity_scaling_factor = 0.1
goal.request.max_acceleration_scaling_factor = 0.1
goal.request.start_state.is_diff = True
constraints = Constraints()
targets = [('left', LEFT_TARGET)]
if args.group == 'both_arms':
    targets.append(('right', RIGHT_TARGET))
for prefix, target in targets:
    for idx, value in enumerate(target, start=1):
        joint = JointConstraint()
        joint.joint_name = f'{prefix}_joint{idx}'
        joint.position = value
        joint.tolerance_above = 0.01
        joint.tolerance_below = 0.01
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
goal.request.goal_constraints.append(constraints)
goal.planning_options.plan_only = False
goal.planning_options.replan = False
goal.planning_options.look_around = False

send = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send, timeout_sec=5.0)
handle = send.result()
if handle is None or not handle.accepted:
    print('GOAL_REJECTED')
    raise SystemExit(3)
result_future = handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=20.0)
result = result_future.result()
if result is None:
    print('NO_RESULT')
    raise SystemExit(4)
print(f'MOVEIT_EXECUTION error_code={result.result.error_code.val}')
node.destroy_node()
rclpy.shutdown()
