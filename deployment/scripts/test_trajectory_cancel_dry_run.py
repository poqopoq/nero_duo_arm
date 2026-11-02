#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

NAMES = [f'left_joint{i}' for i in range(1, 8)] + [
    f'right_joint{i}' for i in range(1, 8)]


def point(sec, positions):
    p = JointTrajectoryPoint()
    p.positions = positions
    p.velocities = [0.0] * len(positions)
    p.time_from_start = Duration(sec=sec, nanosec=0)
    return p


rclpy.init()
node = Node('trajectory_cancel_dry_run_test')
client = ActionClient(
    node, FollowJointTrajectory,
    '/dual_arm_controller/follow_joint_trajectory')
if not client.wait_for_server(timeout_sec=10.0):
    print('ACTION_SERVER_UNAVAILABLE')
    raise SystemExit(2)

goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = NAMES
start = [0.0] * 14
end = [0.2] * 14
goal.trajectory.points = [point(0, start), point(5, end)]
send = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send, timeout_sec=5.0)
handle = send.result()
if handle is None or not handle.accepted:
    print('GOAL_REJECTED')
    raise SystemExit(3)

time.sleep(0.5)
cancel_future = handle.cancel_goal_async()
rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
print('CANCEL_RESPONSE', cancel_future.done() and cancel_future.result() is not None)

result_future = handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
result = result_future.result()
if result is None:
    print('NO_RESULT')
    raise SystemExit(4)
print('GOAL_STATUS', result.status)
print('ERROR_CODE', result.result.error_code)
node.destroy_node()
rclpy.shutdown()
