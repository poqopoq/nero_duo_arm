#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def point(sec, positions):
    p = JointTrajectoryPoint()
    p.positions = positions
    p.velocities = [0.0] * len(positions)
    p.time_from_start = Duration(sec=sec, nanosec=0)
    return p


rclpy.init()
node = Node('trajectory_bridge_dry_run_client')
client = ActionClient(
    node, FollowJointTrajectory,
    '/dual_arm_controller/follow_joint_trajectory')
if not client.wait_for_server(timeout_sec=10.0):
    print('ACTION_SERVER_UNAVAILABLE')
    raise SystemExit(2)

goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = [f'left_joint{i}' for i in range(1, 8)] + [
    f'right_joint{i}' for i in range(1, 8)]
goal.trajectory.points = [
    point(0, [0.0] * 14),
    point(1, [0.05, -0.05, 0.02, 0.05, 0.0, 0.02, 0.0] +
             [-0.05, -0.05, -0.02, 0.05, 0.0, 0.02, 0.0]),
]
send_future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
goal_handle = send_future.result()
if goal_handle is None or not goal_handle.accepted:
    print('GOAL_REJECTED')
    raise SystemExit(3)
result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
result = result_future.result()
if result is None:
    print('NO_RESULT')
    raise SystemExit(4)
print(f'DRY_RUN_RESULT error_code={result.result.error_code}')
node.destroy_node()
rclpy.shutdown()
