import time
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

LEFT_TARGET = [0.10, -0.20, 0.00, 0.30, 0.00, 0.10, 0.00]
RIGHT_TARGET = [-0.10, -0.20, 0.00, 0.30, 0.00, 0.10, 0.00]


def make_request(group, targets):
    req = GetMotionPlan.Request()
    mpr = MotionPlanRequest()
    mpr.group_name = group
    mpr.num_planning_attempts = 5
    mpr.allowed_planning_time = 5.0
    mpr.max_velocity_scaling_factor = 0.1
    mpr.max_acceleration_scaling_factor = 0.1
    mpr.start_state.is_diff = True
    goal = Constraints()
    for prefix, target in targets:
        for idx, value in enumerate(target, start=1):
            jc = JointConstraint()
            jc.joint_name = f'{prefix}_joint{idx}'
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
    mpr.goal_constraints.append(goal)
    req.motion_plan_request = mpr
    return req


rclpy.init()
node = Node('dual_moveit_plan_test')
client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
if not client.wait_for_service(timeout_sec=10.0):
    print('SERVICE_UNAVAILABLE')
    raise SystemExit(2)

for group, targets in (
    ('left_arm', [('left', LEFT_TARGET)]),
    ('both_arms', [('left', LEFT_TARGET), ('right', RIGHT_TARGET)]),
):
    future = client.call_async(make_request(group, targets))
    rclpy.spin_until_future_complete(node, future, timeout_sec=12.0)
    if not future.done() or future.result() is None:
        print(group, 'CALL_FAILED')
        continue
    res = future.result()
    code = res.motion_plan_response.error_code.val
    pts = len(res.motion_plan_response.trajectory.joint_trajectory.points)
    print(f'{group}: error_code={code} trajectory_points={pts}')
node.destroy_node()
rclpy.shutdown()
