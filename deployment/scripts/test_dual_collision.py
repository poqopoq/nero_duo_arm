import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

LEFT_COLLIDE = [-1.23617, 1.5731, -0.30537, 2.07824, 0.08537, 0.1038, 1.24577]
NAMES = [f'left_joint{i}' for i in range(1, 8)] + [f'right_joint{i}' for i in range(1, 8)]

rclpy.init()
node = Node('dual_collision_check')
client = node.create_client(GetStateValidity, '/check_state_validity')
if not client.wait_for_service(timeout_sec=10.0):
    print('SERVICE_UNAVAILABLE')
    raise SystemExit(2)

def check(label, positions):
    req = GetStateValidity.Request()
    req.group_name = 'both_arms'
    req.robot_state = RobotState()
    req.robot_state.joint_state = JointState()
    req.robot_state.joint_state.name = NAMES
    req.robot_state.joint_state.position = positions
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=8.0)
    if not fut.done() or fut.result() is None:
        print(label, 'CALL_FAILED')
        return
    res = fut.result()
    print(label, 'valid=', res.valid, 'contacts=', len(res.contacts))

check('normal_zero', [0.0] * 14)
check('left_into_table', LEFT_COLLIDE + [0.0] * 7)
node.destroy_node()
rclpy.shutdown()
