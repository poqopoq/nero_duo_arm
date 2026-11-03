from __future__ import annotations

import os
import socket
import tempfile
import threading
import time
import unittest

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from dual_nero_vla_bridge.protocol import recv_frame, send_frame
from dual_nero_vla_bridge.vla_bridge import NeroVlaBridge


class FakeIkServer(Node):
    def __init__(self, mode: str = 'success') -> None:
        super().__init__('fake_compute_ik')
        self.mode = mode
        self.requests: list[GetPositionIK.Request] = []
        self.create_service(GetPositionIK, '/compute_ik', self._handle)

    def _handle(self, request, response):
        self.requests.append(request)
        if self.mode == 'failure':
            response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
            return response
        seed = list(request.ik_request.robot_state.joint_state.position)
        offset = 0.2 if self.mode == 'large_step' else 0.01
        response.solution.joint_state.name = list(
            request.ik_request.robot_state.joint_state.name)
        response.solution.joint_state.position = [value + offset for value in seed]
        response.error_code.val = MoveItErrorCodes.SUCCESS
        return response


class FakeTrajectoryServer(Node):
    def __init__(self) -> None:
        super().__init__('fake_trajectory_server')
        self.goals: list[dict] = []
        self.goal_event = threading.Event()
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            '/dual_arm_controller/follow_joint_trajectory',
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
        )

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self.goals.append({
            'joint_names': list(trajectory.joint_names),
            'points': [
                {
                    'positions': list(point.positions),
                    'time_from_start': (
                        point.time_from_start.sec
                        + point.time_from_start.nanosec * 1e-9),
                }
                for point in trajectory.points
            ],
        })
        self.goal_event.set()
        time.sleep(0.02)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


class FakeFeedback(Node):
    def __init__(self) -> None:
        super().__init__('fake_vla_feedback')
        self.tcp_pub = self.create_publisher(PoseStamped, '/left/feedback/tcp_pose', 10)
        self.joint_pub = self.create_publisher(JointState, '/left/feedback/joint_states', 10)
        self.create_timer(0.05, self._publish)

    def _publish(self) -> None:
        pose = PoseStamped()
        pose.header.frame_id = 'left_base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.1
        pose.pose.position.y = 0.2
        pose.pose.position.z = 0.3
        pose.pose.orientation.w = 1.0
        self.tcp_pub.publish(pose)

        joints = JointState()
        joints.header.stamp = pose.header.stamp
        joints.name = [f'joint{index}' for index in range(1, 8)]
        joints.position = [0.0] * 7
        self.joint_pub.publish(joints)


class BridgeClient:
    def __init__(self, socket_path: str) -> None:
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(2.0)
        self.socket.connect(socket_path)

    def close(self) -> None:
        self.socket.close()

    def request(self, command: str, **fields):
        send_frame(self.socket, {'cmd': command, **fields})
        return recv_frame(self.socket)[0]


class MotionPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def _start(self, mode: str, ik_mode: str = 'success', allow_real_motion: bool = False):
        self.tempdir = tempfile.TemporaryDirectory(prefix='nero-vla-motion-test-')
        socket_path = os.path.join(self.tempdir.name, 'bridge.sock')
        self.feedback = FakeFeedback()
        self.ik = FakeIkServer(ik_mode)
        self.trajectory = FakeTrajectoryServer()
        self.bridge = NeroVlaBridge(parameter_overrides=[
            Parameter('socket_path', value=socket_path),
            Parameter('execution_mode', value=mode),
            Parameter('allow_real_motion', value=allow_real_motion),
            Parameter('observation_timeout_sec', value=2.0),
            Parameter('watchdog_timeout_sec', value=5.0),
            Parameter('stop_service_timeout_sec', value=0.01),
            Parameter('ik_timeout_sec', value=0.05),
            Parameter('ik_service_timeout_sec', value=0.3),
            Parameter('trajectory_server_timeout_sec', value=0.5),
            Parameter('trajectory_result_timeout_sec', value=1.0),
            Parameter('trajectory_duration_sec', value=0.05),
        ])
        self.executor = MultiThreadedExecutor(num_threads=4)
        for node in (self.feedback, self.ik, self.trajectory, self.bridge):
            self.executor.add_node(node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.client = BridgeClient(socket_path)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            status = self.client.request('ping')
            if status.get('state_ready') and self.bridge._ik_client.service_is_ready():
                return status
            time.sleep(0.05)
        self.fail('bridge feedback or fake IK service did not become ready')

    def tearDown(self) -> None:
        if hasattr(self, 'client'):
            self.client.close()
        if hasattr(self, 'bridge'):
            self.bridge.shutdown_bridge()
        if hasattr(self, 'executor'):
            self.executor.shutdown()
        for name in ('feedback', 'ik', 'trajectory', 'bridge'):
            node = getattr(self, name, None)
            if node is not None:
                node.destroy_node()
        if hasattr(self, 'tempdir'):
            self.tempdir.cleanup()

    def test_dry_run_filters_without_calling_ik(self) -> None:
        self._start('dry_run')
        response = self.client.request(
            'send_action', action=[0.15, 0.2, 0.3, 0.0, 0.0, 0.0, 0.8])
        self.assertTrue(response['ok'])
        self.assertTrue(response['accepted'])
        self.assertFalse(response['executed'])
        self.assertEqual(response['mode'], 'dry_run')
        self.assertAlmostEqual(response['action'][0], 0.11, places=6)
        self.assertEqual(self.ik.requests, [])
        self.assertEqual(self.trajectory.goals, [])

    def test_ik_dry_run_returns_validated_joint_target(self) -> None:
        self._start('ik_dry_run')
        response = self.client.request(
            'send_action', action=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.8])
        self.assertTrue(response['ok'])
        self.assertTrue(response['accepted'])
        self.assertFalse(response['executed'])
        self.assertEqual(response['mode'], 'ik_dry_run')
        self.assertEqual(len(self.ik.requests), 1)
        request = self.ik.requests[0].ik_request
        self.assertEqual(request.group_name, 'left_arm')
        self.assertEqual(request.ik_link_name, 'left_tcp_link')
        self.assertEqual(request.pose_stamped.header.frame_id, 'left_base_link')
        self.assertEqual(
            list(request.robot_state.joint_state.name),
            [f'left_joint{index}' for index in range(1, 8)],
        )
        self.assertEqual(response['joint_target'], [0.01] * 7)
        self.assertFalse(self.trajectory.goal_event.is_set())

    def test_trajectory_mode_sends_short_joint_trajectory(self) -> None:
        self._start('trajectory', allow_real_motion=True)
        response = self.client.request(
            'send_action', action=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.8])
        self.assertTrue(response['ok'])
        self.assertTrue(response['accepted'])
        self.assertTrue(response['queued'])
        self.assertEqual(response['mode'], 'trajectory')
        self.assertTrue(self.trajectory.goal_event.wait(timeout=2.0))
        self.assertEqual(len(self.trajectory.goals), 1)
        goal = self.trajectory.goals[0]
        self.assertEqual(
            goal['joint_names'], [f'left_joint{index}' for index in range(1, 8)])
        self.assertEqual(len(goal['points']), 1)
        self.assertEqual(goal['points'][0]['positions'], [0.01] * 7)
        self.assertAlmostEqual(goal['points'][0]['time_from_start'], 0.05, places=6)

    def test_ik_failure_latches_motion_fault(self) -> None:
        self._start('trajectory', ik_mode='failure', allow_real_motion=True)
        response = self.client.request(
            'send_action', action=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.8])
        self.assertTrue(response['ok'])
        self.assertFalse(response['accepted'])
        self.assertIn('IK failed', response['error'])
        latched = self.client.request(
            'send_action', action=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.8])
        self.assertFalse(latched['accepted'])
        self.assertIn('motion fault latched', latched['error'])
        self.assertFalse(self.trajectory.goal_event.is_set())

    def test_large_ik_joint_step_is_rejected(self) -> None:
        self._start('ik_dry_run', ik_mode='large_step')
        response = self.client.request(
            'send_action', action=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.8])
        self.assertTrue(response['ok'])
        self.assertFalse(response['accepted'])
        self.assertIn('exceeds', response['error'])


if __name__ == '__main__':
    unittest.main()
