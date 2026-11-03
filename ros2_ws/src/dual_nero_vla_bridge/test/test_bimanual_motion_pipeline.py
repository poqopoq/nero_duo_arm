from __future__ import annotations

import os
import socket
import tempfile
import threading
import time
import unittest

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from dual_nero_vla_bridge.bimanual_bridge import (
    ALL_JOINT_NAMES,
    NeroBimanualVlaBridge,
)
from dual_nero_vla_bridge.protocol import recv_frame, send_frame


class FakeBimanualIkServer(Node):
    def __init__(self, failure_side: str | None = None) -> None:
        super().__init__('fake_bimanual_compute_ik')
        self.failure_side = failure_side
        self.requests: list[GetPositionIK.Request] = []
        self.create_service(GetPositionIK, '/compute_ik', self._handle)

    def _handle(self, request, response):
        self.requests.append(request)
        group_name = request.ik_request.group_name
        if (
            self.failure_side is not None
            and group_name == f'{self.failure_side}_arm'
        ):
            response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
            return response
        seed = list(request.ik_request.robot_state.joint_state.position)
        response.solution.joint_state.name = list(
            request.ik_request.robot_state.joint_state.name)
        response.solution.joint_state.position = [value + 0.01 for value in seed]
        response.error_code.val = MoveItErrorCodes.SUCCESS
        return response


class FakeBimanualTrajectoryServer(Node):
    def __init__(self) -> None:
        super().__init__('fake_bimanual_trajectory_server')
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


class FakeBimanualFeedback(Node):
    def __init__(self) -> None:
        super().__init__('fake_bimanual_vla_feedback')
        self.tcp_pubs = {
            side: self.create_publisher(
                PoseStamped, f'/{side}/feedback/tcp_pose', 10)
            for side in ('left', 'right')
        }
        self.joint_pubs = {
            side: self.create_publisher(
                JointState, f'/{side}/feedback/joint_states', 10)
            for side in ('left', 'right')
        }
        self.create_timer(0.05, self._publish)

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for index, side in enumerate(('left', 'right')):
            pose = PoseStamped()
            pose.header.frame_id = f'{side}_base_link'
            pose.header.stamp = stamp
            pose.pose.position.x = 0.1 + 0.2 * index
            pose.pose.position.y = 0.2
            pose.pose.position.z = 0.3
            pose.pose.orientation.w = 1.0
            self.tcp_pubs[side].publish(pose)

            joints = JointState()
            joints.header.stamp = stamp
            joints.name = [f'joint{joint_index}' for joint_index in range(1, 8)]
            joints.position = [0.0] * 7
            self.joint_pubs[side].publish(joints)


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


def _rot6d_action(left_x: float = 0.1, left_gripper: float = 0.8) -> list[float]:
    identity_rot6d = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return [
        left_x, 0.2, 0.3, *identity_rot6d, left_gripper,
        0.3, 0.2, 0.3, *identity_rot6d, 0.2,
    ]


class BimanualMotionPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def _start(
        self,
        mode: str,
        failure_side: str | None = None,
        allow_real_motion: bool = False,
        schema: str = 'rot6d20',
    ):
        self.tempdir = tempfile.TemporaryDirectory(
            prefix='nero-bimanual-vla-motion-test-')
        socket_path = os.path.join(self.tempdir.name, 'bridge.sock')
        self.feedback = FakeBimanualFeedback()
        self.ik = FakeBimanualIkServer(failure_side)
        self.trajectory = FakeBimanualTrajectoryServer()
        self.bridge = NeroBimanualVlaBridge(parameter_overrides=[
            Parameter('socket_path', value=socket_path),
            Parameter('action_schema', value=schema),
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
        self.executor = MultiThreadedExecutor(num_threads=5)
        for node in (self.feedback, self.ik, self.trajectory, self.bridge):
            self.executor.add_node(node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.client = BridgeClient(socket_path)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            status = self.client.request('ping')
            if status.get('state_ready') and self.bridge._ik_client.service_is_ready():
                return status
            time.sleep(0.05)
        self.fail('bimanual bridge feedback or fake IK service did not become ready')

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

    def test_dry_run_filters_both_arms_without_ik(self) -> None:
        status = self._start('dry_run')
        self.assertTrue(status['bimanual'])
        response = self.client.request(
            'send_action', action=_rot6d_action(left_x=0.15))
        self.assertTrue(response['ok'])
        self.assertTrue(response['accepted'])
        self.assertEqual(response['mode'], 'dry_run')
        self.assertAlmostEqual(response['action'][0], 0.11, places=6)
        self.assertAlmostEqual(response['action'][10], 0.3, places=6)
        self.assertEqual(self.ik.requests, [])
        self.assertEqual(self.trajectory.goals, [])

    def test_ik_dry_run_solves_each_arm(self) -> None:
        self._start('ik_dry_run')
        response = self.client.request('send_action', action=_rot6d_action())
        self.assertTrue(response['ok'])
        self.assertTrue(response['accepted'])
        self.assertEqual(
            [request.ik_request.group_name for request in self.ik.requests],
            ['left_arm', 'right_arm'],
        )
        self.assertEqual(
            [request.ik_request.pose_stamped.header.frame_id
             for request in self.ik.requests],
            ['left_base_link', 'right_base_link'],
        )
        self.assertEqual(
            list(self.ik.requests[0].ik_request.robot_state.joint_state.name),
            list(ALL_JOINT_NAMES),
        )
        self.assertEqual(response['joint_target'], [0.01] * 14)
        self.assertEqual(response['joint_target_by_side']['left'], [0.01] * 7)
        self.assertEqual(response['joint_target_by_side']['right'], [0.01] * 7)
        self.assertFalse(self.trajectory.goal_event.is_set())

    def test_trajectory_sends_one_fourteen_joint_goal(self) -> None:
        self._start('trajectory', allow_real_motion=True)
        response = self.client.request('send_action', action=_rot6d_action())
        self.assertTrue(response['ok'])
        self.assertTrue(response['accepted'])
        self.assertTrue(response['queued'])
        self.assertTrue(self.trajectory.goal_event.wait(timeout=2.0))
        self.assertEqual(len(self.trajectory.goals), 1)
        goal = self.trajectory.goals[0]
        self.assertEqual(goal['joint_names'], list(ALL_JOINT_NAMES))
        self.assertEqual(len(goal['joint_names']), 14)
        self.assertEqual(goal['points'][0]['positions'], [0.01] * 14)
        self.assertAlmostEqual(goal['points'][0]['time_from_start'], 0.05, places=6)

    def test_one_arm_ik_failure_latches_without_goal(self) -> None:
        self._start(
            'trajectory', failure_side='right', allow_real_motion=True)
        response = self.client.request('send_action', action=_rot6d_action())
        self.assertTrue(response['ok'])
        self.assertFalse(response['accepted'])
        self.assertIn('right IK failed', response['error'])
        latched = self.client.request('send_action', action=_rot6d_action())
        self.assertFalse(latched['accepted'])
        self.assertIn('motion fault latched', latched['error'])
        self.assertFalse(self.trajectory.goal_event.is_set())

    def test_axis_angle_schema_is_supported(self) -> None:
        self._start('ik_dry_run', schema='axis_angle14')
        action = [
            0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.8,
            0.3, 0.2, 0.3, 0.0, 0.0, 0.0, 0.2,
        ]
        response = self.client.request('send_action', action=action)
        self.assertTrue(response['accepted'])
        self.assertEqual(response['joint_target'], [0.01] * 14)


if __name__ == '__main__':
    unittest.main()
