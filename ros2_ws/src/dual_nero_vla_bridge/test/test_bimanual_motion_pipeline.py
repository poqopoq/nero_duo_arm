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
from std_msgs.msg import Float32, Int32
from std_srvs.srv import SetBool

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
        self.gripper_pubs = {
            side: self.create_publisher(
                Float32, f'/{side}_gripper/width', 10)
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

            gripper = Float32()
            gripper.data = 0.09
            self.gripper_pubs[side].publish(gripper)


class FakeGripperCommandSink(Node):
    def __init__(self) -> None:
        super().__init__('fake_bimanual_gripper_command_sink')
        self.commands = {'left': [], 'right': []}
        self.command_event = threading.Event()
        for side in ('left', 'right'):
            self.create_subscription(
                Int32,
                f'/{side}_gripper/position_cmd',
                lambda msg, side=side: self._handle(side, msg),
                10,
            )

    def _handle(self, side: str, msg: Int32) -> None:
        self.commands[side].append(int(msg.data))
        self.command_event.set()


class FakeBimanualControlGate(Node):
    def __init__(self) -> None:
        super().__init__('fake_bimanual_control_gate')
        self.enabled = {'left': False, 'right': False}
        self.requests = {'left': [], 'right': []}
        for side in ('left', 'right'):
            self.create_service(
                SetBool,
                f'/{side}/control_enable',
                lambda request, response, side=side: self._handle(
                    side, request, response),
            )

    def _handle(self, side: str, request, response):
        enabled = bool(request.data)
        self.enabled[side] = enabled
        self.requests[side].append(enabled)
        response.success = True
        response.message = 'ok'
        return response


class FakeBimanualStreamCommandSink(Node):
    def __init__(self) -> None:
        super().__init__('fake_bimanual_stream_command_sink')
        self.commands = {'left': [], 'right': []}
        self.command_event = threading.Event()
        for side in ('left', 'right'):
            self.create_subscription(
                JointState,
                f'/{side}/control/joint_states',
                lambda msg, side=side: self._handle(side, msg),
                10,
            )

    def _handle(self, side: str, msg: JointState) -> None:
        self.commands[side].append({
            'name': list(msg.name),
            'position': list(msg.position),
            'stamp': (msg.header.stamp.sec, msg.header.stamp.nanosec),
        })
        self.command_event.set()


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


def _rot6d_action(
        left_x: float = 0.1,
        left_gripper: float = 0.8,
        right_gripper: float = 0.2) -> list[float]:
    identity_rot6d = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    return [
        left_x, 0.2, 0.3, *identity_rot6d, left_gripper,
        0.3, 0.2, 0.3, *identity_rot6d, right_gripper,
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
        allow_gripper_motion: bool = False,
        schema: str = 'rot6d20',
        watchdog_timeout: float = 5.0,
        stream_feedback_timeout: float = 5.0,
        stream_rate_hz: float = 50.0,
    ):
        self.tempdir = tempfile.TemporaryDirectory(
            prefix='nero-bimanual-vla-motion-test-')
        socket_path = os.path.join(self.tempdir.name, 'bridge.sock')
        self.feedback = FakeBimanualFeedback()
        self.ik = FakeBimanualIkServer(failure_side)
        self.trajectory = FakeBimanualTrajectoryServer()
        self.gripper_sink = FakeGripperCommandSink()
        self.control_gate = FakeBimanualControlGate()
        self.stream_sink = FakeBimanualStreamCommandSink()
        self.bridge = NeroBimanualVlaBridge(parameter_overrides=[
            Parameter('socket_path', value=socket_path),
            Parameter('action_schema', value=schema),
            Parameter('execution_mode', value=mode),
            Parameter('allow_real_motion', value=allow_real_motion),
            Parameter('gripper_enabled', value=allow_gripper_motion),
            Parameter('allow_gripper_motion', value=allow_gripper_motion),
            Parameter('observation_timeout_sec', value=2.0),
            Parameter('watchdog_timeout_sec', value=watchdog_timeout),
            Parameter('stop_service_timeout_sec', value=0.01),
            Parameter('stream_command_rate_hz', value=stream_rate_hz),
            Parameter(
                'stream_feedback_timeout_sec', value=stream_feedback_timeout),
            Parameter('ik_timeout_sec', value=0.05),
            Parameter('ik_service_timeout_sec', value=0.3),
            Parameter('trajectory_server_timeout_sec', value=0.5),
            Parameter('trajectory_result_timeout_sec', value=1.0),
            Parameter('trajectory_duration_sec', value=0.05),
            Parameter('gripper_feedback_timeout_sec', value=2.0),
        ])
        self.executor = MultiThreadedExecutor(num_threads=6)
        for node in (
            self.feedback,
            self.gripper_sink,
            self.control_gate,
            self.stream_sink,
            self.ik,
            self.trajectory,
            self.bridge,
        ):
            self.executor.add_node(node)
        self.spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.client = BridgeClient(socket_path)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            status = self.client.request('ping')
            if (
                status.get('state_ready')
                and status.get('gripper_feedback_ready')
                and self.bridge._ik_client.service_is_ready()
            ):
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
        for name in (
            'feedback',
            'gripper_sink',
            'control_gate',
            'stream_sink',
            'ik',
            'trajectory',
            'bridge',
        ):
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
        self.assertFalse(response['gripper_commanded'])
        self.assertEqual(self.gripper_sink.commands, {'left': [], 'right': []})

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

    def test_stream_publishes_both_sides_from_same_target(self) -> None:
        self._start(
            'stream',
            allow_real_motion=True,
            stream_rate_hz=20.0,
        )
        response = self.client.request('send_action', action=_rot6d_action())
        self.assertTrue(response['ok'])
        self.assertTrue(response['accepted'])
        self.assertEqual(response['mode'], 'stream')
        self.assertEqual(response['stream_sequence'], 1)
        self.assertNotIn('trajectory_sequence', response)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if (
                self.stream_sink.commands['left']
                and self.stream_sink.commands['right']
            ):
                common_stamps = (
                    {command['stamp'] for command in self.stream_sink.commands['left']}
                    & {command['stamp'] for command in self.stream_sink.commands['right']}
                )
                if common_stamps:
                    break
            time.sleep(0.01)
        common_stamps = (
            {command['stamp'] for command in self.stream_sink.commands['left']}
            & {command['stamp'] for command in self.stream_sink.commands['right']}
        )
        self.assertTrue(common_stamps)
        self.assertFalse(self.trajectory.goal_event.is_set())
        for side in ('left', 'right'):
            command = self.stream_sink.commands[side][0]
            self.assertEqual(
                command['name'],
                [f'joint{index}' for index in range(1, 8)],
            )
            self.assertEqual(command['position'], [0.01] * 7)
        status = self.client.request('ping')
        self.assertTrue(status['stream']['stream_gate_open'])
        self.assertEqual(self.control_gate.requests['left'][0], True)
        self.assertEqual(self.control_gate.requests['right'][0], True)

    def test_stream_stop_stops_both_sides(self) -> None:
        self._start(
            'stream',
            allow_real_motion=True,
            stream_rate_hz=20.0,
        )
        response = self.client.request('send_action', action=_rot6d_action())
        self.assertTrue(response['accepted'])
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if (
                len(self.stream_sink.commands['left']) >= 2
                and len(self.stream_sink.commands['right']) >= 2
            ):
                break
            time.sleep(0.01)
        self.assertGreaterEqual(len(self.stream_sink.commands['left']), 2)
        self.assertGreaterEqual(len(self.stream_sink.commands['right']), 2)

        self.assertTrue(self.client.request('stop')['stopped'])
        time.sleep(0.1)
        counts = {
            side: len(self.stream_sink.commands[side])
            for side in ('left', 'right')
        }
        time.sleep(0.2)
        for side in ('left', 'right'):
            self.assertLessEqual(
                len(self.stream_sink.commands[side]), counts[side])
        self.assertIn(
            'stop command received', self.client.request('ping')['motion_fault'])
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if (
                False in self.control_gate.requests['left']
                and False in self.control_gate.requests['right']
            ):
                break
            time.sleep(0.02)
        self.assertIn(False, self.control_gate.requests['left'])
        self.assertIn(False, self.control_gate.requests['right'])

    def test_stream_one_arm_ik_failure_latches_without_commands(self) -> None:
        self._start(
            'stream',
            failure_side='right',
            allow_real_motion=True,
        )
        response = self.client.request('send_action', action=_rot6d_action())
        self.assertTrue(response['ok'])
        self.assertFalse(response['accepted'])
        self.assertIn('right IK failed', response['error'])
        latched = self.client.request('send_action', action=_rot6d_action())
        self.assertFalse(latched['accepted'])
        self.assertIn('motion fault latched', latched['error'])
        self.assertEqual(self.stream_sink.commands, {'left': [], 'right': []})
        self.assertFalse(self.trajectory.goal_event.is_set())

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

    def test_enabled_gripper_actions_publish_raw_positions(self) -> None:
        self._start('ik_dry_run', allow_gripper_motion=True)
        response = self.client.request('send_action', action=_rot6d_action())
        self.assertTrue(response['accepted'])
        self.assertTrue(response['gripper_commanded'])
        self.assertEqual(
            response['gripper_positions'], {'left': 198, 'right': 792})
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if (
                self.gripper_sink.commands['left']
                and self.gripper_sink.commands['right']
            ):
                break
            time.sleep(0.02)
        self.assertEqual(self.gripper_sink.commands['left'], [198])
        self.assertEqual(self.gripper_sink.commands['right'], [792])

    def test_closed_normalized_grippers_use_command_limit_990(self) -> None:
        self._start('ik_dry_run', allow_gripper_motion=True)
        response = self.client.request(
            'send_action',
            action=_rot6d_action(left_gripper=0.0, right_gripper=0.0))
        self.assertTrue(response['accepted'])
        self.assertTrue(response['gripper_commanded'])
        self.assertEqual(
            response['gripper_positions'], {'left': 990, 'right': 990})
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if (
                self.gripper_sink.commands['left']
                and self.gripper_sink.commands['right']
            ):
                break
            time.sleep(0.02)
        self.assertEqual(self.gripper_sink.commands['left'], [990])
        self.assertEqual(self.gripper_sink.commands['right'], [990])

    def test_ik_failure_does_not_command_grippers(self) -> None:
        self._start(
            'ik_dry_run',
            failure_side='right',
            allow_gripper_motion=True,
        )
        response = self.client.request('send_action', action=_rot6d_action())
        self.assertFalse(response['accepted'])
        self.assertEqual(self.gripper_sink.commands, {'left': [], 'right': []})


if __name__ == '__main__':
    unittest.main()
