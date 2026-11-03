#!/usr/bin/env python3
"""Expose cached NERO observations and validated motions to LeRobot.

The bridge deliberately does not import LeRobot or any Python 3.12 package.
It runs in the ROS 2 Humble Python 3.10 environment and communicates over a
Unix stream socket using :mod:`dual_nero_vla_bridge.protocol`.

``send_action`` applies workspace and per-step safety filters. Depending on
``execution_mode`` it can stop after filtering, solve and report IK without
motion, or send a short joint trajectory asynchronously. The default is
``dry_run`` and ``trajectory`` mode additionally requires
``allow_real_motion=true``.
"""
from __future__ import annotations

import json
import math
import os
import socket
import threading
import time
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Empty, SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .protocol import ProtocolError, recv_frame, send_frame

STATE_SIZE = 7
LOCAL_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 8))
EXECUTION_MODES = ('dry_run', 'ik_dry_run', 'trajectory')


class MotionError(RuntimeError):
    """Raised when an action cannot be converted into a safe motion."""


def _seconds_to_duration(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


def _as_finite_floats(values: Any, expected_size: int, field: str) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != expected_size:
        raise ValueError(f'{field} must contain {expected_size} values')
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{field} contains a non-finite value')
    return result


def _quaternion_to_axis_angle(x: float, y: float, z: float, w: float) -> list[float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('TCP quaternion is degenerate')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    vector_norm = math.sqrt(x * x + y * y + z * z)
    if vector_norm < 1e-12:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(vector_norm, w)
    scale = angle / vector_norm
    return [x * scale, y * scale, z * scale]


def _axis_angle_to_quaternion(vector: list[float]) -> tuple[float, float, float, float]:
    angle = math.sqrt(sum(value * value for value in vector))
    if angle < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    half = 0.5 * angle
    scale = math.sin(half) / angle
    return vector[0] * scale, vector[1] * scale, vector[2] * scale, math.cos(half)


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_conjugate(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    return -x, -y, -z, w


def _clamp_rotation_step(current: list[float], target: list[float], max_step: float) -> tuple[list[float], bool]:
    if max_step <= 0.0:
        return list(target), False
    q_current = _axis_angle_to_quaternion(current)
    q_target = _axis_angle_to_quaternion(target)
    q_delta = _quaternion_multiply(_quaternion_conjugate(q_current), q_target)
    norm = math.sqrt(sum(value * value for value in q_delta))
    q_delta = tuple(value / norm for value in q_delta)
    if q_delta[3] < 0.0:
        q_delta = tuple(-value for value in q_delta)
    angle = 2.0 * math.acos(max(-1.0, min(1.0, q_delta[3])))
    if angle <= max_step:
        return list(target), False
    axis_norm = math.sqrt(sum(value * value for value in q_delta[:3]))
    if axis_norm < 1e-12:
        return list(current), True
    scale = math.sin(0.5 * max_step) / axis_norm
    q_step = (q_delta[0] * scale, q_delta[1] * scale, q_delta[2] * scale, math.cos(0.5 * max_step))
    q_applied = _quaternion_multiply(q_current, q_step)
    return _quaternion_to_axis_angle(*q_applied), True


class NeroVlaBridge(Node):
    """ROS 2 node hosting a local observation/action socket server."""

    def __init__(self, parameter_overrides: Optional[list[Parameter]] = None) -> None:
        super().__init__(
            'dual_nero_vla_bridge', parameter_overrides=parameter_overrides)

        self.declare_parameter('socket_path', '/tmp/nero_vla_bridge.sock')
        self.declare_parameter('side', 'left')
        self.declare_parameter('tcp_topic', '')
        self.declare_parameter('joint_state_topic', '')
        self.declare_parameter('image_topic', '/camera_scene/color/image_raw')
        self.declare_parameter('image_key', 'camera1')
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('observation_timeout_sec', 0.75)
        self.declare_parameter('image_timeout_sec', 1.5)
        self.declare_parameter('execution_mode', 'dry_run')
        self.declare_parameter('allow_real_motion', False)
        self.declare_parameter('max_translation_step_m', 0.01)
        self.declare_parameter('max_rotation_step_rad', 0.05)
        self.declare_parameter('max_joint_step_rad', 0.15)
        self.declare_parameter('joint_position_min', [
            -2.70526, -1.760, -2.75, -1.01, -2.75, -0.73, -1.5707963,
        ])
        self.declare_parameter('joint_position_max', [
            2.70526, 1.74, 2.75, 2.200, 2.75, 0.95, 1.5707963,
        ])
        self.declare_parameter('workspace_min_xyz', [-2.0, -2.0, -0.2])
        self.declare_parameter('workspace_max_xyz', [2.0, 2.0, 2.0])
        self.declare_parameter('gripper_max_m', 0.1)
        self.declare_parameter('watchdog_timeout_sec', 0.75)
        self.declare_parameter('stop_service_timeout_sec', 0.5)
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('ik_group_name', '')
        self.declare_parameter('ik_link_name', '')
        self.declare_parameter('ik_frame_id', '')
        self.declare_parameter('ik_timeout_sec', 0.1)
        self.declare_parameter('ik_service_timeout_sec', 0.2)
        self.declare_parameter('avoid_collisions', True)
        self.declare_parameter(
            'trajectory_action', '/dual_arm_controller/follow_joint_trajectory')
        self.declare_parameter('trajectory_duration_sec', 0.2)
        self.declare_parameter('trajectory_server_timeout_sec', 0.25)
        self.declare_parameter('trajectory_result_timeout_sec', 2.0)

        self.socket_path = str(self.get_parameter('socket_path').value)
        self.side = str(self.get_parameter('side').value)
        self.tcp_topic = str(self.get_parameter('tcp_topic').value) or f'/{self.side}/feedback/tcp_pose'
        self.joint_state_topic = (
            str(self.get_parameter('joint_state_topic').value)
            or f'/{self.side}/feedback/joint_states'
        )
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.image_key = str(self.get_parameter('image_key').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.observation_timeout = float(self.get_parameter('observation_timeout_sec').value)
        self.image_timeout = float(self.get_parameter('image_timeout_sec').value)
        self.execution_mode = str(self.get_parameter('execution_mode').value)
        if self.execution_mode not in EXECUTION_MODES:
            raise ValueError(
                f'execution_mode must be one of {EXECUTION_MODES}, got {self.execution_mode!r}')
        self.allow_real_motion = bool(self.get_parameter('allow_real_motion').value)
        if self.execution_mode == 'trajectory' and not self.allow_real_motion:
            raise ValueError(
                'execution_mode=trajectory requires allow_real_motion=true')
        self.max_translation_step = float(self.get_parameter('max_translation_step_m').value)
        self.max_rotation_step = float(self.get_parameter('max_rotation_step_rad').value)
        self.max_joint_step = float(self.get_parameter('max_joint_step_rad').value)
        self.joint_position_min = _as_finite_floats(
            self.get_parameter('joint_position_min').value, 7, 'joint_position_min')
        self.joint_position_max = _as_finite_floats(
            self.get_parameter('joint_position_max').value, 7, 'joint_position_max')
        if any(low >= high for low, high in zip(self.joint_position_min, self.joint_position_max)):
            raise ValueError('joint_position_min must be strictly below joint_position_max')
        self.workspace_min = _as_finite_floats(
            self.get_parameter('workspace_min_xyz').value, 3, 'workspace_min_xyz')
        self.workspace_max = _as_finite_floats(
            self.get_parameter('workspace_max_xyz').value, 3, 'workspace_max_xyz')
        if any(low >= high for low, high in zip(self.workspace_min, self.workspace_max)):
            raise ValueError('workspace_min_xyz must be strictly below workspace_max_xyz')
        self.gripper_max_m = max(1e-6, float(self.get_parameter('gripper_max_m').value))
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_sec').value)
        self.stop_service_timeout = float(self.get_parameter('stop_service_timeout_sec').value)
        self.ik_group_name = (
            str(self.get_parameter('ik_group_name').value) or f'{self.side}_arm')
        self.ik_link_name = (
            str(self.get_parameter('ik_link_name').value) or f'{self.side}_tcp_link')
        self.ik_frame_id = (
            str(self.get_parameter('ik_frame_id').value) or f'{self.side}_base_link')
        self.ik_timeout = float(self.get_parameter('ik_timeout_sec').value)
        self.ik_service_timeout = float(
            self.get_parameter('ik_service_timeout_sec').value)
        self.avoid_collisions = bool(self.get_parameter('avoid_collisions').value)
        self.trajectory_action_name = str(
            self.get_parameter('trajectory_action').value)
        self.trajectory_duration = float(
            self.get_parameter('trajectory_duration_sec').value)
        self.trajectory_server_timeout = float(
            self.get_parameter('trajectory_server_timeout_sec').value)
        self.trajectory_result_timeout = float(
            self.get_parameter('trajectory_result_timeout_sec').value)
        if self.max_joint_step <= 0.0:
            raise ValueError('max_joint_step_rad must be positive')
        if self.trajectory_duration <= 0.0:
            raise ValueError('trajectory_duration_sec must be positive')

        self._lock = threading.Lock()
        self._tcp_xyz: Optional[list[float]] = None
        self._tcp_rot: Optional[list[float]] = None
        self._tcp_mono = 0.0
        self._tcp_stamp = 0.0
        self._joints: Optional[list[float]] = None
        self._gripper_width_m: Optional[float] = None
        self._joint_mono = 0.0
        self._joint_stamp = 0.0
        self._image_jpeg: Optional[bytes] = None
        self._image_shape: Optional[tuple[int, int, int]] = None
        self._image_mono = 0.0
        self._image_stamp = 0.0
        self._last_action_mono = 0.0
        self._watchdog_fired = False
        self._motion_request_lock = threading.Lock()
        self._motion_condition = threading.Condition()
        self._pending_joint_target: Optional[list[float]] = None
        self._pending_sequence = 0
        self._active_goal_handle: Any = None
        self._trajectory_abort = threading.Event()
        self._trajectory_worker: Optional[threading.Thread] = None
        self._trajectory_status: dict[str, Any] = {'state': 'idle'}
        self._stop_in_progress = False
        self._motion_fault_reason: Optional[str] = None

        self._shutdown = threading.Event()
        self._server: Optional[socket.socket] = None
        self._server_thread: Optional[threading.Thread] = None

        topic_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.create_subscription(PoseStamped, self.tcp_topic, self._tcp_cb, topic_qos)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_cb, topic_qos)
        self.create_subscription(Image, self.image_topic, self._image_cb, image_qos)

        self._stop_clients = {
            self.side: self.create_client(Empty, f'/{self.side}/emergency_stop'),
        }
        self._gate_clients = {
            self.side: self.create_client(SetBool, f'/{self.side}/control_enable'),
        }
        self._ik_client = self.create_client(
            GetPositionIK, str(self.get_parameter('ik_service').value))
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, self.trajectory_action_name)
        self.create_timer(0.1, self._watchdog_tick)
        if self.execution_mode == 'trajectory':
            self._trajectory_worker = threading.Thread(
                target=self._trajectory_worker_loop,
                name='nero-vla-trajectory',
                daemon=True,
            )
            self._trajectory_worker.start()
        self._start_socket_server()

        self.get_logger().info(
            f'VLA bridge listening at {self.socket_path}; side={self.side}, '
            f'execution_mode={self.execution_mode}, tcp={self.tcp_topic}, '
            f'joints={self.joint_state_topic}, image={self.image_topic}, '
            f'ik={self.ik_group_name}/{self.ik_link_name} in {self.ik_frame_id}'
        )

    def _stamp_to_sec(self, stamp: Any) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _tcp_cb(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.ik_frame_id:
            self.get_logger().warn(
                f'Rejected TCP pose in frame {msg.header.frame_id!r}; '
                f'expected {self.ik_frame_id!r}',
                throttle_duration_sec=5.0,
            )
            return
        pose = msg.pose
        quaternion = pose.orientation
        try:
            rotation = _quaternion_to_axis_angle(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        except ValueError as exc:
            self.get_logger().warn(f'Rejected TCP pose: {exc}')
            return
        with self._lock:
            self._tcp_xyz = [pose.position.x, pose.position.y, pose.position.z]
            self._tcp_rot = rotation
            self._tcp_mono = time.monotonic()
            self._tcp_stamp = self._stamp_to_sec(msg.header.stamp)

    def _joint_cb(self, msg: JointState) -> None:
        positions = {name: float(value) for name, value in zip(msg.name, msg.position)}
        joints = [positions.get(name) for name in LOCAL_JOINT_NAMES]
        if not all(value is not None for value in joints):
            joints = [positions.get(f'{self.side}_{name}') for name in LOCAL_JOINT_NAMES]
        gripper = positions.get('gripper')
        if gripper is None:
            gripper = positions.get(f'{self.side}_gripper')
        with self._lock:
            if all(value is not None for value in joints):
                self._joints = [float(value) for value in joints]
            if gripper is not None:
                self._gripper_width_m = float(gripper)
            self._joint_mono = time.monotonic()
            self._joint_stamp = self._stamp_to_sec(msg.header.stamp)

    def _image_cb(self, msg: Image) -> None:
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(
                f'Unsupported image encoding {msg.encoding}; expected rgb8 or bgr8',
                throttle_duration_sec=5.0,
            )
            return
        row_bytes = int(msg.width) * 3
        if int(msg.step) < row_bytes or len(msg.data) < int(msg.height) * int(msg.step):
            self.get_logger().warn('Malformed camera image', throttle_duration_sec=5.0)
            return
        try:
            buffer = np.frombuffer(msg.data, dtype=np.uint8)
            rows = buffer.reshape(int(msg.height), int(msg.step))[:, :row_bytes]
            image = rows.reshape(int(msg.height), int(msg.width), 3)
            rgb = image if msg.encoding == 'rgb8' else cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            ok, encoded = cv2.imencode(
                '.jpg', rgb, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to encode camera image: {exc}', throttle_duration_sec=5.0)
            return
        with self._lock:
            self._image_jpeg = encoded.tobytes()
            self._image_shape = (int(msg.height), int(msg.width), 3)
            self._image_mono = time.monotonic()
            self._image_stamp = self._stamp_to_sec(msg.header.stamp)

    def _observation_snapshot(self, require_image: bool) -> tuple[Optional[dict[str, Any]], Optional[str]]:
        now = time.monotonic()
        with self._lock:
            if self._tcp_xyz is None or self._tcp_rot is None:
                return None, 'no TCP feedback received'
            if now - self._tcp_mono > self.observation_timeout:
                return None, f'TCP feedback is stale ({now - self._tcp_mono:.3f}s old)'
            if self._joints is None:
                return None, 'no joint feedback received'
            if now - self._joint_mono > self.observation_timeout:
                return None, f'joint feedback is stale ({now - self._joint_mono:.3f}s old)'
            gripper = 0.0
            if self._gripper_width_m is not None:
                gripper = max(0.0, min(1.0, self._gripper_width_m / self.gripper_max_m))
            state = [*self._tcp_xyz, *self._tcp_rot, gripper]
            snapshot = {
                'state': state,
                'joints': list(self._joints),
                'gripper_available': self._gripper_width_m is not None,
                'timestamps': {
                    'tcp': self._tcp_stamp,
                    'joints': self._joint_stamp,
                },
            }
            if require_image:
                if self._image_jpeg is None or self._image_shape is None:
                    return None, 'no camera image received'
                if now - self._image_mono > self.image_timeout:
                    return None, f'camera image is stale ({now - self._image_mono:.3f}s old)'
                snapshot['image_jpeg'] = self._image_jpeg
                snapshot['image_shape'] = list(self._image_shape)
                snapshot['timestamps']['image'] = self._image_stamp
        return snapshot, None

    def _filter_action(self, action: list[float], current_state: list[float]) -> tuple[list[float], bool]:
        applied = list(action)
        clamped = False
        for index in range(3):
            bounded = max(self.workspace_min[index], min(self.workspace_max[index], applied[index]))
            clamped = clamped or bounded != applied[index]
            applied[index] = bounded

        delta = [applied[index] - current_state[index] for index in range(3)]
        distance = math.sqrt(sum(value * value for value in delta))
        if self.max_translation_step > 0.0 and distance > self.max_translation_step:
            scale = self.max_translation_step / distance
            applied[:3] = [current_state[index] + delta[index] * scale for index in range(3)]
            clamped = True

        rotation, rotation_clamped = _clamp_rotation_step(
            current_state[3:6], applied[3:6], self.max_rotation_step)
        applied[3:6] = rotation
        applied[6] = max(0.0, min(1.0, applied[6]))
        return applied, clamped or rotation_clamped

    def _wait_for_future(
        self,
        future: Any,
        timeout: float,
        abort_event: Optional[threading.Event] = None,
    ) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        while not future.done():
            if abort_event is not None and abort_event.is_set():
                return False
            if not rclpy.ok() or time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return True

    def _solve_ik(self, action: list[float], seed: list[float]) -> list[float]:
        if not self._ik_client.wait_for_service(timeout_sec=self.ik_service_timeout):
            raise MotionError(f'IK service unavailable: {self._ik_client.srv_name}')

        request = GetPositionIK.Request()
        ik_request: PositionIKRequest = request.ik_request
        ik_request.group_name = self.ik_group_name
        ik_request.ik_link_name = self.ik_link_name
        ik_request.avoid_collisions = self.avoid_collisions
        ik_request.timeout = _seconds_to_duration(self.ik_timeout)
        ik_request.pose_stamped.header.frame_id = self.ik_frame_id
        ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped.pose.position.x = action[0]
        ik_request.pose_stamped.pose.position.y = action[1]
        ik_request.pose_stamped.pose.position.z = action[2]
        qx, qy, qz, qw = _axis_angle_to_quaternion(action[3:6])
        ik_request.pose_stamped.pose.orientation.x = qx
        ik_request.pose_stamped.pose.orientation.y = qy
        ik_request.pose_stamped.pose.orientation.z = qz
        ik_request.pose_stamped.pose.orientation.w = qw

        robot_state = RobotState()
        robot_state.joint_state.name = [
            f'{self.side}_joint{index}' for index in range(1, 8)]
        robot_state.joint_state.position = list(seed)
        ik_request.robot_state = robot_state

        with self._motion_request_lock:
            future = self._ik_client.call_async(request)
            if not self._wait_for_future(
                future, self.ik_timeout + self.ik_service_timeout + 0.1):
                raise MotionError('IK call timed out')
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            raise MotionError(f'IK call failed: {exc}') from exc
        if response is None:
            raise MotionError('IK call returned no response')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionError(
                f'IK failed with MoveIt error code {response.error_code.val}')
        return self._validate_joint_solution(response.solution.joint_state, seed)

    def _validate_joint_solution(self, joint_state: JointState, seed: list[float]) -> list[float]:
        positions = {
            name: float(value) for name, value in zip(joint_state.name, joint_state.position)
        }
        solution: list[float] = []
        for index in range(1, 8):
            value = positions.get(f'{self.side}_joint{index}')
            if value is None:
                value = positions.get(f'joint{index}')
            if value is None:
                raise MotionError(f'IK solution is missing joint{index}')
            solution.append(value)
        if not all(math.isfinite(value) for value in solution):
            raise MotionError('IK solution contains a non-finite joint value')

        for index, (value, low, high) in enumerate(zip(
            solution, self.joint_position_min, self.joint_position_max), start=1):
            if value < low - 1e-6 or value > high + 1e-6:
                raise MotionError(
                    f'IK joint{index}={value:.6f} is outside [{low:.6f}, {high:.6f}]')

        step = max(abs(value - previous) for value, previous in zip(solution, seed))
        if step > self.max_joint_step:
            raise MotionError(
                f'IK joint step {step:.6f} rad exceeds {self.max_joint_step:.6f} rad')
        return solution

    def _make_short_trajectory(self, joints: list[float]) -> JointTrajectory:
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        point.time_from_start = _seconds_to_duration(self.trajectory_duration)
        trajectory = JointTrajectory()
        trajectory.joint_names = [f'{self.side}_joint{index}' for index in range(1, 8)]
        trajectory.points = [point]
        return trajectory

    def _set_trajectory_status(self, **values: Any) -> None:
        with self._motion_condition:
            self._trajectory_status = {
                **values,
                'updated_mono': time.monotonic(),
            }

    def _trajectory_snapshot(self) -> dict[str, Any]:
        with self._motion_condition:
            return {
                **self._trajectory_status,
                'pending_sequence': self._pending_sequence,
                'fault': self._motion_fault_reason,
            }

    def _queue_joint_target(self, joints: list[float]) -> int:
        with self._motion_condition:
            self._pending_joint_target = list(joints)
            self._pending_sequence += 1
            sequence = self._pending_sequence
            self._motion_condition.notify_all()
        return sequence

    def _abort_motion(self, reason: str) -> None:
        with self._motion_condition:
            if self._motion_fault_reason is None:
                self._motion_fault_reason = reason
            self._pending_joint_target = None
            self._trajectory_abort.set()
            active_goal = self._active_goal_handle
            start_stop_thread = not self._stop_in_progress
            self._stop_in_progress = True
            self._motion_condition.notify_all()
        self.get_logger().error(f'VLA motion aborted: {reason}')
        if active_goal is not None:
            active_goal.cancel_goal_async()
        if start_stop_thread:
            threading.Thread(
                target=self._stop_and_clear_abort,
                name='nero-vla-stop',
                daemon=True,
            ).start()

    def _stop_and_clear_abort(self) -> None:
        try:
            self._request_stop()
        finally:
            with self._motion_condition:
                self._stop_in_progress = False
                if not self._shutdown.is_set():
                    self._trajectory_abort.clear()
                self._motion_condition.notify_all()

    def _execute_trajectory(self, joints: list[float], sequence: int) -> None:
        if not self._trajectory_client.wait_for_server(
            timeout_sec=self.trajectory_server_timeout
        ):
            self._abort_motion(
                f'trajectory action server unavailable: {self.trajectory_action_name}')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._make_short_trajectory(joints)
        self._set_trajectory_status(state='sending', sequence=sequence)
        self.get_logger().info(
            f'Sending trajectory #{sequence}: ' +
            ' '.join(f'{value:+.4f}' for value in joints),
            throttle_duration_sec=1.0,
        )
        send_future = self._trajectory_client.send_goal_async(goal)
        if not self._wait_for_future(
            send_future,
            self.trajectory_server_timeout + 0.2,
            self._trajectory_abort,
        ):
            if self._trajectory_abort.is_set():
                return
            self._abort_motion('timed out sending trajectory goal')
            return

        try:
            goal_handle = send_future.result()
        except Exception as exc:  # noqa: BLE001
            self._abort_motion(f'failed to send trajectory goal: {exc}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self._abort_motion('trajectory goal was rejected')
            return
        with self._motion_condition:
            self._active_goal_handle = goal_handle
        self._set_trajectory_status(state='executing', sequence=sequence)

        result_future = goal_handle.get_result_async()
        timeout = self.trajectory_duration + self.trajectory_result_timeout
        completed = self._wait_for_future(
            result_future, timeout, self._trajectory_abort)
        with self._motion_condition:
            self._active_goal_handle = None
        if not completed:
            if self._trajectory_abort.is_set():
                goal_handle.cancel_goal_async()
                return
            goal_handle.cancel_goal_async()
            self._abort_motion('timed out waiting for trajectory result')
            return

        try:
            wrapped_result = result_future.result()
        except Exception as exc:  # noqa: BLE001
            self._abort_motion(f'failed to read trajectory result: {exc}')
            return
        error_code = getattr(wrapped_result, 'result', None)
        error_code = getattr(error_code, 'error_code', -1)
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._abort_motion(f'trajectory failed with error code {error_code}')
            return
        self._set_trajectory_status(
            state='completed', sequence=sequence, error_code=error_code)

    def _trajectory_worker_loop(self) -> None:
        while not self._shutdown.is_set():
            with self._motion_condition:
                while (
                    self._pending_joint_target is None
                    and not self._trajectory_abort.is_set()
                    and not self._shutdown.is_set()
                ):
                    self._motion_condition.wait(timeout=0.1)
                if self._shutdown.is_set():
                    return
                if self._trajectory_abort.is_set():
                    self._motion_condition.wait(timeout=0.1)
                    continue
                if self._pending_joint_target is None:
                    continue
                joints = self._pending_joint_target
                sequence = self._pending_sequence
                self._pending_joint_target = None
            try:
                self._execute_trajectory(joints, sequence)
            except Exception as exc:  # noqa: BLE001
                self._abort_motion(f'trajectory worker failed: {exc}')

    def _watchdog_tick(self) -> None:
        if self.execution_mode != 'trajectory':
            return
        now = time.monotonic()
        with self._lock:
            active = self._last_action_mono > 0.0 and now - self._last_action_mono <= self.watchdog_timeout
        if active:
            self._watchdog_fired = False
        elif self._last_action_mono > 0.0 and not self._watchdog_fired:
            self._watchdog_fired = True
            self._abort_motion('VLA action timeout')

    def _request_stop(self) -> None:
        if self.execution_mode != 'trajectory' or not self.allow_real_motion:
            self.get_logger().warn('Stop requested without real motion enabled')
            return
        for side, client in self._stop_clients.items():
            if not client.wait_for_service(timeout_sec=self.stop_service_timeout):
                self.get_logger().error(f'{side} emergency_stop service unavailable')
                continue
            client.call_async(Empty.Request())
        for side, client in self._gate_clients.items():
            if not client.wait_for_service(timeout_sec=self.stop_service_timeout):
                continue
            request = SetBool.Request()
            request.data = False
            client.call_async(request)

    def _dispatch(self, request: dict[str, Any]) -> tuple[dict[str, Any], bytes]:
        command = request.get('cmd')
        if command == 'ping':
            status, error = self._observation_snapshot(require_image=False)
            ik_ready = self._ik_client.service_is_ready()
            trajectory_ready = self._trajectory_client.server_is_ready()
            return {
                'ok': True,
                'socket_path': self.socket_path,
                'side': self.side,
                'dry_run': self.execution_mode == 'dry_run',
                'execution_mode': self.execution_mode,
                'real_motion_enabled': (
                    self.execution_mode == 'trajectory' and self.allow_real_motion),
                'ik_ready': ik_ready,
                'trajectory_ready': trajectory_ready,
                'motion_fault': self._motion_fault_reason,
                'trajectory': self._trajectory_snapshot(),
                'state_ready': status is not None,
                'state_error': error,
            }, b''
        if command == 'get_observation':
            snapshot, error = self._observation_snapshot(require_image=True)
            if snapshot is None:
                return {'ok': False, 'error': error or 'observation unavailable'}, b''
            payload = snapshot.pop('image_jpeg')
            return {
                'ok': True,
                'state': snapshot['state'],
                'joints': snapshot['joints'],
                'gripper_available': snapshot['gripper_available'],
                'timestamps': snapshot['timestamps'],
                'image': {
                    'key': self.image_key,
                    'encoding': 'jpeg',
                    'shape': snapshot['image_shape'],
                },
            }, payload
        if command == 'send_action':
            try:
                action = _as_finite_floats(request.get('action'), STATE_SIZE, 'action')
            except (TypeError, ValueError) as exc:
                return {'ok': False, 'error': str(exc)}, b''
            snapshot, error = self._observation_snapshot(require_image=False)
            if snapshot is None:
                return {'ok': False, 'error': error or 'state unavailable'}, b''
            applied, clamped = self._filter_action(action, snapshot['state'])
            with self._lock:
                self._last_action_mono = time.monotonic()
            self._watchdog_fired = False
            self.get_logger().info(
                'VLA action ' + ' '.join(f'{value:+.4f}' for value in applied) +
                (f' (clamped from {action})' if clamped else ''),
                throttle_duration_sec=1.0,
            )
            if self.execution_mode == 'dry_run':
                return {
                    'ok': True,
                    'action': applied,
                    'accepted': True,
                    'clamped': clamped,
                    'executed': False,
                    'mode': self.execution_mode,
                }, b''
            if self.execution_mode == 'trajectory' and self._motion_fault_reason is not None:
                return {
                    'ok': True,
                    'action': applied,
                    'accepted': False,
                    'clamped': clamped,
                    'executed': False,
                    'mode': self.execution_mode,
                    'error': f'motion fault latched: {self._motion_fault_reason}',
                }, b''
            try:
                joint_target = self._solve_ik(applied, snapshot['joints'])
            except MotionError as exc:
                if self.execution_mode == 'trajectory':
                    self._abort_motion(str(exc))
                return {
                    'ok': True,
                    'action': applied,
                    'accepted': False,
                    'clamped': clamped,
                    'executed': False,
                    'mode': self.execution_mode,
                    'error': str(exc),
                }, b''
            self.get_logger().info(
                'IK target ' + ' '.join(f'{value:+.4f}' for value in joint_target),
                throttle_duration_sec=1.0,
            )
            if self.execution_mode == 'ik_dry_run':
                return {
                    'ok': True,
                    'action': applied,
                    'accepted': True,
                    'clamped': clamped,
                    'executed': False,
                    'mode': self.execution_mode,
                    'joint_target': joint_target,
                    'ik_error_code': MoveItErrorCodes.SUCCESS,
                }, b''

            sequence = self._queue_joint_target(joint_target)
            return {
                'ok': True,
                'action': applied,
                'accepted': True,
                'clamped': clamped,
                'executed': False,
                'queued': True,
                'mode': self.execution_mode,
                'joint_target': joint_target,
                'trajectory_sequence': sequence,
                'gripper_commanded': False,
            }, b''
        if command == 'stop':
            if self.execution_mode == 'trajectory':
                self._abort_motion('stop command received')
            else:
                self._request_stop()
            return {
                'ok': True,
                'stopped': True,
                'dry_run': self.execution_mode == 'dry_run',
                'execution_mode': self.execution_mode,
            }, b''
        return {'ok': False, 'error': f'unknown command: {command!r}'}, b''

    def _client_loop(self, conn: socket.socket) -> None:
        with conn:
            while not self._shutdown.is_set():
                try:
                    request, _ = recv_frame(conn)
                    header, payload = self._dispatch(request)
                    send_frame(conn, header, payload)
                except (ConnectionError, BrokenPipeError, socket.timeout):
                    break
                except (ProtocolError, json.JSONDecodeError) as exc:
                    try:
                        send_frame(conn, {'ok': False, 'error': str(exc)})
                    except OSError:
                        break
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f'VLA socket request failed: {exc}')
                    try:
                        send_frame(conn, {'ok': False, 'error': str(exc)})
                    except OSError:
                        break

    def _start_socket_server(self) -> None:
        socket_dir = os.path.dirname(self.socket_path)
        if socket_dir:
            os.makedirs(socket_dir, exist_ok=True)
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass
        self._server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server.bind(self.socket_path)
        os.chmod(self.socket_path, 0o600)
        self._server.listen(4)
        self._server.settimeout(0.5)
        self._server_thread = threading.Thread(
            target=self._accept_loop, name='nero-vla-socket', daemon=True)
        self._server_thread.start()

    def _accept_loop(self) -> None:
        assert self._server is not None
        while not self._shutdown.is_set():
            try:
                conn, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(
                target=self._client_loop, args=(conn,), name='nero-vla-client', daemon=True
            ).start()

    def shutdown_bridge(self) -> None:
        self._shutdown.set()
        with self._motion_condition:
            self._pending_joint_target = None
            self._trajectory_abort.set()
            active_goal = self._active_goal_handle
            self._motion_condition.notify_all()
        if active_goal is not None:
            active_goal.cancel_goal_async()
        if self._trajectory_worker is not None:
            self._trajectory_worker.join(timeout=1.0)
            self._trajectory_worker = None
        if self.execution_mode == 'trajectory' and self.allow_real_motion:
            self._request_stop()
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
            self._server = None
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass
        if self._server_thread is not None:
            self._server_thread.join(timeout=1.0)
            self._server_thread = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NeroVlaBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_bridge()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

