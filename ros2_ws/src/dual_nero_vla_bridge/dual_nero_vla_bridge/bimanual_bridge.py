#!/usr/bin/env python3
"""Expose cached bimanual NERO observations and validated motions to LeRobot.

The node serves a local Unix stream socket.  LeRobot runs in its own Python
3.12 environment and never imports ``rclpy``.  The default action schema is
the 20-D rot6d layout written by the Xense UMI HDF5 converter::

    left_tcp.x/y/z, left_tcp.r1..r6, left_gripper,
    right_tcp.x/y/z, right_tcp.r1..r6, right_gripper

An optional 14-D axis-angle schema is available for checkpoints that use the
legacy NERO contract.  Both schemas are absolute end-effector targets.  IK is
solved for both arms before either side is sent, and the resulting 14 joints
are dispatched in one ``FollowJointTrajectory`` goal so the arms cannot start
from different action steps.
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
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Empty, SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .image_utils import jpeg_size
from .protocol import ProtocolError, recv_frame, send_frame
from .vla_bridge import (
    MotionError,
    _as_finite_floats,
    _axis_angle_to_quaternion,
    _clamp_rotation_step,
    _quaternion_to_axis_angle,
    _seconds_to_duration,
)

SIDES = ('left', 'right')
SCHEMAS = ('rot6d20', 'axis_angle14')
ACTION_SIZES = {'rot6d20': 20, 'axis_angle14': 14}
EXECUTION_MODES = ('dry_run', 'ik_dry_run', 'trajectory')
LOCAL_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 8))
ALL_JOINT_NAMES = tuple(
    f'{side}_joint{index}' for side in SIDES for index in range(1, 8))


def _quaternion_to_rot6d(qx: float, qy: float, qz: float, qw: float) -> list[float]:
    """Return the first two rotation-matrix columns as ``[m00,m10,m20,m01,m11,m21]``."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError('TCP quaternion is degenerate')
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    return [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
        2.0 * (x * y - z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z + x * w),
    ]


def _rot6d_to_quaternion(values: list[float]) -> tuple[float, float, float, float]:
    """Invert :func:`_quaternion_to_rot6d` with Gram-Schmidt."""
    a1 = np.asarray(values[:3], dtype=np.float64)
    a2 = np.asarray(values[3:], dtype=np.float64)
    norm1 = float(np.linalg.norm(a1))
    if norm1 < 1e-12:
        raise ValueError('rot6d first column is degenerate')
    b1 = a1 / norm1
    a2 = a2 - float(np.dot(b1, a2)) * b1
    norm2 = float(np.linalg.norm(a2))
    if norm2 < 1e-12:
        raise ValueError('rot6d columns are collinear')
    b2 = a2 / norm2
    b3 = np.cross(b1, b2)
    matrix = np.stack([b1, b2, b3], axis=1)

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = int(np.argmax(np.diagonal(matrix)))
        i, j, k = diagonal, (diagonal + 1) % 3, (diagonal + 2) % 3
        scale = math.sqrt(max(
            0.0, 1.0 + matrix[i, i] - matrix[j, j] - matrix[k, k])) * 2.0
        if scale < 1e-12:
            raise ValueError('rot6d matrix is degenerate')
        components = np.zeros(3)
        components[i] = 0.25 * scale
        components[j] = (matrix[j, i] + matrix[i, j]) / scale
        components[k] = (matrix[k, i] + matrix[i, k]) / scale
        qw = (matrix[k, j] - matrix[j, k]) / scale
        qx, qy, qz = components
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return float(qx / norm), float(qy / norm), float(qz / norm), float(qw / norm)


def _rot6d_to_axis_angle(values: list[float]) -> list[float]:
    return _quaternion_to_axis_angle(*_rot6d_to_quaternion(values))


def _axis_angle_to_rot6d(values: list[float]) -> list[float]:
    return _quaternion_to_rot6d(*_axis_angle_to_quaternion(values))


class NeroBimanualVlaBridge(Node):
    """ROS 2 node hosting the bimanual observation/action socket server."""

    def __init__(self, parameter_overrides: Optional[list[Parameter]] = None) -> None:
        super().__init__(
            'dual_nero_bimanual_vla_bridge', parameter_overrides=parameter_overrides)

        self.declare_parameter('socket_path', '/tmp/nero_bimanual_vla_bridge.sock')
        self.declare_parameter('action_schema', 'rot6d20')
        self.declare_parameter('left_tcp_topic', '/left/feedback/tcp_pose')
        self.declare_parameter('right_tcp_topic', '/right/feedback/tcp_pose')
        self.declare_parameter('left_joint_state_topic', '/left/feedback/joint_states')
        self.declare_parameter('right_joint_state_topic', '/right/feedback/joint_states')

        self.declare_parameter('camera_scene_topic', '/camera_scene/color/image_raw')
        self.declare_parameter(
            'camera_scene_compressed_topic',
            '/camera_scene/color/image_raw/compressed')
        self.declare_parameter('camera_scene_key', 'side')
        self.declare_parameter('camera_left_wrist_topic', '/camera_left_wrist/color/image_raw')
        self.declare_parameter(
            'camera_left_wrist_compressed_topic',
            '/camera_left_wrist/color/image_raw/compressed')
        self.declare_parameter('camera_left_wrist_key', 'left_wrist')
        self.declare_parameter('camera_right_wrist_topic', '/camera_right_wrist/color/image_raw')
        self.declare_parameter(
            'camera_right_wrist_compressed_topic',
            '/camera_right_wrist/color/image_raw/compressed')
        self.declare_parameter('camera_right_wrist_key', 'right_wrist')
        self.declare_parameter('use_compressed_color', True)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('observation_timeout_sec', 0.75)
        self.declare_parameter('image_timeout_sec', 1.5)

        self.declare_parameter('execution_mode', 'dry_run')
        self.declare_parameter('allow_real_motion', False)
        self.declare_parameter('max_translation_step_m', 0.01)
        self.declare_parameter('max_rotation_step_rad', 0.05)
        self.declare_parameter('max_joint_step_rad', 0.15)
        self.declare_parameter('joint_position_min', [
            -2.70526, -1.760, -2.75, -1.01, -2.75, -0.7330383, -1.5707963,
        ])
        self.declare_parameter('joint_position_max', [
            2.70526, 1.74, 2.75, 2.14, 2.75, 0.8726646, 1.5707963,
        ])
        self.declare_parameter('workspace_min_xyz', [-2.0, -2.0, -0.2])
        self.declare_parameter('workspace_max_xyz', [2.0, 2.0, 2.0])
        self.declare_parameter('gripper_max_m', 0.09)
        self.declare_parameter('left_gripper_width_topic', '/left_gripper/width')
        self.declare_parameter('right_gripper_width_topic', '/right_gripper/width')
        self.declare_parameter(
            'left_gripper_command_topic', '/left_gripper/position_cmd')
        self.declare_parameter(
            'right_gripper_command_topic', '/right_gripper/position_cmd')
        self.declare_parameter('allow_gripper_motion', False)
        self.declare_parameter('gripper_position_min', 0)
        self.declare_parameter('gripper_position_max', 990)
        self.declare_parameter('gripper_feedback_timeout_sec', 1.0)
        self.declare_parameter('watchdog_timeout_sec', 0.75)
        self.declare_parameter('stop_service_timeout_sec', 0.5)

        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('left_ik_group_name', 'left_arm')
        self.declare_parameter('right_ik_group_name', 'right_arm')
        self.declare_parameter('left_ik_link_name', 'left_tcp_link')
        self.declare_parameter('right_ik_link_name', 'right_tcp_link')
        self.declare_parameter('left_ik_frame_id', 'left_base_link')
        self.declare_parameter('right_ik_frame_id', 'right_base_link')
        self.declare_parameter('ik_timeout_sec', 0.1)
        self.declare_parameter('ik_service_timeout_sec', 0.2)
        self.declare_parameter('avoid_collisions', True)
        self.declare_parameter(
            'trajectory_action', '/dual_arm_controller/follow_joint_trajectory')
        self.declare_parameter('trajectory_duration_sec', 0.2)
        self.declare_parameter('trajectory_server_timeout_sec', 0.25)
        self.declare_parameter('trajectory_result_timeout_sec', 2.0)

        self.socket_path = str(self.get_parameter('socket_path').value)
        self.action_schema = str(self.get_parameter('action_schema').value)
        if self.action_schema not in SCHEMAS:
            raise ValueError(
                f'action_schema must be one of {SCHEMAS}, got {self.action_schema!r}')
        self.action_size = ACTION_SIZES[self.action_schema]

        self.tcp_topics = {
            side: str(self.get_parameter(f'{side}_tcp_topic').value)
            for side in SIDES
        }
        self.joint_state_topics = {
            side: str(self.get_parameter(f'{side}_joint_state_topic').value)
            for side in SIDES
        }
        self.ik_group_names = {
            side: str(self.get_parameter(f'{side}_ik_group_name').value)
            for side in SIDES
        }
        self.ik_link_names = {
            side: str(self.get_parameter(f'{side}_ik_link_name').value)
            for side in SIDES
        }
        self.ik_frame_ids = {
            side: str(self.get_parameter(f'{side}_ik_frame_id').value)
            for side in SIDES
        }

        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.observation_timeout = float(self.get_parameter('observation_timeout_sec').value)
        self.image_timeout = float(self.get_parameter('image_timeout_sec').value)
        self.execution_mode = str(self.get_parameter('execution_mode').value)
        if self.execution_mode not in EXECUTION_MODES:
            raise ValueError(
                f'execution_mode must be one of {EXECUTION_MODES}, '
                f'got {self.execution_mode!r}')
        self.allow_real_motion = bool(self.get_parameter('allow_real_motion').value)
        if self.execution_mode == 'trajectory' and not self.allow_real_motion:
            raise ValueError('execution_mode=trajectory requires allow_real_motion=true')

        self.max_translation_step = float(
            self.get_parameter('max_translation_step_m').value)
        self.max_rotation_step = float(
            self.get_parameter('max_rotation_step_rad').value)
        self.max_joint_step = float(self.get_parameter('max_joint_step_rad').value)
        self.joint_position_min = _as_finite_floats(
            self.get_parameter('joint_position_min').value, 7, 'joint_position_min')
        self.joint_position_max = _as_finite_floats(
            self.get_parameter('joint_position_max').value, 7, 'joint_position_max')
        if any(low >= high for low, high in zip(
                self.joint_position_min, self.joint_position_max)):
            raise ValueError('joint_position_min must be strictly below joint_position_max')
        self.workspace_min = _as_finite_floats(
            self.get_parameter('workspace_min_xyz').value, 3, 'workspace_min_xyz')
        self.workspace_max = _as_finite_floats(
            self.get_parameter('workspace_max_xyz').value, 3, 'workspace_max_xyz')
        if any(low >= high for low, high in zip(
                self.workspace_min, self.workspace_max)):
            raise ValueError('workspace_min_xyz must be strictly below workspace_max_xyz')
        self.gripper_max_m = max(1e-6, float(self.get_parameter('gripper_max_m').value))
        self.gripper_width_topics = {
            side: str(self.get_parameter(f'{side}_gripper_width_topic').value)
            for side in SIDES
        }
        self.gripper_command_topics = {
            side: str(self.get_parameter(f'{side}_gripper_command_topic').value)
            for side in SIDES
        }
        self.allow_gripper_motion = bool(
            self.get_parameter('allow_gripper_motion').value)
        self.gripper_position_min = int(
            self.get_parameter('gripper_position_min').value)
        self.gripper_position_max = int(
            self.get_parameter('gripper_position_max').value)
        if self.gripper_position_max <= self.gripper_position_min:
            raise ValueError(
                'gripper_position_max must be greater than gripper_position_min')
        self.gripper_feedback_timeout = max(
            0.1, float(self.get_parameter('gripper_feedback_timeout_sec').value))
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_sec').value)
        self.stop_service_timeout = float(
            self.get_parameter('stop_service_timeout_sec').value)
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

        self.use_compressed_color = bool(
            self.get_parameter('use_compressed_color').value)
        self.camera_specs = [
            {
                'topic': str(self.get_parameter('camera_scene_topic').value),
                'compressed_topic': str(
                    self.get_parameter('camera_scene_compressed_topic').value),
                'key': str(self.get_parameter('camera_scene_key').value),
            },
            {
                'topic': str(self.get_parameter('camera_left_wrist_topic').value),
                'compressed_topic': str(
                    self.get_parameter(
                        'camera_left_wrist_compressed_topic').value),
                'key': str(self.get_parameter('camera_left_wrist_key').value),
            },
            {
                'topic': str(self.get_parameter('camera_right_wrist_topic').value),
                'compressed_topic': str(
                    self.get_parameter(
                        'camera_right_wrist_compressed_topic').value),
                'key': str(self.get_parameter('camera_right_wrist_key').value),
            },
        ]
        for spec in self.camera_specs:
            spec['topic'] = (
                spec['compressed_topic']
                if self.use_compressed_color
                else spec['topic'])
        self.camera_specs = [
            spec for spec in self.camera_specs if spec['topic']]
        keys = [spec['key'] for spec in self.camera_specs]
        if len(set(keys)) != len(keys):
            raise ValueError('camera image keys must be unique')
        if not self.camera_specs:
            raise ValueError('at least one camera topic is required')

        self._lock = threading.Lock()
        self._feedback = {
            side: {
                'tcp_xyz': None,
                'tcp_rot6d': None,
                'tcp_rotvec': None,
                'tcp_mono': 0.0,
                'tcp_stamp': 0.0,
                'joints': None,
                'gripper_width_m': None,
                'gripper_mono': 0.0,
                'joint_mono': 0.0,
                'joint_stamp': 0.0,
            }
            for side in SIDES
        }
        self._images: dict[str, dict[str, Any]] = {}
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
        for side in SIDES:
            self.create_subscription(
                PoseStamped,
                self.tcp_topics[side],
                lambda msg, side=side: self._tcp_cb(side, msg),
                topic_qos,
            )
            self.create_subscription(
                JointState,
                self.joint_state_topics[side],
                lambda msg, side=side: self._joint_cb(side, msg),
                topic_qos,
            )
            self.create_subscription(
                Float32,
                self.gripper_width_topics[side],
                lambda msg, side=side: self._gripper_width_cb(side, msg),
                topic_qos,
            )
        self._gripper_command_publishers = {
            side: self.create_publisher(
                Int32, self.gripper_command_topics[side], topic_qos)
            for side in SIDES
        }
        for spec in self.camera_specs:
            if self.use_compressed_color:
                self.create_subscription(
                    CompressedImage,
                    spec['topic'],
                    lambda msg, key=spec['key']: self._compressed_image_cb(
                        key, msg),
                    image_qos,
                )
            else:
                self.create_subscription(
                    Image,
                    spec['topic'],
                    lambda msg, key=spec['key']: self._image_cb(key, msg),
                    image_qos,
                )

        self._stop_clients = {
            side: self.create_client(Empty, f'/{side}/emergency_stop')
            for side in SIDES
        }
        self._gripper_stop_clients = {
            side: self.create_client(
                Empty, f'/{side}_gripper/emergency_stop')
            for side in SIDES
        }
        self._gate_clients = {
            side: self.create_client(SetBool, f'/{side}/control_enable')
            for side in SIDES
        }
        self._ik_client = self.create_client(
            GetPositionIK, str(self.get_parameter('ik_service').value))
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, self.trajectory_action_name)
        self.create_timer(0.1, self._watchdog_tick)
        if self.execution_mode == 'trajectory':
            self._trajectory_worker = threading.Thread(
                target=self._trajectory_worker_loop,
                name='nero-bimanual-vla-trajectory',
                daemon=True,
            )
            self._trajectory_worker.start()
        self._start_socket_server()

        self.get_logger().info(
            f'Bimanual VLA bridge listening at {self.socket_path}; '
            f'schema={self.action_schema}, execution_mode={self.execution_mode}, '
            f'tcp={self.tcp_topics}, cameras={[s["key"] for s in self.camera_specs]}, '
            f'ik={self.ik_group_names}'
        )

    @staticmethod
    def _stamp_to_sec(stamp: Any) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _tcp_cb(self, side: str, msg: PoseStamped) -> None:
        expected_frame = self.ik_frame_ids[side]
        if msg.header.frame_id and msg.header.frame_id != expected_frame:
            self.get_logger().warn(
                f'Rejected {side} TCP pose in frame {msg.header.frame_id!r}; '
                f'expected {expected_frame!r}',
                throttle_duration_sec=5.0,
            )
            return
        pose = msg.pose
        quaternion = pose.orientation
        try:
            rotation = _quaternion_to_axis_angle(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            rotation6d = _quaternion_to_rot6d(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        except ValueError as exc:
            self.get_logger().warn(f'Rejected {side} TCP pose: {exc}')
            return
        with self._lock:
            feedback = self._feedback[side]
            feedback['tcp_xyz'] = [
                pose.position.x, pose.position.y, pose.position.z]
            feedback['tcp_rotvec'] = rotation
            feedback['tcp_rot6d'] = rotation6d
            feedback['tcp_mono'] = time.monotonic()
            feedback['tcp_stamp'] = self._stamp_to_sec(msg.header.stamp)

    def _joint_cb(self, side: str, msg: JointState) -> None:
        positions = {
            name: float(value) for name, value in zip(msg.name, msg.position)
        }
        joints = [positions.get(name) for name in LOCAL_JOINT_NAMES]
        if not all(value is not None for value in joints):
            joints = [
                positions.get(f'{side}_{name}') for name in LOCAL_JOINT_NAMES]
        gripper = positions.get('gripper')
        if gripper is None:
            gripper = positions.get(f'{side}_gripper')
        with self._lock:
            feedback = self._feedback[side]
            if all(value is not None for value in joints):
                feedback['joints'] = [float(value) for value in joints]
            if gripper is not None:
                feedback['gripper_width_m'] = float(gripper)
            feedback['joint_mono'] = time.monotonic()
            feedback['joint_stamp'] = self._stamp_to_sec(msg.header.stamp)

    def _gripper_width_cb(self, side: str, msg: Float32) -> None:
        width = float(msg.data)
        if not math.isfinite(width) or width < 0.0:
            self.get_logger().warn(
                f'Rejected invalid {side} gripper width: {width}',
                throttle_duration_sec=5.0,
            )
            return
        with self._lock:
            feedback = self._feedback[side]
            feedback['gripper_width_m'] = width
            feedback['gripper_mono'] = time.monotonic()

    def _gripper_feedback_ready(self, now: Optional[float] = None) -> bool:
        if now is None:
            now = time.monotonic()
        with self._lock:
            for side in SIDES:
                feedback = self._feedback[side]
                if feedback['gripper_width_m'] is None:
                    return False
                if now - feedback['gripper_mono'] > self.gripper_feedback_timeout:
                    return False
        return True

    def _send_gripper_commands(self, applied_poses: dict[str, list[float]]) -> dict:
        if not self.allow_gripper_motion:
            return {
                'gripper_commanded': False,
                'gripper_positions': {},
                'gripper_error': 'gripper motion disabled in VLA bridge',
            }
        if self.execution_mode == 'dry_run':
            return {
                'gripper_commanded': False,
                'gripper_positions': {},
                'gripper_error': None,
            }
        now = time.monotonic()
        with self._lock:
            stale = {}
            for side in SIDES:
                feedback = self._feedback[side]
                age = (
                    now - feedback['gripper_mono']
                    if feedback['gripper_mono'] > 0.0 else None
                )
                if (
                    feedback['gripper_width_m'] is None
                    or age is None
                    or age > self.gripper_feedback_timeout
                ):
                    stale[side] = age
        if stale:
            detail = ', '.join(
                f'{side}=' + ('missing' if age is None else f'{age:.2f}s old')
                for side, age in stale.items()
            )
            return {
                'gripper_commanded': False,
                'gripper_positions': {},
                'gripper_error': f'gripper feedback not fresh: {detail}',
            }

        positions = {}
        span = self.gripper_position_max - self.gripper_position_min
        for side in SIDES:
            normalized = max(0.0, min(1.0, float(applied_poses[side][6])))
            positions[side] = int(round(
                self.gripper_position_min + (1.0 - normalized) * span))
        for side in SIDES:
            message = Int32()
            message.data = positions[side]
            self._gripper_command_publishers[side].publish(message)
        self.get_logger().info(
            'VLA gripper positions sent: '
            + ', '.join(f'{side}={position}' for side, position in positions.items()),
            throttle_duration_sec=1.0,
        )
        return {
            'gripper_commanded': True,
            'gripper_positions': positions,
            'gripper_error': None,
        }

    def _image_cb(self, key: str, msg: Image) -> None:
        if msg.encoding not in ('rgb8', 'bgr8', 'yuv422_yuy2', 'yuyv', 'yuv422'):
            self.get_logger().warn(
                f'Unsupported image encoding {msg.encoding} for {key}; '
                'expected RGB8 or YUYV',
                throttle_duration_sec=5.0,
            )
            return
        channels = 2 if msg.encoding in ('yuv422_yuy2', 'yuyv', 'yuv422') else 3
        row_bytes = int(msg.width) * channels
        if int(msg.step) < row_bytes or len(msg.data) < int(msg.height) * int(msg.step):
            self.get_logger().warn(
                f'Malformed camera image for {key}', throttle_duration_sec=5.0)
            return
        try:
            buffer = np.frombuffer(msg.data, dtype=np.uint8)
            rows = buffer.reshape(int(msg.height), int(msg.step))[:, :row_bytes]
            image = rows.reshape(int(msg.height), int(msg.width), channels)
            if channels == 2:
                rgb = cv2.cvtColor(image, cv2.COLOR_YUV2RGB_YUYV)
            elif msg.encoding == 'bgr8':
                rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                rgb = image
            ok, encoded = cv2.imencode(
                '.jpg', rgb, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'Failed to encode {key}: {exc}', throttle_duration_sec=5.0)
            return
        with self._lock:
            self._images[key] = {
                'jpeg': encoded.tobytes(),
                'shape': (int(msg.height), int(msg.width), 3),
                'mono': time.monotonic(),
                'stamp': self._stamp_to_sec(msg.header.stamp),
            }

    def _compressed_image_cb(
        self, key: str, msg: CompressedImage
    ) -> None:
        image_format = msg.format.lower()
        if 'jpeg' not in image_format and 'jpg' not in image_format:
            self.get_logger().warn(
                f'Unsupported compressed image format {msg.format!r} '
                f'for {key}; expected JPEG',
                throttle_duration_sec=5.0,
            )
            return
        jpeg = bytes(msg.data)
        size = jpeg_size(jpeg)
        if size is None:
            self.get_logger().warn(
                f'Malformed compressed camera image for {key}',
                throttle_duration_sec=5.0,
            )
            return
        with self._lock:
            self._images[key] = {
                'jpeg': jpeg,
                'shape': (*size, 3),
                'mono': time.monotonic(),
                'stamp': self._stamp_to_sec(msg.header.stamp),
            }

    def _observation_snapshot(
        self, require_images: bool
    ) -> tuple[Optional[dict[str, Any]], Optional[str]]:
        now = time.monotonic()
        with self._lock:
            state: list[float] = []
            joints_by_side: dict[str, list[float]] = {}
            grippers: dict[str, float] = {}
            pose_by_side: dict[str, list[float]] = {}
            timestamps: dict[str, float] = {}
            for side in SIDES:
                feedback = self._feedback[side]
                if feedback['tcp_xyz'] is None or feedback['tcp_rotvec'] is None:
                    return None, f'no {side} TCP feedback received'
                tcp_age = now - feedback['tcp_mono']
                if tcp_age > self.observation_timeout:
                    return None, f'{side} TCP feedback is stale ({tcp_age:.3f}s old)'
                if feedback['joints'] is None:
                    return None, f'no {side} joint feedback received'
                joint_age = now - feedback['joint_mono']
                if joint_age > self.observation_timeout:
                    return None, (
                        f'{side} joint feedback is stale ({joint_age:.3f}s old)')

                gripper = 0.0
                if feedback['gripper_width_m'] is not None:
                    gripper = max(0.0, min(
                        1.0, feedback['gripper_width_m'] / self.gripper_max_m))
                grippers[side] = gripper
                pose = [
                    *feedback['tcp_xyz'],
                    *feedback['tcp_rotvec'],
                    gripper,
                ]
                pose_by_side[side] = pose
                joints_by_side[side] = list(feedback['joints'])
                if self.action_schema == 'rot6d20':
                    state.extend([
                        *feedback['tcp_xyz'],
                        *feedback['tcp_rot6d'],
                        gripper,
                    ])
                else:
                    state.extend(pose)
                timestamps[f'{side}_tcp'] = feedback['tcp_stamp']
                timestamps[f'{side}_joints'] = feedback['joint_stamp']

            images: list[dict[str, Any]] = []
            if require_images:
                for spec in self.camera_specs:
                    key = spec['key']
                    image = self._images.get(key)
                    if image is None:
                        return None, f'no camera image received for {key!r}'
                    age = now - image['mono']
                    if age > self.image_timeout:
                        return None, (
                            f'camera image {key!r} is stale ({age:.3f}s old)')
                    images.append({'key': key, **image})
                    timestamps[key] = image['stamp']

            return {
                'state': state,
                'joints': joints_by_side,
                'pose_by_side': pose_by_side,
                'grippers': grippers,
                'timestamps': timestamps,
                'images': images,
            }, None

    def _filter_pose(
        self, action: list[float], current_state: list[float]
    ) -> tuple[list[float], bool]:
        applied = list(action)
        clamped = False
        for index in range(3):
            bounded = max(
                self.workspace_min[index],
                min(self.workspace_max[index], applied[index]),
            )
            clamped = clamped or bounded != applied[index]
            applied[index] = bounded

        delta = [applied[index] - current_state[index] for index in range(3)]
        distance = math.sqrt(sum(value * value for value in delta))
        if self.max_translation_step > 0.0 and distance > self.max_translation_step:
            scale = self.max_translation_step / distance
            applied[:3] = [
                current_state[index] + delta[index] * scale for index in range(3)]
            clamped = True

        rotation, rotation_clamped = _clamp_rotation_step(
            current_state[3:6], applied[3:6], self.max_rotation_step)
        applied[3:6] = rotation
        applied[6] = max(0.0, min(1.0, applied[6]))
        return applied, clamped or rotation_clamped

    def _parse_action(self, values: list[float]) -> dict[str, list[float]]:
        parsed: dict[str, list[float]] = {}
        offset = 0
        for side in SIDES:
            if self.action_schema == 'rot6d20':
                xyz = values[offset:offset + 3]
                rotation = _rot6d_to_axis_angle(values[offset + 3:offset + 9])
                gripper = values[offset + 9]
                offset += 10
            else:
                xyz = values[offset:offset + 3]
                rotation = values[offset + 3:offset + 6]
                gripper = values[offset + 6]
                offset += 7
            parsed[side] = [*xyz, *rotation, gripper]
        return parsed

    def _encode_side_action(self, action: list[float]) -> list[float]:
        rotation = (
            _axis_angle_to_rot6d(action[3:6])
            if self.action_schema == 'rot6d20'
            else list(action[3:6])
        )
        return [*action[:3], *rotation, action[6]]

    def _filter_action(
        self, action: list[float], pose_by_side: dict[str, list[float]]
    ) -> tuple[list[float], dict[str, list[float]], bool]:
        internal = self._parse_action(action)
        applied_poses: dict[str, list[float]] = {}
        applied: list[float] = []
        clamped = False
        for side in SIDES:
            filtered, side_clamped = self._filter_pose(
                internal[side], pose_by_side[side])
            applied_poses[side] = filtered
            applied.extend(self._encode_side_action(filtered))
            clamped = clamped or side_clamped
        return applied, applied_poses, clamped

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

    def _solve_side_ik(
        self,
        side: str,
        pose: list[float],
        seed: list[float],
        joint_positions: dict[str, list[float]],
    ) -> list[float]:
        if not self._ik_client.wait_for_service(timeout_sec=self.ik_service_timeout):
            raise MotionError(f'IK service unavailable: {self._ik_client.srv_name}')

        request = GetPositionIK.Request()
        ik_request: PositionIKRequest = request.ik_request
        ik_request.group_name = self.ik_group_names[side]
        ik_request.ik_link_name = self.ik_link_names[side]
        ik_request.avoid_collisions = self.avoid_collisions
        ik_request.timeout = _seconds_to_duration(self.ik_timeout)
        ik_request.pose_stamped.header.frame_id = self.ik_frame_ids[side]
        ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped.pose.position.x = pose[0]
        ik_request.pose_stamped.pose.position.y = pose[1]
        ik_request.pose_stamped.pose.position.z = pose[2]
        qx, qy, qz, qw = _axis_angle_to_quaternion(pose[3:6])
        ik_request.pose_stamped.pose.orientation.x = qx
        ik_request.pose_stamped.pose.orientation.y = qy
        ik_request.pose_stamped.pose.orientation.z = qz
        ik_request.pose_stamped.pose.orientation.w = qw

        robot_state = RobotState()
        robot_state.joint_state.name = list(ALL_JOINT_NAMES)
        robot_state.joint_state.position = [
            *joint_positions['left'], *joint_positions['right']]
        ik_request.robot_state = robot_state

        with self._motion_request_lock:
            future = self._ik_client.call_async(request)
            if not self._wait_for_future(
                    future, self.ik_timeout + self.ik_service_timeout + 0.1):
                raise MotionError(f'{side} IK call timed out')
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            raise MotionError(f'{side} IK call failed: {exc}') from exc
        if response is None:
            raise MotionError(f'{side} IK call returned no response')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionError(
                f'{side} IK failed with MoveIt error code '
                f'{response.error_code.val}')
        return self._validate_joint_solution(
            side, response.solution.joint_state, seed)

    def _validate_joint_solution(
        self, side: str, joint_state: JointState, seed: list[float]
    ) -> list[float]:
        positions = {
            name: float(value)
            for name, value in zip(joint_state.name, joint_state.position)
        }
        solution: list[float] = []
        for index in range(1, 8):
            value = positions.get(f'{side}_joint{index}')
            if value is None:
                value = positions.get(f'joint{index}')
            if value is None:
                raise MotionError(
                    f'{side} IK solution is missing joint{index}')
            solution.append(value)
        if not all(math.isfinite(value) for value in solution):
            raise MotionError(
                f'{side} IK solution contains a non-finite joint value')

        for index, (value, low, high) in enumerate(zip(
                solution, self.joint_position_min, self.joint_position_max),
                start=1):
            if value < low - 1e-6 or value > high + 1e-6:
                raise MotionError(
                    f'{side} IK joint{index}={value:.6f} is outside '
                    f'[{low:.6f}, {high:.6f}]')

        step = max(abs(value - previous) for value, previous in zip(solution, seed))
        if step > self.max_joint_step:
            raise MotionError(
                f'{side} IK joint step {step:.6f} rad exceeds '
                f'{self.max_joint_step:.6f} rad')
        return solution

    def _solve_bimanual(
        self,
        poses: dict[str, list[float]],
        joints_by_side: dict[str, list[float]],
    ) -> dict[str, list[float]]:
        left_seed = list(joints_by_side['left'])
        right_seed = list(joints_by_side['right'])
        left_solution = self._solve_side_ik(
            'left',
            poses['left'],
            left_seed,
            {'left': left_seed, 'right': right_seed},
        )
        right_solution = self._solve_side_ik(
            'right',
            poses['right'],
            right_seed,
            {'left': left_solution, 'right': right_seed},
        )
        return {'left': left_solution, 'right': right_solution}

    def _make_short_trajectory(
        self, joints_by_side: dict[str, list[float]]
    ) -> JointTrajectory:
        point = JointTrajectoryPoint()
        point.positions = [
            *joints_by_side['left'], *joints_by_side['right']]
        point.time_from_start = _seconds_to_duration(self.trajectory_duration)
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ALL_JOINT_NAMES)
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
        self.get_logger().error(f'Bimanual VLA motion aborted: {reason}')
        if active_goal is not None:
            active_goal.cancel_goal_async()
        if start_stop_thread:
            threading.Thread(
                target=self._stop_and_clear_abort,
                name='nero-bimanual-vla-stop',
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
                timeout_sec=self.trajectory_server_timeout):
            self._abort_motion(
                f'trajectory action server unavailable: {self.trajectory_action_name}')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._make_short_trajectory({
            'left': joints[:7],
            'right': joints[7:],
        })
        self._set_trajectory_status(state='sending', sequence=sequence)
        self.get_logger().info(
            f'Sending bimanual trajectory #{sequence}: ' +
            ' '.join(f'{value:+.4f}' for value in joints),
            throttle_duration_sec=1.0,
        )
        send_future = self._trajectory_client.send_goal_async(goal)
        if not self._wait_for_future(
                send_future,
                self.trajectory_server_timeout + 0.2,
                self._trajectory_abort):
            if self._trajectory_abort.is_set():
                return
            self._abort_motion('timed out sending bimanual trajectory goal')
            return

        try:
            goal_handle = send_future.result()
        except Exception as exc:  # noqa: BLE001
            self._abort_motion(f'failed to send bimanual trajectory goal: {exc}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self._abort_motion('bimanual trajectory goal was rejected')
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
            self._abort_motion('timed out waiting for bimanual trajectory result')
            return

        try:
            wrapped_result = result_future.result()
        except Exception as exc:  # noqa: BLE001
            self._abort_motion(f'failed to read bimanual trajectory result: {exc}')
            return
        error_code = getattr(wrapped_result, 'result', None)
        error_code = getattr(error_code, 'error_code', -1)
        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._abort_motion(
                f'bimanual trajectory failed with error code {error_code}')
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
                self._abort_motion(f'bimanual trajectory worker failed: {exc}')

    def _watchdog_tick(self) -> None:
        if self.execution_mode != 'trajectory':
            return
        now = time.monotonic()
        with self._lock:
            active = (
                self._last_action_mono > 0.0
                and now - self._last_action_mono <= self.watchdog_timeout)
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
                self.get_logger().error(
                    f'{side} emergency_stop service unavailable')
                continue
            client.call_async(Empty.Request())
        for side, client in self._gate_clients.items():
            if not client.wait_for_service(timeout_sec=self.stop_service_timeout):
                continue
            request = SetBool.Request()
            request.data = False
            client.call_async(request)
        for side, client in self._gripper_stop_clients.items():
            if not client.wait_for_service(timeout_sec=self.stop_service_timeout):
                continue
            client.call_async(Empty.Request())

    def _dispatch(self, request: dict[str, Any]) -> tuple[dict[str, Any], bytes]:
        command = request.get('cmd')
        if command == 'ping':
            status, error = self._observation_snapshot(require_images=False)
            return {
                'ok': True,
                'bimanual': True,
                'socket_path': self.socket_path,
                'sides': list(SIDES),
                'schema': self.action_schema,
                'dry_run': self.execution_mode == 'dry_run',
                'execution_mode': self.execution_mode,
                'real_motion_enabled': (
                    self.execution_mode == 'trajectory' and self.allow_real_motion),
                'ik_ready': self._ik_client.service_is_ready(),
                'trajectory_ready': self._trajectory_client.server_is_ready(),
                'motion_fault': self._motion_fault_reason,
                'trajectory': self._trajectory_snapshot(),
                'allow_gripper_motion': self.allow_gripper_motion,
                'gripper_feedback_ready': self._gripper_feedback_ready(),
                'gripper_width_topics': dict(self.gripper_width_topics),
                'gripper_command_topics': dict(self.gripper_command_topics),
                'camera_keys': [spec['key'] for spec in self.camera_specs],
                'state_ready': status is not None,
                'state_error': error,
            }, b''
        if command == 'get_observation':
            snapshot, error = self._observation_snapshot(require_images=True)
            if snapshot is None:
                return {
                    'ok': False,
                    'error': error or 'observation unavailable',
                }, b''
            payload = bytearray()
            image_metadata = []
            for image in snapshot['images']:
                jpeg = image['jpeg']
                offset = len(payload)
                payload.extend(jpeg)
                image_metadata.append({
                    'key': image['key'],
                    'encoding': 'jpeg',
                    'shape': list(image['shape']),
                    'offset': offset,
                    'size': len(jpeg),
                })
            return {
                'ok': True,
                'schema': self.action_schema,
                'state': snapshot['state'],
                'joints': snapshot['joints'],
                'grippers': snapshot['grippers'],
                'timestamps': snapshot['timestamps'],
                'images': image_metadata,
            }, bytes(payload)
        if command == 'send_action':
            try:
                action = _as_finite_floats(
                    request.get('action'), self.action_size, 'action')
            except (TypeError, ValueError) as exc:
                return {'ok': False, 'error': str(exc)}, b''
            snapshot, error = self._observation_snapshot(require_images=False)
            if snapshot is None:
                return {'ok': False, 'error': error or 'state unavailable'}, b''
            try:
                applied, applied_poses, clamped = self._filter_action(
                    action, snapshot['pose_by_side'])
            except ValueError as exc:
                return {'ok': False, 'error': str(exc)}, b''
            with self._lock:
                self._last_action_mono = time.monotonic()
            self._watchdog_fired = False
            self.get_logger().info(
                'Bimanual VLA action ' +
                ' '.join(f'{value:+.4f}' for value in applied) +
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
                    'gripper_commanded': False,
                    'gripper_positions': {},
                    'gripper_error': None,
                }, b''
            if (self.execution_mode == 'trajectory'
                    and self._motion_fault_reason is not None):
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
                joint_targets = self._solve_bimanual(
                    applied_poses, snapshot['joints'])
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
            joint_target = [
                *joint_targets['left'], *joint_targets['right']]
            self.get_logger().info(
                'Bimanual IK target ' +
                ' '.join(f'{value:+.4f}' for value in joint_target),
                throttle_duration_sec=1.0,
            )
            if self.execution_mode == 'ik_dry_run':
                result = {
                    'ok': True,
                    'action': applied,
                    'accepted': True,
                    'clamped': clamped,
                    'executed': False,
                    'mode': self.execution_mode,
                    'joint_target': joint_target,
                    'joint_target_by_side': joint_targets,
                    'ik_error_code': MoveItErrorCodes.SUCCESS,
                }
                result.update(self._send_gripper_commands(applied_poses))
                return result, b''

            sequence = self._queue_joint_target(joint_target)
            result = {
                'ok': True,
                'action': applied,
                'accepted': True,
                'clamped': clamped,
                'executed': False,
                'queued': True,
                'mode': self.execution_mode,
                'joint_target': joint_target,
                'joint_target_by_side': joint_targets,
                'trajectory_sequence': sequence,
            }
            result.update(self._send_gripper_commands(applied_poses))
            return result, b''
        if command == 'stop':
            if self.execution_mode == 'trajectory':
                self._abort_motion('stop command received')
            else:
                self._request_stop()
            return {
                'ok': True,
                'stopped': True,
                'bimanual': True,
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
                    self.get_logger().error(
                        f'Bimanual VLA socket request failed: {exc}')
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
            target=self._accept_loop,
            name='nero-bimanual-vla-socket',
            daemon=True,
        )
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
                target=self._client_loop,
                args=(conn,),
                name='nero-bimanual-vla-client',
                daemon=True,
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
            try:
                self._trajectory_worker.join(timeout=1.0)
            except KeyboardInterrupt:
                pass
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
            try:
                self._server_thread.join(timeout=1.0)
            except KeyboardInterrupt:
                pass
            self._server_thread = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NeroBimanualVlaBridge()
    executor = MultiThreadedExecutor(num_threads=4)
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
