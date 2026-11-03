#!/usr/bin/env python3
"""FastAPI dashboard for the table-mounted dual NERO workspace.

The node exposes:

* ``/``                          browser dashboard
* ``/ws``                        joint feedback and one-shot joint commands
* ``/api/config``                joint limits and camera metadata
* ``/api/move``                  MoveIt planning and execution for one or both arms
* ``/api/park``                  MoveIt return to the configured support pose
* ``/api/clear_faults``          clear latched NERO joint-driver faults
* ``/api/recover``               reset motion state and re-enable one arm
* ``/robot_urdf``                xacro-expanded dual-arm URDF for the 3D view
* ``/stream/<camera>/<mode>``    MJPEG streams sourced from ROS Image topics

Motion requests are sent to MoveIt's ``/move_action`` server. MoveIt plans
against the robot and table collision model and executes through the
``dual_arm_controller`` trajectory bridge. The app requires its local
control-gate state to be open before it will submit a goal; the driver's own
control gate remains an independent second interlock.
"""
from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, Union

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, Response, StreamingResponse
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_srvs.srv import Empty, SetBool, Trigger
import uvicorn
import xacro
import yaml

from .image_stream import CompressedImageStream, ImageStream


SIDES = ('left', 'right')
MOVE_TARGETS = ('left', 'right', 'both')
CameraStream = Union[ImageStream, CompressedImageStream]
LOCAL_JOINT_NAMES = [f'joint{index}' for index in range(1, 8)]

# Conservative intersection of the NERO driver limits and the dual-arm URDF.
# joint2/joint4 match the two validated dual_nero_description relaxations so
# the gravity-stable support pose can be commanded without clamping.
JOINT_LIMITS = [
    (-2.705260, 2.705260),   # joint1
    (-1.760000, 1.740000),   # joint2
    (-2.750000, 2.750000),   # joint3
    (-1.010000, 2.200000),   # joint4
    (-2.750000, 2.750000),   # joint5
    (-0.7330383, 0.8726646),  # joint6
    (-1.5707963, 1.5707963),  # joint7
]

CAMERA_DEFS = (
    ('scene', 'Scene', 'scene_color_topic',
     'scene_color_compressed_topic', 'scene_depth_topic'),
    ('left_wrist', 'Left wrist', 'left_wrist_color_topic',
     'left_wrist_color_compressed_topic', 'left_wrist_depth_topic'),
    ('right_wrist', 'Right wrist', 'right_wrist_color_topic',
     'right_wrist_color_compressed_topic', 'right_wrist_depth_topic'),
)


MOVEIT_ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: 'SUCCESS',
    MoveItErrorCodes.FAILURE: 'FAILURE',
    MoveItErrorCodes.PLANNING_FAILED: 'PLANNING_FAILED',
    MoveItErrorCodes.INVALID_MOTION_PLAN: 'INVALID_MOTION_PLAN',
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
    MoveItErrorCodes.CONTROL_FAILED: 'CONTROL_FAILED',
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: 'UNABLE_TO_ACQUIRE_SENSOR_DATA',
    MoveItErrorCodes.TIMED_OUT: 'TIMED_OUT',
    MoveItErrorCodes.PREEMPTED: 'PREEMPTED',
    MoveItErrorCodes.START_STATE_IN_COLLISION: 'START_STATE_IN_COLLISION',
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        'START_STATE_VIOLATES_PATH_CONSTRAINTS',
    MoveItErrorCodes.START_STATE_INVALID: 'START_STATE_INVALID',
    MoveItErrorCodes.GOAL_IN_COLLISION: 'GOAL_IN_COLLISION',
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        'GOAL_VIOLATES_PATH_CONSTRAINTS',
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'GOAL_CONSTRAINTS_VIOLATED',
    MoveItErrorCodes.GOAL_STATE_INVALID: 'GOAL_STATE_INVALID',
    MoveItErrorCodes.INVALID_GROUP_NAME: 'INVALID_GROUP_NAME',
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'INVALID_GOAL_CONSTRAINTS',
    MoveItErrorCodes.INVALID_ROBOT_STATE: 'INVALID_ROBOT_STATE',
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        'COLLISION_CHECKING_UNAVAILABLE',
    MoveItErrorCodes.ROBOT_STATE_STALE: 'ROBOT_STATE_STALE',
    MoveItErrorCodes.SENSOR_INFO_STALE: 'SENSOR_INFO_STALE',
    MoveItErrorCodes.COMMUNICATION_FAILURE: 'COMMUNICATION_FAILURE',
    MoveItErrorCodes.CRASH: 'CRASH',
    MoveItErrorCodes.ABORT: 'ABORT',
}


ARM_STATUS_LABELS = {
    0: 'normal',
    1: 'emergency stop',
    2: 'no IK solution',
    3: 'singularity',
    4: 'target exceeds joint limit',
    5: 'joint communication error',
    6: 'brake not released',
    7: 'collision',
    8: 'teaching drag overspeed',
    9: 'joint status error',
    10: 'other error',
    11: 'teaching record',
    12: 'teaching execution',
    13: 'teaching pause',
    14: 'main controller over-temperature',
    15: 'release resistor over-temperature',
}

MOTOR_FAULT_FIELDS = (
    'undervoltage',
    'motor_overtemp',
    'driver_overcurrent',
    'driver_overtemp',
    'collision_tripped',
    'driver_error',
    'stall_tripped',
)


def clamp_positions(positions: List[float]) -> List[float]:
    if len(positions) != len(JOINT_LIMITS):
        raise ValueError(
            f'expected {len(JOINT_LIMITS)} joint positions, got {len(positions)}'
        )
    return [
        max(low, min(high, float(value)))
        for value, (low, high) in zip(positions, JOINT_LIMITS)
    ]


def _asset_path(root: Path, relative: str) -> Path:
    """Resolve a URL path without allowing ``..`` traversal outside root."""
    requested = Path(relative)
    if requested.is_absolute() or '..' in requested.parts:
        raise HTTPException(status_code=404, detail='asset not found')
    candidate = root / requested
    if not candidate.is_file():
        raise HTTPException(status_code=404, detail='asset not found')
    return candidate


class WebappNode(Node):
    def __init__(self) -> None:
        super().__init__('dual_nero_webapp')

        self.declare_parameter('left_ns', 'left')
        self.declare_parameter('right_ns', 'right')
        self.declare_parameter('http_host', '0.0.0.0')
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('initial_gate_open', False)
        self.declare_parameter('enable_cameras', True)
        self.declare_parameter('depth_min_m', 0.05)
        self.declare_parameter('depth_max_m', 2.0)
        self.declare_parameter('use_compressed_color', True)

        self.declare_parameter(
            'scene_color_compressed_topic',
            '/camera_scene/color/image_raw/compressed')
        self.declare_parameter('scene_color_topic', '/camera_scene/color/image_raw')
        self.declare_parameter(
            'scene_depth_topic', '/camera_scene/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'left_wrist_color_compressed_topic',
            '/camera_left_wrist/color/image_raw/compressed')
        self.declare_parameter(
            'left_wrist_color_topic', '/camera_left_wrist/color/image_raw')
        self.declare_parameter(
            'left_wrist_depth_topic',
            '/camera_left_wrist/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'right_wrist_color_compressed_topic',
            '/camera_right_wrist/color/image_raw/compressed')
        self.declare_parameter(
            'right_wrist_color_topic', '/camera_right_wrist/color/image_raw')
        self.declare_parameter(
            'right_wrist_depth_topic',
            '/camera_right_wrist/aligned_depth_to_color/image_raw')

        self.declare_parameter('urdf_xacro', '')
        self.declare_parameter('support_pose_file', '')
        self.declare_parameter('moveit_action_name', '/move_action')
        self.declare_parameter('moveit_planning_time_sec', 10.0)
        self.declare_parameter('moveit_planning_attempts', 5)
        self.declare_parameter('moveit_velocity_scaling', 0.05)
        self.declare_parameter('moveit_acceleration_scaling', 0.05)
        self.declare_parameter('moveit_goal_tolerance_rad', 0.01)
        self.declare_parameter('moveit_result_timeout_sec', 120.0)
        self.declare_parameter('moveit_dry_run', True)
        self.declare_parameter(
            'trajectory_cancel_service',
            '/dual_nero_trajectory_bridge/cancel_motion')
        self.declare_parameter(
            'trajectory_clear_cancel_service',
            '/dual_nero_trajectory_bridge/clear_cancel')
        self.declare_parameter('table_width', 1.80)
        self.declare_parameter('table_depth', 0.90)
        self.declare_parameter('table_thickness', 0.03)
        self.declare_parameter('base_spacing', 0.70)
        self.declare_parameter('front_edge_offset', 0.06)
        self.declare_parameter('left_base_yaw', 3.141592653589793)
        self.declare_parameter('right_base_yaw', 3.141592653589793)
        self.declare_parameter('include_gripper', False)
        for side in SIDES:
            for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                self.declare_parameter(f'{side}_tcp_{axis}', 0.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.left_ns = self.get_parameter('left_ns').value
        self.right_ns = self.get_parameter('right_ns').value
        namespaces = {'left': self.left_ns, 'right': self.right_ns}

        self.joint_names = list(LOCAL_JOINT_NAMES)
        for side in SIDES:
            self.create_subscription(
                JointState,
                f'/{namespaces[side]}/feedback/joint_states',
                lambda msg, s=side: self._on_feedback(s, msg),
                qos,
            )
            self.create_subscription(
                DiagnosticArray,
                f'/{namespaces[side]}/feedback/arm_status',
                lambda msg, s=side: self._on_arm_status(s, msg),
                qos,
            )
            self.create_subscription(
                DiagnosticArray,
                f'/{namespaces[side]}/feedback/motor_status',
                lambda msg, s=side: self._on_motor_status(s, msg),
                qos,
            )

        initial_gate_open = bool(self.get_parameter('initial_gate_open').value)
        self._gate_open = {side: initial_gate_open for side in SIDES}
        self._motor_enabled: Dict[str, Optional[bool]] = {side: None for side in SIDES}
        self._latest: Dict[str, Dict] = {
            side: {
                'names': list(self.joint_names),
                'positions': [0.0] * len(self.joint_names),
            }
            for side in SIDES
        }
        self._feedback_time = {side: 0.0 for side in SIDES}
        self._latest_lock = threading.Lock()

        self._health_lock = threading.Lock()
        self._health: Dict[str, Dict] = {
            side: {
                'arm_status': None,
                'arm_status_text': None,
                'err_status': None,
                'joint_limit_flags': [],
                'communication_flags': [],
                'enabled_joints': None,
                'joint_count': len(self.joint_names),
                'motor_faults': [],
                'arm_status_time': 0.0,
                'motor_status_time': 0.0,
            }
            for side in SIDES
        }

        self._motion_lock = threading.Lock()
        self._active_motion_lock = threading.Lock()
        self._active_goal_handle = None
        self._active_motion_side: Optional[str] = None
        self._move_client = ActionClient(
            self,
            MoveGroup,
            str(self.get_parameter('moveit_action_name').value),
        )
        self._trajectory_cancel_client = self.create_client(
            Empty,
            str(self.get_parameter('trajectory_cancel_service').value),
        )
        self._trajectory_clear_cancel_client = self.create_client(
            Empty,
            str(self.get_parameter('trajectory_clear_cancel_service').value),
        )

        self._service_clients: Dict[Tuple[str, str], object] = {}
        self._service_lock = threading.Lock()
        self._urdf_xml: Optional[str] = None
        self._urdf_error: Optional[str] = None

        self.cameras: Dict[str, Dict] = {}
        if bool(self.get_parameter('enable_cameras').value):
            self._setup_cameras()

        self._support_pose, self._support_pose_errors = self._load_support_pose()

        self.get_logger().info(
            'dual_nero_webapp ready: MoveIt action '
            + str(self.get_parameter('moveit_action_name').value)
            + f'; HTTP port {self.get_parameter("http_port").value}'
        )

    def _load_support_pose(self) -> Tuple[Dict[str, List[float]], Dict[str, str]]:
        configured_path = str(self.get_parameter('support_pose_file').value).strip()
        if not configured_path:
            message = 'support_pose_file parameter is empty'
            self.get_logger().warn(message)
            return {}, {side: message for side in SIDES}

        path = Path(configured_path).expanduser()
        try:
            document = yaml.safe_load(path.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError) as exc:
            message = f'failed to load support pose {path}: {exc}'
            self.get_logger().warn(message)
            return {}, {side: message for side in SIDES}

        data = document.get('support_pose') if isinstance(document, dict) else None
        if not isinstance(data, dict):
            message = f'{path} must contain a support_pose mapping'
            self.get_logger().warn(message)
            return {}, {side: message for side in SIDES}

        poses: Dict[str, List[float]] = {}
        errors: Dict[str, str] = {}
        for side in SIDES:
            values = data.get(side)
            if isinstance(values, (list, tuple)):
                raw_positions = list(values)
            else:
                raw_positions = [
                    data.get(f'{side}_joint{index}') for index in range(1, 8)]

            if len(raw_positions) != len(JOINT_LIMITS) or any(
                    value is None for value in raw_positions):
                errors[side] = (
                    f'{path}: missing 7 joint values for {side}')
                continue
            try:
                positions = [float(value) for value in raw_positions]
            except (TypeError, ValueError):
                errors[side] = f'{path}: {side} pose contains non-numeric values'
                continue
            if not all(math.isfinite(value) for value in positions):
                errors[side] = f'{path}: {side} pose contains non-finite values'
                continue
            outside = [
                f'joint{index + 1}={value:.6f}'
                for index, (value, (low, high))
                in enumerate(zip(positions, JOINT_LIMITS))
                if not low <= value <= high
            ]
            if outside:
                errors[side] = (
                    f'{path}: {side} pose exceeds joint limits: '
                    + ', '.join(outside))
                continue
            poses[side] = positions

        for message in errors.values():
            self.get_logger().warn(message)
        for side in poses:
            self.get_logger().info(f'{side} support pose loaded from {path}')
        return poses, errors

    def park_status(self) -> Dict[str, Dict]:
        available = {
            side: side in self._support_pose for side in SIDES
        }
        errors = dict(self._support_pose_errors)
        available['both'] = all(available[side] for side in SIDES)
        if not available['both']:
            missing = [
                errors.get(side, f'{side} support pose is not configured')
                for side in SIDES if not available[side]
            ]
            errors['both'] = '; '.join(missing)
        return {'available': available, 'errors': errors}

    def _setup_cameras(self) -> None:
        depth_min = float(self.get_parameter('depth_min_m').value)
        depth_max = float(self.get_parameter('depth_max_m').value)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        use_compressed = bool(self.get_parameter('use_compressed_color').value)
        for name, label, color_param, compressed_param, depth_param in CAMERA_DEFS:
            streams: Dict[str, CameraStream] = {}
            for mode, param in (('color', color_param), ('depth', depth_param)):
                compressed = mode == 'color' and use_compressed
                if compressed:
                    param = compressed_param
                topic = str(self.get_parameter(param).value).strip()
                if not topic:
                    continue
                if compressed:
                    stream = CompressedImageStream(
                        name=f'{name}_{mode}',
                        label=f'{label} {mode}',
                        mode=mode,
                    )
                    msg_type = CompressedImage
                    topic_label = f'{label} {mode} (JPEG)'
                else:
                    stream = ImageStream(
                        name=f'{name}_{mode}',
                        label=f'{label} {mode}',
                        mode=mode,
                        depth_min_m=depth_min,
                        depth_max_m=depth_max,
                    )
                    msg_type = Image
                    topic_label = f'{label} {mode}'
                streams[mode] = stream
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, stream=stream: stream.update(msg),
                    image_qos,
                )
                self.get_logger().info(f'{topic_label}: {topic}')
            if streams:
                self.cameras[name] = {'label': label, 'streams': streams}

    def _on_feedback(self, side: str, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        prefix = f'{side}_'
        positions: List[float] = []
        for index, joint_name in enumerate(self.joint_names):
            value = by_name.get(joint_name)
            if value is None:
                value = by_name.get(prefix + joint_name)
            if value is None and index < len(msg.position):
                value = msg.position[index]
            positions.append(float(value) if value is not None else 0.0)
        with self._latest_lock:
            self._latest[side] = {
                'names': list(self.joint_names),
                'positions': positions,
            }
            self._feedback_time[side] = time.monotonic()

    @staticmethod
    def _diagnostic_values(msg: DiagnosticArray) -> Dict[str, str]:
        if not msg.status:
            return {}
        return {item.key: item.value for item in msg.status[0].values}

    @staticmethod
    def _int_value(values: Dict[str, str], key: str) -> Optional[int]:
        try:
            return int(float(values[key]))
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _bool_list_value(values: Dict[str, str], key: str) -> List[bool]:
        try:
            raw = json.loads(values[key])
        except (KeyError, TypeError, ValueError):
            return []
        return [bool(item) for item in raw] if isinstance(raw, list) else []

    def _on_arm_status(self, side: str, msg: DiagnosticArray) -> None:
        values = self._diagnostic_values(msg)
        arm_status = self._int_value(values, 'arm_status')
        err_status = self._int_value(values, 'err_status')
        with self._health_lock:
            health = self._health[side]
            health['arm_status'] = arm_status
            health['arm_status_text'] = (
                ARM_STATUS_LABELS.get(arm_status, f'unknown ({arm_status})')
                if arm_status is not None else None
            )
            health['err_status'] = err_status
            health['joint_limit_flags'] = self._bool_list_value(
                values, 'joint_angle_limit')
            health['communication_flags'] = self._bool_list_value(
                values, 'communication_status_joint')
            health['arm_status_time'] = time.monotonic()

    def _on_motor_status(self, side: str, msg: DiagnosticArray) -> None:
        enabled_joints = 0
        faults: List[str] = []
        for status in msg.status:
            values = {item.key: item.value for item in status.values}
            if values.get('enabled') == 'true':
                enabled_joints += 1
            for field in MOTOR_FAULT_FIELDS:
                if values.get(field) == 'true':
                    faults.append(f'{status.name}:{field}')
        with self._health_lock:
            health = self._health[side]
            health['enabled_joints'] = enabled_joints
            health['joint_count'] = len(msg.status)
            health['motor_faults'] = faults
            health['motor_status_time'] = time.monotonic()

    def snapshot(self) -> Dict[str, Dict]:
        now = time.monotonic()
        with self._latest_lock:
            result = {side: dict(self._latest[side]) for side in SIDES}
            ages = {
                side: (
                    round(now - self._feedback_time[side], 2)
                    if self._feedback_time[side] > 0.0 else None
                )
                for side in SIDES
            }
        with self._health_lock:
            health = {
                side: {
                    **{
                        key: value
                        for key, value in self._health[side].items()
                        if not key.endswith('_time')
                    },
                    'arm_status_age_sec': (
                        round(now - self._health[side]['arm_status_time'], 2)
                        if self._health[side]['arm_status_time'] > 0.0 else None
                    ),
                    'motor_status_age_sec': (
                        round(now - self._health[side]['motor_status_time'], 2)
                        if self._health[side]['motor_status_time'] > 0.0 else None
                    ),
                }
                for side in SIDES
            }
        for side in SIDES:
            result[side]['feedback_age_sec'] = ages[side]
            result[side]['gate_open'] = self._gate_open[side]
            result[side]['motor_enabled'] = self._motor_enabled[side]
            result[side]['health'] = health[side]
        return result

    def motion_status(self) -> Dict:
        with self._active_motion_lock:
            active_side = self._active_motion_side
        dry_run = bool(self.get_parameter('moveit_dry_run').value)
        return {
            'backend': 'moveit',
            'mode': 'dry_run' if dry_run else 'hardware',
            'dry_run': dry_run,
            'action_name': str(self.get_parameter('moveit_action_name').value),
            'available': bool(self._move_client.server_is_ready()),
            'busy': self._motion_lock.locked(),
            'active_side': active_side,
        }

    @staticmethod
    def _target_sides(target: str) -> Tuple[str, ...]:
        return SIDES if target == 'both' else (target,)

    def _motion_precheck(self, target: str) -> Optional[str]:
        if target not in MOVE_TARGETS:
            return f'unknown move target: {target}'
        now = time.monotonic()
        for side in self._target_sides(target):
            if not self._gate_open[side]:
                return f'{side} control gate is closed'
            if self._motor_enabled[side] is not True:
                return f'{side} motors are not marked enabled'
            with self._latest_lock:
                last_feedback = self._feedback_time[side]
            if last_feedback <= 0.0 or now - last_feedback > 1.5:
                return f'{side} feedback is missing or stale'
        return None

    def _cancel_active_motion(self, side: Optional[str] = None) -> None:
        with self._active_motion_lock:
            handle = self._active_goal_handle
            active_side = self._active_motion_side
        if handle is None:
            return
        matches_active = (
            side is None
            or side == active_side
            or (active_side == 'both' and side in SIDES)
        )
        if not matches_active:
            return
        try:
            cancel_service = str(
                self.get_parameter('trajectory_cancel_service').value)
            if self._trajectory_cancel_client.wait_for_service(timeout_sec=1.0):
                cancel_future = self._trajectory_cancel_client.call_async(
                    Empty.Request())
                if self._wait_future(cancel_future, 1.0):
                    self.get_logger().warn(
                        'cancel requested for active trajectory bridge goal')
                else:
                    self.get_logger().error(
                        f'{cancel_service} did not acknowledge cancellation')
            else:
                self.get_logger().warn(
                    f'{cancel_service} unavailable; cancelling MoveIt goal only')
            future = handle.cancel_goal_async()
            if not self._wait_future(future, 1.0):
                self.get_logger().warn(
                    f'MoveIt cancel handshake did not finish for {active_side}')
            else:
                response = future.result()
                count = len(response.goals_canceling) if response is not None else 0
                self.get_logger().warn(
                    f'MoveIt cancel requested for active {active_side} goal; '
                    f'goals_canceling={count}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'failed to cancel MoveIt goal: {exc}')

    def cancel_motion(self, side: Optional[str] = None) -> Dict:
        if side is not None and side not in MOVE_TARGETS:
            return {'ok': False, 'error': f'unknown move target: {side}'}
        with self._active_motion_lock:
            active_side = self._active_motion_side
            handle = self._active_goal_handle
        if handle is None or active_side is None:
            return {'ok': False, 'error': 'no active MoveIt motion'}
        if side is not None and side != active_side:
            return {
                'ok': False,
                'error': f'active motion is on {active_side}, not {side}',
            }
        self._cancel_active_motion(active_side)
        return {
            'ok': True,
            'side': active_side,
            'message': f'{active_side} cancellation requested',
        }

    def _clear_trajectory_cancel(self) -> Optional[str]:
        service = str(
            self.get_parameter('trajectory_clear_cancel_service').value)
        if not self._trajectory_clear_cancel_client.wait_for_service(
                timeout_sec=1.0):
            return f'{service} unavailable; refusing motion'
        future = self._trajectory_clear_cancel_client.call_async(Empty.Request())
        if not self._wait_future(future, 1.0):
            return f'{service} did not acknowledge in time; refusing motion'
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            return f'{service} failed: {exc}; refusing motion'
        return None

    def _make_move_goal(
        self,
        target: str,
        positions_by_side: Dict[str, List[float]],
    ):
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = (
            'both_arms' if target == 'both' else f'{target}_arm')
        goal.request.num_planning_attempts = int(
            self.get_parameter('moveit_planning_attempts').value)
        goal.request.allowed_planning_time = float(
            self.get_parameter('moveit_planning_time_sec').value)
        goal.request.max_velocity_scaling_factor = float(
            self.get_parameter('moveit_velocity_scaling').value)
        goal.request.max_acceleration_scaling_factor = float(
            self.get_parameter('moveit_acceleration_scaling').value)
        goal.request.start_state.is_diff = True

        tolerance = float(
            self.get_parameter('moveit_goal_tolerance_rad').value)
        constraints = Constraints()
        for side in self._target_sides(target):
            for index, position in enumerate(positions_by_side[side], start=1):
                constraint = JointConstraint()
                constraint.joint_name = f'{side}_joint{index}'
                constraint.position = float(position)
                constraint.tolerance_above = tolerance
                constraint.tolerance_below = tolerance
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False
        return goal

    def _move_target(
        self,
        target: str,
        positions_by_side: Dict[str, List[float]],
    ) -> Dict:
        error = self._motion_precheck(target)
        if error is not None:
            return {'ok': False, 'error': error}
        label = 'both arms' if target == 'both' else target
        try:
            clamped = {
                side: clamp_positions(positions_by_side[side])
                for side in self._target_sides(target)
            }
        except (TypeError, ValueError) as exc:
            return {'ok': False, 'error': str(exc)}
        except KeyError as exc:
            return {'ok': False, 'error': f'missing positions for {exc.args[0]}'}

        if not self._motion_lock.acquire(blocking=False):
            return {
                'ok': False,
                'error': 'another MoveIt motion is already in progress',
            }
        try:
            dry_run = bool(self.get_parameter('moveit_dry_run').value)
            self.get_logger().info(
                f'{label} MoveIt motion requested; dry_run={dry_run}')
            action_name = str(self.get_parameter('moveit_action_name').value)
            if not self._move_client.wait_for_server(timeout_sec=1.0):
                return {
                    'ok': False,
                    'error': (
                        f'MoveIt action {action_name} is unavailable; start '
                        'moveit_execution or webapp.launch.py with '
                        'start_moveit:=true'),
                }

            clear_error = self._clear_trajectory_cancel()
            if clear_error is not None:
                return {'ok': False, 'error': clear_error}

            goal = self._make_move_goal(target, clamped)
            send_future = self._move_client.send_goal_async(goal)
            if not self._wait_future(send_future, 10.0):
                return {
                    'ok': False,
                    'error': 'MoveIt did not acknowledge the goal in time',
                }
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                return {'ok': False, 'error': 'MoveIt rejected the goal'}

            with self._active_motion_lock:
                self._active_goal_handle = goal_handle
                self._active_motion_side = target
            try:
                result_future = goal_handle.get_result_async()
                timeout = float(
                    self.get_parameter('moveit_result_timeout_sec').value)
                if not self._wait_future(result_future, timeout):
                    self._cancel_active_motion(target)
                    return {
                        'ok': False,
                        'error': 'MoveIt motion timed out; goal cancelled',
                    }
                response = result_future.result()
            finally:
                with self._active_motion_lock:
                    self._active_goal_handle = None
                    self._active_motion_side = None

            if response is None:
                return {'ok': False, 'error': 'MoveIt returned no result'}
            code = int(response.result.error_code.val)
            if code != MoveItErrorCodes.SUCCESS:
                name = MOVEIT_ERROR_NAMES.get(code, f'ERROR_{code}')
                return {
                    'ok': False,
                    'error': f'MoveIt motion failed: {name}',
                    'moveit_error_code': code,
                }
            suffix = ' (dry-run; no driver command sent)' if dry_run else ''
            self.get_logger().info(
                f'{label} MoveIt motion completed; dry_run={dry_run}')
            result_positions = (
                {side: list(values) for side, values in clamped.items()}
                if target == 'both' else list(clamped[target])
            )
            return {
                'ok': True,
                'side': target,
                'dry_run': dry_run,
                'positions': result_positions,
                'message': f'{label} MoveIt motion completed{suffix}',
            }
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'MoveIt request failed: {exc}')
            return {'ok': False, 'error': f'MoveIt request failed: {exc}'}
        finally:
            self._motion_lock.release()

    def move_to(self, side: str, positions: List[float]) -> Dict:
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        return self._move_target(side, {side: positions})

    def move_both(self, positions_by_side: Dict[str, List[float]]) -> Dict:
        if not isinstance(positions_by_side, dict):
            return {'ok': False, 'error': 'positions must map left and right to lists'}
        missing = [side for side in SIDES if side not in positions_by_side]
        if missing:
            return {
                'ok': False,
                'error': 'missing positions for ' + ', '.join(missing),
            }
        return self._move_target('both', positions_by_side)

    def park(self, side: str) -> Dict:
        if side not in MOVE_TARGETS:
            return {'ok': False, 'error': f'unknown move target: {side}'}

        positions_by_side: Dict[str, List[float]] = {}
        for target_side in self._target_sides(side):
            positions = self._support_pose.get(target_side)
            if positions is None:
                error = self._support_pose_errors.get(
                    target_side, f'{target_side} support pose is not configured')
                return {'ok': False, 'error': error}
            positions_by_side[target_side] = positions

        result = self._move_target(side, positions_by_side)
        if result.get('ok'):
            suffix = ' (dry-run; no driver command sent)' if result.get('dry_run') else ''
            label = 'both arms' if side == 'both' else side
            result['message'] = f'{label} MoveIt park motion completed{suffix}'
        return result

    def _get_service_client(self, side: str, kind: str):
        key = (side, kind)
        with self._service_lock:
            client = self._service_clients.get(key)
            if client is not None:
                return client
            namespace = self._namespace_for(side)
            if kind == 'gate':
                client = self.create_client(SetBool, f'/{namespace}/control_enable')
            elif kind == 'enable':
                client = self.create_client(SetBool, f'/{namespace}/enable_agx_arm')
            elif kind == 'estop':
                client = self.create_client(Empty, f'/{namespace}/emergency_stop')
            elif kind == 'clear_faults':
                client = self.create_client(Trigger, f'/{namespace}/clear_joint_error')
            elif kind == 'reset_motion':
                client = self.create_client(Trigger, f'/{namespace}/reset_motion')
            else:
                raise ValueError(f'unknown service kind: {kind}')
            self._service_clients[key] = client
            return client

    def _namespace_for(self, side: str) -> str:
        return self.left_ns if side == 'left' else self.right_ns

    @staticmethod
    def _wait_future(future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def set_gate(self, side: str, enabled: bool, timeout: float = 2.0) -> Dict:
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        if not enabled:
            # Fail closed locally before waiting for either the driver service
            # or the MoveIt cancel handshake.
            self._gate_open[side] = False
            self._cancel_active_motion(side)
        client = self._get_service_client(side, 'gate')
        if not client.wait_for_service(timeout_sec=1.0):
            namespace = self._namespace_for(side)
            return {'ok': False, 'error': f'/{namespace}/control_enable unavailable'}
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        if not self._wait_future(future, timeout):
            return {'ok': False, 'error': 'control gate request timed out'}
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'error': f'control gate request failed: {exc}'}
        if response is None or not response.success:
            return {
                'ok': False,
                'error': getattr(response, 'message', 'control gate request rejected'),
            }
        if enabled:
            self._gate_open[side] = True
        self.get_logger().info(f'{side} control gate {"opened" if enabled else "closed"}')
        return {'ok': True, 'side': side, 'enabled': bool(enabled)}

    def set_enabled(self, side: str, enabled: bool, timeout: float = 8.0) -> Dict:
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        if not enabled:
            self._motor_enabled[side] = False
            self._cancel_active_motion(side)
        client = self._get_service_client(side, 'enable')
        if not client.wait_for_service(timeout_sec=1.0):
            namespace = self._namespace_for(side)
            return {'ok': False, 'error': f'/{namespace}/enable_agx_arm unavailable'}
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        if not self._wait_future(future, timeout):
            return {'ok': False, 'error': 'motor enable request timed out'}
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'error': f'motor enable request failed: {exc}'}
        if response is None or not response.success:
            return {
                'ok': False,
                'error': getattr(response, 'message', 'motor enable request rejected'),
            }
        self._motor_enabled[side] = bool(enabled)
        self.get_logger().info(f'{side} motors {"enabled" if enabled else "disabled"}')
        return {'ok': True, 'side': side, 'enabled': bool(enabled)}

    def _call_trigger(self, side: str, kind: str, timeout: float = 3.0) -> Dict:
        client = self._get_service_client(side, kind)
        if not client.wait_for_service(timeout_sec=1.0):
            namespace = self._namespace_for(side)
            service_name = {
                'clear_faults': 'clear_joint_error',
                'reset_motion': 'reset_motion',
            }[kind]
            return {
                'ok': False,
                'side': side,
                'error': f'/{namespace}/{service_name} unavailable',
            }
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout):
            return {'ok': False, 'side': side, 'error': f'{kind} request timed out'}
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'side': side, 'error': f'{kind} request failed: {exc}'}
        if response is None:
            return {'ok': False, 'side': side, 'error': f'{kind} returned no response'}
        result = {
            'ok': bool(response.success),
            'side': side,
            'message': response.message,
        }
        if not response.success:
            result['error'] = response.message or f'{kind} rejected'
        return result

    def clear_faults(self, side: str) -> Dict:
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        if self._motion_lock.locked():
            return {'ok': False, 'error': 'refusing fault clear during motion'}
        result = self._call_trigger(side, 'clear_faults')
        if result.get('ok'):
            self.get_logger().info(
                f'{side} joint fault-clear command sent: {result.get("message")}')
        return result

    def reset_motion(self, side: str) -> Dict:
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        if self._gate_open[side]:
            return {
                'ok': False,
                'error': 'close the control gate before resetting the motion controller',
            }
        if self._motion_lock.locked():
            return {'ok': False, 'error': 'refusing controller reset during motion'}
        result = self._call_trigger(side, 'reset_motion')
        if result.get('ok'):
            self.get_logger().info(
                f'{side} motion-controller reset sent: {result.get("message")}')
        return result

    def recover(self, side: str) -> Dict:
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        if self._gate_open[side]:
            return {
                'ok': False,
                'error': 'close the control gate before recovering the arm',
            }
        if self._motion_lock.locked():
            return {'ok': False, 'error': 'refusing recovery while a motion is active'}
        feedback_age = time.monotonic() - self._feedback_time[side]
        if self._feedback_time[side] <= 0.0 or feedback_age > 2.0:
            return {
                'ok': False,
                'error': 'feedback is missing or stale; refusing recovery',
            }

        reset_result = self.reset_motion(side)
        if not reset_result.get('ok'):
            return reset_result
        clear_result = self.clear_faults(side)
        if not clear_result.get('ok'):
            return clear_result
        enable_result = self.set_enabled(side, True)
        if not enable_result.get('ok'):
            return enable_result

        message = (
            f'{side} motion controller reset and joint faults cleared; '
            'motors re-enabled. The control gate remains closed.'
        )
        self.get_logger().info(message)
        return {
            'ok': True,
            'side': side,
            'enabled': True,
            'message': message,
            'reset_message': reset_result.get('message'),
            'clear_message': clear_result.get('message'),
        }

    def emergency_stop(self, side: str, timeout: float = 2.0) -> Dict:
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        client = self._get_service_client(side, 'estop')
        if not client.wait_for_service(timeout_sec=1.0):
            namespace = self._namespace_for(side)
            gate_result = self.set_gate(side, False)
            gate_closed = bool(gate_result.get('ok'))
            gate_error = None if gate_closed else gate_result.get('error')
            stop_error = f'/{namespace}/emergency_stop unavailable'
            return {
                'ok': False,
                'side': side,
                'stop_sent': False,
                'stop_confirmed': False,
                'gate_closed': gate_closed,
                'gate_error': gate_error,
                'error': stop_error,
            }
        future = client.call_async(Empty.Request())
        # Hardware stop is dispatched first. Closing the driver gate and
        # cancelling the MoveIt goal are best-effort follow-ups, so neither
        # handshake can delay the emergency-stop request.
        self._gate_open[side] = False
        gate_result = self.set_gate(side, False)
        gate_closed = bool(gate_result.get('ok'))
        gate_error = None if gate_closed else gate_result.get('error')
        stop_error = None
        if not self._wait_future(future, timeout):
            stop_error = 'emergency stop request timed out'
        else:
            try:
                future.result()
            except Exception as exc:  # noqa: BLE001
                stop_error = f'emergency stop request failed: {exc}'
        stop_confirmed = stop_error is None
        self.get_logger().warn(
            f'{side} emergency stop requested; '
            f'confirmed={stop_confirmed}, gate_closed={gate_closed}')
        errors = []
        if stop_error is not None:
            errors.append(stop_error)
        if not gate_closed:
            errors.append(f'control gate close failed: {gate_error}')
        return {
            'ok': stop_confirmed and gate_closed,
            'side': side,
            'stop_sent': True,
            'stop_confirmed': stop_confirmed,
            'gate_closed': gate_closed,
            'gate_error': gate_error,
            'error': '; '.join(errors) if errors else None,
        }

    def emergency_stop_all(self) -> Dict:
        results: Dict[str, Dict] = {}

        def stop_side(side: str) -> None:
            results[side] = self.emergency_stop(side)

        threads = [
            threading.Thread(target=stop_side, args=(side,), daemon=True)
            for side in SIDES
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        failed = [side for side in SIDES if not results[side].get('ok')]
        return {'ok': not failed, 'failed': failed, 'results': results}

    def camera_info(self) -> Dict[str, Dict]:
        return {
            name: {
                'label': data['label'],
                'streams': {
                    mode: stream.info()
                    for mode, stream in data['streams'].items()
                },
            }
            for name, data in self.cameras.items()
        }

    def get_camera_stream(self, camera: str, mode: str) -> Optional[CameraStream]:
        data = self.cameras.get(camera)
        if data is None:
            return None
        return data['streams'].get(mode)

    def load_urdf_xml(self) -> str:
        if self._urdf_xml is not None:
            return self._urdf_xml
        if self._urdf_error is not None:
            raise RuntimeError(self._urdf_error)
        xacro_path = str(self.get_parameter('urdf_xacro').value).strip()
        if not xacro_path:
            xacro_path = str(
                Path(get_package_share_directory('dual_nero_description'))
                / 'urdf' / 'two_nero_tabletop.urdf.xacro'
            )
        mappings = {
            'table_width': str(self.get_parameter('table_width').value),
            'table_depth': str(self.get_parameter('table_depth').value),
            'table_thickness': str(self.get_parameter('table_thickness').value),
            'base_spacing': str(self.get_parameter('base_spacing').value),
            'front_edge_offset': str(self.get_parameter('front_edge_offset').value),
            'left_base_yaw': str(self.get_parameter('left_base_yaw').value),
            'right_base_yaw': str(self.get_parameter('right_base_yaw').value),
            'include_gripper': str(bool(self.get_parameter('include_gripper').value)).lower(),
        }
        for side in SIDES:
            for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                mappings[f'{side}_tcp_{axis}'] = str(
                    self.get_parameter(f'{side}_tcp_{axis}').value)
        try:
            document = xacro.process_file(xacro_path, mappings=mappings)
            self._urdf_xml = document.toprettyxml(indent='  ')
            self.get_logger().info(f'Loaded browser URDF from {xacro_path}')
            return self._urdf_xml
        except Exception as exc:  # noqa: BLE001
            self._urdf_error = f'failed to expand {xacro_path}: {exc}'
            self.get_logger().error(self._urdf_error)
            raise RuntimeError(self._urdf_error) from exc


def build_app(node: WebappNode, static_dir: Path) -> FastAPI:
    app = FastAPI(title='Dual NERO webapp')

    agx_share = Path(get_package_share_directory('agx_arm_description'))

    @app.get('/static/{filename:path}')
    async def static_file(filename: str) -> FileResponse:
        return FileResponse(
            _asset_path(static_dir, filename),
            headers={'Cache-Control': 'no-store, max-age=0'},
        )

    @app.get('/pkg/agx_arm_description/{filename:path}')
    async def agx_asset(filename: str) -> FileResponse:
        return FileResponse(_asset_path(agx_share, filename))

    @app.get('/', response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        return HTMLResponse(
            (static_dir / 'index.html').read_text(),
            headers={'Cache-Control': 'no-store, max-age=0'},
        )

    @app.get('/robot_urdf')
    async def robot_urdf() -> Response:
        try:
            xml = node.load_urdf_xml()
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        return Response(xml, media_type='application/xml')

    @app.get('/api/config')
    async def config() -> Dict:
        return {
            'sides': list(SIDES),
            'move_targets': list(MOVE_TARGETS),
            'joint_names': list(node.joint_names),
            'limits': [list(limit) for limit in JOINT_LIMITS],
            'urdf_url': '/robot_urdf',
            'cameras': [
                {
                    'name': name,
                    'label': data['label'],
                    'modes': list(data['streams'].keys()),
                }
                for name, data in node.cameras.items()
            ],
            'initial_gate_open': bool(node.get_parameter('initial_gate_open').value),
            'park': node.park_status(),
            'motion': node.motion_status(),
        }

    @app.get('/api/state')
    async def state() -> Dict:
        return {
            'type': 'state',
            'data': node.snapshot(),
            'motion': node.motion_status(),
        }

    @app.get('/api/motion')
    async def motion_status() -> Dict:
        return node.motion_status()

    @app.get('/api/cameras')
    async def cameras() -> Dict:
        return {'server_time': time.time(), 'cameras': node.camera_info()}

    @app.get('/api/health')
    async def health() -> Dict:
        return {'ok': True, 'server_time': time.time()}

    @app.get('/stream/{camera}/{mode}')
    async def stream(camera: str, mode: str) -> StreamingResponse:
        image_stream = node.get_camera_stream(camera, mode)
        if image_stream is None:
            raise HTTPException(status_code=404, detail='camera stream not configured')

        async def frames():
            last_seq = -1
            loop = asyncio.get_running_loop()
            while True:
                if image_stream.sequence() != last_seq:
                    jpeg, seq = await loop.run_in_executor(
                        None, image_stream.latest_jpeg)
                    if jpeg is not None and seq != last_seq:
                        last_seq = seq
                        yield (
                            b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Cache-Control: no-cache\r\n'
                            b'Pragma: no-cache\r\n\r\n'
                            + jpeg + b'\r\n'
                        )
                await asyncio.sleep(0.001)

        return StreamingResponse(
            frames(),
            media_type='multipart/x-mixed-replace; boundary=frame',
            headers={'Cache-Control': 'no-store'},
        )

    async def run_blocking(function, *args):
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, function, *args)

    @app.post('/api/control_gate')
    async def control_gate(request: Request) -> Dict:
        body = await request.json()
        side = str(body.get('side', ''))
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        return await run_blocking(node.set_gate, side, bool(body.get('enabled', False)))

    @app.post('/api/enable')
    async def enable(request: Request) -> Dict:
        body = await request.json()
        side = str(body.get('side', ''))
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        return await run_blocking(node.set_enabled, side, bool(body.get('enabled', False)))

    @app.post('/api/clear_faults')
    async def clear_faults(request: Request) -> Dict:
        body = await request.json()
        side = str(body.get('side', ''))
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        return await run_blocking(node.clear_faults, side)

    @app.post('/api/recover')
    async def recover(request: Request) -> Dict:
        body = await request.json()
        side = str(body.get('side', ''))
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        return await run_blocking(node.recover, side)

    @app.post('/api/park')
    async def park(request: Request) -> Dict:
        body = await request.json()
        side = str(body.get('side', ''))
        if side not in MOVE_TARGETS:
            return {'ok': False, 'error': f'unknown move target: {side}'}
        return await run_blocking(node.park, side)

    @app.post('/api/move')
    async def move(request: Request) -> Dict:
        body = await request.json()
        side = str(body.get('side', ''))
        positions = body.get('positions')
        if side == 'both':
            if not isinstance(positions, dict):
                return {
                    'ok': False,
                    'error': 'both-arm positions must map left and right to lists',
                }
            return await run_blocking(node.move_both, positions)
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown move target: {side}'}
        if not isinstance(positions, list):
            return {'ok': False, 'error': 'positions must be a list'}
        return await run_blocking(node.move_to, side, positions)

    @app.post('/api/cancel_motion')
    async def cancel_motion(request: Request) -> Dict:
        body = await request.json()
        side = body.get('side')
        if side is not None:
            side = str(side)
            if side not in MOVE_TARGETS:
                return {'ok': False, 'error': f'unknown move target: {side}'}
        return await run_blocking(node.cancel_motion, side)

    @app.post('/api/emergency_stop')
    async def emergency_stop(request: Request) -> Dict:
        body = await request.json()
        side = str(body.get('side', ''))
        if side not in SIDES:
            return {'ok': False, 'error': f'unknown side: {side}'}
        return await run_blocking(node.emergency_stop, side)

    @app.post('/api/emergency_stop_all')
    async def emergency_stop_all() -> Dict:
        return await run_blocking(node.emergency_stop_all)

    clients: Set[WebSocket] = set()

    @app.websocket('/ws')
    async def websocket_endpoint(websocket: WebSocket) -> None:
        await websocket.accept()
        clients.add(websocket)
        await websocket.send_text(json.dumps({
            'type': 'state',
            'data': node.snapshot(),
            'motion': node.motion_status(),
        }))
        try:
            while True:
                raw = await websocket.receive_text()
                try:
                    message = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                side = message.get('side')
                positions = message.get('positions')
                if side == 'both' and isinstance(positions, dict):
                    result = await run_blocking(node.move_both, positions)
                elif side in SIDES and isinstance(positions, list):
                    result = await run_blocking(node.move_to, side, positions)
                else:
                    continue
                await websocket.send_text(json.dumps({
                    'type': 'motion_result',
                    'data': result,
                }))
        except WebSocketDisconnect:
            pass
        finally:
            clients.discard(websocket)

    async def broadcast_state() -> None:
        while True:
            await asyncio.sleep(0.1)
            if not clients:
                continue
            payload = json.dumps({
                'type': 'state',
                'data': node.snapshot(),
                'motion': node.motion_status(),
            })
            dead: List[WebSocket] = []
            for websocket in list(clients):
                try:
                    await websocket.send_text(payload)
                except Exception:  # noqa: BLE001
                    dead.append(websocket)
            for websocket in dead:
                clients.discard(websocket)

    @app.on_event('startup')
    async def start_broadcast() -> None:
        app.state.broadcast_task = asyncio.create_task(broadcast_state())

    @app.on_event('shutdown')
    async def stop_broadcast() -> None:
        task = getattr(app.state, 'broadcast_task', None)
        if task is not None:
            task.cancel()

    return app


def main() -> None:
    rclpy.init()
    node = WebappNode()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    static_dir = Path(get_package_share_directory('dual_nero_webapp')) / 'static'
    host = str(node.get_parameter('http_host').value)
    port = int(node.get_parameter('http_port').value)
    app = build_app(node, static_dir)
    try:
        uvicorn.run(app, host=host, port=port, log_level='info')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
