#!/usr/bin/python3
"""ROS 2 driver for an AGILINK OmniPicker 3 gripper over CAN FD."""
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter as NodeParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Empty, SetBool, Trigger

from .omnipicker_gripper import (
    DEFAULT_BITRATE,
    DEFAULT_DATA_BITRATE,
    FAULT_NONE,
    POSITION_MAX,
    POSITION_MIN,
    STATE_DROPPED,
    STATE_REACHED,
    STATE_STALLED,
    CanTransportError,
    OmniPickerStatus,
    control_frame,
    decode_position_query,
    decode_status,
    make_transport,
    position_from_can,
    position_query_frame,
)
from .zqwl_canfd import DEFAULT_SERIAL_BAUD, STABLE_PORT, ZqwlCanFd, ZqwlError


LEFT_STABLE_PORT = (
    '/dev/serial/by-id/'
    'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00')
PORTS = {'left': LEFT_STABLE_PORT, 'right': STABLE_PORT}
DEFAULT_POSITION_MAX = 1000
DEFAULT_OPENING_MAX_MM = 120.0


class OmniPickerNode(Node):
    """OP3 driver that retains the former gripper ROS interface."""

    def __init__(self, side: str = 'right') -> None:
        side = side.strip().lower()
        if side not in PORTS:
            raise ValueError(f'side must be left or right, got {side!r}')
        self._side = side
        prefix = f'{side}_gripper'
        super().__init__(f'{side}_gripper_node')

        self.declare_parameter('transport', 'zqwl')
        self.declare_parameter('port', PORTS[side])
        self.declare_parameter('serial_baudrate', DEFAULT_SERIAL_BAUD)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('device_id', 1)
        self.declare_parameter('channel', 0)
        self.declare_parameter('bitrate', DEFAULT_BITRATE)
        self.declare_parameter('data_bitrate', DEFAULT_DATA_BITRATE)
        self.declare_parameter('can_fd', True)
        self.declare_parameter('brs', True)
        self.declare_parameter('save_params', False)
        self.declare_parameter('allow_motion', False)
        self.declare_parameter('enable_motor_on_start', False)
        self.declare_parameter('auto_enable_on_command', True)
        self.declare_parameter('feedback_rate', 20.0)
        self.declare_parameter('feedback_timeout_sec', 1.0)
        self.declare_parameter('receive_timeout_sec', 0.025)
        self.declare_parameter('status_rate', 20.0)
        self.declare_parameter('goal_timeout_sec', 10.0)
        self.declare_parameter('latch_motion_timeout', True)
        self.declare_parameter('cancel_stops_motion', True)
        self.declare_parameter('disable_on_shutdown', True)
        self.declare_parameter('allow_nonzero_effort', False)
        self.declare_parameter('position_tolerance', 8)
        self.declare_parameter('treat_closed_stall_as_grasp', True)
        self.declare_parameter('motion_speed_percent', 50)
        self.declare_parameter('motion_force_percent', 100)
        self.declare_parameter('motion_torque_percent', 100)  # compatibility alias
        self.declare_parameter('motion_acceleration_percent', 100)
        self.declare_parameter('motion_deceleration_percent', 100)
        self.declare_parameter('motion_ramp_ms', 200)  # compatibility, ignored
        self.declare_parameter('max_current', 200)  # compatibility, ignored
        self.declare_parameter('cancel_disables_motor', True)  # compatibility
        self.declare_parameter('parallel_gripper_width_mm', DEFAULT_OPENING_MAX_MM)
        self.declare_parameter('joint_name', f'{side}_op3_joint1')
        self.declare_parameter('joint_open_m', 0.0)
        self.declare_parameter('joint_closed_m', 0.02)
        self.declare_parameter('position_min', POSITION_MIN)
        self.declare_parameter('position_max', DEFAULT_POSITION_MAX)
        self.declare_parameter('command_max', DEFAULT_POSITION_MAX)
        self.declare_parameter('opening_min_mm', 0.0)
        self.declare_parameter('opening_max_mm', DEFAULT_OPENING_MAX_MM)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('state_topic', f'/{prefix}/state')
        self.declare_parameter('position_topic', f'/{prefix}/position')
        self.declare_parameter('width_topic', f'/{prefix}/width')
        self.declare_parameter('status_topic', f'/{prefix}/status')
        self.declare_parameter('command_topic', f'/{prefix}/position_cmd')
        self.declare_parameter('speed_topic', f'/{prefix}/speed_cmd')
        self.declare_parameter('enable_service', f'/{prefix}/enable')
        self.declare_parameter('estop_service', f'/{prefix}/emergency_stop')
        self.declare_parameter('clear_fault_service', f'/{prefix}/clear_fault')
        self.declare_parameter(
            'clear_obstruction_service', f'/{prefix}/clear_obstruction')
        self.declare_parameter(
            'action_name', f'/{prefix}_controller/gripper_cmd')

        gp = self.get_parameter
        self._transport_kind = str(gp('transport').value).strip().lower()
        if self._transport_kind not in {'zqwl', 'socketcan'}:
            raise ValueError("transport must be 'zqwl' or 'socketcan'")
        self._port = str(gp('port').value)
        self._serial_baudrate = int(gp('serial_baudrate').value)
        self._can_interface = str(gp('can_interface').value)
        self._device_id = int(gp('device_id').value)
        self._channel = int(gp('channel').value)
        self._bitrate = int(gp('bitrate').value)
        self._data_bitrate = int(gp('data_bitrate').value)
        self._can_fd = bool(gp('can_fd').value)
        self._brs = bool(gp('brs').value)
        self._save_params = bool(gp('save_params').value)
        self._allow_motion = bool(gp('allow_motion').value)
        self._auto_enable_on_command = bool(
            gp('auto_enable_on_command').value)
        self._feedback_rate = float(gp('feedback_rate').value)
        self._feedback_timeout = float(gp('feedback_timeout_sec').value)
        self._receive_timeout = float(gp('receive_timeout_sec').value)
        self._status_rate = float(gp('status_rate').value)
        self._goal_timeout = float(gp('goal_timeout_sec').value)
        self._latch_motion_timeout = bool(
            gp('latch_motion_timeout').value)
        self._cancel_stops_motion = bool(gp('cancel_stops_motion').value)
        self._disable_on_shutdown = bool(gp('disable_on_shutdown').value)
        self._allow_nonzero_effort = bool(gp('allow_nonzero_effort').value)
        self._position_tolerance = int(gp('position_tolerance').value)
        self._treat_closed_stall_as_grasp = bool(
            gp('treat_closed_stall_as_grasp').value)
        self._motion_speed_percent = int(
            gp('motion_speed_percent').value)
        self._motion_force_percent = int(
            gp('motion_force_percent').value)
        self._motion_accel_percent = int(
            gp('motion_acceleration_percent').value)
        self._motion_decel_percent = int(
            gp('motion_deceleration_percent').value)
        self._parallel_width_mm = float(
            gp('parallel_gripper_width_mm').value)
        self._joint_name = str(gp('joint_name').value)
        self._joint_open_m = float(gp('joint_open_m').value)
        self._joint_closed_m = float(gp('joint_closed_m').value)
        self._position_min = int(gp('position_min').value)
        self._position_max = int(gp('position_max').value)
        self._command_max = min(
            self._position_max, int(gp('command_max').value))
        self._opening_min = float(gp('opening_min_mm').value)
        self._opening_max = float(gp('opening_max_mm').value)
        self._action_name = str(gp('action_name').value)
        self._estop_service = str(gp('estop_service').value)
        self._clear_fault_service = str(gp('clear_fault_service').value)
        self._clear_obstruction_service = str(
            gp('clear_obstruction_service').value)
        self._validate_runtime_configuration()
        if not self._can_fd and self._brs:
            self.get_logger().warn(
                'brs=true requires CAN-FD; disabling BRS for classic CAN')
            self._brs = False

        self._callback_group = ReentrantCallbackGroup()
        self._io_lock = threading.Lock()
        self._motion_lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._transport = None
        self._io_thread: Optional[threading.Thread] = None
        self._adapter_error: Optional[str] = None
        self._target_position: Optional[int] = None
        self._position: Optional[int] = None
        self._position_byte: Optional[int] = None
        self._joint_value: Optional[float] = None
        self._width_mm: Optional[float] = None
        self._status: Optional[OmniPickerStatus] = None
        self._feedback_monotonic = 0.0
        self._enabled = bool(self._allow_motion and
                             bool(gp('enable_motor_on_start').value))
        self._state = 'ready' if self._allow_motion else 'motion disabled'
        self._fault_reason: Optional[str] = None
        self._last_error: Optional[str] = None
        self._estop_latched = False
        self._motion_active = False
        self._motion_target_position: Optional[int] = None
        self._motion_start_position: Optional[int] = None
        self._motion_deadline = 0.0
        self._motion_result = ''
        self._motion_stalled = False
        self._motion_failed = False
        self._motion_owner = ''
        self._last_io_error_log = 0.0
        self._io_error_count = 0

        self._state_pub = self.create_publisher(
            JointState, str(gp('state_topic').value), 10)
        self._joint_states_pub = self.create_publisher(
            JointState, str(gp('joint_states_topic').value), 10)
        self._position_pub = self.create_publisher(
            Int32, str(gp('position_topic').value), 10)
        self._width_pub = self.create_publisher(
            Float32, str(gp('width_topic').value), 10)
        self._status_pub = self.create_publisher(
            DiagnosticArray, str(gp('status_topic').value), 10)

        self._command_sub = self.create_subscription(
            Int32,
            str(gp('command_topic').value),
            self._command_callback,
            10,
            callback_group=self._callback_group,
        )
        self._speed_sub = self.create_subscription(
            Int32,
            str(gp('speed_topic').value),
            self._speed_callback,
            10,
            callback_group=self._callback_group,
        )
        self._enable_srv = self.create_service(
            SetBool,
            str(gp('enable_service').value),
            self._enable_callback,
            callback_group=self._callback_group,
        )
        self._estop_srv = self.create_service(
            Empty,
            self._estop_service,
            self._estop_callback,
            callback_group=self._callback_group,
        )
        self._clear_fault_srv = self.create_service(
            Trigger,
            self._clear_fault_service,
            self._clear_fault_callback,
            callback_group=self._callback_group,
        )
        self._clear_obstruction_srv = self.create_service(
            Trigger,
            self._clear_obstruction_service,
            self._clear_obstruction_callback,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            GripperCommand,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.create_timer(
            1.0 / self._status_rate,
            self._publish_status,
            callback_group=self._callback_group,
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._open_transport()
        self._io_thread = threading.Thread(
            target=self._io_loop, name=f'{self._side}-op3-canfd',
            daemon=True)
        self._io_thread.start()
        endpoint = (
            self._port if self._transport_kind == 'zqwl'
            else self._can_interface)
        mode = 'CAN-FD' if self._can_fd else 'CAN'
        self.get_logger().info(
            f'{self._side} OmniPicker 3 ready: transport={self._transport_kind}, '
            f'endpoint={endpoint}, mode={mode}, id={self._device_id}, '
            f'bitrate={self._bitrate}, data_bitrate='
            f'{self._data_bitrate if self._can_fd else "n/a"}, '
            f'allow_motion={self._allow_motion}')

    def _validate_runtime_configuration(self) -> None:
        if not 1 <= self._device_id <= 0x7FF:
            raise ValueError('device_id must be 1..0x7FF')
        if not 0 <= self._channel <= 3:
            raise ValueError('channel must be 0..3')
        if self._position_max <= self._position_min:
            raise ValueError('position_max must be greater than position_min')
        if self._command_max <= self._position_min:
            raise ValueError('command_max must be greater than position_min')
        if self._opening_max < self._opening_min:
            raise ValueError('opening_max_mm must be >= opening_min_mm')
        if self._parallel_width_mm <= 0.0:
            raise ValueError('parallel_gripper_width_mm must be positive')
        if self._feedback_rate <= 0.0 or self._status_rate <= 0.0:
            raise ValueError('feedback_rate and status_rate must be positive')
        if self._receive_timeout < 0.0:
            raise ValueError('receive_timeout_sec must be non-negative')
        if self._feedback_timeout <= 0.0 or self._goal_timeout <= 0.0:
            raise ValueError('timeouts must be positive')
        self._validate_motion_percentages(
            self._motion_speed_percent,
            self._motion_force_percent,
            self._motion_accel_percent,
            self._motion_decel_percent,
        )

    @staticmethod
    def _validate_motion_percentages(*values: int) -> None:
        names = (
            'motion_speed_percent',
            'motion_force_percent',
            'motion_acceleration_percent',
            'motion_deceleration_percent',
        )
        for name, value in zip(names, values):
            if not 1 <= value <= 100:
                raise ValueError(f'{name} must be 1..100')

    def _open_transport(self) -> None:
        try:
            self._transport = make_transport(
                self._transport_kind,
                port=self._port,
                serial_baudrate=self._serial_baudrate,
                channel=self._channel,
                bitrate=self._bitrate,
                data_bitrate=self._data_bitrate,
                can_interface=self._can_interface,
                can_fd=self._can_fd,
                brs=self._brs,
                save_params=self._save_params,
            )
            self._transport.open()
            self._adapter_error = None
        except (CanTransportError, OSError, ZqwlError, ValueError) as exc:
            self._transport = None
            self._adapter_error = str(exc)
            self._set_fault(f'CAN transport unavailable: {exc}')
            self.get_logger().error(
                f'Cannot initialize OmniPicker 3 {self._transport_kind} '
                f'transport: {exc}')

    def _position_to_joint_value(self, position: int) -> float:
        span = self._position_max - self._position_min
        ratio = (float(position) - self._position_min) / span
        ratio = min(1.0, max(0.0, ratio))
        return self._joint_open_m + ratio * (
            self._joint_closed_m - self._joint_open_m)

    def _joint_value_to_position(self, joint_value: float) -> int:
        span = self._joint_closed_m - self._joint_open_m
        if abs(span) < 1e-9:
            raise ValueError('joint_open_m and joint_closed_m must differ')
        ratio = (joint_value - self._joint_open_m) / span
        ratio = min(1.0, max(0.0, ratio))
        position = self._position_min + ratio * (
            self._position_max - self._position_min)
        return int(round(position))

    def _position_to_width(self, position: int) -> float:
        span = self._position_max - self._position_min
        ratio = (float(position) - self._position_min) / span
        ratio = min(1.0, max(0.0, ratio))
        return self._opening_max - ratio * (
            self._opening_max - self._opening_min)

    def _set_fault(self, reason: str) -> None:
        with self._state_lock:
            self._fault_reason = reason
            self._last_error = reason
            self._state = 'fault'
            self._target_position = None
            self._motion_active = False
        self.get_logger().error(reason)

    def _clear_runtime_fault(self) -> None:
        with self._state_lock:
            self._fault_reason = None
            if not self._estop_latched:
                self._state = 'ready' if self._allow_motion else (
                    'motion disabled')

    def _feedback_fresh(self) -> bool:
        return (
            self._feedback_monotonic > 0.0
            and time.monotonic() - self._feedback_monotonic
            <= self._feedback_timeout
        )

    def _send_target(self, position: int) -> None:
        transport = self._transport
        if transport is None:
            raise CanTransportError('CAN transport is not open')
        frame = control_frame(
            position=position,
            speed_percent=self._motion_speed_percent,
            force_percent=self._motion_force_percent,
            acceleration_percent=self._motion_accel_percent,
            deceleration_percent=self._motion_decel_percent,
            device_id=self._device_id,
            channel=self._channel,
            fd=self._can_fd,
            brs=self._brs,
            position_min=self._position_min,
            position_max=self._position_max,
        )
        transport.send(frame)

    def _handle_position_query(self, position_raw: int) -> None:
        ratio = min(1.0, max(0.0, position_raw / 4095.0))
        position = int(round(
            self._position_min
            + (1.0 - ratio) * (self._position_max - self._position_min)))
        now = time.monotonic()
        with self._state_lock:
            self._position = position
            self._position_byte = int(round((1.0 - ratio) * 255.0))
            self._joint_value = self._position_to_joint_value(position)
            self._width_mm = self._position_to_width(position)
            self._feedback_monotonic = now
            if self._motion_active and self._motion_target_position is not None:
                if abs(position - self._motion_target_position) <= self._position_tolerance:
                    self._motion_active = False
                    self._motion_result = f'reached {position}'
                    self._state = 'enabled'
            if self._status is None:
                self._status = OmniPickerStatus(position_raw=position_raw)
            else:
                self._status = OmniPickerStatus(
                    fault_code=self._status.fault_code,
                    state_code=self._status.state_code,
                    position_byte=self._position_byte,
                    velocity_byte=self._status.velocity_byte,
                    force_byte=self._status.force_byte,
                    position_raw=position_raw,
                )
        self._publish_feedback_messages()

    def _handle_status(self, status: OmniPickerStatus) -> None:
        if status.position_byte is None:
            return
        position = position_from_can(
            status.position_byte, self._position_min, self._position_max)
        now = time.monotonic()
        closing_stall = False
        with self._state_lock:
            self._status = status
            self._position_byte = status.position_byte
            self._position = position
            self._joint_value = self._position_to_joint_value(position)
            self._width_mm = self._position_to_width(position)
            self._feedback_monotonic = now
            if status.fault_code != FAULT_NONE:
                self._fault_reason = status.fault_name
                self._last_error = f'OP3 fault: {status.fault_name}'
                self._state = 'fault'
                self._target_position = None
                self._motion_failed = True
                self._motion_active = False
            elif self._estop_latched:
                self._state = 'fault'
            elif self._motion_active:
                target = self._motion_target_position
                start = self._motion_start_position
                error = (
                    abs(position - target)
                    if target is not None else self._position_tolerance + 1)
                reached = error <= self._position_tolerance
                inferred_closing = (
                    target > start if start is not None
                    else target >= (self._position_min + self._position_max) / 2)
                closing_stall = (
                    status.state_code == STATE_STALLED
                    and target is not None
                    and inferred_closing
                    and self._treat_closed_stall_as_grasp
                )
                if status.state_code == STATE_DROPPED:
                    self._motion_failed = True
                    self._motion_result = 'object dropped'
                    self._motion_active = False
                    self._last_error = self._motion_result
                elif status.state_code == STATE_STALLED and not closing_stall:
                    self._motion_failed = True
                    self._motion_result = (
                        f'motion stalled away from current target '
                        f'(actual={position}, target={target})')
                    self._motion_active = False
                    self._last_error = self._motion_result
                elif reached or status.state_code == STATE_REACHED:
                    self._motion_active = False
                    self._motion_result = f'reached {position}'
                elif closing_stall:
                    self._motion_active = False
                    self._motion_result = f'closed stall at {position}'
                else:
                    self._motion_result = 'moving'
                self._motion_stalled = closing_stall
                self._state = 'moving' if self._motion_active else 'enabled'
            elif self._enabled:
                self._state = 'enabled'
            else:
                self._state = 'ready'
        self._publish_feedback_messages()

    def _io_loop(self) -> None:
        period = 1.0 / self._feedback_rate
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            with self._state_lock:
                target = self._target_position
                should_send = bool(
                    self._allow_motion and self._enabled and target is not None
                    and self._estop_latched is False
                    and self._fault_reason is None)
            if should_send:
                try:
                    with self._io_lock:
                        self._send_target(int(target))
                    self._io_error_count = 0
                except CanTransportError as exc:
                    self._io_error_count += 1
                    if (self._io_error_count >= 3
                            or loop_start - self._last_io_error_log > 2.0):
                        self._last_io_error_log = loop_start
                        self._set_fault(f'CAN-FD send failed: {exc}')
            try:
                with self._io_lock:
                    transport = self._transport
                    if transport is not None and self._can_fd:
                        transport.send(position_query_frame(
                            device_id=self._device_id,
                            channel=self._channel,
                            brs=self._brs))
                    frames = (
                        transport.receive(self._receive_timeout)
                        if transport is not None else [])
                for frame in frames:
                    queried_position = decode_position_query(
                        frame, self._device_id)
                    if queried_position is not None:
                        self._handle_position_query(queried_position)
                        continue
                    status = decode_status(frame, self._device_id)
                    if status is not None:
                        self._handle_status(status)
            except CanTransportError as exc:
                if loop_start - self._last_io_error_log > 2.0:
                    self._last_io_error_log = loop_start
                    self.get_logger().warn(f'CAN-FD receive failed: {exc}')
            with self._state_lock:
                active = self._motion_active
            if active and not self._feedback_fresh():
                if self._latch_motion_timeout:
                    self._set_fault(
                        'OmniPicker status feedback timed out during motion')
            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0.0:
                self._stop_event.wait(remaining)

    def _begin_motion(self, position: int, owner: str) -> tuple[bool, str]:
        if not self._allow_motion:
            return False, 'motion is disabled (set allow_motion:=true)'
        with self._motion_lock, self._state_lock:
            if self._estop_latched:
                return False, 'emergency stop is latched'
            if self._fault_reason is not None:
                return False, f'gripper fault: {self._fault_reason}'
            if self._transport is None:
                return False, f'CAN transport unavailable: {self._adapter_error}'
            if (self._motion_active and self._motion_owner
                    and self._motion_owner != owner):
                return False, (
                    f'another motion is active ({self._motion_owner})')
            start = self._position
            self._target_position = int(position)
            self._enabled = True
            self._motion_active = True
            self._motion_target_position = int(position)
            self._motion_start_position = start
            self._motion_deadline = time.monotonic() + self._goal_timeout
            self._motion_result = 'moving'
            self._motion_stalled = False
            self._motion_failed = False
            self._motion_owner = owner
            self._state = 'moving'
        return True, 'moving'

    def _stop_motion(self, reason: str) -> None:
        with self._state_lock:
            self._target_position = None
            self._enabled = False
            self._motion_active = False
            self._motion_result = reason
            self._state = 'ready' if self._allow_motion else 'motion disabled'

    def _enable_callback(
            self, request: SetBool.Request, response: SetBool.Response):
        if request.data and not self._allow_motion:
            response.success = False
            response.message = 'motion is disabled (set allow_motion:=true)'
            return response
        if request.data and self._estop_latched:
            response.success = False
            response.message = 'clear the latched emergency stop first'
            return response
        with self._state_lock:
            self._enabled = bool(request.data)
            if request.data:
                self._state = 'enabled'
            else:
                self._wildcard_stop_locked('disabled by service')
                self._state = 'ready' if self._allow_motion else 'motion disabled'
        response.success = True
        response.message = (
            'OP3 logical command stream enabled; the motor remains '
            'auto-enabled by firmware'
            if request.data else
            'OP3 command stream stopped. The documented simple CAN protocol '
            'does not expose a motor-disable command.')
        return response

    def _wildcard_stop_locked(self, reason: str) -> None:
        self._target_position = None
        self._motion_active = False
        self._motion_result = reason

    def _estop_callback(self, _request, response):
        with self._state_lock:
            self._estop_latched = True
            self._enabled = False
            self._fault_reason = 'emergency stop'
            self._last_error = 'emergency stop'
            self._wildcard_stop_locked('emergency stop')
            self._state = 'fault'
        self.get_logger().error(
            'OmniPicker software emergency stop latched; CAN-FD motor '
            'commands stopped. Remove hazards before clearing the latch.')
        return Empty.Response()

    def _clear_fault_callback(
            self, _request: Trigger.Request, response: Trigger.Response):
        with self._state_lock:
            self._estop_latched = False
            self._fault_reason = None
            self._last_error = None
            self._state = 'ready' if self._allow_motion else 'motion disabled'
        response.success = True
        response.message = (
            'software fault/estop latch cleared; hardware faults are only '
            'cleared if the OP3 status reports no fault')
        return response

    def _clear_obstruction_callback(
            self, _request: Trigger.Request, response: Trigger.Response):
        with self._state_lock:
            position = self._position
            target = self._target_position
        if position is None:
            response.success = False
            response.message = 'no OP3 position feedback is available'
            return response
        open_target = max(self._position_min, position - 120)
        if target is not None:
            open_target = max(open_target, min(target, position) - 120)
        ok, reason = self._begin_motion(open_target, 'clear_obstruction')
        response.success = ok
        response.message = (
            f'opening toward {open_target}' if ok else reason)
        return response

    def _command_callback(self, msg: Int32) -> None:
        target = max(self._command_max, self._position_min)
        position = max(self._position_min, min(int(msg.data), target))
        if self._auto_enable_on_command and self._allow_motion:
            with self._state_lock:
                self._enabled = True
        ok, reason = self._begin_motion(position, 'position_cmd')
        if not ok:
            with self._state_lock:
                self._last_error = reason
            self.get_logger().warn(f'Gripper position command rejected: {reason}')
        else:
            with self._state_lock:
                self._last_error = None
            self.get_logger().debug(
                f'OmniPicker target set by topic: {position}')

    def _speed_callback(self, msg: Int32) -> None:
        speed = int(msg.data)
        if not 1 <= speed <= 100:
            self.get_logger().warn(
                f'Gripper speed command must be 1..100, got {speed}')
            return
        with self._state_lock:
            self._motion_speed_percent = speed
        self.get_logger().info(f'OmniPicker speed set to {speed}%')

    def _goal_callback(self, goal_request):
        command = goal_request.command
        if not self._allow_motion:
            self.get_logger().warn('Rejecting gripper goal: allow_motion=false')
            return GoalResponse.REJECT
        lower, upper = sorted((
            self._joint_open_m, self._joint_closed_m))
        if not lower <= command.position <= upper:
            self.get_logger().warn(
                f'Rejecting gripper goal outside [{lower}, {upper}]: '
                f'{command.position}')
            return GoalResponse.REJECT
        if command.max_effort > 0.0 and not self._allow_nonzero_effort:
            self.get_logger().warn(
                'GripperCommand.max_effort is ignored; configure force with '
                'motion_force_percent')
        if self._estop_latched or self._fault_reason is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        result = GripperCommand.Result()
        position = self._joint_value_to_position(goal_handle.request.command.position)
        with self._motion_lock:
            if self._motion_active:
                self.get_logger().warn(
                    'Rejecting gripper goal while another motion is active: '
                    f'{self._motion_owner}')
                goal_handle.abort()
                result.reached_goal = False
                return result
            ok, reason = self._begin_motion(position, 'action')
        if not ok:
            self.get_logger().warn(f'Cannot execute gripper goal: {reason}')
            goal_handle.abort()
            result.position = (
                float(self._joint_value)
                if self._joint_value is not None else 0.0)
            result.reached_goal = False
            return result

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                if self._cancel_stops_motion:
                    self._stop_motion('goal canceled')
                goal_handle.canceled()
                result.position = (
                    float(self._joint_value)
                    if self._joint_value is not None else 0.0)
                result.reached_goal = False
                return result
            with self._state_lock:
                active = self._motion_active
                reached = (
                    self._position is not None
                    and abs(self._position - position)
                    <= self._position_tolerance)
                stalled = self._motion_stalled
                failed = self._motion_failed
                deadline = self._motion_deadline
                joint = self._joint_value
            if not active:
                result.position = float(joint) if joint is not None else 0.0
                result.reached_goal = bool(reached)
                result.stalled = bool(stalled)
                if failed:
                    goal_handle.abort()
                elif reached or stalled:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            if time.monotonic() >= deadline:
                self._set_fault(
                    f'OmniPicker goal timed out before feedback reached '
                    f'position {position}')
                goal_handle.abort()
                result.position = float(joint) if joint is not None else 0.0
                result.reached_goal = False
                return result
            time.sleep(0.02)
        goal_handle.abort()
        result.reached_goal = False
        return result

    def _publish_feedback_messages(self) -> None:
        with self._state_lock:
            position = self._position
            joint = self._joint_value
            width = self._width_mm
            status = self._status
        if position is None or joint is None or width is None:
            return
        stamp = self.get_clock().now().to_msg()
        message = JointState()
        message.header.stamp = stamp
        message.name = [self._joint_name]
        message.position = [float(joint)]
        message.velocity = [
            float(status.velocity_byte) if status is not None else 0.0]
        message.effort = [
            float(status.force_byte) if status is not None else 0.0]
        self._state_pub.publish(message)
        self._joint_states_pub.publish(message)
        position_message = Int32()
        position_message.data = int(position)
        self._position_pub.publish(position_message)
        width_message = Float32()
        width_message.data = float(width)
        self._width_pub.publish(width_message)

    def _status_values(self) -> dict[str, str]:
        now = time.monotonic()
        with self._state_lock:
            status = self._status
            feedback_age = (
                now - self._feedback_monotonic
                if self._feedback_monotonic > 0.0 else float('inf'))
            target = self._target_position
            return {
                'protocol': 'omnipicker3_canfd',
                'transport': self._transport_kind,
                'state': self._state,
                'enabled': str(self._enabled).lower(),
                'can_connected': str(self._transport is not None).lower(),
                'allow_motion': str(self._allow_motion).lower(),
                'auto_enable_on_command': str(
                    self._auto_enable_on_command).lower(),
                'feedback_fresh': str(feedback_age <= self._feedback_timeout).lower(),
                'feedback_age_sec': (
                    f'{feedback_age:.3f}' if feedback_age != float('inf') else 'inf'),
                'side': self._side,
                'fault': str((
                    status.fault_code if status is not None else FAULT_NONE)),
                'fault_reason': self._fault_reason or '',
                'last_error': self._last_error or '',
                'estop_latched': str(self._estop_latched).lower(),
                'motion_active': str(self._motion_active).lower(),
                'motion_source': self._motion_owner,
                'motion_target_position': (
                    str(target) if target is not None else ''),
                'motion_start_position': (
                    str(self._motion_start_position)
                    if self._motion_start_position is not None else ''),
                'command_position': (
                    str(target) if target is not None else ''),
                'feedback_position': (
                    str(self._position) if self._position is not None else ''),
                'position_min': str(self._position_min),
                'position_max': str(self._position_max),
                'command_min': str(self._position_min),
                'command_max': str(self._command_max),
                'controller_enabled': str(self._enabled).lower(),
                'enable_register': '1' if self._enabled else '0',
                'max_current_register': '',
                'motion_result': self._motion_result,
                'motion_stalled': str(self._motion_stalled).lower(),
                'position': (
                    str(self._position) if self._position is not None else ''),
                'joint_value': (
                    f'{self._joint_value:.6f}'
                    if self._joint_value is not None else ''),
                'joint_rad': (
                    f'{self._joint_value:.6f}'
                    if self._joint_value is not None else ''),
                'width_mm': (
                    f'{self._width_mm:.3f}'
                    if self._width_mm is not None else ''),
                'device_state': (
                    status.state_name if status is not None else 'unknown'),
                'device_position_byte': (
                    str(status.position_byte) if status is not None else ''),
                'device_velocity_byte': (
                    str(status.velocity_byte) if status is not None else ''),
                'device_force_byte': (
                    str(status.force_byte) if status is not None else ''),
                'drop_alarm': str((
                    status is not None and status.state_code == STATE_DROPPED
                )).lower(),
                'force_control_available': 'false',
                'motion_speed_percent': str(self._motion_speed_percent),
                'motion_force_percent': str(self._motion_force_percent),
                'motion_torque_percent': str(self._motion_force_percent),
                'motion_acceleration_percent': str(self._motion_accel_percent),
                'motion_deceleration_percent': str(self._motion_decel_percent),
                'motion_ramp_ms': '200',
                'torque_limit_supported': 'true',
                'endpoint': (
                    self._port if self._transport_kind == 'zqwl'
                    else self._can_interface),
                'adapter_error': self._adapter_error or '',
            }

    def _publish_status(self) -> None:
        values = self._status_values()
        if values['state'] == 'fault':
            level = DiagnosticStatus.ERROR
        elif values['can_connected'] != 'true':
            level = DiagnosticStatus.WARN
        elif (
            values['motion_active'] == 'true'
            and values['feedback_fresh'] != 'true'
        ):
            level = DiagnosticStatus.ERROR
        elif values['drop_alarm'] == 'true':
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.OK
        if values['fault_reason']:
            message = f"fault: {values['fault_reason']}"
        elif values['last_error']:
            message = values['last_error']
        elif values['can_connected'] != 'true':
            message = 'CAN transport unavailable'
        elif values['motion_active'] == 'true':
            message = f"moving to {values['motion_target_position']}"
        else:
            message = values['state']
        status = DiagnosticStatus()
        status.level = level
        status.name = f'{self._side}_gripper'
        status.hardware_id = values['endpoint']
        status.message = message
        status.values = [
            KeyValue(key=key, value=value)
            for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_pub.publish(array)

    def _on_set_parameters(self, parameters: list[NodeParameter]):
        speed = self._motion_speed_percent
        force = self._motion_force_percent
        accel = self._motion_accel_percent
        decel = self._motion_decel_percent
        tolerance = self._position_tolerance
        goal_timeout = self._goal_timeout
        for parameter in parameters:
            if parameter.name in {
                    'motion_speed_percent', 'motion_force_percent',
                    'motion_torque_percent', 'motion_acceleration_percent',
                    'motion_deceleration_percent', 'position_tolerance',
                    'motion_ramp_ms', 'max_current'}:
                if parameter.type_ != NodeParameter.Type.INTEGER:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{parameter.name} must be an integer')
                value = int(parameter.value)
                if parameter.name == 'motion_speed_percent':
                    speed = value
                elif parameter.name in {
                        'motion_force_percent', 'motion_torque_percent'}:
                    force = value
                elif parameter.name == 'motion_acceleration_percent':
                    accel = value
                elif parameter.name == 'motion_deceleration_percent':
                    decel = value
                elif parameter.name == 'position_tolerance':
                    tolerance = value
                # motion_ramp_ms and max_current are accepted only so old
                # Web/MoveIt clients do not fail; OP3 does not use them.
            elif parameter.name == 'goal_timeout_sec':
                if parameter.type_ not in {
                        NodeParameter.Type.DOUBLE,
                        NodeParameter.Type.INTEGER}:
                    return SetParametersResult(
                        successful=False,
                        reason='goal_timeout_sec must be numeric')
                goal_timeout = float(parameter.value)
        try:
            self._validate_motion_percentages(speed, force, accel, decel)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        if tolerance <= 0:
            return SetParametersResult(
                successful=False, reason='position_tolerance must be positive')
        if goal_timeout <= 0.0:
            return SetParametersResult(
                successful=False, reason='goal_timeout_sec must be positive')
        with self._state_lock:
            self._motion_speed_percent = speed
            self._motion_force_percent = force
            self._motion_accel_percent = accel
            self._motion_decel_percent = decel
            self._position_tolerance = tolerance
            self._goal_timeout = goal_timeout
        return SetParametersResult(successful=True)

    def close(self) -> None:
        self._stop_event.set()
        if self._io_thread is not None:
            self._io_thread.join(timeout=1.0)
        with self._state_lock:
            self._enabled = False
            self._target_position = None
            self._motion_active = False
        with self._io_lock:
            transport = self._transport
            self._transport = None
            if transport is None:
                return
            if self._disable_on_shutdown and hasattr(transport, 'disable'):
                transport.disable()
            transport.close()


def main(args=None, side: str = 'right') -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = OmniPickerNode(side=side)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except KeyboardInterrupt:
                pass
        if node is not None:
            try:
                node.close()
            finally:
                try:
                    node.destroy_node()
                except Exception:
                    pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass


def left_main(args=None) -> None:
    main(args=args, side='left')


if __name__ == '__main__':
    main()
