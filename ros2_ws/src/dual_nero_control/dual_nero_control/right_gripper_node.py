#!/usr/bin/python3
"""ROS 2 driver for a CTAG2F90-D gripper over a ZQWL adapter."""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter as NodeParameter
from control_msgs.action import GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Empty, SetBool, Trigger

from .ctag_gripper import (
    FAULT_STATUS_B_DROP_ALARM,
    MOTOR_STATUS_ENABLED,
    MOTOR_STATUS_POSITION_REACHED,
    MOTOR_STATUS_READY,
    MOTOR_STATUS_SPEED_REACHED,
    MOTOR_STATUS_TORQUE_REACHED,
    MOTOR_STATUS_ZERO_SPEED,
    POSITION_MAX,
    POSITION_MIN,
    REG_ACCEL_RAMP,
    REG_BUS_VOLTAGE,
    REG_ENABLE,
    REG_FAULT_STATUS,
    REG_FAULT_STATUS_B,
    REG_FEEDBACK,
    REG_MAX_CURRENT,
    REG_MAX_SPEED,
    REG_MODE,
    REG_MOTOR_STATUS,
    REG_SPEED,
    REG_SUPPLY_TEMPERATURE,
    REG_TARGET_POSITION,
    REG_TORQUE,
    REG_USER_CURRENT,
    alarm_reset_frame,
    decode_feedback,
    decode_register_u16,
    enable_frame,
    joint_from_position,
    motion_config_frames,
    motion_frames,
    opening_from_position,
    position_from_joint,
    register_read_request_frame,
    scalar_write_frame,
)
from .zqwl_canfd import DEFAULT_SERIAL_BAUD, STABLE_PORT, ZqwlCanFd, ZqwlError


LEFT_STABLE_PORT = (
    '/dev/serial/by-id/'
    'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00')
PORTS = {'left': LEFT_STABLE_PORT, 'right': STABLE_PORT}
DEFAULT_COMMAND_MAX = 990
DEFAULT_MAX_CURRENT = 200
MOTION_SETTLE_CONFIRM_SEC = 0.5
# Conservative vendor speed-register estimate used only for motion deadlines.
# Measured travel is faster at the high end, so this cannot cut a move short.
RAW_COUNTS_PER_SECOND_PER_SPEED_PERCENT = 0.6


class CtagGripperNode(Node):
    def __init__(self, side: str = 'right') -> None:
        side = side.strip().lower()
        if side not in PORTS:
            raise ValueError(f'side must be left or right, got {side!r}')
        self._side = side
        prefix = f'{side}_gripper'
        super().__init__(f'{side}_gripper_node')

        self.declare_parameter('port', PORTS[side])
        self.declare_parameter('serial_baudrate', DEFAULT_SERIAL_BAUD)
        self.declare_parameter('device_id', 1)
        self.declare_parameter('channel', 0)
        self.declare_parameter('bitrate', 1_000_000)
        self.declare_parameter('can_fd', False)
        self.declare_parameter('data_bitrate', 5_000_000)
        self.declare_parameter('brs', False)
        self.declare_parameter('save_params', False)
        self.declare_parameter('allow_motion', False)
        self.declare_parameter('enable_motor_on_start', False)
        # The direct gripper interface is intentionally small: a position
        # command, a speed command, emergency stop and obstruction clearing.
        # Auto-enable keeps that interface usable without a separate setup
        # step while the explicit enable service remains available for Web and
        # MoveIt compatibility.
        self.declare_parameter('auto_enable_on_command', True)
        self.declare_parameter('require_feedback_for_motion', True)
        self.declare_parameter('feedback_timeout_sec', 1.0)
        self.declare_parameter('feedback_rate', 20.0)
        self.declare_parameter('status_rate', 20.0)
        self.declare_parameter('goal_timeout_sec', 10.0)
        self.declare_parameter('motion_timeout_margin_sec', 5.0)
        self.declare_parameter('latch_motion_timeout', True)
        self.declare_parameter('cancel_disables_motor', True)
        self.declare_parameter('disable_on_shutdown', True)
        self.declare_parameter('allow_nonzero_effort', False)
        self.declare_parameter('watchdog_rate', 20.0)
        self.declare_parameter('position_tolerance', 25)
        self.declare_parameter('treat_closed_stall_as_grasp', True)
        self.declare_parameter('motion_speed_percent', 25)
        self.declare_parameter('motion_torque_percent', 100)
        self.declare_parameter('motion_ramp_ms', 200)
        self.declare_parameter('max_current', DEFAULT_MAX_CURRENT)
        self.declare_parameter('joint_name', f'{side}_ctag_left_1_joint')
        self.declare_parameter('joint_open_rad', 0.0)
        self.declare_parameter('joint_closed_rad', -0.90838)
        self.declare_parameter('position_min', POSITION_MIN)
        self.declare_parameter('position_max', POSITION_MAX)
        self.declare_parameter('command_max', DEFAULT_COMMAND_MAX)
        self.declare_parameter('opening_min_mm', 0.0)
        self.declare_parameter('opening_max_mm', 90.0)
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
        self._port = str(gp('port').value)
        self._serial_baudrate = int(gp('serial_baudrate').value)
        self._device_id = int(gp('device_id').value)
        self._channel = int(gp('channel').value)
        self._bitrate = int(gp('bitrate').value)
        self._can_fd = bool(gp('can_fd').value)
        self._data_bitrate = int(gp('data_bitrate').value)
        self._brs = bool(gp('brs').value)
        self._save_params = bool(gp('save_params').value)
        self._allow_motion = bool(gp('allow_motion').value)
        self._enable_motor_on_start = bool(
            gp('enable_motor_on_start').value)
        self._auto_enable_on_command = bool(
            gp('auto_enable_on_command').value)
        self._require_feedback = bool(
            gp('require_feedback_for_motion').value)
        self._feedback_timeout = max(
            0.05, float(gp('feedback_timeout_sec').value))
        self._feedback_rate = max(1.0, float(gp('feedback_rate').value))
        self._status_rate = max(1.0, float(gp('status_rate').value))
        self._goal_timeout = max(0.1, float(gp('goal_timeout_sec').value))
        self._motion_timeout_margin = max(
            0.5, float(gp('motion_timeout_margin_sec').value))
        self._latch_motion_timeout = bool(
            gp('latch_motion_timeout').value)
        self._cancel_disables_motor = bool(
            gp('cancel_disables_motor').value)
        self._disable_on_shutdown = bool(gp('disable_on_shutdown').value)
        self._allow_nonzero_effort = bool(
            gp('allow_nonzero_effort').value)
        self._watchdog_rate = max(1.0, float(gp('watchdog_rate').value))
        self._position_tolerance = max(
            0, int(gp('position_tolerance').value))
        self._treat_closed_stall_as_grasp = bool(
            gp('treat_closed_stall_as_grasp').value)
        self._motion_speed_percent = int(gp('motion_speed_percent').value)
        self._motion_torque_percent = int(gp('motion_torque_percent').value)
        self._motion_ramp_ms = int(gp('motion_ramp_ms').value)
        self._max_current = int(gp('max_current').value)
        self._joint_name = str(gp('joint_name').value)
        self._joint_open = float(gp('joint_open_rad').value)
        self._joint_closed = float(gp('joint_closed_rad').value)
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

        if self._position_max <= self._position_min:
            raise ValueError('position_max must be greater than position_min')
        if self._command_max <= self._position_min:
            raise ValueError('command_max must be greater than position_min')
        self._validate_motion_config(
            self._motion_speed_percent,
            self._motion_torque_percent,
            self._motion_ramp_ms,
            self._max_current,
        )
        if self._brs and not self._can_fd:
            self.get_logger().warn(
                'brs=true requires CAN-FD; disabling BRS for classic CAN')
            self._brs = False

        self._callback_group = ReentrantCallbackGroup()
        self._io_lock = threading.Lock()
        self._state_lock = threading.RLock()
        # Serialize enable, motion and stop requests.  RLock is required
        # because _start_motion holds it while requesting auto-enable.
        self._motion_start_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._client: Optional[ZqwlCanFd] = None
        self._adapter_error: Optional[str] = None
        self._io_thread: Optional[threading.Thread] = None
        self._position: Optional[int] = None
        self._joint_rad: Optional[float] = None
        self._feedback_monotonic = 0.0
        self._controller_target_position: Optional[int] = None
        self._controller_target_monotonic = 0.0
        self._control_mode: Optional[int] = None
        self._control_mode_monotonic = 0.0
        self._motor_status_bits: Optional[int] = None
        self._motor_status_monotonic = 0.0
        self._fault_bits: Optional[int] = None
        self._fault_status_monotonic = 0.0
        self._fault_bits_b: Optional[int] = None
        self._fault_status_b_monotonic = 0.0
        self._user_current: Optional[int] = None
        self._user_current_monotonic = 0.0
        self._bus_voltage_raw: Optional[int] = None
        self._bus_voltage_monotonic = 0.0
        self._temperature_raw: Optional[int] = None
        self._temperature_monotonic = 0.0
        self._max_speed_register: Optional[int] = None
        self._max_speed_monotonic = 0.0
        self._max_current_register: Optional[int] = None
        self._max_current_monotonic = 0.0
        self._enable_register: Optional[int] = None
        self._enable_register_monotonic = 0.0
        self._speed_register: Optional[int] = None
        self._speed_register_monotonic = 0.0
        self._torque_register: Optional[int] = None
        self._torque_register_monotonic = 0.0
        self._ramp_register: Optional[int] = None
        self._ramp_register_monotonic = 0.0
        self._last_io_error_log = 0.0
        self._last_feedback_error_log = 0.0
        self._safe_disable_last_attempt = 0.0
        self._enabled = False
        self._state = 'disabled'
        self._fault_reason: Optional[str] = None
        self._last_error: Optional[str] = None
        self._estop_latched = False
        self._motion_active = False
        self._motion_target_position: Optional[int] = None
        self._motion_deadline = 0.0
        self._motion_source = ''
        self._motion_start_position: Optional[int] = None
        self._motion_in_tolerance_since = 0.0
        self._motion_is_closing = False
        self._motion_result = ''
        self._motion_stalled = False
        self._last_motion_reached = False
        self._last_motion_stalled = False
        self._last_motion_failed = False
        self._last_command_position: Optional[int] = None
        self._startup_enable_pending = False

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
        self._watchdog_timer = self.create_timer(
            1.0 / self._watchdog_rate,
            self._watchdog_tick,
            callback_group=self._callback_group,
        )
        self._status_timer = self.create_timer(
            1.0 / self._status_rate,
            self._publish_status,
            callback_group=self._callback_group,
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._open_adapter()
        self._io_thread = threading.Thread(
            target=self._io_loop, name='gripper-can-rx', daemon=True)
        self._io_thread.start()

        if self._allow_motion:
            with self._state_lock:
                self._state = 'ready'
            if self._client is not None and not self._write_motion_config():
                self.get_logger().warn(
                    'Initial gripper motion configuration, including '
                    f'0x0533 max_current={self._max_current}, could not be sent')
        if self._client is None:
            if self._allow_motion:
                self._set_fault('CAN adapter unavailable')
        else:
            self._startup_enable_pending = bool(
                self._enable_motor_on_start and self._allow_motion)

        self.get_logger().info(
            f'{side.capitalize()} gripper driver ready: port={self._port}, '
            f'allow_motion={self._allow_motion}, '
            f'action={self._action_name}, '
            f'command=/{prefix}/position_cmd, '
            f'speed=/{prefix}/speed_cmd, '
            f'estop={self._estop_service}, '
            f'clear_obstruction={self._clear_obstruction_service}')

    def _open_adapter(self) -> None:
        try:
            client = ZqwlCanFd(
                port=self._port,
                serial_baudrate=self._serial_baudrate,
                timeout=1.0,
            )
            client.open()
            client.configure_can(
                channel=self._channel,
                bitrate=self._bitrate,
                data_bitrate=self._data_bitrate if self._can_fd else None,
                fd=self._can_fd,
                save=self._save_params,
            )
            self._client = client
            self._adapter_error = None
            mode = 'CAN-FD' if self._can_fd else 'classic CAN'
            self.get_logger().info(
                f'ZQWL adapter configured: {mode}, '
                f'channel={self._channel}, bitrate={self._bitrate}')
        except (ZqwlError, OSError) as exc:
            self._client = None
            self._adapter_error = str(exc)
            if isinstance(exc, PermissionError):
                self.get_logger().error(
                    'Cannot open gripper CAN adapter: permission denied. '
                    'Run the node under the dialout group (for example '
                    "`sg dialout -c '... ros2 launch ...'`) and restart it.")
            self.get_logger().error(
                f'Cannot initialize gripper CAN adapter: {exc}')

    @staticmethod
    def _validate_motion_config(
            speed_percent: int,
            torque_percent: int,
            ramp_ms: int,
            max_current: int) -> None:
        if not 1 <= speed_percent <= 100:
            raise ValueError('motion_speed_percent must be 1..100')
        if not 1 <= torque_percent <= 100:
            raise ValueError('motion_torque_percent must be 1..100')
        if not 0 <= ramp_ms <= 1000:
            raise ValueError('motion_ramp_ms must be 0..1000')
        if not 1 <= max_current <= 1000:
            raise ValueError('max_current must be 1..1000')

    def _set_runtime_speed(self, speed_percent: int, source: str) -> bool:
        if not 1 <= speed_percent <= 100:
            self.get_logger().warn(
                f'Gripper speed rejected from {source}: expected 1..100, '
                f'got {speed_percent}')
            return False
        with self._state_lock:
            changed = speed_percent != self._motion_speed_percent
            self._motion_speed_percent = speed_percent
            motion_active = self._motion_active
        if changed and motion_active:
            # The vendor reads 0x0104 for the active profile as well.  Send the
            # new configuration immediately so a speed change is not deferred
            # until the next position command.
            self._write_motion_config()
        if changed:
            self.get_logger().info(
                f'Gripper speed set to {speed_percent}% from {source}')
        return True

    def _motion_timeout_for(self, distance: int) -> float:
        with self._state_lock:
            speed = self._motion_speed_percent
        travel_sec = abs(distance) / max(
            0.1,
            RAW_COUNTS_PER_SECOND_PER_SPEED_PERCENT * speed,
        )
        return max(
            self._goal_timeout,
            travel_sec + getattr(self, '_motion_timeout_margin', 5.0),
        )

    def _on_set_parameters(
            self, parameters: list[NodeParameter]) -> SetParametersResult:
        speed = self._motion_speed_percent
        torque = self._motion_torque_percent
        ramp = self._motion_ramp_ms
        max_current = self._max_current
        goal_timeout = self._goal_timeout
        for parameter in parameters:
            if parameter.name not in {
                    'goal_timeout_sec',
                    'motion_speed_percent',
                    'motion_torque_percent',
                    'motion_ramp_ms',
                    'max_current'}:
                continue
            if parameter.name == 'goal_timeout_sec':
                if parameter.type_ not in {
                        NodeParameter.Type.DOUBLE,
                        NodeParameter.Type.INTEGER}:
                    return SetParametersResult(
                        successful=False,
                        reason='goal_timeout_sec must be numeric',
                    )
                goal_timeout = float(parameter.value)
                if goal_timeout <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='goal_timeout_sec must be greater than zero',
                    )
                continue
            if parameter.type_ != NodeParameter.Type.INTEGER:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} must be an integer',
                )
            value = int(parameter.value)
            if parameter.name == 'motion_speed_percent':
                speed = value
            elif parameter.name == 'motion_torque_percent':
                torque = value
            elif parameter.name == 'max_current':
                max_current = value
            else:
                ramp = value
        try:
            self._validate_motion_config(
                speed, torque, ramp, max_current)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        with self._state_lock:
            previous_max_current = self._max_current
            self._goal_timeout = goal_timeout
            self._motion_speed_percent = speed
            self._motion_torque_percent = torque
            self._motion_ramp_ms = ramp
            self._max_current = max_current
            motion_active = self._motion_active
        if motion_active or max_current != previous_max_current:
            self._write_motion_config()
        return SetParametersResult(successful=True)

    def _status_values(self) -> dict[str, str]:
        now = time.monotonic()
        with self._state_lock:
            state = self._state
            enabled = self._enabled
            fault_reason = self._fault_reason
            last_error = self._last_error
            estop_latched = self._estop_latched
            motion_active = self._motion_active
            motion_source = self._motion_source
            motion_target = self._motion_target_position
            motion_start = self._motion_start_position
            motion_is_closing = self._motion_is_closing
            motion_result = self._motion_result
            motion_stalled = self._motion_stalled
            position = self._position
            feedback_monotonic = self._feedback_monotonic
            controller_target = self._controller_target_position
            controller_target_monotonic = self._controller_target_monotonic
            control_mode = self._control_mode
            control_mode_monotonic = self._control_mode_monotonic
            motor_status_bits = self._motor_status_bits
            motor_status_monotonic = self._motor_status_monotonic
            fault_bits = self._fault_bits
            fault_status_monotonic = self._fault_status_monotonic
            fault_bits_b = self._fault_bits_b
            fault_status_b_monotonic = self._fault_status_b_monotonic
            user_current = self._user_current
            user_current_monotonic = self._user_current_monotonic
            bus_voltage_raw = self._bus_voltage_raw
            bus_voltage_monotonic = self._bus_voltage_monotonic
            temperature_raw = self._temperature_raw
            temperature_monotonic = self._temperature_monotonic
            max_speed_register = self._max_speed_register
            max_speed_monotonic = self._max_speed_monotonic
            max_current_register = self._max_current_register
            max_current_monotonic = self._max_current_monotonic
            enable_register = self._enable_register
            speed_register = self._speed_register
            torque_register = self._torque_register
            ramp_register = self._ramp_register
            last_command = self._last_command_position
            speed_percent = self._motion_speed_percent
            torque_percent = self._motion_torque_percent
            ramp_ms = self._motion_ramp_ms
            max_current_setpoint = self._max_current
            can_connected = self._client is not None
            allow_motion = self._allow_motion
            auto_enable = self._auto_enable_on_command
            port = self._port

        feedback_age = (
            max(0.0, now - feedback_monotonic)
            if position is not None else None)
        feedback_fresh = (
            feedback_age is not None
            and feedback_age <= self._feedback_timeout)
        controller_target_age = (
            max(0.0, now - controller_target_monotonic)
            if controller_target is not None else None)
        control_mode_age = (
            max(0.0, now - control_mode_monotonic)
            if control_mode is not None else None)
        motor_status_age = (
            max(0.0, now - motor_status_monotonic)
            if motor_status_bits is not None else None)
        fault_status_age = (
            max(0.0, now - fault_status_monotonic)
            if fault_bits is not None else None)
        fault_status_b_age = (
            max(0.0, now - fault_status_b_monotonic)
            if fault_bits_b is not None else None)
        user_current_age = (
            max(0.0, now - user_current_monotonic)
            if user_current is not None else None)
        bus_voltage_age = (
            max(0.0, now - bus_voltage_monotonic)
            if bus_voltage_raw is not None else None)
        temperature_age = (
            max(0.0, now - temperature_monotonic)
            if temperature_raw is not None else None)
        max_speed_age = (
            max(0.0, now - max_speed_monotonic)
            if max_speed_register is not None else None)
        max_current_age = (
            max(0.0, now - max_current_monotonic)
            if max_current_register is not None else None)
        return {
            'side': self._side,
            'state': state,
            'enabled': str(enabled).lower(),
            'fault_reason': fault_reason or '',
            'last_error': last_error or '',
            'estop_latched': str(estop_latched).lower(),
            'motion_active': str(motion_active).lower(),
            'motion_source': motion_source,
            'motion_target_position': (
                str(motion_target) if motion_target is not None else ''),
            'motion_start_position': (
                str(motion_start) if motion_start is not None else ''),
            'motion_is_closing': str(motion_is_closing).lower(),
            'motion_result': motion_result,
            'motion_stalled': str(motion_stalled).lower(),
            'command_position': (
                str(last_command) if last_command is not None else ''),
            'feedback_position': str(position) if position is not None else '',
            'command_min': str(self._position_min),
            'command_max': str(self._command_max),
            'feedback_age_sec': (
                f'{feedback_age:.3f}' if feedback_age is not None else ''),
            'feedback_fresh': str(feedback_fresh).lower(),
            'controller_target_position': (
                str(controller_target)
                if controller_target is not None else ''),
            'controller_target_age_sec': (
                f'{controller_target_age:.3f}'
                if controller_target_age is not None else ''),
            'control_mode': (
                str(control_mode) if control_mode is not None else ''),
            'control_mode_age_sec': (
                f'{control_mode_age:.3f}'
                if control_mode_age is not None else ''),
            'motor_status_bits': (
                f'0x{motor_status_bits:04X}'
                if motor_status_bits is not None else ''),
            'motor_status_age_sec': (
                f'{motor_status_age:.3f}'
                if motor_status_age is not None else ''),
            'motor_position_reached': str(bool(
                motor_status_bits is not None
                and motor_status_bits & MOTOR_STATUS_POSITION_REACHED)).lower(),
            'motor_speed_reached': str(bool(
                motor_status_bits is not None
                and motor_status_bits & MOTOR_STATUS_SPEED_REACHED)).lower(),
            'motor_zero_speed': str(bool(
                motor_status_bits is not None
                and motor_status_bits & MOTOR_STATUS_ZERO_SPEED)).lower(),
            'motor_torque_reached': str(bool(
                motor_status_bits is not None
                and motor_status_bits & MOTOR_STATUS_TORQUE_REACHED)).lower(),
            'motor_ready': str(bool(
                motor_status_bits is not None
                and motor_status_bits & MOTOR_STATUS_READY)).lower(),
            'controller_enabled': str(bool(
                motor_status_bits is not None
                and motor_status_bits & MOTOR_STATUS_ENABLED)).lower(),
            'fault_bits': (
                f'0x{fault_bits:04X}' if fault_bits is not None else ''),
            'fault_status_age_sec': (
                f'{fault_status_age:.3f}'
                if fault_status_age is not None else ''),
            'fault_bits_b': (
                f'0x{fault_bits_b:04X}' if fault_bits_b is not None else ''),
            'fault_status_b_age_sec': (
                f'{fault_status_b_age:.3f}'
                if fault_status_b_age is not None else ''),
            'drop_alarm': str(bool(
                fault_bits_b is not None
                and fault_bits_b & FAULT_STATUS_B_DROP_ALARM)).lower(),
            'bus_voltage_v': (
                f'{bus_voltage_raw / 100.0:.2f}'
                if bus_voltage_raw is not None else ''),
            'bus_voltage_age_sec': (
                f'{bus_voltage_age:.3f}'
                if bus_voltage_age is not None else ''),
            'temperature_c': (
                f'{temperature_raw / 100.0:.2f}'
                if temperature_raw is not None else ''),
            'temperature_age_sec': (
                f'{temperature_age:.3f}'
                if temperature_age is not None else ''),
            'max_speed_register': (
                str(max_speed_register)
                if max_speed_register is not None else ''),
            'max_speed_age_sec': (
                f'{max_speed_age:.3f}'
                if max_speed_age is not None else ''),
            'max_current_register': (
                str(max_current_register)
                if max_current_register is not None else ''),
            'max_current_age_sec': (
                f'{max_current_age:.3f}'
                if max_current_age is not None else ''),
            'enable_register': (
                str(enable_register) if enable_register is not None else ''),
            'speed_register': (
                str(speed_register) if speed_register is not None else ''),
            'torque_register': (
                str(torque_register) if torque_register is not None else ''),
            'ramp_register': (
                str(ramp_register) if ramp_register is not None else ''),
            'max_current_setpoint': str(max_current_setpoint),
            'user_current_raw': (
                str(user_current) if user_current is not None else ''),
            'user_current_age_sec': (
                f'{user_current_age:.3f}'
                if user_current_age is not None else ''),
            'can_connected': str(can_connected).lower(),
            'allow_motion': str(allow_motion).lower(),
            'auto_enable_on_command': str(auto_enable).lower(),
            'motion_speed_percent': str(speed_percent),
            'motion_torque_percent': str(torque_percent),
            'motion_ramp_ms': str(ramp_ms),
            'torque_limit_supported': 'true',
            'torque_feedback_available': str(
                user_current is not None).lower(),
            'current_feedback_available': str(
                user_current is not None).lower(),
            'force_control_available': 'false',
            'port': port,
            'adapter_error': getattr(self, '_adapter_error', '') or '',
        }

    def _publish_status(self) -> None:
        values = self._status_values()
        state = values['state']
        if state == 'fault':
            level = DiagnosticStatus.ERROR
        elif values['can_connected'] != 'true':
            level = DiagnosticStatus.WARN
        elif (
            state in {'enabled', 'moving'}
            and values['feedback_fresh'] != 'true'
        ):
            level = DiagnosticStatus.ERROR
        elif values['feedback_fresh'] != 'true':
            level = DiagnosticStatus.WARN
        elif values['drop_alarm'] == 'true':
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.OK

        if values['fault_reason']:
            message = f"fault: {values['fault_reason']}"
        elif values['last_error']:
            message = values['last_error']
        elif values['can_connected'] != 'true':
            message = 'CAN adapter unavailable'
        elif state == 'moving':
            message = f"moving to {values['motion_target_position']}"
        elif values['motion_stalled'] == 'true' and values['motion_result']:
            message = values['motion_result']
        else:
            message = state

        status = DiagnosticStatus()
        status.level = level
        status.name = f'{self._side}_gripper'
        status.hardware_id = values['port']
        status.message = message
        status.values = [
            KeyValue(key=key, value=value)
            for key, value in values.items()
        ]
        message_array = DiagnosticArray()
        message_array.header.stamp = self.get_clock().now().to_msg()
        message_array.status = [status]
        self._status_pub.publish(message_array)

    def _io_loop(self) -> None:
        period = 1.0 / self._feedback_rate
        read_frames = tuple(
            register_read_request_frame(
                address,
                device_id=self._device_id,
                channel=self._channel,
                fd=self._can_fd,
                brs=self._brs,
                payload_size=(
                    4 if address in {REG_FEEDBACK, REG_TARGET_POSITION}
                    else 2),
            )
            for address in (
                REG_FEEDBACK,
                REG_TARGET_POSITION,
                REG_MODE,
                REG_MOTOR_STATUS,
                REG_FAULT_STATUS,
                REG_FAULT_STATUS_B,
                REG_USER_CURRENT,
                REG_BUS_VOLTAGE,
                REG_SUPPLY_TEMPERATURE,
                REG_MAX_SPEED,
                REG_MAX_CURRENT,
                REG_ENABLE,
                REG_SPEED,
                REG_TORQUE,
                REG_ACCEL_RAMP,
            )
        )
        while not self._stop_event.is_set():
            started = time.monotonic()
            client = self._client
            if client is not None:
                try:
                    with self._io_lock:
                        for frame in read_frames:
                            client.send(frame)
                        for message in client.messages(period):
                            if not hasattr(message, 'can_id'):
                                continue
                            address = self._register_address(message.can_id)
                            if address is None:
                                continue
                            try:
                                if address == REG_FEEDBACK:
                                    self._on_feedback(
                                        decode_feedback(message.data))
                                elif address == REG_TARGET_POSITION:
                                    self._on_controller_target(
                                        decode_feedback(message.data))
                                elif address == REG_MODE:
                                    self._on_control_mode(
                                        decode_register_u16(message.data))
                                elif address == REG_MOTOR_STATUS:
                                    self._on_motor_status(
                                        decode_register_u16(message.data))
                                elif address == REG_FAULT_STATUS:
                                    self._on_fault_status(
                                        decode_register_u16(message.data))
                                elif address == REG_FAULT_STATUS_B:
                                    self._on_fault_status_b(
                                        decode_register_u16(message.data))
                                elif address == REG_USER_CURRENT:
                                    self._on_user_current(
                                        decode_register_u16(message.data))
                                elif address == REG_BUS_VOLTAGE:
                                    self._set_scalar_telemetry(
                                        '_bus_voltage_raw',
                                        '_bus_voltage_monotonic',
                                        decode_register_u16(message.data))
                                elif address == REG_SUPPLY_TEMPERATURE:
                                    self._set_scalar_telemetry(
                                        '_temperature_raw',
                                        '_temperature_monotonic',
                                        decode_register_u16(message.data))
                                elif address == REG_MAX_SPEED:
                                    self._set_scalar_telemetry(
                                        '_max_speed_register',
                                        '_max_speed_monotonic',
                                        decode_register_u16(message.data))
                                elif address == REG_MAX_CURRENT:
                                    self._set_scalar_telemetry(
                                        '_max_current_register',
                                        '_max_current_monotonic',
                                        decode_register_u16(message.data))
                                elif address == REG_ENABLE:
                                    self._set_scalar_telemetry(
                                        '_enable_register',
                                        '_enable_register_monotonic',
                                        decode_register_u16(message.data))
                                elif address == REG_SPEED:
                                    self._set_scalar_telemetry(
                                        '_speed_register',
                                        '_speed_register_monotonic',
                                        decode_register_u16(message.data))
                                elif address == REG_TORQUE:
                                    self._set_scalar_telemetry(
                                        '_torque_register',
                                        '_torque_register_monotonic',
                                        decode_register_u16(message.data))
                                elif address == REG_ACCEL_RAMP:
                                    self._set_scalar_telemetry(
                                        '_ramp_register',
                                        '_ramp_register_monotonic',
                                        decode_register_u16(message.data))
                            except ValueError:
                                continue
                except (ZqwlError, OSError, ValueError) as exc:
                    now = time.monotonic()
                    if now - self._last_io_error_log >= 2.0:
                        self._last_io_error_log = now
                        self.get_logger().warn(
                            f'Gripper CAN feedback failed: {exc}')
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def _register_address(self, can_id: int) -> Optional[int]:
        address = (can_id >> 8) & 0xFFFF
        device = can_id & 0x7F
        if device != self._device_id:
            return None
        if address not in {
                REG_FEEDBACK,
                REG_TARGET_POSITION,
                REG_MODE,
                REG_MOTOR_STATUS,
                REG_FAULT_STATUS,
                REG_FAULT_STATUS_B,
                REG_USER_CURRENT,
                REG_BUS_VOLTAGE,
                REG_SUPPLY_TEMPERATURE,
                REG_MAX_SPEED,
                REG_MAX_CURRENT,
                REG_ENABLE,
                REG_SPEED,
                REG_TORQUE,
                REG_ACCEL_RAMP}:
            return None
        return address

    def _set_scalar_telemetry(
            self,
            value_attribute: str,
            monotonic_attribute: str,
            value: int) -> None:
        with self._state_lock:
            setattr(self, value_attribute, value)
            setattr(self, monotonic_attribute, time.monotonic())

    def _on_controller_target(self, position: int) -> None:
        with self._state_lock:
            self._controller_target_position = position
            self._controller_target_monotonic = time.monotonic()

    def _on_control_mode(self, mode: int) -> None:
        with self._state_lock:
            self._control_mode = mode
            self._control_mode_monotonic = time.monotonic()

    def _on_motor_status(self, bits: int) -> None:
        now = time.monotonic()
        with self._state_lock:
            previous = self._motor_status_bits
            self._motor_status_bits = bits
            self._motor_status_monotonic = now
        torque_reached = bits & MOTOR_STATUS_TORQUE_REACHED
        zero_speed = bits & MOTOR_STATUS_ZERO_SPEED
        if torque_reached and not (previous or 0) & MOTOR_STATUS_TORQUE_REACHED:
            self.get_logger().info(
                'Gripper reports torque reached'
                + (' at zero speed' if zero_speed else '')
                + f' (0x0401=0x{bits:04X}, feedback={self._position})')

    def _on_fault_status(self, bits: int) -> None:
        now = time.monotonic()
        with self._state_lock:
            previous = self._fault_bits
            self._fault_bits = bits
            self._fault_status_monotonic = now
        if bits and bits != previous:
            self.get_logger().warn(
                f'Gripper controller fault register 0x0402=0x{bits:04X}')

    def _on_fault_status_b(self, bits: int) -> None:
        now = time.monotonic()
        with self._state_lock:
            previous = self._fault_bits_b
            self._fault_bits_b = bits
            self._fault_status_b_monotonic = now
        if bits and bits != previous:
            self.get_logger().warn(
                f'Gripper controller alarm register 0x0403=0x{bits:04X}'
                + (' (drop alarm)' if bits & FAULT_STATUS_B_DROP_ALARM else ''))

    def _on_user_current(self, current: int) -> None:
        with self._state_lock:
            self._user_current = current
            self._user_current_monotonic = time.monotonic()

    def _on_feedback(self, position: int) -> None:
        if self._stop_event.is_set():
            return
        now = time.monotonic()
        if not self._position_min <= position <= self._position_max:
            self._set_fault(
                f'feedback position {position} outside '
                f'{self._position_min}..{self._position_max}')
            if now - self._last_feedback_error_log >= 2.0:
                self._last_feedback_error_log = now
                self.get_logger().error(
                    f'Rejected gripper feedback position {position}; '
                    f'expected {self._position_min}..{self._position_max}')
            return

        with self._state_lock:
            self._position = position
            self._feedback_monotonic = now
            within_tolerance = (
                self._motion_active
                and self._motion_target_position is not None
                and abs(position - self._motion_target_position)
                <= self._position_tolerance
            )
            if within_tolerance:
                if self._motion_in_tolerance_since <= 0.0:
                    self._motion_in_tolerance_since = now
            else:
                self._motion_in_tolerance_since = 0.0
            status_age = now - self._motor_status_monotonic
            zero_speed = bool(
                self._motor_status_bits is not None
                and status_age <= max(0.25, 5.0 / self._feedback_rate)
                and self._motor_status_bits & MOTOR_STATUS_ZERO_SPEED)
            settled = bool(
                within_tolerance
                and self._motion_in_tolerance_since > 0.0
                and (
                    zero_speed
                    or now - self._motion_in_tolerance_since
                    >= MOTION_SETTLE_CONFIRM_SEC))
            if settled:
                self._motion_active = False
                self._motion_target_position = None
                self._motion_source = ''
                self._motion_start_position = None
                self._motion_in_tolerance_since = 0.0
                self._motion_is_closing = False
                self._motion_stalled = False
                self._motion_result = (
                    f'position reached: target={self._last_command_position}, '
                    f'feedback={position}')
                self._last_motion_reached = True
                self._last_motion_stalled = False
                self._last_motion_failed = False
                if self._state == 'moving':
                    self._state = 'enabled' if self._enabled else 'disabled'
            else:
                contact = bool(
                    self._motion_active
                    and self._motion_is_closing
                    and self._motion_target_position is not None
                    and self._motion_start_position is not None
                    and position - self._motion_start_position
                    > max(self._position_tolerance // 2, 5)
                    and self._motor_status_bits is not None
                    and status_age <= max(0.25, 5.0 / self._feedback_rate)
                    and self._contact_status_matches(
                        self._motor_status_bits, status_age)
                )
                if contact:
                    target = self._motion_target_position
                    self._motion_active = False
                    self._motion_target_position = None
                    self._motion_source = ''
                    self._motion_start_position = None
                    self._motion_in_tolerance_since = 0.0
                    self._motion_is_closing = False
                    self._motion_stalled = True
                    self._motion_result = (
                        'closed on object or mechanical stop; '
                        f'target={target}, feedback={position}')
                    self._last_motion_reached = False
                    self._last_motion_stalled = True
                    self._last_motion_failed = False
                    if self._state == 'moving':
                        self._state = 'enabled' if self._enabled else 'disabled'
                    self.get_logger().info(
                        f'Gripper contact detected: target={target}, '
                        f'feedback={position}')
            if (self._state == 'disabled' and self._allow_motion
                    and not self._motion_active):
                self._state = 'ready'
        self._joint_rad = joint_from_position(
            position,
            position_min=self._position_min,
            position_max=self._position_max,
            opening_min_mm=self._opening_min,
            opening_max_mm=self._opening_max,
            joint_open_rad=self._joint_open,
            joint_closed_rad=self._joint_closed,
        )
        opening_mm = opening_from_position(
            position,
            position_min=self._position_min,
            position_max=self._position_max,
            opening_min_mm=self._opening_min,
            opening_max_mm=self._opening_max,
        )
        stamp = self.get_clock().now().to_msg()

        position_msg = Int32()
        position_msg.data = int(position)

        width_msg = Float32()
        width_msg.data = float(opening_mm / 1000.0)

        state = JointState()
        state.header.stamp = stamp
        state.name = [self._joint_name]
        state.position = [float(self._joint_rad)]
        try:
            self._position_pub.publish(position_msg)
            self._width_pub.publish(width_msg)
            self._state_pub.publish(state)
            self._joint_states_pub.publish(state)
        except Exception:
            if not self._stop_event.is_set():
                raise

    def _feedback_is_fresh(self) -> bool:
        now = time.monotonic()
        return (
            self._position is not None
            and now - self._feedback_monotonic
            <= self._feedback_timeout
        )

    def _closed_stall_detected(self) -> bool:
        """Return true when a closing command has reached a hard stop/grasp.

        The vendor reports ``torque reached + zero speed`` when the jaws meet
        an object or the mechanical stop.  That is normal gripper behavior,
        not a driver fault, so the motion layer must not latch a timeout.
        """
        with self._state_lock:
            bits = self._motor_status_bits
            status_monotonic = self._motor_status_monotonic
            position = self._position
            target = self._motion_target_position
            start_position = self._motion_start_position
            closing = self._motion_is_closing
        if (
            not closing
            or position is None
            or target is None
            or start_position is None
            or bits is None
        ):
            return False
        status_age = time.monotonic() - status_monotonic
        if status_age > max(0.25, 5.0 / self._feedback_rate):
            return False
        if abs(position - target) <= self._position_tolerance:
            return False
        # Zero speed plus torque reached is also reported when the motor never
        # starts.  Only classify it as a grasp after the jaws have actually
        # travelled in the closing direction during this command.
        closing_progress = position - start_position
        if closing_progress <= max(self._position_tolerance // 2, 5):
            return False
        return self._contact_status_matches(bits, status_age)

    def _contact_status_matches(self, bits: int, status_age: float) -> bool:
        """Return true only for a confirmed stalled contact status.

        Zero speed and torque reached are also visible briefly while a new
        move starts, and a stale status from the previous command still has
        POSITION_REACHED set.  A real grasp or mechanical stop has not reached
        the new target, so only the 0x001C combination is unambiguous.
        """
        if bits is None or status_age > max(0.25, 5.0 / self._feedback_rate):
            return False
        contact_bits = MOTOR_STATUS_TORQUE_REACHED | MOTOR_STATUS_ZERO_SPEED
        return bool(
            (bits & contact_bits) == contact_bits
            and not bits & MOTOR_STATUS_POSITION_REACHED)

    def _set_fault(self, reason: str) -> None:
        with self._state_lock:
            first_fault = self._fault_reason is None
            if first_fault:
                self._fault_reason = reason
            self._last_error = reason
            self._state = 'fault'
            self._motion_active = False
            self._motion_target_position = None
            self._motion_source = ''
            self._motion_start_position = None
            self._motion_is_closing = False
            self._motion_stalled = False
            self._motion_result = reason
            self._last_motion_failed = True
            self._startup_enable_pending = False
        if first_fault:
            self.get_logger().error(f'Gripper fault latched: {reason}')

    def _safe_disable(self, reason: str, force: bool = False) -> bool:
        with self._motion_start_lock:
            return self._safe_disable_locked(reason, force=force)

    def _safe_disable_locked(self, reason: str, force: bool = False) -> bool:
        self._set_fault(reason)
        now = time.monotonic()
        if not force:
            with self._state_lock:
                retry = now - self._safe_disable_last_attempt >= 1.0
                self._safe_disable_last_attempt = now
            if not retry:
                return False
        else:
            with self._state_lock:
                self._safe_disable_last_attempt = now
        if self._client is None:
            with self._state_lock:
                self._enabled = False
            return False
        return self._write_enable(False)

    def _write_enable(self, enabled: bool) -> bool:
        client = self._client
        if client is None:
            return False
        frame = enable_frame(
            enabled,
            device_id=self._device_id,
            channel=self._channel,
            fd=self._can_fd,
            brs=self._brs,
        )
        try:
            with self._io_lock:
                client.send(frame)
        except (ZqwlError, OSError) as exc:
            with self._state_lock:
                self._last_error = f'Cannot write gripper enable: {exc}'
            if rclpy.ok():
                self.get_logger().error(f'Cannot write gripper enable: {exc}')
            if enabled:
                self._set_fault(f'enable write failed: {exc}')
            return False
        with self._state_lock:
            self._enabled = enabled
            if enabled:
                if self._state != 'fault':
                    self._state = 'enabled'
                self._startup_enable_pending = False
            else:
                self._motion_active = False
                self._motion_target_position = None
                self._motion_source = ''
                self._motion_start_position = None
                self._motion_is_closing = False
                self._motion_stalled = False
                if self._state != 'fault':
                    self._state = 'disabled'
        if rclpy.ok():
            self.get_logger().info(
                'Gripper motor enable requested: '
                f'{"on" if enabled else "off"}')
        return True

    def _write_alarm_reset(self) -> bool:
        client = self._client
        if client is None:
            return False
        frame = alarm_reset_frame(
            device_id=self._device_id,
            channel=self._channel,
            fd=self._can_fd,
            brs=self._brs,
        )
        try:
            with self._io_lock:
                client.send(frame)
        except (ZqwlError, OSError) as exc:
            with self._state_lock:
                self._last_error = f'Cannot reset gripper alarm: {exc}'
            self.get_logger().error(
                f'Cannot reset gripper hardware alarm: {exc}')
            return False
        return True

    def _write_motion_config(self) -> bool:
        client = self._client
        if client is None:
            return False
        with self._state_lock:
            speed = self._motion_speed_percent
            torque = self._motion_torque_percent
            ramp = self._motion_ramp_ms
            max_current = self._max_current
        config_frames = motion_config_frames(
            speed, torque, ramp,
            device_id=self._device_id,
            channel=self._channel,
            fd=self._can_fd,
            brs=self._brs,
        )
        config_frames = (
            scalar_write_frame(
                REG_MAX_CURRENT, max_current,
                self._device_id, self._channel, self._can_fd, self._brs),
            *config_frames,
        )
        try:
            with self._io_lock:
                for frame in config_frames:
                    client.send(frame)
        except (ZqwlError, OSError) as exc:
            self.get_logger().error(
                f'Cannot write gripper motion configuration: {exc}')
            return False
        return True

    def _write_motion(self, position: int) -> bool:
        client = self._client
        if client is None:
            return False
        if not self._write_motion_config():
            return False
        position_frame, trigger_frame = motion_frames(
            position,
            device_id=self._device_id,
            channel=self._channel,
            fd=self._can_fd,
            brs=self._brs,
        )
        try:
            with self._io_lock:
                client.send(position_frame)
                client.send(trigger_frame)
        except (ZqwlError, OSError) as exc:
            self.get_logger().error(
                f'Cannot write gripper position {position}: {exc}')
            return False
        with self._state_lock:
            self._last_command_position = position
        return True

    def _request_enable(self, enabled: bool, source: str = 'service'):
        with self._motion_start_lock:
            return self._request_enable_locked(enabled, source=source)

    def _request_enable_locked(self, enabled: bool, source: str = 'service'):
        if enabled:
            with self._state_lock:
                if self._estop_latched:
                    reason = 'emergency stop is latched; clear the fault first'
                elif self._fault_reason is not None:
                    reason = f'fault latched: {self._fault_reason}'
                elif not self._allow_motion:
                    reason = 'allow_motion is false'
                elif self._motion_active:
                    reason = 'a gripper motion is already active'
                elif self._require_feedback and not self._feedback_is_fresh():
                    reason = 'no fresh 0x0418 feedback'
                else:
                    reason = None
            if reason is not None:
                self.get_logger().warn(
                    f'Gripper enable rejected from {source}: {reason}')
                return False, reason

        if not self._write_enable(enabled):
            return False, 'CAN enable write failed'

        with self._state_lock:
            self._enabled = enabled
            if enabled:
                self._state = 'enabled'
            else:
                self._motion_active = False
                self._motion_target_position = None
                self._motion_source = ''
                self._motion_start_position = None
                self._motion_is_closing = False
                self._motion_stalled = False
                self._state = 'fault' if self._fault_reason else 'disabled'
        return True, 'enable command sent' if enabled else 'disable command sent'

    def _start_motion(self, position: int, source: str) -> tuple[bool, str]:
        if not self._position_min <= position <= self._command_max:
            reason = (
                f'expected {self._position_min}..{self._command_max}, '
                f'got {position}')
            self.get_logger().warn(f'Gripper motion rejected: {reason}')
            return False, reason
        if not self._motion_start_lock.acquire(timeout=0.25):
            reason = 'another gripper command is being processed'
            self.get_logger().warn(f'Gripper motion rejected: {reason}')
            return False, reason
        try:
            with self._state_lock:
                retarget = (
                    self._motion_active and source in {'topic', 'speed'})
                if self._estop_latched:
                    reason = 'emergency stop is latched; clear the fault first'
                elif self._fault_reason is not None:
                    reason = f'fault latched: {self._fault_reason}'
                elif not self._allow_motion:
                    reason = 'allow_motion is false'
                elif self._motion_active and not retarget:
                    reason = 'a gripper motion is already active'
                elif (
                    self._require_feedback
                    and not self._feedback_is_fresh()
                ):
                    reason = 'no fresh 0x0418 feedback'
                elif not self._enabled and not self._auto_enable_on_command:
                    reason = (
                        'motor is not enabled; call '
                        f'/{self._side}_gripper/enable first')
                else:
                    reason = None
            if reason is not None:
                self.get_logger().warn(
                    f'Gripper motion rejected from {source}: {reason}')
                return False, reason

            with self._state_lock:
                if (
                    self._motion_active
                    and self._motion_target_position == position
                ):
                    # Position controllers and VLA policies commonly publish
                    # the same target at tens of hertz.  Treating every repeat
                    # as a fresh move would reset the contact/travel baseline
                    # and turn a valid grasp into a timeout.
                    return True, 'target is already active'
                if (
                    not self._motion_active
                    and self._last_command_position == position
                    and (
                        self._last_motion_reached
                        or self._last_motion_stalled
                    )
                ):
                    # A completed target may also be republished by a
                    # controller. Do not restart a successful grasp or an
                    # intentional contact stop until a different target is
                    # requested.
                    return True, 'target already completed'

            if not self._enabled:
                ok, message = self._request_enable(
                    True, source=f'{source} auto-enable')
                if not ok:
                    return False, message

            with self._state_lock:
                start_position = self._position
                distance = (
                    abs(position - start_position)
                    if start_position is not None else 0)
                self._motion_active = True
                self._motion_target_position = position
                self._last_command_position = position
                self._motion_deadline = (
                    time.monotonic() + self._motion_timeout_for(distance))
                self._motion_source = source
                self._motion_start_position = start_position
                self._motion_is_closing = (
                    start_position is not None and position > start_position)
                self._motion_in_tolerance_since = 0.0
                self._motion_result = ''
                self._motion_stalled = False
                self._last_motion_reached = False
                self._last_motion_stalled = False
                self._last_motion_failed = False
                self._state = 'moving'
            if not self._write_motion(position):
                with self._state_lock:
                    self._motion_active = False
                    self._motion_target_position = None
                    self._motion_source = ''
                    self._motion_start_position = None
                    self._motion_is_closing = False
                self._safe_disable('motion write failed', force=True)
                return False, 'CAN motion write failed'
            self.get_logger().info(
                f'Gripper position command: {position} '
                f'({"retargeted" if retarget else "new"}, source={source})')
            return True, 'motion command sent'
        finally:
            self._motion_start_lock.release()

    def _finish_motion(
        self,
        reason: str,
        is_error: bool = False,
        disable_motor: bool = False,
        stalled: bool = False,
        reached: bool = False,
    ) -> None:
        with self._motion_start_lock:
            with self._state_lock:
                was_active = self._motion_active
                self._motion_active = False
                self._motion_target_position = None
                self._motion_source = ''
                self._motion_start_position = None
                self._motion_in_tolerance_since = 0.0
                self._motion_is_closing = False
                self._motion_result = reason
                self._motion_stalled = stalled
                self._last_motion_reached = reached
                self._last_motion_stalled = stalled
                self._last_motion_failed = is_error
                self._last_error = reason if is_error else None
                if self._state == 'moving':
                    self._state = (
                        'fault' if self._fault_reason is not None
                        else ('enabled' if self._enabled else 'disabled'))
            if is_error:
                self.get_logger().warn(reason)
            if disable_motor and was_active:
                self._safe_disable(reason, force=True)

    def _command_callback(self, msg: Int32) -> None:
        position = int(msg.data)
        self._start_motion(position, source='topic')

    def _speed_callback(self, msg: Int32) -> None:
        self._set_runtime_speed(int(msg.data), source='speed topic')

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        response.success, response.message = self._request_enable(
            bool(request.data), source='enable service')
        return response

    def _estop_callback(self, _request, response):
        with self._motion_start_lock:
            with self._state_lock:
                self._estop_latched = True
                self._state = 'fault'
                self._fault_reason = 'emergency stop'
                self._motion_active = False
                self._motion_target_position = None
                self._motion_source = ''
                self._motion_start_position = None
                self._motion_is_closing = False
                self._motion_stalled = False
                self._motion_result = 'emergency stop'
                self._last_motion_failed = True
                self._startup_enable_pending = False
            disabled = self._safe_disable('emergency stop', force=True)
        self.get_logger().error(
            f'Gripper emergency stop latched; disable_sent={disabled}')
        return response

    def _clear_fault_callback(self, _request, response):
        with self._motion_start_lock:
            with self._state_lock:
                if self._enabled:
                    response.success = False
                    response.message = (
                        'disable the motor before clearing faults')
                    return response
                if self._require_feedback and not self._feedback_is_fresh():
                    response.success = False
                    response.message = (
                        'cannot clear fault without fresh feedback')
                    return response
            if not self._write_alarm_reset():
                response.success = False
                response.message = (
                    'cannot write hardware alarm reset (0x0110)')
                return response
            with self._state_lock:
                had_fault = (
                    self._fault_reason is not None or self._estop_latched)
                self._fault_reason = None
                self._last_error = None
                self._estop_latched = False
                self._motion_active = False
                self._motion_target_position = None
                self._motion_source = ''
                self._motion_start_position = None
                self._motion_is_closing = False
                self._motion_stalled = False
                self._motion_result = ''
                self._last_motion_reached = False
                self._last_motion_stalled = False
                self._last_motion_failed = False
                self._state = 'ready' if self._allow_motion else 'disabled'
        response.success = True
        response.message = (
            'fault cleared and hardware alarm reset sent; motor remains disabled'
            if had_fault else 'no fault was latched')
        return response

    def _clear_obstruction_callback(self, _request, response):
        """Stop, reset the hardware alarm and move the jaws back open.

        This is the one-step recovery operation used by the direct gripper
        interface.  It deliberately disables the motor before touching the
        vendor alarm register, then issues an open command through the normal
        safety gate.  The motor is therefore never re-enabled while a stale
        e-stop or controller fault is still latched.
        """
        with self._motion_start_lock:
            with self._state_lock:
                had_fault = (
                    self._fault_reason is not None or self._estop_latched)
                position = self._position
            if self._enabled:
                # Stop cleanly first without injecting a synthetic fault.
                # A real fault/e-stop remains latched until the reset below.
                self._write_enable(False)
            with self._state_lock:
                if self._enabled:
                    response.success = False
                    response.message = (
                        'cannot clear obstruction: motor disable was not '
                        'confirmed')
                    return response
                if self._require_feedback and not self._feedback_is_fresh():
                    response.success = False
                    response.message = (
                        'cannot clear obstruction without fresh feedback')
                    return response
            if not self._write_alarm_reset():
                response.success = False
                response.message = (
                    'cannot clear obstruction: hardware alarm reset '
                    '(0x0110) write failed')
                return response
            with self._state_lock:
                self._fault_reason = None
                self._last_error = None
                self._estop_latched = False
                self._motion_active = False
                self._motion_target_position = None
                self._motion_source = ''
                self._motion_start_position = None
                self._motion_is_closing = False
                self._motion_stalled = False
                self._motion_result = ''
                self._last_motion_reached = False
                self._last_motion_stalled = False
                self._last_motion_failed = False
                self._state = 'ready' if self._allow_motion else 'disabled'

            if not self._allow_motion:
                response.success = True
                response.message = (
                    'obstruction latch cleared and hardware alarm reset; '
                    'motion is disabled by allow_motion=false')
                return response
            if position is not None and position <= max(1, self._position_min):
                response.success = True
                response.message = (
                    'obstruction latch cleared; gripper is already open')
                return response
            if not self._enabled:
                enabled, detail = self._request_enable(
                    True, source='clear obstruction')
                if not enabled:
                    response.success = False
                    response.message = (
                        'obstruction latch cleared, but motor enable for the '
                        f'opening move failed: {detail}')
                    return response
            opened, detail = self._start_motion(
                0, source='clear_obstruction')
            response.success = opened
            if opened:
                response.message = (
                    'obstruction latch cleared and hardware alarm reset; '
                    'open command sent')
            else:
                response.message = (
                    'obstruction latch cleared, but the open command was '
                    f'rejected: {detail}')
        if had_fault:
            self.get_logger().warn(
                'Gripper obstruction recovery requested after a latched fault')
        return response

    def _watchdog_tick(self) -> None:
        now = time.monotonic()
        disable_reason: Optional[str] = None
        timeout_reason: Optional[str] = None
        stall_reason: Optional[str] = None
        startup_enable = False
        with self._state_lock:
            if self._enabled and not self._feedback_is_fresh():
                disable_reason = 'feedback timeout while motor enabled'
            elif self._estop_latched and self._enabled:
                disable_reason = 'emergency stop'
            elif self._fault_reason is not None and self._enabled:
                disable_reason = self._fault_reason

            if self._motion_active and now > self._motion_deadline:
                source = self._motion_source or 'unknown'
                target = self._motion_target_position
                if (
                    self._treat_closed_stall_as_grasp
                    and self._motion_is_closing
                    and self._closed_stall_detected()
                ):
                    stall_reason = (
                        f'closed on object or mechanical stop from {source}; '
                        f'target={target}, feedback={self._position}')
                else:
                    timeout_reason = (
                        f'motion timeout from {source}; target={target}, '
                        f'feedback={self._position}')

            if (
                timeout_reason is None
                and disable_reason is None
                and self._startup_enable_pending
                and not self._enabled
                and not self._estop_latched
                and self._fault_reason is None
                and self._feedback_is_fresh()
            ):
                startup_enable = True

        if timeout_reason is not None:
            self._finish_motion(
                timeout_reason,
                is_error=True,
                disable_motor=self._latch_motion_timeout)
            return
        if stall_reason is not None:
            self._finish_motion(
                stall_reason,
                is_error=False,
                disable_motor=False,
                stalled=True,
                reached=False)
            return
        if disable_reason is not None:
            self._safe_disable(disable_reason)
            return
        if startup_enable:
            ok, message = self._request_enable(True, source='startup')
            if not ok:
                self.get_logger().warn(
                    f'Startup gripper enable deferred: {message}')

    def _goal_callback(self, goal_request):
        if not self._allow_motion:
            self.get_logger().warn(
                'Rejecting gripper goal because allow_motion is false')
            return GoalResponse.REJECT
        with self._state_lock:
            if self._estop_latched or self._fault_reason is not None:
                self.get_logger().warn(
                    'Rejecting gripper goal while a fault is latched')
                return GoalResponse.REJECT
            if self._motion_active:
                self.get_logger().warn(
                    'Rejecting gripper goal while another motion is active')
                return GoalResponse.REJECT
        effort = float(goal_request.command.max_effort)
        if effort > 0.0 and not self._allow_nonzero_effort:
            self.get_logger().warn(
                'Rejecting gripper goal: max_effort is not implemented; '
                'send max_effort=0 or start with allow_nonzero_effort:=true')
            return GoalResponse.REJECT
        target = float(goal_request.command.position)
        if not math.isfinite(target):
            return GoalResponse.REJECT
        lower, upper = sorted((self._joint_open, self._joint_closed))
        if target < lower - 1e-3 or target > upper + 1e-3:
            self.get_logger().warn(
                f'Rejecting gripper goal {target:.4f} rad outside '
                f'[{lower:.4f}, {upper:.4f}]')
            return GoalResponse.REJECT
        command_closed = joint_from_position(
            self._command_max,
            position_min=self._position_min,
            position_max=self._position_max,
            opening_min_mm=self._opening_min,
            opening_max_mm=self._opening_max,
            joint_open_rad=self._joint_open,
            joint_closed_rad=self._joint_closed,
        )
        if target < command_closed - 1e-3:
            self.get_logger().warn(
                f'Rejecting gripper goal {target:.4f} rad: command position '
                f'is limited to {self._command_max} '
                f'({command_closed:.4f} rad)')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        result = GripperCommand.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.reached_goal = False
            return result
        target_rad = float(goal_handle.request.command.position)
        target_position = position_from_joint(
            target_rad,
            position_min=self._position_min,
            position_max=self._position_max,
            opening_min_mm=self._opening_min,
            opening_max_mm=self._opening_max,
            joint_open_rad=self._joint_open,
            joint_closed_rad=self._joint_closed,
        )
        target_position = min(target_position, self._command_max)
        started, message = self._start_motion(target_position, source='action')
        if not started:
            self.get_logger().warn(f'Gripper goal rejected: {message}')
            goal_handle.abort()
            result.reached_goal = False
            return result

        with self._state_lock:
            deadline = self._motion_deadline
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                cancel_reason = (
                    'action canceled; gripper motor was disabled'
                    if self._cancel_disables_motor
                    else 'action canceled; CAN target is not retracted')
                self._finish_motion(
                    cancel_reason,
                    is_error=True,
                    disable_motor=self._cancel_disables_motor)
                goal_handle.canceled()
                result.reached_goal = False
                return result
            with self._state_lock:
                active = self._motion_active
                reached = self._last_motion_reached
                stalled = self._last_motion_stalled
                failed = self._last_motion_failed
                motion_result = self._motion_result
            if not active:
                result.position = (
                    float(self._joint_rad)
                    if self._joint_rad is not None else 0.0)
                result.reached_goal = bool(reached)
                result.stalled = bool(stalled)
                if failed or not (reached or stalled):
                    goal_handle.abort()
                    self.get_logger().warn(
                        'Gripper goal aborted: '
                        f'{motion_result or "motion ended before reaching the target"}')
                else:
                    goal_handle.succeed()
                    if stalled:
                        self.get_logger().info(
                            f'Gripper goal stalled at contact: {motion_result}')
                return result
            if time.monotonic() >= deadline:
                self._watchdog_tick()
                if not self._motion_active:
                    continue
            time.sleep(0.02)

        result.position = (
            float(self._joint_rad) if self._joint_rad is not None else 0.0)
        result.reached_goal = False
        result.stalled = False
        goal_handle.abort()
        return result

    def close(self) -> None:
        self._stop_event.set()
        io_thread = self._io_thread
        if io_thread is not None:
            try:
                io_thread.join(timeout=1.0)
            except KeyboardInterrupt:
                pass
        if self._disable_on_shutdown and self._client is not None:
            for attempt in range(2):
                try:
                    self._write_enable(False)
                    break
                except KeyboardInterrupt:
                    if rclpy.ok():
                        if attempt == 0:
                            self.get_logger().warn(
                                'Shutdown interrupted while disabling gripper '
                                'motor; retrying')
                        else:
                            self.get_logger().warn(
                                'Shutdown interrupted again; disabling the CAN '
                                'channel without another motor-enable write')
        with self._io_lock:
            client = self._client
            self._client = None
            if client is None:
                return
            try:
                client.disable_can(self._channel)
            except (ZqwlError, OSError, KeyboardInterrupt) as exc:
                if rclpy.ok():
                    self.get_logger().warn(
                        f'Cannot disable CAN channel: {exc}')
            finally:
                client.close()


def main(args=None, side: str = 'right') -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = CtagGripperNode(side=side)
        executor = MultiThreadedExecutor(num_threads=3)
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
