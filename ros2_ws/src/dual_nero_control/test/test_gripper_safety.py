from __future__ import annotations

import threading
import time
import unittest
from unittest.mock import patch

from std_srvs.srv import Empty, Trigger
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter
from std_msgs.msg import Int32

from dual_nero_control.ctag_gripper import joint_from_position
from dual_nero_control.right_gripper_node import CtagGripperNode


class _Logger:
    def __init__(self) -> None:
        self.messages = []

    def info(self, message: str, **_kwargs) -> None:
        self.messages.append(('info', message))

    def warn(self, message: str, **_kwargs) -> None:
        self.messages.append(('warn', message))

    def error(self, message: str, **_kwargs) -> None:
        self.messages.append(('error', message))


class _GoalCommand:
    def __init__(self, position: float) -> None:
        self.position = position
        self.max_effort = 0.0


class _GoalRequest:
    def __init__(self, position: float) -> None:
        self.command = _GoalCommand(position)


class _GoalHandle:
    def __init__(self, position: float) -> None:
        self.request = _GoalRequest(position)
        self.is_cancel_requested = False
        self.final_status = None

    def abort(self) -> None:
        self.final_status = 'abort'

    def succeed(self) -> None:
        self.final_status = 'succeed'

    def canceled(self) -> None:
        self.final_status = 'canceled'


class _SafetyHarness(CtagGripperNode):
    """Exercise the driver safety state machine without opening hardware."""

    def __init__(self) -> None:
        self._side = 'right'
        self._state_lock = threading.RLock()
        self._motion_start_lock = threading.RLock()
        self._allow_motion = True
        self._allow_nonzero_effort = False
        self._auto_enable_on_command = False
        self._require_feedback = True
        self._feedback_timeout = 1.0
        self._feedback_rate = 20.0
        self._goal_timeout = 3.0
        self._latch_motion_timeout = True
        self._cancel_disables_motor = True
        self._position_tolerance = 25
        self._treat_closed_stall_as_grasp = True
        self._position_min = 0
        self._position_max = 1000
        self._command_max = 990
        self._position = 0
        self._joint_rad = 0.0
        self._joint_open = 0.0
        self._joint_closed = -0.90838
        self._opening_min = 0.0
        self._opening_max = 90.0
        self._feedback_monotonic = time.monotonic()
        self._feedback_fresh = True
        self._controller_target_position = None
        self._controller_target_monotonic = 0.0
        self._control_mode = None
        self._control_mode_monotonic = 0.0
        self._motor_status_bits = None
        self._motor_status_monotonic = 0.0
        self._fault_bits = None
        self._fault_status_monotonic = 0.0
        self._fault_bits_b = None
        self._fault_status_b_monotonic = 0.0
        self._user_current = None
        self._user_current_monotonic = 0.0
        self._bus_voltage_raw = None
        self._bus_voltage_monotonic = 0.0
        self._temperature_raw = None
        self._temperature_monotonic = 0.0
        self._max_speed_register = None
        self._max_speed_monotonic = 0.0
        self._max_current_register = None
        self._max_current_monotonic = 0.0
        self._enable_register = None
        self._enable_register_monotonic = 0.0
        self._speed_register = None
        self._speed_register_monotonic = 0.0
        self._torque_register = None
        self._torque_register_monotonic = 0.0
        self._ramp_register = None
        self._ramp_register_monotonic = 0.0
        self._enabled = False
        self._state = 'ready'
        self._fault_reason = None
        self._last_error = None
        self._estop_latched = False
        self._motion_active = False
        self._motion_target_position = None
        self._motion_deadline = 0.0
        self._motion_source = ''
        self._motion_start_position = None
        self._motion_in_tolerance_since = 0.0
        self._motion_is_closing = False
        self._motion_result = ''
        self._motion_stalled = False
        self._last_motion_reached = False
        self._last_motion_stalled = False
        self._last_motion_failed = False
        self._last_command_position = None
        self._startup_enable_pending = False
        self._safe_disable_last_attempt = 0.0
        self._client = object()
        self._port = '/dev/null'
        self._motion_speed_percent = 25
        self._motion_torque_percent = 100
        self._motion_ramp_ms = 200
        self._max_current = 200
        self._logger = _Logger()
        self.enable_writes = []
        self.alarm_resets = 0
        self.motion_writes = []
        self.config_writes = 0
        self.fail_motion_write = False

    def get_logger(self):
        return self._logger

    def _feedback_is_fresh(self) -> bool:
        return self._feedback_fresh

    def _write_enable(self, enabled: bool) -> bool:
        self.enable_writes.append(enabled)
        if not enabled:
            self._enabled = False
        return True

    def _write_alarm_reset(self) -> bool:
        self.alarm_resets += 1
        return True

    def _write_motion_config(self) -> bool:
        self.config_writes += 1
        return True

    def _write_motion(self, position: int) -> bool:
        if self.fail_motion_write:
            return False
        self.motion_writes.append(position)
        return True


class _FakeIoThread:
    def __init__(self, events: list) -> None:
        self.events = events

    def join(self, timeout=None) -> None:
        self.events.append(('join', timeout))


class _FakeCanClient:
    def __init__(self, events: list) -> None:
        self.events = events

    def disable_can(self, channel: int) -> None:
        self.events.append(('disable_can', channel))

    def close(self) -> None:
        self.events.append(('close',))


class _CloseHarness(_SafetyHarness):
    def __init__(self, interrupt_writes: int) -> None:
        super().__init__()
        self.events = []
        self._interrupt_writes = interrupt_writes
        self._disable_on_shutdown = True
        self._channel = 0
        self._stop_event = threading.Event()
        self._io_thread = _FakeIoThread(self.events)
        self._client = _FakeCanClient(self.events)
        self._io_lock = threading.Lock()

    def _write_enable(self, enabled: bool) -> bool:
        self.events.append(('write_enable', enabled))
        if sum(event[0] == 'write_enable' for event in self.events) <= (
                self._interrupt_writes):
            raise KeyboardInterrupt
        self._enabled = enabled
        return True


class GripperSafetyTest(unittest.TestCase):
    def test_close_survives_keyboard_interrupt_and_disables_can(self) -> None:
        node = _CloseHarness(interrupt_writes=1)

        node.close()

        self.assertEqual(
            node.events,
            [
                ('join', 1.0),
                ('write_enable', False),
                ('write_enable', False),
                ('disable_can', 0),
                ('close',),
            ],
        )
        self.assertFalse(node._enabled)

    def test_enable_requires_fresh_feedback(self) -> None:
        node = _SafetyHarness()
        node._feedback_fresh = False

        ok, reason = node._request_enable(True)

        self.assertFalse(ok)
        self.assertIn('fresh', reason)
        self.assertEqual(node.enable_writes, [])

    def test_enable_rejects_latched_emergency_stop(self) -> None:
        node = _SafetyHarness()
        node._estop_latched = True

        ok, reason = node._request_enable(True)

        self.assertFalse(ok)
        self.assertIn('emergency stop', reason)
        self.assertEqual(node.enable_writes, [])

    def test_emergency_stop_latches_and_disables(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'

        node._estop_callback(None, Empty.Response())

        self.assertTrue(node._estop_latched)
        self.assertEqual(node._fault_reason, 'emergency stop')
        self.assertEqual(node._state, 'fault')
        self.assertFalse(node._enabled)
        self.assertEqual(node.enable_writes, [False])

    def test_clear_fault_requires_disabled_and_fresh_feedback(self) -> None:
        node = _SafetyHarness()
        node._estop_latched = True
        node._fault_reason = 'emergency stop'
        node._enabled = True

        response = node._clear_fault_callback(None, Trigger.Response())
        self.assertFalse(response.success)
        self.assertIn('disable', response.message)

        node._enabled = False
        node._feedback_fresh = False
        response = node._clear_fault_callback(None, Trigger.Response())
        self.assertFalse(response.success)
        self.assertIn('feedback', response.message)

        node._feedback_fresh = True
        response = node._clear_fault_callback(None, Trigger.Response())
        self.assertTrue(response.success)
        self.assertEqual(node.alarm_resets, 1)
        self.assertIn('hardware alarm reset', response.message)
        self.assertFalse(node._estop_latched)
        self.assertIsNone(node._fault_reason)

    def test_second_motion_is_rejected(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'

        first, _ = node._start_motion(100, source='test')
        second, reason = node._start_motion(200, source='test')

        self.assertTrue(first)
        self.assertFalse(second)
        self.assertIn('already active', reason)
        self.assertEqual(node.motion_writes, [100])

    def test_topic_command_retargets_active_motion(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'

        first, _ = node._start_motion(100, source='topic')
        second, reason = node._start_motion(200, source='topic')

        self.assertTrue(first)
        self.assertTrue(second, reason)
        self.assertEqual(node.motion_writes, [100, 200])
        self.assertEqual(node._motion_target_position, 200)

    def test_repeated_same_target_is_idempotent(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'

        first, _ = node._start_motion(100, source='topic')
        started_at = node._motion_start_position
        second, reason = node._start_motion(100, source='topic')

        self.assertTrue(first)
        self.assertTrue(second, reason)
        self.assertEqual(reason, 'target is already active')
        self.assertEqual(node.motion_writes, [100])
        self.assertEqual(node._motion_start_position, started_at)

    def test_repeated_completed_target_does_not_restart_motion(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'

        first, _ = node._start_motion(100, source='topic')
        node._finish_motion('position reached', reached=True)
        repeated, reason = node._start_motion(100, source='topic')

        self.assertTrue(first)
        self.assertTrue(repeated, reason)
        self.assertEqual(reason, 'target already completed')
        self.assertFalse(node._motion_active)
        next_target, next_reason = node._start_motion(0, source='topic')
        self.assertTrue(next_target, next_reason)
        self.assertEqual(node.motion_writes, [100, 0])

    def test_repeated_stalled_target_does_not_restart_motion(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'

        first, _ = node._start_motion(900, source='topic')
        node._finish_motion('closed on object', stalled=True)
        repeated, reason = node._start_motion(900, source='topic')

        self.assertTrue(first)
        self.assertTrue(repeated, reason)
        self.assertEqual(reason, 'target already completed')
        self.assertFalse(node._motion_active)
        self.assertEqual(node.motion_writes, [900])

    def test_speed_topic_updates_runtime_speed(self) -> None:
        node = _SafetyHarness()

        node._speed_callback(Int32(data=60))
        self.assertEqual(node._motion_speed_percent, 60)

        node._speed_callback(Int32(data=0))
        self.assertEqual(node._motion_speed_percent, 60)

    def test_position_command_auto_enables_when_configured(self) -> None:
        node = _SafetyHarness()
        node._auto_enable_on_command = True

        ok, reason = node._start_motion(120, source='topic')

        self.assertTrue(ok, reason)
        self.assertTrue(node._enabled)
        self.assertEqual(node.enable_writes, [True])
        self.assertEqual(node.motion_writes, [120])

    def test_clear_obstruction_disables_resets_and_opens(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'fault'
        node._fault_reason = 'opening stalled'
        node._estop_latched = True
        node._position = 500

        response = node._clear_obstruction_callback(
            None, Trigger.Response())

        self.assertTrue(response.success, response.message)
        self.assertEqual(node.enable_writes, [False, True])
        self.assertEqual(node.alarm_resets, 1)
        self.assertFalse(node._estop_latched)
        self.assertIsNone(node._fault_reason)
        self.assertEqual(node.motion_writes, [0])

    def test_motion_requires_enable(self) -> None:
        node = _SafetyHarness()

        ok, reason = node._start_motion(100, source='test')

        self.assertFalse(ok)
        self.assertIn('not enabled', reason)
        self.assertEqual(node.motion_writes, [])

    def test_command_limit_accepts_990(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'

        ok, reason = node._start_motion(990, source='test')

        self.assertTrue(ok, reason)
        self.assertEqual(node.motion_writes, [990])

    def test_command_limit_rejects_991_without_can_write(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'

        ok, reason = node._start_motion(991, source='test')

        self.assertFalse(ok)
        self.assertIn('expected 0..990', reason)
        self.assertEqual(node.motion_writes, [])
        self.assertFalse(node._motion_active)

    def test_action_goal_accepts_command_limit_and_rejects_full_close(
            self) -> None:
        node = _SafetyHarness()

        accepted = node._goal_callback(
            _GoalRequest(joint_from_position(990)))
        rejected = node._goal_callback(_GoalRequest(-0.91))

        self.assertEqual(accepted, GoalResponse.ACCEPT)
        self.assertEqual(rejected, GoalResponse.REJECT)

    def test_action_aborts_when_motion_ends_without_reaching_goal(self) -> None:
        node = _SafetyHarness()
        node._enabled = True

        def fake_start_motion(_position, source):
            self.assertEqual(source, 'action')
            node._motion_active = False
            node._motion_result = (
                'motion timeout from action; target=508, feedback=496')
            node._last_motion_reached = False
            node._last_motion_stalled = False
            node._last_motion_failed = True
            return True, 'motion command sent'

        node._start_motion = fake_start_motion
        goal_handle = _GoalHandle(-0.5)

        with patch(
                'dual_nero_control.right_gripper_node.rclpy.ok',
                return_value=True):
            result = node._execute_callback(goal_handle)

        self.assertEqual(goal_handle.final_status, 'abort')
        self.assertFalse(result.reached_goal)
        self.assertFalse(result.stalled)

    def test_motion_write_failure_latches_fault_and_disables(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'
        node.fail_motion_write = True

        ok, reason = node._start_motion(100, source='test')

        self.assertFalse(ok)
        self.assertIn('write failed', reason)
        self.assertEqual(node._state, 'fault')
        self.assertIn('motion write failed', node._fault_reason)
        self.assertEqual(node.enable_writes, [False])

    def test_goal_timeout_parameter_updates_runtime_value(self) -> None:
        node = _SafetyHarness()

        result = node._on_set_parameters([
            Parameter(
                'goal_timeout_sec', Parameter.Type.DOUBLE, 10.0),
        ])

        self.assertTrue(result.successful)
        self.assertEqual(node._goal_timeout, 10.0)

    def test_goal_timeout_parameter_rejects_nonpositive_value(self) -> None:
        node = _SafetyHarness()

        result = node._on_set_parameters([
            Parameter(
                'goal_timeout_sec', Parameter.Type.DOUBLE, 0.0),
        ])

        self.assertFalse(result.successful)
        self.assertIn('greater than zero', result.reason)
        self.assertEqual(node._goal_timeout, 3.0)

    def test_max_current_parameter_defaults_to_200_and_is_configurable(
            self) -> None:
        node = _SafetyHarness()
        self.assertEqual(node._max_current, 200)

        result = node._on_set_parameters([
            Parameter('max_current', Parameter.Type.INTEGER, 300),
        ])

        self.assertTrue(result.successful)
        self.assertEqual(node._max_current, 300)
        self.assertEqual(node.config_writes, 1)

    def test_max_current_parameter_rejects_out_of_range_value(self) -> None:
        node = _SafetyHarness()

        result = node._on_set_parameters([
            Parameter('max_current', Parameter.Type.INTEGER, 1001),
        ])

        self.assertFalse(result.successful)
        self.assertIn('1..1000', result.reason)
        self.assertEqual(node._max_current, 200)
        self.assertEqual(node.config_writes, 0)

    def test_motion_timeout_latches_fault_and_disables(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'moving'
        node._motion_active = True
        node._motion_target_position = 700
        node._motion_source = 'topic'
        node._motion_deadline = time.monotonic() - 1.0

        node._watchdog_tick()

        self.assertFalse(node._enabled)
        self.assertFalse(node._motion_active)
        self.assertEqual(node._state, 'fault')
        self.assertIn('motion timeout', node._fault_reason)
        self.assertEqual(node.enable_writes, [False])

    def test_motion_timeout_can_remain_nonfatal_for_debugging(self) -> None:
        node = _SafetyHarness()
        node._latch_motion_timeout = False
        node._enabled = True
        node._state = 'moving'
        node._motion_active = True
        node._motion_target_position = 700
        node._motion_source = 'topic'
        node._motion_deadline = time.monotonic() - 1.0

        node._watchdog_tick()

        self.assertTrue(node._enabled)
        self.assertFalse(node._motion_active)
        self.assertEqual(node._state, 'enabled')
        self.assertIsNone(node._fault_reason)
        self.assertIn('motion timeout', node._last_error)
        self.assertEqual(node.enable_writes, [])

    def test_closed_stall_is_reported_as_grasp_not_fault(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'moving'
        node._motion_active = True
        node._motion_target_position = 990
        node._motion_start_position = 300
        node._motion_is_closing = True
        node._motion_source = 'topic'
        node._motion_deadline = time.monotonic() - 1.0
        node._position = 900
        node._feedback_monotonic = time.monotonic()
        node._motor_status_bits = 0x001C
        node._motor_status_monotonic = time.monotonic()

        node._watchdog_tick()

        self.assertTrue(node._enabled)
        self.assertFalse(node._motion_active)
        self.assertEqual(node._state, 'enabled')
        self.assertIsNone(node._fault_reason)
        self.assertTrue(node._last_motion_stalled)
        self.assertFalse(node._last_motion_reached)
        self.assertFalse(node._last_motion_failed)
        self.assertIn('closed on object', node._motion_result)
        self.assertEqual(node.enable_writes, [])

    def test_stale_position_reached_status_is_not_contact(self) -> None:
        node = _SafetyHarness()

        self.assertFalse(node._contact_status_matches(0x001D, 0.0))
        self.assertTrue(node._contact_status_matches(0x001C, 0.0))

    def test_closed_command_without_travel_times_out(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'moving'
        node._motion_active = True
        node._motion_target_position = 685
        node._motion_start_position = 28
        node._motion_is_closing = True
        node._motion_source = 'topic'
        node._motion_deadline = time.monotonic() - 1.0
        node._position = 28
        node._feedback_monotonic = time.monotonic()
        node._motor_status_bits = 0x001C
        node._motor_status_monotonic = time.monotonic()

        node._watchdog_tick()

        self.assertFalse(node._enabled)
        self.assertEqual(node._state, 'fault')
        self.assertIn('motion timeout', node._fault_reason)
        self.assertFalse(node._last_motion_stalled)
        self.assertTrue(node._last_motion_failed)
        self.assertEqual(node.enable_writes, [False])

    def test_opening_stall_still_latches_timeout_fault(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'moving'
        node._motion_active = True
        node._motion_target_position = 0
        node._motion_start_position = 900
        node._motion_is_closing = False
        node._motion_source = 'topic'
        node._motion_deadline = time.monotonic() - 1.0
        node._position = 500
        node._feedback_monotonic = time.monotonic()
        node._motor_status_bits = 0x001C
        node._motor_status_monotonic = time.monotonic()

        node._watchdog_tick()

        self.assertFalse(node._enabled)
        self.assertEqual(node._state, 'fault')
        self.assertIn('motion timeout', node._fault_reason)
        self.assertEqual(node.enable_writes, [False])

    def test_successful_finish_clears_previous_error(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'moving'
        node._motion_active = True
        node._last_error = 'old timeout'

        node._finish_motion('action reached target')

        self.assertIsNone(node._last_error)
        self.assertEqual(node._state, 'enabled')

    def test_fault_cancels_deferred_startup_enable(self) -> None:
        node = _SafetyHarness()
        node._startup_enable_pending = True

        node._set_fault('invalid feedback')

        self.assertFalse(node._startup_enable_pending)
        self.assertEqual(node._state, 'fault')
        self.assertEqual(node._last_error, 'invalid feedback')

    def test_status_values_report_actual_driver_state(self) -> None:
        node = _SafetyHarness()
        node._enabled = True
        node._state = 'enabled'
        node._last_command_position = 420

        values = node._status_values()

        self.assertEqual(values['state'], 'enabled')
        self.assertEqual(values['enabled'], 'true')
        self.assertEqual(values['feedback_fresh'], 'true')
        self.assertEqual(values['command_position'], '420')
        self.assertEqual(values['motion_speed_percent'], '25')
        self.assertEqual(values['motion_torque_percent'], '100')
        self.assertEqual(values['torque_limit_supported'], 'true')
        self.assertEqual(values['torque_feedback_available'], 'false')
        self.assertEqual(values['force_control_available'], 'false')

    def test_status_values_report_motor_telemetry(self) -> None:
        node = _SafetyHarness()
        node._motor_status_bits = 0x801F
        node._motor_status_monotonic = time.monotonic()
        node._controller_target_position = 420
        node._controller_target_monotonic = time.monotonic()
        node._control_mode = 0
        node._control_mode_monotonic = time.monotonic()
        node._fault_bits = 0
        node._fault_status_monotonic = time.monotonic()
        node._user_current = 321
        node._user_current_monotonic = time.monotonic()

        values = node._status_values()

        self.assertEqual(values['motor_status_bits'], '0x801F')
        self.assertEqual(values['controller_target_position'], '420')
        self.assertEqual(values['control_mode'], '0')
        self.assertEqual(values['motor_position_reached'], 'true')
        self.assertEqual(values['motor_zero_speed'], 'true')
        self.assertEqual(values['motor_torque_reached'], 'true')
        self.assertEqual(values['motor_ready'], 'true')
        self.assertEqual(values['controller_enabled'], 'true')
        self.assertEqual(values['fault_bits'], '0x0000')
        self.assertEqual(values['user_current_raw'], '321')
        self.assertEqual(values['current_feedback_available'], 'true')

    def test_status_values_report_supply_and_config_telemetry(self) -> None:
        node = _SafetyHarness()
        now = time.monotonic()
        node._fault_bits_b = 0x1000
        node._fault_status_b_monotonic = now
        node._bus_voltage_raw = 2343
        node._bus_voltage_monotonic = now
        node._temperature_raw = 3668
        node._temperature_monotonic = now
        node._max_speed_register = 5000
        node._max_speed_monotonic = now
        node._max_current_register = 100
        node._max_current_monotonic = now
        node._enable_register = 1
        node._speed_register = 100
        node._torque_register = 100
        node._ramp_register = 100

        values = node._status_values()

        self.assertEqual(values['fault_bits_b'], '0x1000')
        self.assertEqual(values['drop_alarm'], 'true')
        self.assertEqual(values['bus_voltage_v'], '23.43')
        self.assertEqual(values['temperature_c'], '36.68')
        self.assertEqual(values['max_speed_register'], '5000')
        self.assertEqual(values['max_current_register'], '100')
        self.assertEqual(values['enable_register'], '1')
        self.assertEqual(values['speed_register'], '100')
        self.assertEqual(values['torque_register'], '100')
        self.assertEqual(values['ramp_register'], '100')
        self.assertEqual(values['max_current_setpoint'], '200')

    def test_motion_config_validation(self) -> None:
        CtagGripperNode._validate_motion_config(25, 100, 200, 200)
        with self.assertRaises(ValueError):
            CtagGripperNode._validate_motion_config(0, 100, 200, 200)
        with self.assertRaises(ValueError):
            CtagGripperNode._validate_motion_config(25, 0, 200, 200)
        with self.assertRaises(ValueError):
            CtagGripperNode._validate_motion_config(25, 100, 1001, 200)
        with self.assertRaises(ValueError):
            CtagGripperNode._validate_motion_config(25, 100, 200, 0)
        with self.assertRaises(ValueError):
            CtagGripperNode._validate_motion_config(25, 100, 200, 1001)


if __name__ == '__main__':
    unittest.main()
