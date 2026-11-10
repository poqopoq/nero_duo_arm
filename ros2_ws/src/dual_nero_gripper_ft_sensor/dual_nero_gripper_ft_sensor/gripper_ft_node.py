#!/usr/bin/env python3
"""ROS 2 serial driver for the gripper six-axis force/torque sensor V0.1.0."""

from __future__ import annotations

from concurrent.futures import Future, TimeoutError as FutureTimeoutError
from dataclasses import dataclass, field
import queue
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Trigger

from dual_nero_gripper_ft_sensor_msgs.msg import SixAxisForce
from dual_nero_gripper_ft_sensor_msgs.srv import SetBaudRate, SetSampleRate

from .protocol import (
    BAUD_RATE_CODES,
    CMD_GET_FIRMWARE_VERSION,
    CMD_GET_SERIAL_NUMBER,
    CMD_MEASUREMENT,
    CMD_SET_BAUD_RATE,
    CMD_SET_SAMPLE_RATE,
    CMD_START_AUTO_UPLOAD,
    CMD_STOP_AUTO_UPLOAD,
    SAMPLE_RATE_CODES,
    Frame,
    FrameParser,
    ProtocolError,
    build_frame,
    decode_measurements,
    decode_serial_number,
    format_ack,
    format_firmware_version,
)

try:
    import serial
except ImportError:  # pragma: no cover - exercised only on incomplete installs
    serial = None


@dataclass
class _CommandResult:
    success: bool
    message: str
    frame: Optional[Frame] = None


@dataclass
class _Command:
    message_type: int
    payload: bytes
    future: Future
    baud_rate: Optional[int] = None
    cancelled: threading.Event = field(default_factory=threading.Event)


class GripperFtSensorNode(Node):
    """Read the gripper force sensor stream and expose it through ROS 2."""

    def __init__(self) -> None:
        super().__init__("gripper_ft_node")

        self._serial_port = str(self.declare_parameter(
            "serial_port", "/dev/ttyUSB0").value)
        self._baud_rate = int(self.declare_parameter(
            "baud_rate", 921600).value)
        self._frame_id = str(self.declare_parameter(
            "frame_id", "gripper_ft_sensor").value)
        self._sensor_id = str(self.declare_parameter(
            "sensor_id", "").value).strip() or self.get_name()
        self._expected_serial_number = str(self.declare_parameter(
            "expected_serial_number", "").value).strip()
        self._query_serial_number_on_start = bool(self.declare_parameter(
            "query_serial_number_on_start", True).value)
        self._require_serial_match = bool(self.declare_parameter(
            "require_serial_match", False).value)
        data_topic = str(self.declare_parameter(
            "data_topic", "/gripper_ft/data").value)
        raw_topic = str(self.declare_parameter(
            "raw_topic", "/gripper_ft/raw").value)
        self._publish_raw = bool(self.declare_parameter(
            "publish_raw", False).value)
        self._reconnect_interval = float(self.declare_parameter(
            "reconnect_interval", 2.0).value)
        self._response_timeout = float(self.declare_parameter(
            "response_timeout", 1.0).value)

        if self._baud_rate <= 0:
            raise ValueError("baud_rate must be positive")
        if self._reconnect_interval <= 0.0:
            raise ValueError("reconnect_interval must be positive")
        if self._response_timeout <= 0.0:
            raise ValueError("response_timeout must be positive")

        self._data_publisher = self.create_publisher(SixAxisForce, data_topic, 10)
        self._raw_publisher = None
        if self._publish_raw:
            self._raw_publisher = self.create_publisher(
                UInt8MultiArray, raw_topic, 10)

        self._commands: queue.Queue[Optional[_Command]] = queue.Queue(maxsize=8)
        self._active_command: Optional[_Command] = None
        self._active_deadline = 0.0
        self._serial = None
        self._serial_connected = False
        self._parser = FrameParser()
        self._stop_event = threading.Event()
        self._last_warning_time = 0.0
        self._sensor_serial_number = ""
        self._identity_pending = False
        self._identity_deadline = 0.0
        self._publish_allowed = not self._require_serial_match
        self._service_wait_timeout = self._response_timeout + 2.0
        self._worker: Optional[threading.Thread] = None

        self.create_service(
            Trigger, "~/start_streaming",
            lambda request, response: self._trigger_command(
                CMD_START_AUTO_UPLOAD, response))
        self.create_service(
            Trigger, "~/stop_streaming",
            lambda request, response: self._trigger_command(
                CMD_STOP_AUTO_UPLOAD, response))
        self.create_service(
            Trigger, "~/get_version",
            self._handle_get_version)
        self.create_service(
            Trigger, "~/get_serial_number",
            self._handle_get_serial_number)
        self.create_service(
            SetSampleRate, "~/set_sample_rate",
            self._handle_set_sample_rate)
        self.create_service(
            SetBaudRate, "~/set_baud_rate",
            self._handle_set_baud_rate)

        if serial is None:
            self.get_logger().error(
                "pyserial is not installed; install python3-serial")
            return

        self._worker = threading.Thread(
            target=self._serial_worker,
            name="gripper_ft_serial",
            daemon=True,
        )
        self._worker.start()

    def _run_command(
            self,
            message_type: int,
            payload: bytes = b"",
            baud_rate: Optional[int] = None) -> _CommandResult:
        if not self._serial_connected:
            return _CommandResult(
                False,
                f"serial port {self._serial_port!r} is not connected",
            )
        if self._identity_pending:
            return _CommandResult(
                False, "sensor serial-number query is still in progress")

        command = _Command(
            message_type=message_type,
            payload=payload,
            future=Future(),
            baud_rate=baud_rate,
        )
        try:
            self._commands.put_nowait(command)
        except queue.Full:
            return _CommandResult(False, "command queue is full")

        try:
            return command.future.result(timeout=self._service_wait_timeout)
        except FutureTimeoutError:
            command.cancelled.set()
            return _CommandResult(False, "timed out waiting for serial worker")

    def _trigger_command(
            self, message_type: int, response: Trigger.Response
    ) -> Trigger.Response:
        result = self._run_command(message_type)
        response.success = result.success
        response.message = result.message
        return response

    def _handle_get_version(
            self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        result = self._run_command(CMD_GET_FIRMWARE_VERSION)
        if not result.success:
            response.success = False
            response.message = result.message
            return response
        if result.frame is None or len(result.frame.data) != 3:
            response.success = False
            payload = b"" if result.frame is None else result.frame.data
            response.message = format_firmware_version(payload)
            return response
        response.success = True
        response.message = format_firmware_version(result.frame.data)
        return response

    def _handle_get_serial_number(
            self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        result = self._run_command(CMD_GET_SERIAL_NUMBER)
        if not result.success or result.frame is None:
            response.success = False
            response.message = result.message
            return response
        serial_number = decode_serial_number(result.frame.data)
        self._sensor_serial_number = serial_number
        if not serial_number:
            response.success = False
            response.message = "sensor returned an empty serial number"
        elif (self._expected_serial_number
              and serial_number != self._expected_serial_number):
            response.success = False
            response.message = (
                f"{serial_number}; expected {self._expected_serial_number}")
            if self._require_serial_match:
                self._publish_allowed = False
        else:
            response.success = True
            response.message = serial_number
            self._publish_allowed = True
        return response

    def _handle_set_sample_rate(
            self, request: SetSampleRate.Request,
            response: SetSampleRate.Response
    ) -> SetSampleRate.Response:
        rate = int(request.sample_rate_hz)
        code = SAMPLE_RATE_CODES.get(rate)
        if code is None:
            response.success = False
            response.message = (
                f"unsupported sample rate {rate} Hz; supported: "
                f"{', '.join(str(value) for value in SAMPLE_RATE_CODES)} Hz"
            )
            return response

        result = self._run_command(CMD_SET_SAMPLE_RATE, bytes((code,)))
        response.success = result.success
        response.message = (
            f"sample rate set to {rate} Hz ({result.message})"
            if result.success else result.message
        )
        return response

    def _handle_set_baud_rate(
            self, request: SetBaudRate.Request,
            response: SetBaudRate.Response
    ) -> SetBaudRate.Response:
        baud_rate = int(request.baud_rate)
        code = BAUD_RATE_CODES.get(baud_rate)
        if code is None:
            response.success = False
            response.message = (
                f"unsupported baud rate {baud_rate}; supported: "
                f"{', '.join(str(value) for value in BAUD_RATE_CODES)}"
            )
            return response

        result = self._run_command(
            CMD_SET_BAUD_RATE, bytes((code,)), baud_rate=baud_rate)
        response.success = result.success
        response.message = result.message
        if result.success:
            try:
                self.set_parameters([
                    Parameter("baud_rate", value=baud_rate),
                ])
            except Exception as exc:  # parameter updates must not fail the call
                self.get_logger().warning(
                    f"sensor baud changed, but baud_rate parameter update "
                    f"failed: {exc}")
        return response

    def _serial_worker(self) -> None:
        while not self._stop_event.is_set():
            if self._serial is None:
                if not self._connect_serial():
                    self._stop_event.wait(self._reconnect_interval)
                    continue

            try:
                self._start_next_command()
                data = self._serial.read(4096)
                if data:
                    for frame in self._parser.feed(data):
                        self._handle_frame(frame)
                self._expire_active_command()
                self._expire_identity_query()
            except (OSError, serial.SerialException) as exc:
                self._handle_serial_error(exc)
            except Exception as exc:  # keep the driver alive after bad data
                self.get_logger().error(
                    f"unexpected serial worker error: {exc}", exc_info=True)
                self._handle_serial_error(exc)

        self._fail_active_command("node is shutting down")
        self._close_serial()

    def _connect_serial(self) -> bool:
        if serial is None:
            self._stop_event.wait(self._reconnect_interval)
            return False
        try:
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=0.05,
                write_timeout=1.0,
            )
        except (OSError, serial.SerialException) as exc:
            self._warn_periodically(
                f"cannot open serial port {self._serial_port!r} at "
                f"{self._baud_rate} baud: {exc}")
            return False

        self._parser = FrameParser()
        self._serial_connected = True
        self.get_logger().info(
            f"connected to {self._serial_port} at {self._baud_rate} baud")
        self._start_identity_query()
        return True

    def _start_identity_query(self) -> None:
        self._sensor_serial_number = ""
        self._publish_allowed = not self._require_serial_match
        self._identity_pending = False
        if not self._query_serial_number_on_start:
            if self._require_serial_match:
                self.get_logger().error(
                    "require_serial_match is true, but serial-number "
                    "querying on start is disabled; no data will publish")
            return
        if self._serial is None:
            return
        try:
            wire_frame = build_frame(CMD_GET_SERIAL_NUMBER)
            self._serial.write(wire_frame)
            self._serial.flush()
        except (OSError, serial.SerialException) as exc:
            self.get_logger().warning(
                f"failed to request sensor serial number: {exc}")
            return
        self._identity_pending = True
        self._identity_deadline = time.monotonic() + self._response_timeout
        self.get_logger().debug(
            f"requested sensor serial number: "
            f"{wire_frame.hex(' ').upper()}")

    def _start_next_command(self) -> None:
        if self._active_command is not None or self._identity_pending:
            return
        try:
            command = self._commands.get_nowait()
        except queue.Empty:
            return

        if command is None:
            self._stop_event.set()
            return
        if command.cancelled.is_set():
            command.future.set_result(
                _CommandResult(False, "command was cancelled"))
            return
        if self._serial is None:
            command.future.set_result(
                _CommandResult(False, "serial port is not connected"))
            return

        wire_frame = build_frame(command.message_type, command.payload)
        self._serial.write(wire_frame)
        self._serial.flush()
        self._active_command = command
        self._active_deadline = time.monotonic() + self._response_timeout
        self.get_logger().debug(
            f"sent command 0x{command.message_type:02X}: "
            f"{wire_frame.hex(' ').upper()}")

    def _handle_frame(self, frame: Frame) -> None:
        if (self._identity_pending
                and frame.message_type == CMD_GET_SERIAL_NUMBER):
            self._complete_identity_query(frame)
            return

        if frame.message_type == CMD_MEASUREMENT:
            self._publish_measurement(frame)
            return

        if (self._active_command is not None
                and frame.message_type == self._active_command.message_type):
            self._complete_active_command(frame)
            return

        self.get_logger().debug(
            f"ignoring unsolicited frame type 0x{frame.message_type:02X}: "
            f"{frame.hex_data}")

    def _publish_measurement(self, frame: Frame) -> None:
        if not self._publish_allowed:
            return
        try:
            values = decode_measurements(frame.data)
        except ProtocolError as exc:
            self.get_logger().warning(f"invalid measurement frame: {exc}")
            return

        if not rclpy.ok():
            return
        message = SixAxisForce()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.sensor_id = self._sensor_id
        message.serial_number = self._sensor_serial_number
        message.data = list(values)
        self._data_publisher.publish(message)

        if self._raw_publisher is not None:
            raw_message = UInt8MultiArray()
            raw_message.data = list(frame.data)
            self._raw_publisher.publish(raw_message)

    def _complete_identity_query(self, frame: Frame) -> None:
        self._identity_pending = False
        serial_number = decode_serial_number(frame.data)
        self._sensor_serial_number = serial_number
        if not serial_number:
            self.get_logger().warning(
                "sensor returned an empty serial number")
            return

        if not self._expected_serial_number:
            self.get_logger().info(
                f"sensor_id={self._sensor_id} serial_number={serial_number}")
            self._publish_allowed = True
            return

        if serial_number == self._expected_serial_number:
            self.get_logger().info(
                f"sensor_id={self._sensor_id} serial_number={serial_number} "
                "matches expected identity")
            self._publish_allowed = True
            return

        message = (
            f"sensor_id={self._sensor_id} reports serial_number="
            f"{serial_number}, expected {self._expected_serial_number}")
        if self._require_serial_match:
            self._publish_allowed = False
            self.get_logger().error(
                f"{message}; refusing to publish mislabelled force data")
        else:
            self.get_logger().warning(message)

    def _expire_identity_query(self) -> None:
        if not self._identity_pending:
            return
        if time.monotonic() < self._identity_deadline:
            return
        self._identity_pending = False
        message = (
            f"no serial-number response within "
            f"{self._response_timeout:.2f} s")
        if self._require_serial_match:
            self._publish_allowed = False
            self.get_logger().error(
                f"{message}; no data will publish until identity is verified")
        else:
            self.get_logger().warning(message)

    def _complete_active_command(self, frame: Frame) -> None:
        command = self._active_command
        self._active_command = None
        if command is None:
            return

        message = format_ack(frame.data)
        success = True
        if command.baud_rate is not None:
            target_baud = command.baud_rate
            if target_baud != self._baud_rate:
                self._close_serial()
                self._baud_rate = target_baud
                if self._connect_serial():
                    message += f"; serial reopened at {target_baud} baud"
                else:
                    success = False
                    message += (
                        f"; sensor acknowledged, but reopening the serial "
                        f"port at {target_baud} baud failed")

        if not command.future.done():
            command.future.set_result(
                _CommandResult(success, message, frame))

    def _expire_active_command(self) -> None:
        if self._active_command is None:
            return
        if time.monotonic() < self._active_deadline:
            return
        self._fail_active_command(
            f"no response within {self._response_timeout:.2f} s")

    def _fail_active_command(self, reason: str) -> None:
        command = self._active_command
        self._active_command = None
        if command is not None and not command.future.done():
            command.future.set_result(_CommandResult(False, reason))

    def _handle_serial_error(self, exc: Exception) -> None:
        self._warn_periodically(f"serial communication error: {exc}")
        self._fail_active_command(f"serial communication error: {exc}")
        self._close_serial()

    def _close_serial(self) -> None:
        self._serial_connected = False
        self._identity_pending = False
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, serial.SerialException):
                pass
            self._serial = None

    def _warn_periodically(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_time >= 5.0:
            self._last_warning_time = now
            self.get_logger().warning(message)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        try:
            self._commands.put_nowait(None)
        except queue.Full:
            pass
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GripperFtSensorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
