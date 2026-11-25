#!/usr/bin/env python3
"""ROS 2 node for the two OP3 six-axis force sensors over the gripper RS485.

Publishes one ``SixAxisForce`` per sensor face.  The node only reads the
tactile register (0x06); it never sends a motion command.
"""
from __future__ import annotations

import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from dual_nero_gripper_ft_sensor_msgs.msg import SixAxisForce

from .op3_rs485 import (
    AXES,
    DEFAULT_BAUD,
    DEFAULT_DEVICE_ID,
    SUB_FACE_A,
    SUB_FACE_B,
    Op3Rs485Link,
)


DEFAULT_PORT = (
    '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BH00URZP-if00-port0')
MAX_BAUD_WARNING = 460800


class GripperFtRs485Node(Node):
    """Read face A and face B through the OmniPicker 3 RS485 register link."""

    def __init__(self) -> None:
        super().__init__('gripper_ft_rs485_node')

        self._port = str(self.declare_parameter(
            'serial_port', DEFAULT_PORT).value)
        self._baud = int(self.declare_parameter(
            'baud_rate', DEFAULT_BAUD).value)
        self._device_id = int(self.declare_parameter(
            'device_id', DEFAULT_DEVICE_ID).value)
        self._combined_query = bool(self.declare_parameter(
            'combined_query', True).value)
        self._rate_hz = float(self.declare_parameter(
            'rate_hz', 0.0).value)
        self._response_timeout = float(self.declare_parameter(
            'response_timeout', 0.05).value)
        self._reconnect_interval = float(self.declare_parameter(
            'reconnect_interval', 1.0).value)
        self._expected_sensor_count = int(self.declare_parameter(
            'expected_sensor_count', 2).value)
        self._rate_log_period = float(self.declare_parameter(
            'rate_log_period', 5.0).value)

        self._frame_id_a = str(self.declare_parameter(
            'frame_id_face_a', 'gripper_ft_face_a').value)
        self._frame_id_b = str(self.declare_parameter(
            'frame_id_face_b', 'gripper_ft_face_b').value)
        self._sensor_id_a = str(self.declare_parameter(
            'sensor_id_face_a', 'face_a').value)
        self._sensor_id_b = str(self.declare_parameter(
            'sensor_id_face_b', 'face_b').value)
        self._topic_a = str(self.declare_parameter(
            'data_topic_face_a', '/gripper_ft/a').value)
        self._topic_b = str(self.declare_parameter(
            'data_topic_face_b', '/gripper_ft/b').value)

        offset_a = list(self.declare_parameter(
            'offset_face_a', [0.0] * 6).value)
        offset_b = list(self.declare_parameter(
            'offset_face_b', [0.0] * 6).value)
        self._offset_a = tuple(float(v) for v in offset_a[:6]) + (0.0,) * max(
            0, 6 - len(offset_a))
        self._offset_b = tuple(float(v) for v in offset_b[:6]) + (0.0,) * max(
            0, 6 - len(offset_b))

        qos_reliability = str(self.declare_parameter(
            'qos_reliability', 'best_effort').value).strip().lower()
        qos_depth = int(self.declare_parameter('qos_depth', 1).value)
        if qos_reliability not in ('best_effort', 'reliable'):
            raise ValueError(
                "qos_reliability must be 'best_effort' or 'reliable'")
        if qos_depth <= 0:
            raise ValueError('qos_depth must be positive')
        if self._rate_hz < 0.0:
            raise ValueError('rate_hz must be >= 0')
        if self._response_timeout <= 0.0:
            raise ValueError('response_timeout must be positive')
        if self._baud > MAX_BAUD_WARNING:
            self.get_logger().warn(
                f'baud_rate={self._baud} is above the value the vendor SDK '
                f'enforces ({MAX_BAUD_WARNING}); only use it after '
                'reconfiguring uart_baudrate on the gripper')

        qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if qos_reliability == 'best_effort'
                         else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
        )
        self._pub_a = self.create_publisher(SixAxisForce, self._topic_a, qos)
        self._pub_b = self.create_publisher(SixAxisForce, self._topic_b, qos)

        self._stop_event = threading.Event()
        self._worker = threading.Thread(
            target=self._worker_loop, name='op3_ft_rs485', daemon=True)
        self._samples = 0
        self._errors = 0
        self._last_log = time.monotonic()
        self._sensor_count: int | None = None
        self._warn_latency_timer()
        self._worker.start()
        self.get_logger().info(
            f'reading OP3 force sensors on {self._port} @ {self._baud} '
            f'device_id={self._device_id} combined_query={self._combined_query} '
            f'topics=({self._topic_a}, {self._topic_b})')

    def _warn_latency_timer(self) -> None:
        tty = os.path.basename(os.path.realpath(self._port))
        path = f'/sys/bus/usb-serial/devices/{tty}/latency_timer'
        try:
            value = int(Path(path).read_text().strip())
        except (OSError, ValueError):
            return
        if value > 1:
            self.get_logger().warn(
                f'FTDI latency_timer={value} ms caps the read loop near '
                f'{1000.0 / value:.0f} Hz. Set it to 1 ms with '
                'deployment/scripts/install_ftdi_low_latency.sh (needs sudo).')

    def _fill(self, message, sensor_id: str, frame_id: str,
              values: tuple[float, ...], offsets: tuple[float, ...],
              stamp) -> None:
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.sensor_id = sensor_id
        message.serial_number = ''
        message.data = [float(v - o) for v, o in zip(values, offsets)]

    def _publish(self, values_a, values_b, stamp) -> None:
        msg_a = SixAxisForce()
        self._fill(msg_a, self._sensor_id_a, self._frame_id_a,
                   values_a, self._offset_a, stamp)
        self._pub_a.publish(msg_a)
        msg_b = SixAxisForce()
        self._fill(msg_b, self._sensor_id_b, self._frame_id_b,
                   values_b, self._offset_b, stamp)
        self._pub_b.publish(msg_b)
        self._samples += 1

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            link = Op3Rs485Link(
                self._port, self._baud, self._device_id,
                timeout=self._response_timeout)
            try:
                link.open()
                self._sensor_count = link.query_count()
                if self._sensor_count is None:
                    raise RuntimeError('no tactile sensor-count reply')
                if (self._expected_sensor_count > 0
                        and self._sensor_count != self._expected_sensor_count):
                    self.get_logger().warn(
                        f'sensor_count={self._sensor_count}, expected '
                        f'{self._expected_sensor_count}')
                else:
                    self.get_logger().info(
                        f'sensor_count={self._sensor_count}')
                self._run_read_loop(link)
            except Exception as exc:  # noqa: BLE001 - reconnect on any error
                if not self._stop_event.is_set():
                    self.get_logger().warn(
                        f'RS485 read failed: {exc}; retrying in '
                        f'{self._reconnect_interval:.1f}s')
                    self._stop_event.wait(self._reconnect_interval)
            finally:
                link.close()
                if not self._stop_event.is_set():
                    time.sleep(0.05)

    def _run_read_loop(self, link: Op3Rs485Link) -> None:
        period = 1.0 / self._rate_hz if self._rate_hz > 0.0 else 0.0
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            if self._combined_query:
                sample = link.read_all()
                if sample is None:
                    self._errors += 1
                else:
                    stamp = self.get_clock().now().to_msg()
                    self._publish(sample.face_a, sample.face_b, stamp)
            else:
                face_a = link.read_face(SUB_FACE_A)
                face_b = link.read_face(SUB_FACE_B)
                if face_a is None or face_b is None:
                    self._errors += 1
                else:
                    stamp = self.get_clock().now().to_msg()
                    self._publish(face_a, face_b, stamp)
            self._maybe_log_rate()
            if period:
                sleep = period - (time.monotonic() - loop_start)
                if sleep > 0:
                    time.sleep(sleep)

    def _maybe_log_rate(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_log
        if elapsed < self._rate_log_period:
            return
        rate = self._samples / elapsed
        self.get_logger().info(
            f'{rate:.1f} Hz, errors={self._errors}, '
            f'sensor_count={self._sensor_count}')
        self._samples = 0
        self._errors = 0
        self._last_log = now

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperFtRs485Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
