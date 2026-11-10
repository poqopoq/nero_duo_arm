"""Binary protocol for the gripper six-axis force/torque sensor V0.1.0."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable


HEADER = b"\xAA\x55"
MAX_PAYLOAD_LEN = 128
MEASUREMENT_FLOAT_COUNT = 6
MEASUREMENT_PAYLOAD_LEN = MEASUREMENT_FLOAT_COUNT * 4

CMD_MEASUREMENT = 0x01
CMD_SET_BAUD_RATE = 0x02
CMD_GET_FIRMWARE_VERSION = 0x03
CMD_GET_SERIAL_NUMBER = 0x04
CMD_START_AUTO_UPLOAD = 0x09
CMD_STOP_AUTO_UPLOAD = 0x0A
CMD_SET_SAMPLE_RATE = 0x0F

BAUD_RATE_CODES = {
    115200: 0x00,
    230400: 0x01,
    460800: 0x02,
    921600: 0x03,
}

SAMPLE_RATE_CODES = {
    500: 0x04,
    1000: 0x05,
}


class ProtocolError(ValueError):
    """Raised when a frame or command payload violates the protocol."""


@dataclass(frozen=True)
class Frame:
    """A CRC-validated protocol frame."""

    message_type: int
    data: bytes

    @property
    def hex_data(self) -> str:
        return self.data.hex(" ").upper()


@dataclass
class ParserStats:
    frames: int = 0
    crc_errors: int = 0
    discarded_bytes: int = 0


def crc16_modbus(data: bytes) -> int:
    """Return the numerical Modbus CRC16 value for *data*."""

    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_frame(message_type: int, payload: bytes = b"") -> bytes:
    """Build a protocol frame.

    Although the vendor document labels the CRC big-endian, every example
    places the low CRC byte first (the standard Modbus RTU wire order).
    """

    if not 0 <= message_type <= 0xFF:
        raise ProtocolError(f"message type out of range: {message_type}")
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ProtocolError(
            f"payload is {len(payload)} bytes; maximum is {MAX_PAYLOAD_LEN}")

    body = HEADER + bytes((message_type, len(payload))) + payload
    crc = crc16_modbus(body)
    return body + crc.to_bytes(2, "little")


def decode_measurements(payload: bytes) -> tuple[float, ...]:
    """Decode the six little-endian float values in a measurement frame."""

    if len(payload) != MEASUREMENT_PAYLOAD_LEN:
        raise ProtocolError(
            "measurement payload must be "
            f"{MEASUREMENT_PAYLOAD_LEN} bytes, got {len(payload)}")
    return struct.unpack("<6f", payload)


def format_firmware_version(payload: bytes) -> str:
    """Format the three-byte firmware version (major.minor.patch)."""

    if len(payload) != 3:
        return f"unexpected version payload: {payload.hex(' ').upper()}"
    return ".".join(str(value) for value in payload)


def decode_serial_number(payload: bytes) -> str:
    """Decode the NUL-padded ASCII serial number."""

    return payload.split(b"\x00", 1)[0].decode("ascii", errors="replace").strip()


def format_ack(payload: bytes) -> str:
    if not payload:
        return "acknowledged"
    return f"acknowledged, data={payload.hex(' ').upper()}"


def frames_from_bytes(data: bytes) -> list[Frame]:
    """Parse all complete frames from *data* without carrying extra bytes."""

    parser = FrameParser()
    return parser.feed(data)


class FrameParser:
    """Incremental and resynchronising parser for the AA 55 byte stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = ParserStats()

    def feed(self, data: bytes | bytearray | memoryview) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []

        while True:
            header_index = self._buffer.find(HEADER)
            if header_index < 0:
                self._discard_before_possible_header()
                break
            if header_index:
                self._discard(header_index)

            if len(self._buffer) < 4:
                break

            payload_len = self._buffer[3]
            if payload_len > MAX_PAYLOAD_LEN:
                self._discard(1)
                continue

            frame_len = 4 + payload_len + 2
            if len(self._buffer) < frame_len:
                break

            body = bytes(self._buffer[:frame_len - 2])
            expected_crc = crc16_modbus(body)
            received_crc = int.from_bytes(
                self._buffer[frame_len - 2:frame_len], "little")
            if expected_crc != received_crc:
                self.stats.crc_errors += 1
                self._discard(1)
                continue

            frames.append(Frame(message_type=body[2], data=body[4:]))
            self.stats.frames += 1
            del self._buffer[:frame_len]

        return frames

    def _discard_before_possible_header(self) -> None:
        if len(self._buffer) > 1:
            self._discard(len(self._buffer) - 1)

    def _discard(self, count: int) -> None:
        if count <= 0:
            return
        self.stats.discarded_bytes += count
        del self._buffer[:count]


def encode_command(
        message_type: int,
        payload: bytes | Iterable[int] = b"") -> bytes:
    """Small convenience wrapper used by the ROS node and tests."""

    if not isinstance(payload, bytes):
        payload = bytes(payload)
    return build_frame(message_type, payload)
