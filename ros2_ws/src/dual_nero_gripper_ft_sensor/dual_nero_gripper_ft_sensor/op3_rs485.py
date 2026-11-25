"""OP3 RS485 register protocol for the two on-gripper six-axis force sensors.

The OmniPicker 3 controller exposes two six-axis force sensors through
register ``REG_TACTILE = 0x06``:

* sub ``0x01`` -> face A
* sub ``0x02`` -> face B
* sub ``0x00`` -> both faces in one reply

Wire frame (same envelope as the OP3 CAN register protocol)::

    EE 55 | 29-bit id (4B little-endian) | len | data | CRC16 (2B little-endian)

The 29-bit id is built as::

    bits 28..24  sub register
    bits 23..18  register high 6 bits
    bits 17..16  register low 2 bits
    bits 15..8   product id (0x01)
    bit  7       read/write (0 = read)
    bits 6..0    device id
"""
from __future__ import annotations

from dataclasses import dataclass
import struct
import time

import serial


SOF = b'\xee\x55'
PRODUCT_ID = 0x01

REG_TACTILE = 0x06
QUERY_DATA = 0x00
QUERY_COUNT = 0x01
QUERY_POINTS = 0x02
QUERY_LENGTH = 0x03
QUERY_FRAMES = 0x04

SUB_ALL = 0x00
SUB_FACE_A = 0x01
SUB_FACE_B = 0x02

AXES = ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz')
FACE_COUNT = 2
FACE_PAYLOAD_LEN = 24
ALL_PAYLOAD_LEN = FACE_COUNT * FACE_PAYLOAD_LEN

DEFAULT_BAUD = 460800
DEFAULT_DEVICE_ID = 1


class Op3Rs485Error(RuntimeError):
    """Raised when the OP3 RS485 link cannot complete a transaction."""


def crc16(data: bytes) -> int:
    """Standard Modbus CRC16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_can_id(
        device_id: int, reg: int, sub: int, write: bool = False) -> int:
    """Build the OP3 29-bit register id."""
    std_id = ((sub & 0x1F) << 6) | ((reg >> 2) & 0x3F)
    ext_id = (((reg & 0x03) << 16) | (PRODUCT_ID << 8)
              | ((1 if write else 0) << 7) | (device_id & 0x7F))
    return (std_id << 18) | ext_id


def wire_frame(can_id: int, data: bytes) -> bytes:
    if len(data) > 64:
        raise ValueError('RS485 payload exceeds 64 bytes')
    body = SOF + can_id.to_bytes(4, 'little') + bytes([len(data)]) + data
    return body + crc16(body).to_bytes(2, 'little')


def tactile_request(
        device_id: int, query_type: int, sub: int = SUB_ALL) -> tuple[int, bytes]:
    can_id = build_can_id(device_id, REG_TACTILE, sub, False)
    return can_id, bytes([query_type])


def decode_face(payload: bytes) -> tuple[float, ...] | None:
    """Decode ``total_frames, frame_index, 6 x float`` for one face."""
    if len(payload) < 2 + FACE_PAYLOAD_LEN:
        return None
    return struct.unpack('<6f', payload[2:2 + FACE_PAYLOAD_LEN])


def decode_all(payload: bytes) -> tuple[tuple[float, ...], tuple[float, ...]] | None:
    """Decode ``total_frames, frame_index, faceA(6f), faceB(6f)``."""
    if len(payload) < 2 + ALL_PAYLOAD_LEN:
        return None
    values = struct.unpack('<12f', payload[2:2 + ALL_PAYLOAD_LEN])
    return values[:6], values[6:12]


def _extract(
        buffer: bytes, want_id: int) -> tuple[bytes | None, bytes]:
    """Pull one complete matching frame out of ``buffer``."""
    while True:
        start = buffer.find(SOF)
        if start < 0:
            # Keep a lone trailing 0xEE so a split SOF is not discarded.
            keep = buffer[-1:] if buffer.endswith(SOF[:1]) else b''
            return None, keep
        if start:
            buffer = buffer[start:]
        if len(buffer) < 9:
            return None, buffer
        length = buffer[6]
        if length > 64:
            buffer = buffer[1:]
            continue
        end = 9 + length
        if len(buffer) < end:
            return None, buffer
        raw = buffer[:end]
        buffer = buffer[end:]
        can_id = int.from_bytes(raw[2:6], 'little')
        crc_ok = crc16(raw[:-2]) == int.from_bytes(raw[-2:], 'little')
        if can_id == want_id and crc_ok:
            return raw[7:-2], buffer


@dataclass
class ForceSample:
    face_a: tuple[float, ...]
    face_b: tuple[float, ...]


class Op3Rs485Link:
    """Blocking request/response master for the OP3 RS485 force registers."""

    def __init__(
            self,
            port: str,
            baud_rate: int = DEFAULT_BAUD,
            device_id: int = DEFAULT_DEVICE_ID,
            timeout: float = 0.05,
            write_timeout: float = 0.2) -> None:
        self.port = port
        self.baud_rate = baud_rate
        self.device_id = device_id
        self.timeout = timeout
        self.write_timeout = write_timeout
        self._serial: serial.Serial | None = None
        self._rx = b''

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self) -> None:
        self._serial = serial.Serial(
            self.port,
            self.baud_rate,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=self.timeout,
            write_timeout=self.write_timeout,
            exclusive=True,
        )
        self._rx = b''

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def _transact(self, can_id: int, data: bytes,
                  timeout: float | None = None) -> bytes | None:
        if self._serial is None:
            raise Op3Rs485Error('serial link is not open')
        request = wire_frame(can_id, data)
        # No tcflush/tcdrain per transaction: at 1 ms FTDI latency they add
        # ~0.5-1 ms each.  The frame parser resynchronises on SOF/CRC anyway.
        self._serial.write(request)
        deadline = time.monotonic() + (self.timeout if timeout is None
                                       else timeout)
        while time.monotonic() < deadline:
            waiting = self._serial.in_waiting
            chunk = self._serial.read(waiting if waiting else 1)
            if not chunk:
                continue
            self._rx += chunk
            payload, self._rx = _extract(self._rx, can_id)
            if payload is not None:
                return payload
        return None

    def query(self, query_type: int, sub: int = SUB_ALL) -> bytes | None:
        can_id, data = tactile_request(self.device_id, query_type, sub)
        return self._transact(can_id, data)

    def query_count(self) -> int | None:
        payload = self.query(QUERY_COUNT, SUB_ALL)
        if not payload:
            return None
        return int(payload[0])

    def read_all(self) -> ForceSample | None:
        payload = self.query(QUERY_DATA, SUB_ALL)
        if payload is None:
            return None
        decoded = decode_all(payload)
        if decoded is None:
            return None
        return ForceSample(face_a=decoded[0], face_b=decoded[1])

    def read_face(self, sub: int) -> tuple[float, ...] | None:
        payload = self.query(QUERY_DATA, sub)
        if payload is None:
            return None
        return decode_face(payload)
