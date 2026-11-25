#!/usr/bin/env python3
"""Read the two OP3 six-axis force sensors over the gripper's RS485 link.

The force sensors are not independent RS485 nodes.  They are exposed by the
OmniPicker 3 controller through register ``REG_TACTILE = 0x06``:

    sub 0x01 -> face A,  sub 0x02 -> face B

Each reply is ``total_frames, frame_index, <6 little-endian float>`` (24 bytes
of payload).  The wire frame is the same ``EE55`` register envelope used by the
gripper::

    EE 55 | 29-bit id (4B LE) | len | data | CRC16 (2B LE)

Axis names follow the vendor layout (Fx Fy Fz Mx My Mz) but the protocol docs
do not fix units/signs, so treat the numbers as raw until calibrated.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

import serial

SOF = b'\xee\x55'
PRODUCT_ID = 0x01
REG_TACTILE = 0x06
QUERY_DATA = 0x00
QUERY_COUNT = 0x01
SUB_ALL = 0x00
SUB_FACE_A = 0x01
SUB_FACE_B = 0x02
AXES = ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz')

DEFAULT_PORT = (
    '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BH00URZP-if00-port0')
DEFAULT_BAUD = 460800
DEFAULT_DEVICE_ID = 1


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_can_id(device_id: int, reg: int, sub: int, write: bool = False) -> int:
    std_id = ((sub & 0x1F) << 6) | ((reg >> 2) & 0x3F)
    ext_id = (((reg & 0x03) << 16) | (PRODUCT_ID << 8)
              | ((1 if write else 0) << 7) | (device_id & 0x7F))
    return (std_id << 18) | ext_id


def wire_frame(can_id: int, data: bytes) -> bytes:
    body = SOF + can_id.to_bytes(4, 'little') + bytes([len(data)]) + data
    return body + crc16(body).to_bytes(2, 'little')


class Link:
    def __init__(self, port: str, baud: int, timeout: float = 0.3):
        self.serial = serial.Serial(port, baud, timeout=0.03, exclusive=True)
        self.timeout = timeout
        self.buffer = b''

    def close(self) -> None:
        self.serial.close()

    def transact(self, can_id: int, data: bytes) -> bytes | None:
        self.serial.reset_input_buffer()
        self.serial.write(wire_frame(can_id, data))
        self.serial.flush()
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            chunk = self.serial.read(self.serial.in_waiting or 1)
            if chunk:
                self.buffer += chunk
                frame = self._take(can_id)
                if frame is not None:
                    return frame
        return None

    def _take(self, want_id: int) -> bytes | None:
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                # Keep a trailing partial SOF byte; drop other noise.
                self.buffer = (self.buffer[-1:]
                               if self.buffer.endswith(SOF[:1]) else b'')
                return None
            if start:
                self.buffer = self.buffer[start:]
            if len(self.buffer) < 9:
                return None
            length = self.buffer[6]
            if length > 64:
                self.buffer = self.buffer[1:]
                continue
            end = 9 + length
            if len(self.buffer) < end:
                return None
            raw = self.buffer[:end]
            self.buffer = self.buffer[end:]
            can_id = int.from_bytes(raw[2:6], 'little')
            crc_ok = crc16(raw[:-2]) == int.from_bytes(raw[-2:], 'little')
            if can_id == want_id and crc_ok:
                return raw[7:-2]
        return None


def query(link: Link, device_id: int, query_type: int,
          sub: int) -> bytes | None:
    can_id = build_can_id(device_id, REG_TACTILE, sub, False)
    return link.transact(can_id, bytes([query_type]))


def read_face(link: Link, device_id: int, sub: int):
    data = query(link, device_id, QUERY_DATA, sub)
    if data is None or len(data) < 2:
        return None
    payload = data[2:]
    if len(payload) != 24:
        return None
    return struct.unpack('<6f', payload)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Read the two OP3 six-axis force sensors over RS485.')
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    parser.add_argument('--device-id', type=int, default=DEFAULT_DEVICE_ID)
    parser.add_argument('--rate', type=float, default=50.0,
                        help='readout rate in Hz')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds to run (0 = forever)')
    args = parser.parse_args()
    if args.baud != DEFAULT_BAUD:
        print('error: OmniPicker 3 RS485 uses a fixed 460800 baud',
              file=sys.stderr)
        return 2

    try:
        link = Link(args.port, args.baud)
    except Exception as exc:  # noqa: BLE001
        print(f'error: cannot open {args.port}: {exc}', file=sys.stderr)
        return 1

    try:
        count = query(link, args.device_id, QUERY_COUNT, SUB_ALL)
        print(f'sensor_count={int.from_bytes(count or b"", "little")} '
              f'port={args.port} device_id={args.device_id}', flush=True)
        period = 1.0 / args.rate if args.rate > 0 else 0.0
        start = time.monotonic()
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            loop = time.monotonic()
            face_a = read_face(link, args.device_id, SUB_FACE_A)
            face_b = read_face(link, args.device_id, SUB_FACE_B)
            stamp = time.monotonic() - start
            def fmt(values):
                if values is None:
                    return 'no-data'
                return ' '.join(f'{name}={value:8.3f}'
                                for name, value in zip(AXES, values))
            print(f'{stamp:8.3f}s  A[{fmt(face_a)}]  B[{fmt(face_b)}]',
                  flush=True)
            sleep = period - (time.monotonic() - loop)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
