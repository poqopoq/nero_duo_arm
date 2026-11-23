"""Protocol and CAN-FD transports for AGILINK OmniPicker 3.

The public control protocol is documented in the AGIBOT OmniPicker manual.
A downstream frame uses the configured standard CAN node ID and eight bytes::

    reserved, position, velocity, force, acceleration, deceleration, 0, 0

The upstream status frame uses the same ID::

    fault, state, position, velocity, force, reserved, reserved, reserved

OP3 positions in these frames are one byte.  ``0`` is closed and ``255`` is
fully open.  The ROS-facing command contract remains ``0`` open and
``position_max`` closed to stay compatible with the existing MoveIt setup.
"""
from __future__ import annotations

import select
import socket
import struct
from dataclasses import dataclass
from typing import Protocol

from .zqwl_canfd import CanFrame, ZqwlCanFd, ZqwlError

CAN_POSITION_OPEN = 0xFF
CAN_POSITION_CLOSED = 0x00
CAN_POSITION_MAX = 0xFF
POSITION_MIN = 0
POSITION_MAX = 1000
DEFAULT_BITRATE = 1_000_000
DEFAULT_DATA_BITRATE = 5_000_000
POSITION_QUERY_BASE_ID = 0x01130100

FAULT_NONE = 0
FAULT_OVERHEAT = 1
FAULT_OVERSPEED = 2
FAULT_INITIALIZATION = 3
FAULT_LIMIT = 4
FAULT_NAMES = {
    FAULT_NONE: 'none',
    FAULT_OVERHEAT: 'overheated',
    FAULT_OVERSPEED: 'speed limit exceeded',
    FAULT_INITIALIZATION: 'initialization fault',
    FAULT_LIMIT: 'limit exceeded',
}

STATE_REACHED = 0
STATE_MOVING = 1
STATE_STALLED = 2
STATE_DROPPED = 3
STATE_NAMES = {
    STATE_REACHED: 'reached',
    STATE_MOVING: 'moving',
    STATE_STALLED: 'stalled',
    STATE_DROPPED: 'object dropped',
}

# Linux SocketCAN constants.  Python normally exposes these through socket,
# but keeping fallbacks makes the module testable on older builds.
_SOL_CAN_RAW = getattr(socket, 'SOL_CAN_RAW', 101)
_CAN_RAW = getattr(socket, 'CAN_RAW', 1)
_CAN_RAW_FD_FRAMES = getattr(socket, 'CAN_RAW_FD_FRAMES', 5)
_CAN_RAW_LOOPBACK = getattr(socket, 'CAN_RAW_LOOPBACK', 3)
_CANFD_BRS = 0x01
_CAN_EFF_FLAG = 0x80000000
_CANFD_MTU = 72
_CAN_MTU = 16


class CanTransportError(RuntimeError):
    """Raised when a CAN transport cannot send or decode a frame."""


class CanTransport(Protocol):
    def open(self) -> None: ...
    def send(self, frame: CanFrame) -> None: ...
    def receive(self, timeout: float) -> list[CanFrame]: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class OmniPickerStatus:
    fault_code: int = 0
    state_code: int = STATE_REACHED
    position_byte: int | None = None
    velocity_byte: int = 0
    force_byte: int = 0
    position_raw: int | None = None

    @property
    def fault_name(self) -> str:
        return FAULT_NAMES.get(self.fault_code, f'unknown ({self.fault_code})')

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state_code, f'unknown ({self.state_code})')


def _percent_byte(percent: int) -> int:
    if not 0 <= percent <= 100:
        raise ValueError('percent must be 0..100')
    return int(round(percent * CAN_POSITION_MAX / 100.0))


def position_to_can(
        position: int,
        position_min: int = POSITION_MIN,
        position_max: int = POSITION_MAX) -> int:
    """Map ROS raw position (0=open) to OP3 frame position (255=open)."""
    if position_max <= position_min:
        raise ValueError('position_max must be greater than position_min')
    ratio = (float(position) - position_min) / (position_max - position_min)
    ratio = min(1.0, max(0.0, ratio))
    return int(CAN_POSITION_OPEN * (1.0 - ratio))


def position_from_can(
        position_byte: int,
        position_min: int = POSITION_MIN,
        position_max: int = POSITION_MAX) -> int:
    """Map an OP3 status position byte to the ROS raw position contract."""
    if not 0 <= position_byte <= CAN_POSITION_MAX:
        raise ValueError('OP3 position must be 0..255')
    if position_max <= position_min:
        raise ValueError('position_max must be greater than position_min')
    ratio = 1.0 - position_byte / CAN_POSITION_MAX
    return int(round(position_min + ratio * (position_max - position_min)))


def control_frame(
        position: int,
        speed_percent: int = 50,
        force_percent: int = 100,
        acceleration_percent: int = 100,
        deceleration_percent: int = 100,
        device_id: int = 1,
        channel: int = 0,
        fd: bool = True,
        brs: bool = True,
        position_min: int = POSITION_MIN,
        position_max: int = POSITION_MAX) -> CanFrame:
    """Build the documented OP3 downstream frame."""
    if not 1 <= device_id <= 0x7FF:
        raise ValueError('device_id must be 1..0x7FF')
    if brs and not fd:
        raise ValueError('BRS requires a CAN-FD frame')
    data = bytes((
        0,
        position_to_can(position, position_min, position_max),
        _percent_byte(speed_percent),
        _percent_byte(force_percent),
        _percent_byte(acceleration_percent),
        _percent_byte(deceleration_percent),
        0,
        0,
    ))
    return CanFrame(
        channel=channel,
        can_id=device_id,
        data=data,
        extended=False,
        fd=fd,
        brs=brs,
    )


def position_query_frame(
        device_id: int = 1,
        channel: int = 0,
        brs: bool = True) -> CanFrame:
    """Build the read-only 0x24 position query used by the OP3 SDK."""
    if not 1 <= device_id <= 0x7FF:
        raise ValueError('device_id must be 1..0x7FF')
    return CanFrame(
        channel=channel,
        can_id=POSITION_QUERY_BASE_ID | device_id,
        data=bytes(64),
        extended=True,
        fd=True,
        brs=brs,
    )


def decode_position_query(
        frame: CanFrame,
        device_id: int = 1) -> int | None:
    """Decode the read-only OP3 position response (int16 little endian)."""
    if (not frame.extended
            or frame.can_id != (POSITION_QUERY_BASE_ID | device_id)
            or len(frame.data) < 2):
        return None
    position = int.from_bytes(frame.data[:2], 'little', signed=False)
    if position > 4095:
        return None
    return position


def decode_status(
        frame: CanFrame,
        device_id: int = 1) -> OmniPickerStatus | None:
    """Decode an OP3 upstream status frame, ignoring unrelated traffic."""
    if frame.extended or frame.can_id != device_id or len(frame.data) != 8:
        return None
    fault, state, position, velocity, force = frame.data[:5]
    if fault > FAULT_LIMIT or state > STATE_DROPPED:
        # A ZQWL channel can echo the downstream frame.  Its second byte is
        # normally a target position that is outside the status enum.
        return None
    return OmniPickerStatus(fault, state, position, velocity, force)


class ZqwlCanTransport:
    """Adapter around the project's ZQWL UCANFD CDC transport."""

    def __init__(
            self,
            port: str,
            serial_baudrate: int,
            channel: int = 0,
            bitrate: int = DEFAULT_BITRATE,
            data_bitrate: int = DEFAULT_DATA_BITRATE,
            fd: bool = True,
            save: bool = False,
            timeout: float = 1.0) -> None:
        self.port = port
        self.serial_baudrate = serial_baudrate
        self.channel = channel
        self.bitrate = bitrate
        self.data_bitrate = data_bitrate
        self.fd = fd
        self.save = save
        self.timeout = timeout
        self._client: ZqwlCanFd | None = None

    def open(self) -> None:
        client = ZqwlCanFd(
            port=self.port,
            serial_baudrate=self.serial_baudrate,
            timeout=self.timeout,
        )
        client.open()
        try:
            client.configure_can(
                channel=self.channel,
                bitrate=self.bitrate,
                data_bitrate=self.data_bitrate if self.fd else None,
                fd=self.fd,
                save=self.save,
            )
        except Exception:
            client.close()
            raise
        self._client = client

    def send(self, frame: CanFrame) -> None:
        if self._client is None:
            raise CanTransportError('ZQWL transport is not open')
        try:
            self._client.send(frame)
        except (OSError, ZqwlError) as exc:
            raise CanTransportError(str(exc)) from exc

    def receive(self, timeout: float) -> list[CanFrame]:
        if self._client is None:
            raise CanTransportError('ZQWL transport is not open')
        try:
            return list(self._client.receive(max(0.0, timeout)))
        except (OSError, ZqwlError) as exc:
            raise CanTransportError(str(exc)) from exc

    def disable(self) -> None:
        if self._client is not None:
            try:
                self._client.disable_can(self.channel)
            except (OSError, ZqwlError):
                pass

    def close(self) -> None:
        client = self._client
        self._client = None
        if client is not None:
            client.close()


class SocketCanTransport:
    """Linux SocketCAN transport with optional CAN-FD frames."""

    def __init__(
            self,
            interface: str,
            fd: bool = True,
            brs: bool = True,
            loopback: bool = False) -> None:
        if not interface:
            raise ValueError('SocketCAN interface must not be empty')
        if brs and not fd:
            raise ValueError('BRS requires a CAN-FD frame')
        self.interface = interface
        self.fd = fd
        self.brs = brs
        self.loopback = loopback
        self._socket: socket.socket | None = None

    def open(self) -> None:
        if not hasattr(socket, 'AF_CAN'):
            raise CanTransportError('SocketCAN is not available on this system')
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, _CAN_RAW)
        try:
            if self.fd:
                sock.setsockopt(_SOL_CAN_RAW, _CAN_RAW_FD_FRAMES, 1)
            sock.setsockopt(
                _SOL_CAN_RAW, _CAN_RAW_LOOPBACK, int(self.loopback))
            sock.bind((self.interface,))
        except Exception:
            sock.close()
            raise
        self._socket = sock

    def send(self, frame: CanFrame) -> None:
        sock = self._socket
        if sock is None:
            raise CanTransportError('SocketCAN transport is not open')
        if frame.can_id > 0x1FFFFFFF:
            raise CanTransportError('CAN ID exceeds 29 bits')
        socket_id = frame.can_id | (_CAN_EFF_FLAG if frame.extended else 0)
        if frame.fd:
            data = frame.data.ljust(64, b'\0')[:64]
            flags = _CANFD_BRS if frame.brs else 0
            packet = struct.pack('<IBBBB64s', socket_id, len(frame.data),
                                 flags, 0, 0, data)
        else:
            data = frame.data.ljust(8, b'\0')[:8]
            packet = struct.pack('<IB3x8s', socket_id, len(frame.data), data)
        try:
            sock.send(packet)
        except OSError as exc:
            raise CanTransportError(str(exc)) from exc

    def receive(self, timeout: float) -> list[CanFrame]:
        sock = self._socket
        if sock is None:
            raise CanTransportError('SocketCAN transport is not open')
        if not select.select([sock], [], [], max(0.0, timeout))[0]:
            return []
        packets: list[CanFrame] = []
        try:
            while True:
                ready, _, _ = select.select([sock], [], [], 0.0)
                if not ready:
                    break
                packet = sock.recv(_CANFD_MTU)
                if len(packet) == _CANFD_MTU:
                    can_id, length, flags, _, _, data = struct.unpack(
                        '<IBBBB64s', packet)
                    packets.append(CanFrame(
                        channel=0,
                        can_id=can_id & 0x1FFFFFFF,
                        data=data[:length],
                        extended=bool(can_id & _CAN_EFF_FLAG),
                        fd=True,
                        brs=bool(flags & _CANFD_BRS),
                    ))
                elif len(packet) == _CAN_MTU:
                    can_id, length, data = struct.unpack(
                        '<IB3x8s', packet)
                    packets.append(CanFrame(
                        channel=0,
                        can_id=can_id & 0x1FFFFFFF,
                        data=data[:length],
                        extended=bool(can_id & _CAN_EFF_FLAG),
                        fd=False,
                        brs=False,
                    ))
        except OSError as exc:
            raise CanTransportError(str(exc)) from exc
        return packets

    def close(self) -> None:
        sock = self._socket
        self._socket = None
        if sock is not None:
            sock.close()


def make_transport(
        transport: str,
        *,
        port: str = '',
        serial_baudrate: int = 921_600,
        channel: int = 0,
        bitrate: int = DEFAULT_BITRATE,
        data_bitrate: int = DEFAULT_DATA_BITRATE,
        can_interface: str = 'can0',
        can_fd: bool = True,
        brs: bool = True,
        save_params: bool = False,
        timeout: float = 1.0) -> CanTransport:
    """Construct one of the CAN transports accepted by the ROS node."""
    normalized = transport.strip().lower()
    if normalized == 'zqwl':
        return ZqwlCanTransport(
            port=port,
            serial_baudrate=serial_baudrate,
            channel=channel,
            bitrate=bitrate,
            data_bitrate=data_bitrate,
            fd=can_fd,
            save=save_params,
            timeout=timeout,
        )
    if normalized == 'socketcan':
        return SocketCanTransport(can_interface, fd=can_fd, brs=brs)
    raise ValueError(f'unsupported CAN transport {transport!r}')
