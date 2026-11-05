"""Minimal ZQWL UCANFD CDC transport used by the gripper tooling.

The adapter exposes a CDC-ACM serial port, not SocketCAN or SLCAN.  This
module implements only the vendor's documented configuration, heartbeat and
CAN/CAN-FD frame formats.  It deliberately contains no CTAG gripper command
definitions.
"""
from __future__ import annotations

import errno
import os
import select
import termios
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple, Union

STABLE_PORT = (
    '/dev/serial/by-id/'
    'usb-ZQWL-CANFD_ZQWL-CANFD_6A649697220A-if00')
DEFAULT_SERIAL_BAUD = 921600

CONFIG_HEADER = b'\x49\x3b'
CONFIG_TAIL = b'\x45\x2e'
CAN_HEADER = 0x5A
CAN_TAIL = 0xA5

FUNC_DEVICE_INFO = 0x40
FUNC_SERIAL_NUMBER = 0x41
FUNC_CAN_PARAMETERS = 0x42
FUNC_FILTER = 0x43
FUNC_SYSTEM_CONTROL = 0x44
FUNC_HEARTBEAT = 0xFF
FUNC_HEARTBEAT_4CH = 0xFE

READ = 0x52
WRITE = 0x57

# The adapter uses different common-baud code tables for the arbitration
# phase and the CAN-FD data phase (see ZQWL protocol V1.05, section 2.2).
ARBITRATION_CODE_TO_BITRATE = {
    0x0: 1_000_000,
    0x1: 800_000,
    0x2: 500_000,
    0x3: 400_000,
    0x4: 250_000,
    0x5: 200_000,
    0x6: 125_000,
    0x7: 100_000,
    0x8: 50_000,
    0x9: 40_000,
    0xA: 25_000,
    0xB: 20_000,
    0xC: 15_000,
    0xD: 10_000,
    0xE: 5_000,
}
ARBITRATION_BITRATE_TO_CODE = {
    bitrate: code for code, bitrate in ARBITRATION_CODE_TO_BITRATE.items()}

DATA_CODE_TO_BITRATE = {
    0x0: 5_000_000,
    0x1: 4_000_000,
    0x2: 2_000_000,
    0x3: 1_000_000,
    0x4: 800_000,
    0x5: 500_000,
    0x6: 400_000,
    0x7: 250_000,
    0x8: 200_000,
    0x9: 125_000,
    0xA: 100_000,
}
DATA_BITRATE_TO_CODE = {
    bitrate: code for code, bitrate in DATA_CODE_TO_BITRATE.items()}

DLC_TO_LENGTH = {
    0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
    9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64,
}
LENGTH_TO_DLC = {length: dlc for dlc, length in DLC_TO_LENGTH.items()}


class ZqwlError(RuntimeError):
    """Base error for the ZQWL transport."""


class ZqwlProtocolError(ZqwlError):
    """The adapter returned an invalid or unexpected frame."""


@dataclass(frozen=True)
class ConfigResponse:
    function: int
    write: bool
    data: bytes


@dataclass(frozen=True)
class Heartbeat:
    raw: bytes

    @property
    def channels(self) -> int:
        return 4 if self.raw[1] == FUNC_HEARTBEAT_4CH else 2

    @property
    def status(self) -> int:
        """Return the status byte of a 1/2-channel heartbeat (D8)."""
        if self.channels != 2:
            raise ValueError(
                'status decoding is only defined for 1/2-channel heartbeats')
        return self.raw[10]

    def channel_enabled(self, channel: int) -> bool:
        if channel not in (0, 1):
            raise ValueError('channel must be 0 or 1')
        return bool(self.status & (0x20 if channel == 0 else 0x10))

    def bus_state(self, channel: int) -> str:
        if channel not in (0, 1):
            raise ValueError('channel must be 0 or 1')
        shift = 2 if channel == 0 else 0
        return (
            'ok', 'warning', 'passive_error', 'offline')[
                (self.status >> shift) & 0x03]


@dataclass(frozen=True)
class CanFrame:
    channel: int
    can_id: int
    data: bytes
    extended: bool = False
    remote: bool = False
    fd: bool = False
    brs: bool = False

    def format(self) -> str:
        flags = []
        if self.extended:
            flags.append('extended')
        if self.fd:
            flags.append('fd')
        if self.brs:
            flags.append('brs')
        if self.remote:
            flags.append('remote')
        suffix = f" [{' '.join(flags)}]" if flags else ''
        return (
            f'can{self.channel} '
            f'{self.can_id:0{8 if self.extended else 3}X}'
            f'#{self.data.hex().upper()}{suffix}')


ParsedMessage = Union[ConfigResponse, Heartbeat, CanFrame]


def encode_config_request(
        function: int, data: bytes = b'', write: bool = False) -> bytes:
    if len(data) > 16:
        raise ValueError('configuration payload must be at most 16 bytes')
    payload = bytes(data).ljust(16, b'\x00')
    access = WRITE if write else READ
    return CONFIG_HEADER + bytes((function, access)) + payload + CONFIG_TAIL


def encode_can_parameters(
        channel: int,
        bitrate: int,
        data_bitrate: Optional[int] = None,
        fd: bool = False) -> bytes:
    if channel not in range(4):
        raise ValueError('channel must be 0..3')
    if bitrate not in ARBITRATION_BITRATE_TO_CODE:
        raise ValueError(f'unsupported CAN bitrate: {bitrate}')
    if fd and data_bitrate is None:
        raise ValueError('CAN-FD configuration requires a data bitrate')
    if data_bitrate is not None and data_bitrate not in DATA_BITRATE_TO_CODE:
        raise ValueError(
            f'unsupported CAN-FD data bitrate: {data_bitrate}')

    data = bytearray(16)
    data[0] = channel
    data[1] = 0
    arbitration_code = ARBITRATION_BITRATE_TO_CODE[bitrate]
    data_code = DATA_BITRATE_TO_CODE[data_bitrate] if data_bitrate else 0
    data[2] = (arbitration_code << 4) | data_code
    return encode_config_request(FUNC_CAN_PARAMETERS, bytes(data), write=True)


def encode_system_control(
        channel: int,
        enable: bool,
        save: bool = False,
        reset: bool = False) -> bytes:
    if channel not in range(4):
        raise ValueError('channel must be 0..3')
    data = bytearray(16)
    data[0] = int(save)
    data[1] = int(reset)
    data[2 + channel] = int(enable)
    return encode_config_request(FUNC_SYSTEM_CONTROL, bytes(data), write=True)


def encode_can_frame(frame: CanFrame) -> bytes:
    if frame.channel not in range(8):
        raise ValueError('channel must be 0..7')
    if frame.extended:
        if not 0 <= frame.can_id <= 0x1FFFFFFF:
            raise ValueError('extended CAN ID out of range')
    elif not 0 <= frame.can_id <= 0x7FF:
        raise ValueError('standard CAN ID out of range')
    if frame.fd:
        if len(frame.data) not in LENGTH_TO_DLC:
            raise ValueError('unsupported CAN-FD payload length')
    elif len(frame.data) > 8:
        raise ValueError('classic CAN payload must be at most 8 bytes')

    dlc = LENGTH_TO_DLC[len(frame.data)]
    frame_info_1 = dlc | ((frame.channel & 0x01) << 7)
    frame_info_2 = (
        (((frame.channel >> 1) & 0x03) << 3)
        | (int(frame.extended) << 2)
        | (int(frame.remote) << 1)
        | int(frame.brs))
    identifier = frame.can_id | (0x80000000 if frame.fd else 0)
    return (
        bytes((CAN_HEADER, frame_info_1, frame_info_2))
        + identifier.to_bytes(4, 'big')
        + bytes(frame.data)
        + bytes((CAN_TAIL,)))


class FrameParser:
    """Incrementally split the adapter byte stream into protocol messages."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> List[ParsedMessage]:
        self._buffer.extend(data)
        messages: List[ParsedMessage] = []
        while self._buffer:
            start = self._find_start()
            if start is None:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if self._buffer[0] == CONFIG_HEADER[0]:
                message = self._parse_config()
            else:
                message = self._parse_can_or_heartbeat()
            if message is None:
                break
            parsed, length = message
            if length == 0:
                continue
            if parsed is not None:
                messages.append(parsed)
            del self._buffer[:length]
        return messages

    def _find_start(self) -> Optional[int]:
        positions = (
            self._buffer.find(CONFIG_HEADER[0:1]),
            self._buffer.find(bytes((CAN_HEADER,))),
        )
        valid = [position for position in positions if position >= 0]
        return min(valid) if valid else None

    def _parse_config(self) -> Optional[Tuple[Optional[ConfigResponse], int]]:
        if len(self._buffer) < 2:
            return None
        if self._buffer[1] != CONFIG_HEADER[1]:
            del self._buffer[0]
            return None, 0
        length = 22
        if len(self._buffer) < length:
            return None
        candidate = bytes(self._buffer[:length])
        if candidate[-2:] != CONFIG_TAIL:
            del self._buffer[0]
            return None, 0
        if candidate[3] not in (READ, WRITE):
            del self._buffer[0]
            return None, 0
        return ConfigResponse(
            function=candidate[2],
            write=candidate[3] == WRITE,
            data=candidate[4:20],
        ), length

    def _parse_can_or_heartbeat(
            self) -> Optional[Tuple[Optional[ParsedMessage], int]]:
        if len(self._buffer) < 2:
            return None
        function_or_dlc = self._buffer[1]
        if function_or_dlc in (FUNC_HEARTBEAT, FUNC_HEARTBEAT_4CH):
            length = 17 if function_or_dlc == FUNC_HEARTBEAT else 32
            if len(self._buffer) < length:
                return None
            if self._buffer[length - 1] != CAN_TAIL:
                del self._buffer[0]
                return None, 0
            return Heartbeat(bytes(self._buffer[:length])), length

        dlc = function_or_dlc & 0x0F
        if dlc > 8:
            if len(self._buffer) < 7:
                return None
            if not (int.from_bytes(self._buffer[3:7], 'big') & 0x80000000):
                del self._buffer[0]
                return None, 0
        payload_length = DLC_TO_LENGTH[dlc]
        length = 8 + payload_length
        if len(self._buffer) < length:
            return None
        candidate = bytes(self._buffer[:length])
        if candidate[-1] != CAN_TAIL:
            del self._buffer[0]
            return None, 0

        frame_info_2 = candidate[2]
        channel = (
            (((frame_info_2 >> 4) & 0x01) << 2)
            | (((frame_info_2 >> 3) & 0x01) << 1)
            | ((candidate[1] >> 7) & 0x01))
        identifier = int.from_bytes(candidate[3:7], 'big')
        frame = CanFrame(
            channel=channel,
            can_id=identifier & 0x1FFFFFFF,
            data=candidate[7:7 + payload_length],
            extended=bool(frame_info_2 & 0x04),
            remote=bool(frame_info_2 & 0x02),
            fd=bool(identifier & 0x80000000),
            brs=bool(frame_info_2 & 0x01),
        )
        return frame, length


_SERIAL_BAUD_CONSTANTS: Dict[int, int] = {}
for _baud in (
        9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000,
        576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000,
        3000000, 3500000, 4000000):
    _constant = getattr(termios, f'B{_baud}', None)
    if _constant is not None:
        _SERIAL_BAUD_CONSTANTS[_baud] = _constant


class ZqwlCanFd:
    def __init__(
            self,
            port: str = STABLE_PORT,
            serial_baudrate: int = DEFAULT_SERIAL_BAUD,
            timeout: float = 0.5) -> None:
        self.port = port
        self.serial_baudrate = serial_baudrate
        self.timeout = timeout
        self._fd: Optional[int] = None
        self._parser = FrameParser()

    def __enter__(self) -> 'ZqwlCanFd':
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._fd is not None

    def open(self) -> None:
        if self._fd is not None:
            return
        if self.serial_baudrate not in _SERIAL_BAUD_CONSTANTS:
            raise ZqwlError(
                f'unsupported host serial baudrate: {self.serial_baudrate}')
        try:
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            if exc.errno == errno.EACCES:
                raise ZqwlError(
                    f'cannot open {self.port}: permission denied. Add the user '
                    'to dialout and start a new session (`newgrp dialout`), '
                    'or temporarily use '
                    f'`sudo chmod 666 {os.path.realpath(self.port)}`.') from exc
            raise ZqwlError(f'cannot open {self.port}: {exc}') from exc

        try:
            attributes = termios.tcgetattr(fd)
            attributes[0] = 0
            attributes[1] = 0
            attributes[2] = (
                attributes[2]
                & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB
                    | getattr(termios, 'CRTSCTS', 0))
            ) | termios.CS8 | termios.CREAD | termios.CLOCAL
            attributes[3] = 0
            attributes[4] = _SERIAL_BAUD_CONSTANTS[self.serial_baudrate]
            attributes[5] = _SERIAL_BAUD_CONSTANTS[self.serial_baudrate]
            attributes[6][termios.VMIN] = 0
            attributes[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attributes)
            termios.tcflush(fd, termios.TCIFLUSH)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd
        self._parser = FrameParser()

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def read_device_info(self) -> bytes:
        return self._request(FUNC_DEVICE_INFO)

    def read_serial_number(self) -> bytes:
        return self._request(FUNC_SERIAL_NUMBER)

    def configure_can(
            self,
            channel: int = 0,
            bitrate: int = 1_000_000,
            data_bitrate: Optional[int] = None,
            fd: bool = False,
            save: bool = False) -> None:
        self._write_config(
            encode_can_parameters(
                channel, bitrate, data_bitrate=data_bitrate, fd=fd),
            FUNC_CAN_PARAMETERS,
        )
        self._write_config(
            encode_system_control(channel, enable=True, save=save),
            FUNC_SYSTEM_CONTROL,
        )

    def disable_can(self, channel: int = 0) -> None:
        self._write_config(
            encode_system_control(channel, enable=False, save=False),
            FUNC_SYSTEM_CONTROL,
        )

    def send(self, frame: CanFrame) -> None:
        self._write(encode_can_frame(frame))

    def receive(self, duration: float = 0.2) -> List[CanFrame]:
        deadline = time.monotonic() + max(0.0, duration)
        frames: List[CanFrame] = []
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            for message in self._read_messages(remaining):
                if isinstance(message, CanFrame):
                    frames.append(message)
        return frames

    def messages(self, duration: float) -> Iterable[ParsedMessage]:
        deadline = time.monotonic() + max(0.0, duration)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            yield from self._read_messages(remaining)

    def _request(self, function: int) -> bytes:
        self._write(encode_config_request(function))
        return self._await_config(function)

    def _write_config(
            self,
            payload: bytes,
            function: int,
            ack_timeout: float = 0.25) -> Optional[bytes]:
        """Write a configuration frame, tolerating a missing acknowledgement.

        Some ZQWL firmware applies write-only configuration frames without
        returning the matching 0x49/0x3B response.  The vendor example treats
        these writes as fire-and-forget, so a missing ACK must not make an
        otherwise usable CAN channel look unavailable.
        """
        self._write(payload)
        try:
            return self._await_config(function, timeout=ack_timeout)
        except ZqwlProtocolError:
            return None

    def _await_config(
            self,
            function: int,
            timeout: Optional[float] = None) -> bytes:
        deadline = time.monotonic() + (
            self.timeout if timeout is None else max(0.0, timeout))
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ZqwlProtocolError(
                    f'timed out waiting for function 0x{function:02X} response')
            for message in self._read_messages(remaining):
                if isinstance(message, ConfigResponse) and (
                        message.function == function):
                    return message.data

    def _read_messages(self, timeout: float) -> List[ParsedMessage]:
        fd = self._require_fd()
        ready, _, _ = select.select([fd], [], [], max(0.0, timeout))
        if not ready:
            return []
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            return []
        if not data:
            return []
        return self._parser.feed(data)

    def _write(self, payload: bytes) -> None:
        fd = self._require_fd()
        offset = 0
        deadline = time.monotonic() + self.timeout
        while offset < len(payload):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ZqwlProtocolError('timed out writing to ZQWL adapter')
            _, ready, _ = select.select([], [fd], [], remaining)
            if not ready:
                raise ZqwlProtocolError('timed out writing to ZQWL adapter')
            try:
                written = os.write(fd, payload[offset:])
            except BlockingIOError:
                continue
            if written <= 0:
                raise ZqwlProtocolError('ZQWL adapter write failed')
            offset += written

    def _require_fd(self) -> int:
        if self._fd is None:
            raise ZqwlError('ZQWL adapter is not open')
        return self._fd


def decode_serial_number(data: bytes) -> str:
    serial = bytes(data).rstrip(b'\x00')
    return ''.join(chr(value) if 32 <= value < 127 else '.' for value in serial)
