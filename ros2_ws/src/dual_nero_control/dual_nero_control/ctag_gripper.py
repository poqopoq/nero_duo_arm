"""CTAG2F90-D CAN register protocol and jaw-to-position mapping.

The gripper uses the CFEF-RAP register layout on a 29-bit extended CAN ID:

    bits 0-6   device id
    bit 7      write flag
    bits 8-23  register address

The adapter is a ZQWL CDC-ACM device.  It can carry either classic CAN
frames or CAN-FD frames, but the gripper package supplied by the vendor uses
classic 1 Mbps extended frames by default.
"""
from __future__ import annotations

import math
from typing import Optional

from .zqwl_canfd import CanFrame

REG_ENABLE = 0x0100
REG_MODE = 0x0101
REG_POSITION = 0x0102
REG_SPEED = 0x0104
REG_TORQUE = 0x0105
REG_ACCEL_RAMP = 0x0106
REG_TRIGGER = 0x0108
REG_ALARM_RESET = 0x0110
REG_MOTOR_STATUS = 0x0401
REG_FAULT_STATUS = 0x0402
REG_FAULT_STATUS_B = 0x0403
REG_SUPPLY_TEMPERATURE = 0x0405
REG_BUS_VOLTAGE = 0x0407
REG_FEEDBACK = 0x0418
REG_TARGET_POSITION = 0x041C
REG_USER_CURRENT = 0x041B
REG_MAX_SPEED = 0x0521
REG_MAX_CURRENT = 0x0533

MOTOR_STATUS_POSITION_REACHED = 0x0001
MOTOR_STATUS_SPEED_REACHED = 0x0002
MOTOR_STATUS_ZERO_SPEED = 0x0004
MOTOR_STATUS_TORQUE_REACHED = 0x0008
MOTOR_STATUS_READY = 0x0010
MOTOR_STATUS_ENABLED = 0x8000

FAULT_STATUS_B_DROP_ALARM = 0x1000

POSITION_MIN = 0
POSITION_MAX = 1000


def register_can_id(address: int, write: bool, device_id: int = 1) -> int:
    if not 0 <= address <= 0xFFFF:
        raise ValueError('register address must be 0..0xFFFF')
    if not 1 <= device_id <= 0x7F:
        raise ValueError('device id must be 1..127')
    return (address << 8) | (0x80 if write else 0x00) | device_id


def position_payload(position: int) -> bytes:
    """Encode the 32-bit position as high-word then low-word, little-endian."""
    if not 0 <= position <= 0xFFFFFFFF:
        raise ValueError('position must fit in an unsigned 32-bit value')
    high = (position >> 16) & 0xFFFF
    low = position & 0xFFFF
    return high.to_bytes(2, 'little') + low.to_bytes(2, 'little')


def decode_feedback(data: bytes) -> int:
    if len(data) < 4:
        raise ValueError('feedback payload must contain at least four bytes')
    high = int.from_bytes(data[0:2], 'little')
    low = int.from_bytes(data[2:4], 'little')
    return (high << 16) | low


def decode_register_u16(data: bytes) -> int:
    """Decode a 16-bit vendor register response carried in a CAN payload."""
    if len(data) < 2:
        raise ValueError('register response must contain at least two bytes')
    return int.from_bytes(data[0:2], 'little')


def _frame(can_id: int, data: bytes, channel: int, fd: bool, brs: bool) -> CanFrame:
    if not fd and brs:
        raise ValueError('BRS requires a CAN-FD frame')
    return CanFrame(
        channel=channel,
        can_id=can_id,
        data=data,
        extended=True,
        fd=fd,
        brs=brs,
    )


def feedback_request_frame(
        device_id: int = 1,
        channel: int = 0,
        fd: bool = False,
        brs: bool = False) -> CanFrame:
    return register_read_request_frame(
        REG_FEEDBACK,
        device_id=device_id,
        channel=channel,
        fd=fd,
        brs=brs,
        payload_size=4,
    )


def register_read_request_frame(
        address: int,
        device_id: int = 1,
        channel: int = 0,
        fd: bool = False,
        brs: bool = False,
        payload_size: int = 2) -> CanFrame:
    """Build a read request for one vendor register.

    The vendor examples send a zero-filled read payload whose size matches the
    requested value.  The CTAG position feedback uses a four-byte payload while
    scalar status registers use two bytes.
    """
    if not 0 <= payload_size <= 64:
        raise ValueError('payload_size must be 0..64')
    return _frame(
        register_can_id(address, False, device_id),
        bytes(payload_size),
        channel,
        fd,
        brs,
    )


def enable_frame(
        enabled: bool,
        device_id: int = 1,
        channel: int = 0,
        fd: bool = False,
        brs: bool = False) -> CanFrame:
    return _frame(
        register_can_id(REG_ENABLE, True, device_id),
        b'\x01\x00' if enabled else b'\x00\x00',
        channel,
        fd,
        brs,
    )


def alarm_reset_frame(
        device_id: int = 1,
        channel: int = 0,
        fd: bool = False,
        brs: bool = False) -> CanFrame:
    """Build the vendor's write-one alarm reset frame (``0x0110``)."""
    return _frame(
        register_can_id(REG_ALARM_RESET, True, device_id),
        b'\x01\x00',
        channel,
        fd,
        brs,
    )


def scalar_write_frame(
        address: int,
        value: int,
        device_id: int = 1,
        channel: int = 0,
        fd: bool = False,
        brs: bool = False) -> CanFrame:
    """Build a write frame for a 16-bit scalar register."""
    if not 0 <= value <= 0xFFFF:
        raise ValueError('register value must fit in an unsigned 16-bit value')
    return _frame(
        register_can_id(address, True, device_id),
        value.to_bytes(2, 'little'),
        channel,
        fd,
        brs,
    )


def motion_config_frames(
        speed_percent: int,
        torque_percent: int,
        ramp_ms: int,
        device_id: int = 1,
        channel: int = 0,
        fd: bool = False,
        brs: bool = False) -> tuple[CanFrame, CanFrame, CanFrame]:
    """Build vendor-documented speed, torque-limit and ramp writes.

    ``0x0105`` is a maximum torque/current percentage in absolute-position
    mode.  It is not a closed-loop force target, and the vendor register
    table explicitly marks force-control mode (``0x0101 = 4``) unimplemented.
    """
    if not 1 <= speed_percent <= 100:
        raise ValueError('speed_percent must be 1..100')
    if not 1 <= torque_percent <= 100:
        raise ValueError('torque_percent must be 1..100')
    if not 0 <= ramp_ms <= 1000:
        raise ValueError('ramp_ms must be 0..1000')
    return (
        scalar_write_frame(
            REG_SPEED, speed_percent, device_id, channel, fd, brs),
        scalar_write_frame(
            REG_TORQUE, torque_percent, device_id, channel, fd, brs),
        scalar_write_frame(
            REG_ACCEL_RAMP, ramp_ms, device_id, channel, fd, brs),
    )


def motion_frames(
        position: int,
        device_id: int = 1,
        channel: int = 0,
        fd: bool = False,
        brs: bool = False) -> tuple[CanFrame, CanFrame]:
    if not POSITION_MIN <= position <= POSITION_MAX:
        raise ValueError(
            f'position must be {POSITION_MIN}..{POSITION_MAX}, got {position}')
    return (
        _frame(
            register_can_id(REG_POSITION, True, device_id),
            position_payload(position),
            channel,
            fd,
            brs,
        ),
        _frame(
            register_can_id(REG_TRIGGER, True, device_id),
            b'\x01\x00',
            channel,
            fd,
            brs,
        ),
    )


# Geometry from the vendor CTAG2F90-D URDF/kinematics package.  These
# constants describe the four-bar linkage that couples the command register
# (0 = open, 1000 = closed) to the revolute finger joint.
_A = (-30.14132741, 104.14702225)
_C = (-70.50, 141.51248153)
_C_OFFSET = (_C[0] - _A[0], _C[1] - _A[1])
_FINGER_PAD_OFFSET = 51.0


def _rotate_xy(point: tuple[float, float], angle_deg: float) -> tuple[float, float]:
    angle = math.radians(angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return (
        point[0] * cosine - point[1] * sine,
        point[0] * sine + point[1] * cosine,
    )


def _solve_a_cos_b_sin_eq_c(
        a: float,
        b: float,
        c: float,
        lower_deg: float = -90.0,
        upper_deg: float = 0.0) -> Optional[float]:
    radius = math.hypot(a, b)
    if radius == 0.0 or abs(c / radius) > 1.0:
        return None
    phase = math.atan2(b, a)
    alpha = math.acos(c / radius)
    candidates = (phase - alpha, phase + alpha)
    valid = []
    for angle in candidates:
        if abs(a * math.cos(angle) + b * math.sin(angle) - c) > 1e-6:
            continue
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        degrees = math.degrees(angle)
        if lower_deg <= degrees <= upper_deg:
            valid.append(angle)
    if not valid:
        return None
    middle = math.radians((lower_deg + upper_deg) * 0.5)
    return min(valid, key=lambda value: abs(value - middle))


def opening_from_joint_rad(joint_rad: float) -> float:
    """Return the finger-pad opening in millimetres for a joint angle."""
    rotated = _rotate_xy(_C_OFFSET, math.degrees(joint_rad))
    return 2.0 * abs(_A[0] + rotated[0]) - _FINGER_PAD_OFFSET


def joint_from_opening_mm(opening_mm: float) -> Optional[float]:
    """Return the finger joint angle in radians for a pad opening."""
    target_support = opening_mm + _FINGER_PAD_OFFSET
    target_x_c = -target_support / 2.0
    angle = _solve_a_cos_b_sin_eq_c(
        _C_OFFSET[0],
        -_C_OFFSET[1],
        target_x_c - _A[0],
    )
    return angle


def opening_from_position(
        position: int,
        position_min: int = POSITION_MIN,
        position_max: int = POSITION_MAX,
        opening_min_mm: float = 0.0,
        opening_max_mm: float = 90.0) -> float:
    if position_max <= position_min:
        raise ValueError('position_max must be greater than position_min')
    ratio = (float(position) - position_min) / (position_max - position_min)
    ratio = max(0.0, min(1.0, ratio))
    return opening_max_mm - ratio * (opening_max_mm - opening_min_mm)


def joint_from_position(
        position: int,
        position_min: int = POSITION_MIN,
        position_max: int = POSITION_MAX,
        opening_min_mm: float = 0.0,
        opening_max_mm: float = 90.0,
        joint_open_rad: float = 0.0,
        joint_closed_rad: float = -0.90838) -> float:
    opening = opening_from_position(
        position, position_min, position_max, opening_min_mm, opening_max_mm)
    joint = joint_from_opening_mm(opening)
    if joint is None:
        joint = joint_closed_rad
    lower, upper = sorted((joint_open_rad, joint_closed_rad))
    return max(lower, min(upper, joint))


def position_from_joint(
        joint_rad: float,
        position_min: int = POSITION_MIN,
        position_max: int = POSITION_MAX,
        opening_min_mm: float = 0.0,
        opening_max_mm: float = 90.0,
        joint_open_rad: float = 0.0,
        joint_closed_rad: float = -0.90838) -> int:
    lower, upper = sorted((joint_open_rad, joint_closed_rad))
    bounded = max(lower, min(upper, joint_rad))
    opening = opening_from_joint_rad(bounded)
    if opening_max_mm == opening_min_mm:
        return position_min
    ratio = (opening_max_mm - opening) / (opening_max_mm - opening_min_mm)
    ratio = max(0.0, min(1.0, ratio))
    return int(round(position_min + ratio * (position_max - position_min)))
