#!/usr/bin/env python3
"""Direct CAN-FD open/close/status tool for AGILINK OmniPicker 3."""
from __future__ import annotations

import argparse
import grp
import json
import os
import shlex
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'ros2_ws' / 'src' / 'dual_nero_control'))

from dual_nero_control.omnipicker_gripper import (  # noqa: E402
    DEFAULT_BITRATE,
    DEFAULT_DATA_BITRATE,
    STATE_DROPPED,
    STATE_STALLED,
    CanTransportError,
    SocketCanTransport,
    ZqwlCanTransport,
    control_frame,
    decode_position_query,
    decode_status,
    position_from_can,
    position_query_frame,
)
from dual_nero_control.zqwl_canfd import (  # noqa: E402
    DEFAULT_SERIAL_BAUD,
    STABLE_PORT,
    ZqwlError,
)


PORTS = {
    'left': (
        '/dev/serial/by-id/'
        'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00'
    ),
    'right': STABLE_PORT,
}
CONFIG_PATH = REPO_ROOT / 'deployment' / 'config' / 'gripper_can.json'
POSITIONS = {'open': 0, 'close': 1000}
DEFAULT_SPEED = 50
DEFAULT_FORCE = 100
DEFAULT_ACCELERATION = 100
DEFAULT_DECELERATION = 100
DEFAULT_TOLERANCE = 8


def _ensure_dialout_group() -> None:
    try:
        gid = grp.getgrnam('dialout').gr_gid
    except KeyError:
        return
    if gid in os.getgroups() or os.environ.get('GRIPPER_CAN_REEXEC') == '1':
        return
    command = shlex.join([sys.executable, *sys.argv])
    env = dict(os.environ, GRIPPER_CAN_REEXEC='1')
    os.execvpe('sg', ['sg', 'dialout', '-c', command], env)


def _load_side_config(side: str) -> dict[str, object]:
    try:
        data = json.loads(CONFIG_PATH.read_text())
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError) as exc:
        print(f'warning: cannot read {CONFIG_PATH}: {exc}', file=sys.stderr)
        return {}
    if not isinstance(data, dict):
        print(f'warning: {CONFIG_PATH} must contain a JSON object',
              file=sys.stderr)
        return {}
    side_config = data.get(side, {})
    if not isinstance(side_config, dict):
        print(f'warning: {CONFIG_PATH} side {side!r} must be an object',
              file=sys.stderr)
        return {}
    return side_config


def _config_int(
        config: dict[str, object], key: str, default: int) -> int:
    value = config.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        print(f'warning: {CONFIG_PATH} {key} must be an integer; '
              f'using {default}', file=sys.stderr)
        return default
    return value


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Direct OmniPicker 3 CAN-FD control.')
    parser.add_argument('side', choices=('left', 'right'))
    parser.add_argument(
        'action', choices=('open', 'close', 'position', 'status'))
    parser.add_argument(
        '--target', type=int, default=None,
        help='ROS raw position for action=position (0=open, 1000=closed)')
    parser.add_argument(
        '--transport', choices=('zqwl', 'socketcan'), default='zqwl')
    parser.add_argument('--port', default=None, help='ZQWL serial device')
    parser.add_argument(
        '--interface', default=None,
        help='SocketCAN interface (default: can0 for left, can1 for right)')
    parser.add_argument(
        '--device-id', type=int, default=None,
        help='OP3 standard CAN node ID (default: config value or 1)')
    parser.add_argument(
        '--speed', type=int, default=None,
        help='motion speed percentage, 1..100')
    parser.add_argument(
        '--force', type=int, default=None,
        help='motion force/torque percentage, 1..100')
    parser.add_argument(
        '--acceleration', type=int, default=None,
        help='acceleration percentage, 1..100')
    parser.add_argument(
        '--deceleration', type=int, default=None,
        help='deceleration percentage, 1..100')
    parser.add_argument(
        '--tolerance', type=int, default=DEFAULT_TOLERANCE,
        help='feedback tolerance in the ROS 0..1000 position contract')
    parser.add_argument(
        '--timeout', type=float, default=10.0,
        help='motion timeout in seconds')
    parser.add_argument(
        '--classic-can', action='store_true',
        help='send classic CAN instead of CAN-FD (OP3 compatibility mode)')
    parser.add_argument(
        '--keep-enabled', action='store_true',
        help='leave the ZQWL CAN channel configured after the command')
    return parser.parse_args()


def _validate_percent(name: str, value: int) -> None:
    if not 1 <= value <= 100:
        raise ValueError(f'--{name} must be 1..100')


def _wait_for_position(
        transport,
        device_id: int,
        target: int,
        speed: int,
        force: int,
        acceleration: int,
        deceleration: int,
        can_fd: bool,
        brs: bool,
        tolerance: int,
        timeout: float) -> tuple[bool, int | None, object | None]:
    deadline = time.monotonic() + timeout
    last_position = None
    last_status = None
    next_print = 0.0
    while time.monotonic() < deadline:
        if can_fd:
            transport.send(position_query_frame(
                device_id=device_id, brs=brs))
        transport.send(control_frame(
            target,
            speed_percent=speed,
            force_percent=force,
            acceleration_percent=acceleration,
            deceleration_percent=deceleration,
            device_id=device_id,
            fd=can_fd,
            brs=brs,
        ))
        for frame in transport.receive(0.03):
            queried = decode_position_query(frame, device_id)
            if queried is not None:
                last_position = int(round(
                    (1.0 - min(1.0, max(0.0, queried / 4095.0))) * 1000))
                if abs(last_position - target) <= tolerance:
                    return True, last_position, last_status
                continue
            status = decode_status(frame, device_id)
            if status is None:
                continue
            last_status = status
            last_position = position_from_can(status.position_byte)
            now = time.monotonic()
            if now >= next_print:
                print(
                    f'position={last_position} byte={status.position_byte} '
                    f'state={status.state_name} fault={status.fault_name}',
                    flush=True)
                next_print = now + 0.5
            if status.fault_code:
                return False, last_position, status
            if status.state_code == STATE_DROPPED:
                return False, last_position, status
            if abs(last_position - target) <= tolerance:
                return True, last_position, status
            if status.state_code == STATE_STALLED:
                closing = target > (last_position or 0)
                return closing, last_position, status
        time.sleep(0.02)
    return False, last_position, last_status


def main() -> int:
    args = _parse_args()
    if args.transport == 'zqwl':
        _ensure_dialout_group()
    side_config = _load_side_config(args.side)
    device_id = args.device_id if args.device_id is not None else _config_int(
        side_config, 'device_id', 1)
    speed = args.speed if args.speed is not None else _config_int(
        side_config, 'speed', DEFAULT_SPEED)
    force = args.force if args.force is not None else _config_int(
        side_config, 'force_percent', DEFAULT_FORCE)
    acceleration = (
        args.acceleration if args.acceleration is not None else _config_int(
            side_config, 'acceleration_percent', DEFAULT_ACCELERATION))
    deceleration = (
        args.deceleration if args.deceleration is not None else _config_int(
            side_config, 'deceleration_percent', DEFAULT_DECELERATION))
    try:
        _validate_percent('speed', speed)
        _validate_percent('force', force)
        _validate_percent('acceleration', acceleration)
        _validate_percent('deceleration', deceleration)
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    if not 1 <= device_id <= 0x7FF:
        print('error: --device-id must be 1..0x7FF', file=sys.stderr)
        return 2
    if args.tolerance < 0 or args.timeout <= 0.0:
        print('error: tolerance must be >=0 and timeout >0', file=sys.stderr)
        return 2
    if args.action == 'position':
        if args.target is None:
            print('error: action=position requires --target', file=sys.stderr)
            return 2
        target = max(0, min(1000, args.target))
    elif args.action == 'status':
        target = None
    else:
        target = POSITIONS[args.action]
    can_fd = not args.classic_can
    brs = can_fd

    if args.transport == 'zqwl':
        port = args.port or PORTS[args.side]
        transport = ZqwlCanTransport(
            port=port,
            serial_baudrate=DEFAULT_SERIAL_BAUD,
            bitrate=DEFAULT_BITRATE,
            data_bitrate=DEFAULT_DATA_BITRATE,
            fd=can_fd,
        )
        endpoint = port
    else:
        iface = args.interface or ('can0' if args.side == 'left' else 'can1')
        transport = SocketCanTransport(iface, fd=can_fd, brs=brs)
        endpoint = iface

    configured = False
    try:
        transport.open()
        configured = True
        mode = 'CAN-FD 1M/5M BRS' if can_fd else 'classic CAN 1M'
        print(
            f'side={args.side} action={args.action} transport='
            f'{args.transport} endpoint={endpoint} mode={mode} '
            f'id={device_id}',
            flush=True)
        if target is None:
            if not can_fd:
                print(
                    'error: status query requires CAN-FD; the read-only OP3 '
                    'position request is an extended 64-byte frame',
                    file=sys.stderr)
                return 2
            deadline = time.monotonic() + min(args.timeout, 2.0)
            while time.monotonic() < deadline:
                transport.send(position_query_frame(
                    device_id=device_id, brs=brs))
                found = None
                for frame in transport.receive(0.1):
                    found = decode_position_query(frame, device_id)
                    if found is not None:
                        break
                if found is not None:
                    position = int(round(
                        (1.0 - min(1.0, max(0.0, found / 4095.0))) * 1000))
                    print(
                        f'ok: position_raw={found} position={position}',
                        flush=True)
                    return 0
            print('error: no read-only position response', file=sys.stderr)
            return 1
        reached, final, status = _wait_for_position(
            transport, device_id, target, speed, force,
            acceleration, deceleration, can_fd, brs,
            args.tolerance, args.timeout)
        if reached:
            suffix = (
                f' state={status.state_name} fault={status.fault_name}'
                if status is not None else '')
            print(f'ok: target={target} feedback={final}{suffix}', flush=True)
            return 0
        suffix = (
            f' state={status.state_name} fault={status.fault_name}'
            if status is not None else '')
        print(
            f'error: target={target} feedback={final}{suffix}',
            file=sys.stderr)
        return 1
    except (OSError, ZqwlError, CanTransportError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    finally:
        if not args.keep_enabled and configured:
            if hasattr(transport, 'disable'):
                transport.disable()
            transport.close()
        elif args.keep_enabled:
            print('CAN channel remains configured', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
