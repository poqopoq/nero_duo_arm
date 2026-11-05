#!/usr/bin/env python3
"""Direct ZQWL CAN control for the CTAG2F90-D gripper."""
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

from dual_nero_control.ctag_gripper import (  # noqa: E402
    REG_FEEDBACK,
    REG_MAX_CURRENT,
    decode_feedback,
    decode_register_u16,
    enable_frame,
    feedback_request_frame,
    motion_config_frames,
    motion_frames,
    register_read_request_frame,
    scalar_write_frame,
)
from dual_nero_control.zqwl_canfd import (  # noqa: E402
    CanFrame,
    ZqwlCanFd,
    ZqwlError,
)


PORTS = {
    'left': (
        '/dev/serial/by-id/'
        'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00'
    ),
    'right': (
        '/dev/serial/by-id/'
        'usb-ZQWL-CANFD_ZQWL-CANFD_6A649697220A-if00'
    ),
}
CONFIG_PATH = REPO_ROOT / 'deployment' / 'config' / 'gripper_can.json'
DEFAULT_SPEED = 25
DEFAULT_MAX_CURRENT = 200
POSITIONS = {'open': 0, 'close': 980}
TOLERANCE = 10


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


def _read_register(
        client: ZqwlCanFd,
        address: int,
        payload_size: int = 2,
        timeout: float = 0.5) -> bytes | None:
    request = register_read_request_frame(
        address, payload_size=payload_size)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        client.send(request)
        for message in client.messages(0.05):
            if not isinstance(message, CanFrame):
                continue
            if message.can_id & 0x80:
                continue
            received = (message.can_id >> 8) & 0xFFFF
            if received == address:
                return message.data
    return None


def _write_register_verified(
        client: ZqwlCanFd,
        address: int,
        value: int,
        timeout: float = 2.0) -> bool:
    client.send(scalar_write_frame(address, value))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.1)
        data = _read_register(client, address, timeout=0.1)
        if data is not None and decode_register_u16(data) == value:
            return True
    return False


def _read_feedback(client: ZqwlCanFd, timeout: float = 0.5) -> int | None:
    data = _read_register(client, REG_FEEDBACK, payload_size=4, timeout=timeout)
    return decode_feedback(data) if data is not None else None


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


def _wait_for_position(
        client: ZqwlCanFd, start: int, target: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    last = start
    stable_since: float | None = None
    next_print = 0.0
    while time.monotonic() < deadline:
        position = _read_feedback(client)
        now = time.monotonic()
        if position is None:
            continue
        if now >= next_print:
            print(f'feedback={position}', flush=True)
            next_print = now + 0.5
        if abs(position - target) <= TOLERANCE:
            return True
        if position == last:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= 1.0:
                return target > start and position > start + 5
        else:
            last = position
            stable_since = None
        time.sleep(0.05)
    return False


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Direct CAN open/close for the CTAG2F90-D gripper.')
    parser.add_argument('side', choices=('left', 'right'))
    parser.add_argument('action', choices=tuple(POSITIONS))
    parser.add_argument(
        '--keep-enabled',
        action='store_true',
        help='leave the motor and CAN channel enabled after the command',
    )
    parser.add_argument(
        '--speed',
        type=int,
        default=None,
        metavar='PERCENT',
        help='motion speed percentage, 1..100 (default: config value or 25)',
    )
    parser.add_argument(
        '--max-current',
        type=int,
        default=None,
        metavar='VALUE',
        help='0x0533 current-limit register, 1..1000 '
             '(default: config value or 200)',
    )
    return parser.parse_args()


def main() -> int:
    _ensure_dialout_group()
    args = _parse_args()
    side_config = _load_side_config(args.side)
    if args.speed is None:
        args.speed = _config_int(side_config, 'speed', DEFAULT_SPEED)
    if args.max_current is None:
        args.max_current = _config_int(
            side_config, 'max_current', DEFAULT_MAX_CURRENT)
    if not 1 <= args.speed <= 100:
        print('error: --speed must be 1..100', file=sys.stderr)
        return 2
    if not 1 <= args.max_current <= 1000:
        print('error: --max-current must be 1..1000', file=sys.stderr)
        return 2
    target = POSITIONS[args.action]
    client = ZqwlCanFd(port=PORTS[args.side], timeout=0.5)
    configured = False
    enabled = False
    original_max_current: int | None = None
    max_current_changed = False
    try:
        client.open()
        client.configure_can(channel=0, bitrate=1_000_000, fd=False)
        configured = True
        time.sleep(0.2)
        start = _read_feedback(client)
        if start is None:
            print('error: no 0x0418 feedback from gripper', file=sys.stderr)
            return 1
        print(
            f'side={args.side} action={args.action} speed={args.speed}% '
            f'max_current={args.max_current} start={start}',
            flush=True)

        max_current_data = _read_register(client, REG_MAX_CURRENT)
        if max_current_data is None:
            print('error: no 0x0533 response from gripper', file=sys.stderr)
            return 1
        original_max_current = decode_register_u16(max_current_data)
        print(
            f'max_current: {original_max_current} -> {args.max_current}',
            flush=True)
        if original_max_current != args.max_current:
            if not _write_register_verified(
                    client, REG_MAX_CURRENT, args.max_current):
                print('error: 0x0533 write was not confirmed', file=sys.stderr)
                return 1
            max_current_changed = True

        client.send(enable_frame(True))
        enabled = True
        time.sleep(0.2)
        for frame in motion_config_frames(args.speed, 100, 200):
            client.send(frame)
        for frame in motion_frames(target):
            client.send(frame)

        reached = _wait_for_position(client, start, target, timeout=10.0)
        final = _read_feedback(client)
        if reached:
            print(f'ok: target={target} feedback={final}', flush=True)
            return 0
        print(f'error: target={target} feedback={final}', file=sys.stderr)
        return 1
    except (OSError, ZqwlError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    finally:
        if not args.keep_enabled and enabled:
            try:
                client.send(enable_frame(False))
            except (OSError, ZqwlError):
                pass
        if configured and max_current_changed and original_max_current is not None:
            try:
                if not _write_register_verified(
                        client, REG_MAX_CURRENT, original_max_current):
                    print('warning: 0x0533 restore was not confirmed',
                          file=sys.stderr)
            except (OSError, ZqwlError):
                pass
        if not args.keep_enabled and configured:
            try:
                client.disable_can(0)
            except (OSError, ZqwlError):
                pass
        if args.keep_enabled:
            print('motor and CAN channel remain enabled', flush=True)
        client.close()


if __name__ == '__main__':
    raise SystemExit(main())
