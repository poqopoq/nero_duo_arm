#!/usr/bin/env python3
"""Read-only diagnostic probe for the ZQWL UCANFD adapter."""
from __future__ import annotations

import argparse
import sys
import time

from dual_nero_control.zqwl_canfd import (
    CanFrame,
    ConfigResponse,
    DEFAULT_SERIAL_BAUD,
    Heartbeat,
    STABLE_PORT,
    ZqwlCanFd,
    ZqwlError,
    decode_serial_number,
)


def _hex(data: bytes) -> str:
    return ' '.join(f'{value:02X}' for value in data) or '(empty)'


def _parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Query a ZQWL UCANFD adapter and optionally monitor CAN traffic. '
            'This tool never transmits CAN frames.'))
    parser.add_argument('--port', default=STABLE_PORT)
    parser.add_argument(
        '--serial-baud', type=int, default=DEFAULT_SERIAL_BAUD,
        help='host CDC baud rate; the adapter auto-detects it')
    parser.add_argument('--timeout', type=float, default=0.5)
    parser.add_argument(
        '--monitor', action='store_true',
        help='configure and enable CAN during the observation window')
    parser.add_argument('--channel', type=int, default=0)
    parser.add_argument('--bitrate', type=int, default=1_000_000)
    parser.add_argument('--fd', action='store_true')
    parser.add_argument('--data-bitrate', type=int)
    parser.add_argument(
        '--duration', type=float, default=10.0,
        help='CAN observation window in seconds')
    parser.add_argument(
        '--save', action='store_true',
        help='persist the CAN configuration in adapter flash')
    parser.add_argument(
        '--keep-enabled', action='store_true',
        help='do not disable CAN after the observation window')
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    client = ZqwlCanFd(
        port=args.port,
        serial_baudrate=args.serial_baud,
        timeout=args.timeout,
    )
    configured = False
    try:
        client.open()
        print(f'port: {args.port}')
        info = client.read_device_info()
        print(f'device info: {_hex(info)}')
        serial_number = client.read_serial_number()
        print(f'serial raw: {_hex(serial_number)}')
        print(f'serial text: {decode_serial_number(serial_number)}')

        if not args.monitor:
            print('monitor: skipped (pass --monitor to enable CAN receive)')
            return 0

        data_bitrate = args.data_bitrate
        if args.fd and data_bitrate is None:
            data_bitrate = args.bitrate
        print(
            f'configuring CAN{args.channel}: bitrate={args.bitrate} '
            f'fd={args.fd} data_bitrate={data_bitrate} save={args.save}')
        client.configure_can(
            channel=args.channel,
            bitrate=args.bitrate,
            data_bitrate=data_bitrate,
            fd=args.fd,
            save=args.save,
        )
        configured = True
        print(f'observing for {args.duration:.1f} s; Ctrl-C to stop')

        deadline = time.monotonic() + max(0.0, args.duration)
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            for message in client.messages(min(remaining, 0.5)):
                stamp = time.monotonic()
                if isinstance(message, CanFrame):
                    print(f'{stamp:.6f} {message.format()}')
                elif isinstance(message, Heartbeat):
                    detail = ''
                    if message.channels == 2:
                        detail = (
                            f' can0='
                            f'{"on" if message.channel_enabled(0) else "off"}'
                            f' bus0={message.bus_state(0)}'
                            f' can1='
                            f'{"on" if message.channel_enabled(1) else "off"}'
                            f' bus1={message.bus_state(1)}')
                    print(
                        f'{stamp:.6f} heartbeat {_hex(message.raw)}'
                        f'{detail}')
                elif isinstance(message, ConfigResponse):
                    print(
                        f'{stamp:.6f} config function=0x{message.function:02X} '
                        f'data={_hex(message.data)}')
        return 0
    except KeyboardInterrupt:
        return 130
    except ZqwlError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    finally:
        if configured and not args.keep_enabled:
            try:
                client.disable_can(args.channel)
                print('CAN disabled')
            except Exception as exc:
                print(f'warning: could not disable CAN: {exc}', file=sys.stderr)
        client.close()


if __name__ == '__main__':
    raise SystemExit(main())
