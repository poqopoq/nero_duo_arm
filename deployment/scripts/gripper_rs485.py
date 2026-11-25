#!/usr/bin/env python3
"""Direct RS485 control tool for AGILINK OmniPicker 3.

Uses the official AGILINK ``omnihand`` Python SDK.  The OP3 RS485 link runs at
a fixed 460800 baud and speaks the full SDK protocol (not Modbus and not the
simplified 0x4141 frame from the product manual).

Install once::

    pip install --user /path/to/omnihand-<ver>-cp310-*.whl

Actions:
    status     read-only: position, stroke, error, current
    open       move to the fully open position
    close      move to the fully closed position
    position   move to --target in the ROS 0..1000 contract
"""
from __future__ import annotations

import argparse
import sys
import time

try:
    from omnihand import HandType, OmniPicker3
except ImportError:  # pragma: no cover - user-facing guidance
    print('error: omnihand SDK not installed. Install the AGILINK wheel with\n'
          '  /usr/bin/python3 -m pip install --user '
          '/path/to/omnihand-<ver>-cp310-linux_x86_64.whl', file=sys.stderr)
    raise SystemExit(2)


DEFAULT_PORT = (
    '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BH00URZP-if00-port0')
# OP3 RS485 is fixed at 460800; the SDK rejects any other value.
DEFAULT_BAUD = 460800
DEFAULT_DEVICE_ID = 1
DEFAULT_TOLERANCE = 40  # wire ticks, ~1% of the 0..4095 range
DEFAULT_TIMEOUT = 5.0

WIRE_MIN = 0
WIRE_MAX = 4095
ROS_OPEN = 0
ROS_CLOSED = 1000


def ros_to_wire(target: int) -> int:
    """ROS 0=open..1000=closed -> OP3 0..4095 (0=closed, 4095=open)."""
    target = max(ROS_OPEN, min(ROS_CLOSED, target))
    return int(round((1.0 - target / ROS_CLOSED) * WIRE_MAX))


def make_picker(port: str, device_id: int, baud: int):
    picker = OmniPicker3.create_hand_by_rs485(
        hand_type=HandType.UNKNOWN,
        hand_device_id=device_id,
        serial_port=port,
        baud_rate=baud,
    )
    picker.set_request_interval(5)
    picker.set_frame_recv_timeout(200)
    if not picker.init():
        raise RuntimeError(f'OmniPicker3 init failed on {port} @ {baud}')
    return picker


def read_status(picker) -> dict:
    status = {
        'position': picker.get_joint_position(1),
        'stroke': picker.get_stroke(),
    }
    try:
        status['error'] = picker.get_error_report(1)
    except Exception:  # noqa: BLE001 - optional on some firmware
        status['error'] = None
    try:
        status['current'] = picker.get_current_report(1)
    except Exception:  # noqa: BLE001
        status['current'] = None
    return status


def describe(status: dict) -> str:
    return (f"position={status['position']}/{WIRE_MAX} "
            f"stroke={status['stroke']*0.01:.2f}mm "
            f"error={status['error']} current={status['current']}mA")


def move(picker, wire_target: int, tolerance: int, timeout: float) -> bool:
    # set_joint_position uses the firmware default current.  mix_control_by_pt
    # (position + current limit) did not move this OP3 firmware in testing.
    picker.set_joint_position(1, wire_target)
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        time.sleep(0.05)
        last = picker.get_joint_position(1)
        if abs(last - wire_target) <= tolerance:
            return True
    print(f'error: timeout target={wire_target} last={last}', file=sys.stderr)
    return False


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Direct OmniPicker 3 RS485 control via the official SDK.')
    parser.add_argument('action', choices=('status', 'open', 'close', 'position'))
    parser.add_argument('--target', type=int, default=None,
                        help='ROS position 0=open..1000=closed (action=position)')
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    parser.add_argument('--device-id', type=int, default=DEFAULT_DEVICE_ID)
    parser.add_argument('--tolerance', type=int, default=DEFAULT_TOLERANCE)
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.action == 'position' and args.target is None:
        print('error: action=position requires --target', file=sys.stderr)
        return 2
    if args.baud != DEFAULT_BAUD:
        print('error: OmniPicker3 RS485 uses a fixed 460800 baud',
              file=sys.stderr)
        return 2
    if not 1 <= args.device_id <= 0x7FF:
        print('error: --device-id must be 1..0x7FF', file=sys.stderr)
        return 2

    try:
        picker = make_picker(args.port, args.device_id, args.baud)
    except Exception as exc:  # noqa: BLE001
        print(f'error: cannot open {args.port} @ {args.baud}: {exc}',
              file=sys.stderr)
        return 1

    if args.action == 'status':
        print('ok: ' + describe(read_status(picker)), flush=True)
        return 0

    if args.action == 'open':
        wire_target = WIRE_MAX
    elif args.action == 'close':
        wire_target = WIRE_MIN
    else:
        wire_target = ros_to_wire(args.target)

    print(f'ros_target={args.target} wire_target={wire_target}', flush=True)
    ok = move(picker, wire_target, args.tolerance, args.timeout)
    print(('ok: ' if ok else 'failed: ') + describe(read_status(picker)),
          flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
