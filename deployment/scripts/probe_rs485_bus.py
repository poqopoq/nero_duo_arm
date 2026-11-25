#!/usr/bin/env python3
"""Read-only RS485 bus probe for the OmniPicker gripper and FT sensors.

This tool never sends a motion command.  It only issues:

* Modbus RTU 0x03 (Read Holding Registers) to registers 0 and 20-24, which is
  the read-only status path documented for the OmniPicker gripper.
* The FT force-sensor ``0x04`` serial-number query (``AA 55 04 00 33 2C``).

Any device that answers is reported with the raw frame, the decoded fields when
recognised, and a CRC check.  A silent bus means a wiring / polarity / power /
mode problem, not a protocol problem.
"""
from __future__ import annotations

import argparse
import time

import serial


DEFAULT_BAUDS = (115200, 460800, 921600)
DEFAULT_ADDRESSES = tuple(range(1, 9))
FT_SN_QUERY = bytes((0xAA, 0x55, 0x04, 0x00, 0x33, 0x2C))

FAULT_NAMES = {
    0: 'none', 1: 'overheat', 2: 'overspeed', 3: 'initialization',
    4: 'limit',
}
STATE_NAMES = {
    0: 'reached', 1: 'moving', 2: 'stalled', 3: 'object dropped',
}


def crc16(data: bytes) -> int:
    """Standard Modbus CRC16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def modbus_read(address: int, register: int, count: int) -> bytes:
    body = bytes((
        address, 0x03, (register >> 8) & 0xFF, register & 0xFF,
        (count >> 8) & 0xFF, count & 0xFF,
    ))
    crc = crc16(body)
    return body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def describe(frame: bytes) -> str:
    if len(frame) >= 5 and frame[1] in (0x03, 0x10):
        payload = frame[:-2]
        crc_ok = crc16(payload) == (frame[-1] << 8 | frame[-2])
        return (
            f"Modbus fc=0x{frame[1]:02x} slave={frame[0]} "
            f"crc={'OK' if crc_ok else 'BAD'} raw={frame.hex(' ')}"
        )
    if len(frame) >= 6 and frame[:2] == FT_SN_QUERY[:2] and frame[2] == 0x04:
        return f"FT-sensor SN reply raw={frame.hex(' ')}"
    return f"unrecognised raw={frame.hex(' ')}"


def parse_int_list(value: str) -> list[int]:
    return [int(part, 0) for part in value.replace(' ', '').split(',') if part]


def ee55_frame(can_id: int, data: bytes) -> bytes:
    body = (b'\xee\x55' + can_id.to_bytes(4, 'little')
            + bytes([len(data)]) + data)
    crc = crc16(body)
    return body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def parse_ee55(buffer: bytes) -> list[tuple[int, bytes, bool]]:
    frames = []
    while True:
        start = buffer.find(b'\xee\x55')
        if start < 0 or len(buffer) < start + 9:
            break
        length = buffer[start + 6]
        if length > 64:
            buffer = buffer[start + 1:]
            continue
        if len(buffer) < start + 9 + length:
            break
        raw = buffer[start:start + 9 + length]
        can_id = int.from_bytes(raw[2:6], 'little')
        crc_ok = crc16(raw[:-2]) == int.from_bytes(raw[-2:], 'little')
        frames.append((can_id, raw[7:-2], crc_ok))
        buffer = buffer[start + 9 + length:]
    return frames


def probe_ee55_registers(link: serial.Serial, baud: int) -> int:
    """Scan Reading/EtherCAT-register devices (OmniPicker + Changingtek)."""
    hits = 0
    for register in (0x0000, 0x0100, 0x0200):
        link.reset_input_buffer()
        for device_id in range(0, 128):
            can_id = ((register << 8) | device_id) & 0x1FFFFFFF
            link.write(ee55_frame(can_id, b'\x00\x00'))
        link.flush()
        time.sleep(0.5)
        reply = link.read(link.in_waiting or 0)
        frames = parse_ee55(reply)
        if frames:
            hits += len(frames)
            for can_id, data, crc_ok in frames[:8]:
                print(f'  {baud:>7} bps EE55 reg=0x{register:04x} '
                      f'id=0x{can_id:08x} data={data.hex(" ")} '
                      f'crc={"OK" if crc_ok else "BAD"}')
    return hits


def probe_sdk(port: str, device_id: int) -> int:
    """Try the official AGILINK SDK, which is how OP3 actually talks RS485."""
    try:
        from omnihand import HandType, OmniPicker3
    except ImportError:
        print('  SDK: omnihand not installed; skipped')
        return 0
    try:
        picker = OmniPicker3.create_hand_by_rs485(
            hand_type=HandType.UNKNOWN,
            hand_device_id=device_id,
            serial_port=port,
            baud_rate=460800,
        )
        picker.set_request_interval(5)
        picker.set_frame_recv_timeout(200)
        if not picker.init():
            print('  SDK @460800: init returned False')
            return 0
        print(f'  SDK @460800: init OK, position='
              f'{picker.get_joint_position(1)}')
        return 1
    except Exception as exc:  # noqa: BLE001
        print(f'  SDK @460800: {exc}')
        return 0


def probe_port(
        port: str, bauds: list[int], addresses: list[int],
        timeout: float) -> int:
    print(f"\n=== {port} ===")
    hits = 0
    for baud in bauds:
        try:
            link = serial.Serial(port, baud, timeout=timeout, exclusive=True)
        except Exception as exc:  # noqa: BLE001 - report any open failure
            print(f"  {baud:>7} bps: OPEN FAILED: {exc}")
            continue
        with link:
            for address in addresses:
                for request in (
                        modbus_read(address, 0x0000, 1),
                        modbus_read(address, 0x0014, 5),
                        FT_SN_QUERY):
                    link.reset_input_buffer()
                    link.write(request)
                    link.flush()
                    time.sleep(timeout)
                    reply = link.read(64)
                    if reply:
                        hits += 1
                        print(
                            f"  {baud:>7} bps addr={address}: "
                            f"sent {request.hex(' ')} -> {describe(reply)}")
            hits += probe_ee55_registers(link, baud)
    hits += probe_sdk(port, addresses[0] if addresses else 1)
    if not hits:
        print("  no reply on any baud/address and SDK probe failed")
    return hits


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Read-only RS485 probe: Modbus 0x03 + FT sensor SN query. '
            'Sends no motion command.'))
    parser.add_argument('ports', nargs='+', help='Serial ports, e.g. /dev/ttyUSB6')
    parser.add_argument(
        '--baud', default=','.join(str(b) for b in DEFAULT_BAUDS),
        help='Comma-separated baud rates (default 115200,460800,921600)')
    parser.add_argument(
        '--addr', default=','.join(str(a) for a in DEFAULT_ADDRESSES),
        help='Comma-separated Modbus slave addresses (default 1..8)')
    parser.add_argument('--timeout', type=float, default=0.2)
    args = parser.parse_args()

    bauds = parse_int_list(args.baud)
    addresses = parse_int_list(args.addr)

    total = 0
    for port in args.ports:
        total += probe_port(port, bauds, addresses, args.timeout)

    print(f"\nTotal replies: {total}")
    return 0 if total else 1


if __name__ == '__main__':
    raise SystemExit(main())
