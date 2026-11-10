#!/usr/bin/env python3
"""Identify connected gripper force sensors by their hardware serial number."""

from __future__ import annotations

import argparse
import glob
import os
from pathlib import Path
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
SENSOR_PACKAGE = REPO_ROOT / "ros2_ws/src/dual_nero_gripper_ft_sensor"
sys.path.insert(0, str(SENSOR_PACKAGE))

import serial

from dual_nero_gripper_ft_sensor.protocol import (
    CMD_GET_FIRMWARE_VERSION,
    CMD_GET_SERIAL_NUMBER,
    FrameParser,
    build_frame,
    decode_serial_number,
    format_firmware_version,
)


def path_for_port(port: str) -> str:
    target = os.path.realpath(port)
    for by_path in sorted(glob.glob("/dev/serial/by-path/*")):
        if os.path.realpath(by_path) == target:
            return by_path
    return "-"


def identify(
        port: str, baud: int, timeout: float,
        sn_only: bool = False) -> tuple[str, str, int]:
    parser = FrameParser()
    serial_number = ""
    firmware_version = ""
    measurement_frames = 0

    with serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05,
            write_timeout=1.0,
            exclusive=True,
    ) as link:
        link.reset_input_buffer()
        commands = (
            (CMD_GET_SERIAL_NUMBER,)
            if sn_only
            else (CMD_GET_SERIAL_NUMBER, CMD_GET_FIRMWARE_VERSION)
        )
        for command in commands:
            link.write(build_frame(command))
        link.flush()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for frame in parser.feed(link.read(4096)):
                if frame.message_type == CMD_GET_SERIAL_NUMBER:
                    serial_number = decode_serial_number(frame.data)
                elif frame.message_type == CMD_GET_FIRMWARE_VERSION:
                    firmware_version = format_firmware_version(frame.data)
                elif frame.message_type == 0x01:
                    measurement_frames += 1
            if serial_number and (sn_only or firmware_version):
                break

    return serial_number, firmware_version, measurement_frames


def main() -> int:
    argument_parser = argparse.ArgumentParser(
        description="Read gripper force-sensor hardware SN from serial ports")
    argument_parser.add_argument(
        "ports", nargs="*", help="Serial ports; default: /dev/ttyUSB*")
    argument_parser.add_argument("--baud", type=int, default=921600)
    argument_parser.add_argument("--timeout", type=float, default=1.5)
    argument_parser.add_argument(
        "--sn-only", action="store_true",
        help="Send only command 0x04 and wait for the serial number")
    args = argument_parser.parse_args()

    ports = args.ports or sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        print("No serial ports found. Connect the sensor adapters first.")
        return 2

    identified = 0
    print("port\tby-path\tserial_number\tfirmware", flush=True)
    for port in ports:
        try:
            serial_number, firmware, measurement_frames = identify(
                port, args.baud, args.timeout, sn_only=args.sn_only)
        except PermissionError:
            print(
                f"{port}\t{path_for_port(port)}\tPERMISSION_DENIED\t-\n"
                "Use: sg dialout -c '/usr/bin/python3 "
                f"{Path(__file__).resolve()} {' '.join(ports)}'",
                file=sys.stderr,
                flush=True,
            )
            continue
        except serial.SerialException as exc:
            print(
                f"{port}\t{path_for_port(port)}\tOPEN_FAILED:{exc}\t-",
                file=sys.stderr,
                flush=True,
            )
            continue

        if serial_number:
            identified += 1
            print(
                f"{port}\t{path_for_port(port)}\t{serial_number}\t"
                f"{firmware or '-'}"
            )
        else:
            status = (
                "DATA_ONLY_NO_SN" if measurement_frames
                else "NO_DATA_NO_SN"
            )
            print(
                f"{port}\t{path_for_port(port)}\t{status}\t"
                f"{firmware or '-'}",
                file=sys.stderr,
                flush=True,
            )
            if measurement_frames:
                print(
                    f"{port}: data link works; connect sensor yellow RX to "
                    "adapter TX to read the SN.",
                    file=sys.stderr,
                    flush=True,
                )
            else:
                print(
                    f"{port}: no data and no SN; verify sensor power, common "
                    "ground, black TX -> adapter RX, and yellow RX -> adapter TX.",
                    file=sys.stderr,
                    flush=True,
                )

    return 0 if identified else 1


if __name__ == "__main__":
    raise SystemExit(main())
