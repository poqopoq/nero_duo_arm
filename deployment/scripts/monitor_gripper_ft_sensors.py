#!/usr/bin/env python3
"""Monitor gripper force-sensor sample rate, touch response, and CRC errors."""

from __future__ import annotations

import argparse
import glob
import os
from pathlib import Path
import sys
import threading
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
SENSOR_PACKAGE = REPO_ROOT / "ros2_ws/src/dual_nero_gripper_ft_sensor"
sys.path.insert(0, str(SENSOR_PACKAGE))

import serial

from dual_nero_gripper_ft_sensor.protocol import (
    CMD_MEASUREMENT,
    FrameParser,
    decode_measurements,
)


AXIS_COUNT = 6


def path_for_port(port: str) -> str:
    target = os.path.realpath(port)
    for by_path in sorted(glob.glob("/dev/serial/by-path/*")):
        if os.path.realpath(by_path) == target:
            return by_path
    return "-"


def sample_span(samples: list[tuple[float, ...]]) -> list[float]:
    if not samples:
        return [0.0] * AXIS_COUNT
    return [
        max(sample[axis] for sample in samples)
        - min(sample[axis] for sample in samples)
        for axis in range(AXIS_COUNT)
    ]


def monitor_port(
        port: str,
        baud: int,
        interval: float,
        stop: threading.Event,
        state: dict,
        lock: threading.Lock,
) -> None:
    parser = FrameParser()
    interval_start = time.monotonic()
    interval_count = 0
    interval_changed = 0
    interval_samples: list[tuple[float, ...]] = []
    previous: tuple[float, ...] | None = None
    latest: tuple[float, ...] | None = None
    total_count = 0
    total_changed = 0
    max_spans = [0.0] * AXIS_COUNT

    try:
        with serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05,
                exclusive=True,
        ) as link:
            link.reset_input_buffer()
            while not stop.is_set():
                data = link.read(1)
                if data:
                    waiting = link.in_waiting
                    if waiting:
                        data += link.read(waiting)
                    for frame in parser.feed(data):
                        if frame.message_type != CMD_MEASUREMENT:
                            continue
                        try:
                            values = decode_measurements(frame.data)
                        except ValueError:
                            continue

                        previous = latest
                        latest = values
                        interval_count += 1
                        total_count += 1
                        interval_samples.append(values)
                        if previous is not None and values != previous:
                            interval_changed += 1
                            total_changed += 1

                now = time.monotonic()
                elapsed = now - interval_start
                if elapsed < interval:
                    continue

                spans = sample_span(interval_samples)
                for axis, span in enumerate(spans):
                    max_spans[axis] = max(max_spans[axis], span)
                touch_axis = max(range(AXIS_COUNT), key=spans.__getitem__)
                with lock:
                    state[port] = {
                        "rate_hz": interval_count / elapsed,
                        "changed_hz": interval_changed / elapsed,
                        "values": latest,
                        "spans": spans,
                        "touch_axis": touch_axis,
                        "touch_span": spans[touch_axis],
                        "crc_errors": parser.stats.crc_errors,
                    }

                interval_start = now
                interval_count = 0
                interval_changed = 0
                interval_samples = []

        with lock:
            item = state.setdefault(port, {})
            item.update({
                "stopped": True,
                "total_count": total_count,
                "total_changed": total_changed,
                "max_spans": max_spans,
            })
    except (OSError, serial.SerialException) as exc:
        with lock:
            state[port] = {
                "error": str(exc),
                "stopped": True,
                "total_count": total_count,
                "total_changed": total_changed,
                "max_spans": max_spans,
            }


def format_values(values: tuple[float, ...] | None) -> str:
    if values is None:
        return "no frame"
    return " ".join(f"{value:8.3f}" for value in values)


def print_live(
        ports: list[str], state: dict, lock: threading.Lock,
        threshold: float) -> None:
    with lock:
        lines = []
        for port in ports:
            item = state.get(port)
            if item is None:
                continue
            if "error" in item:
                lines.append(f"{port}: ERROR {item['error']}")
                continue
            if item.get("values") is None:
                lines.append(f"{port}: no frame")
                continue

            spans = " ".join(
                f"{span:7.3f}" for span in item["spans"])
            marker = "  <== TOUCH" if item["touch_span"] >= threshold else ""
            lines.append(
                f"{port}: rate={item['rate_hz']:7.1f}Hz "
                f"changed={item['changed_hz']:7.1f}Hz "
                f"span[{item['touch_axis']}]={item['touch_span']:8.3f} "
                f"crc={item['crc_errors']} "
                f"spans=[{spans}] | {format_values(item['values'])}"
                f"{marker}")
    print("\n".join(lines), flush=True)


def print_summary(
        ports: list[str], state: dict, lock: threading.Lock,
        elapsed: float) -> None:
    print("\nSUMMARY", flush=True)
    print(
        "port\taverage_hz\tchanged_hz\tmax_span[axis]\tcrc\ttotal_frames",
        flush=True,
    )
    with lock:
        for port in ports:
            item = state.get(port, {})
            total = int(item.get("total_count", 0))
            if "error" in item:
                print(f"{port}\tOPEN_FAILED:{item['error']}", flush=True)
                continue
            if total == 0:
                print(f"{port}\tNO_DATA", flush=True)
                continue
            max_spans = item.get("max_spans", [0.0] * AXIS_COUNT)
            axis = max(range(AXIS_COUNT), key=max_spans.__getitem__)
            print(
                f"{port}\t{total / elapsed:.1f}\t"
                f"{item.get('total_changed', 0) / elapsed:.1f}\t"
                f"{max_spans[axis]:.3f}[{axis}]\t"
                f"{item.get('crc_errors', 0)}\t{total}",
                flush=True,
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Monitor gripper force-sensor frame rate and touch response. "
            "Run with /usr/bin/python3 so pyserial is available."))
    parser.add_argument(
        "ports", nargs="*",
        help="Serial ports; default: all /dev/ttyUSB*")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument(
        "--duration", type=float, default=30.0,
        help="Monitoring duration in seconds (default: 30)")
    parser.add_argument(
        "--interval", type=float, default=0.5,
        help="Display interval in seconds (default: 0.5)")
    parser.add_argument(
        "--threshold", type=float, default=0.5,
        help="Highlight touch when the largest axis span exceeds this value")
    args = parser.parse_args()

    if args.duration <= 0.0 or args.interval <= 0.0:
        print("error: duration and interval must be positive", file=sys.stderr)
        return 2

    ports = args.ports or sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        print("No serial ports found. Connect the sensor adapters first.")
        return 2

    print(f"Monitoring {len(ports)} port(s) for {args.duration:.1f} s", flush=True)
    print("Touch the sensor for 2-3 seconds and watch the TOUCH marker.", flush=True)
    for port in ports:
        print(f"  {port} -> {path_for_port(port)}", flush=True)
    print(
        "rate=frame rate, changed=rate of frames whose six values changed, "
        "span=peak-to-peak range in the display window",
        flush=True,
    )

    stop = threading.Event()
    state: dict = {}
    lock = threading.Lock()
    threads = [
        threading.Thread(
            target=monitor_port,
            args=(port, args.baud, args.interval, stop, state, lock),
            daemon=True,
        )
        for port in ports
    ]
    for thread in threads:
        thread.start()

    start = time.monotonic()
    try:
        deadline = start + args.duration
        while time.monotonic() < deadline:
            time.sleep(args.interval)
            print_live(ports, state, lock, args.threshold)
            print("-" * 100, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = max(time.monotonic() - start, 1e-9)
        stop.set()
        for thread in threads:
            thread.join(timeout=1.0)
        print_summary(ports, state, lock, elapsed)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
