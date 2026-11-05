from __future__ import annotations

import os
import select
import threading
import time
import unittest

from dual_nero_control.zqwl_canfd import (
    ARBITRATION_CODE_TO_BITRATE,
    ARBITRATION_BITRATE_TO_CODE,
    CONFIG_TAIL,
    DATA_CODE_TO_BITRATE,
    DATA_BITRATE_TO_CODE,
    FUNC_CAN_PARAMETERS,
    FUNC_DEVICE_INFO,
    FUNC_HEARTBEAT,
    FUNC_HEARTBEAT_4CH,
    FUNC_SERIAL_NUMBER,
    FUNC_SYSTEM_CONTROL,
    CanFrame,
    ConfigResponse,
    FrameParser,
    Heartbeat,
    ZqwlCanFd,
    decode_serial_number,
    encode_can_frame,
    encode_can_parameters,
    encode_config_request,
    encode_system_control,
)


def _config_payload(data: bytes) -> bytes:
    return bytes(data).ljust(16, b'\x00')


def _read_exact(fd: int, count: int, timeout: float = 2.0) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + timeout
    while len(data) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(f'expected {count} bytes, got {len(data)}')
        ready, _, _ = select.select([fd], [], [], remaining)
        if not ready:
            raise TimeoutError(f'expected {count} bytes, got {len(data)}')
        chunk = os.read(fd, count - len(data))
        if not chunk:
            raise EOFError('PTY closed while reading')
        data.extend(chunk)
    return bytes(data)


class PtyAdapter:
    """Emulate the adapter's CDC endpoint through a pseudo-terminal."""

    def __init__(self, handler) -> None:
        self.master_fd, self.slave_fd = os.openpty()
        self.path = os.ttyname(self.slave_fd)
        self._handler = handler
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            self._handler(self.master_fd)
        except BaseException as exc:  # propagate assertion failures to the test
            self._error = exc

    def close(self) -> None:
        os.close(self.slave_fd)
        self._thread.join(timeout=3.0)
        os.close(self.master_fd)
        if self._thread.is_alive():
            raise AssertionError('PTY adapter handler did not finish')
        if self._error is not None:
            raise self._error


class ConfigRequestEncodingTest(unittest.TestCase):
    def test_device_info_query_matches_vendor_example(self) -> None:
        expected = bytes.fromhex(
            '49 3B 40 52 00 00 00 00 00 00 00 00 00 00 00 00 '
            '00 00 00 00 45 2E')
        self.assertEqual(encode_config_request(FUNC_DEVICE_INFO), expected)

    def test_serial_number_query_matches_vendor_example(self) -> None:
        expected = bytes.fromhex(
            '49 3B 41 52 00 00 00 00 00 00 00 00 00 00 00 00 '
            '00 00 00 00 45 2E')
        self.assertEqual(encode_config_request(FUNC_SERIAL_NUMBER), expected)

    def test_can_parameter_example_uses_data_domain_baud_code(self) -> None:
        expected = bytes.fromhex(
            '49 3B 42 57 00 00 45 00 00 00 00 00 00 00 00 00 '
            '00 00 00 00 45 2E')
        self.assertEqual(
            encode_can_parameters(
                channel=0, bitrate=250_000, data_bitrate=500_000),
            expected)

    def test_can_parameter_channel_and_same_rate_fd(self) -> None:
        encoded = encode_can_parameters(
            channel=1, bitrate=1_000_000, data_bitrate=1_000_000, fd=True)
        self.assertEqual(encoded[2], FUNC_CAN_PARAMETERS)
        self.assertEqual(encoded[3], 0x57)
        self.assertEqual(encoded[4], 1)
        self.assertEqual(encoded[5], 0)
        self.assertEqual(encoded[6], 0x03)
        self.assertEqual(encoded[-2:], CONFIG_TAIL)

    def test_system_control_example_matches_vendor_example(self) -> None:
        expected = bytes.fromhex(
            '49 3B 44 57 01 00 01 00 00 00 00 00 00 00 00 00 '
            '00 00 00 00 45 2E')
        self.assertEqual(
            encode_system_control(
                channel=0, enable=True, save=True),
            expected)

    def test_system_control_selects_channel(self) -> None:
        self.assertEqual(
            encode_system_control(channel=1, enable=True)[4:8],
            bytes((0, 0, 0, 1)))
        self.assertEqual(
            encode_system_control(channel=0, enable=False)[4:8],
            bytes((0, 0, 0, 0)))

    def test_parameter_validation(self) -> None:
        with self.assertRaises(ValueError):
            encode_can_parameters(channel=4, bitrate=1_000_000)
        with self.assertRaises(ValueError):
            encode_can_parameters(channel=0, bitrate=123_456)
        with self.assertRaises(ValueError):
            encode_can_parameters(
                channel=0, bitrate=1_000_000, data_bitrate=123_456)
        with self.assertRaises(ValueError):
            encode_can_parameters(channel=0, bitrate=1_000_000, fd=True)

    def test_baud_code_tables_are_distinct(self) -> None:
        self.assertEqual(ARBITRATION_CODE_TO_BITRATE[0x4], 250_000)
        self.assertEqual(DATA_CODE_TO_BITRATE[0x5], 500_000)
        self.assertEqual(ARBITRATION_BITRATE_TO_CODE[250_000], 0x4)
        self.assertEqual(DATA_BITRATE_TO_CODE[500_000], 0x5)
        self.assertNotEqual(
            ARBITRATION_BITRATE_TO_CODE[500_000],
            DATA_BITRATE_TO_CODE[500_000])


class FrameEncodingTest(unittest.TestCase):
    def test_standard_frame_layout(self) -> None:
        frame = CanFrame(channel=0, can_id=0x123, data=b'\x01\x02\x03')
        encoded = encode_can_frame(frame)
        self.assertEqual(encoded[0], 0x5A)
        self.assertEqual(encoded[1], 0x03)
        self.assertEqual(encoded[2], 0x00)
        self.assertEqual(encoded[3:7], bytes.fromhex('00 00 01 23'))
        self.assertEqual(encoded[7:-1], b'\x01\x02\x03')
        self.assertEqual(encoded[-1], 0xA5)
        self.assertEqual(len(encoded), 11)

    def test_channel_and_extended_layout(self) -> None:
        frame = CanFrame(
            channel=3, can_id=0x18FF50E5, data=bytes(range(8)),
            extended=True)
        encoded = encode_can_frame(frame)
        self.assertEqual(encoded[1], 0x08 | 0x80)
        self.assertEqual(encoded[2], 0x0C)
        self.assertEqual(encoded[3:7], bytes.fromhex('18 FF 50 E5'))

    def test_fd_frame_sets_protocol_bit_and_brs(self) -> None:
        frame = CanFrame(
            channel=0, can_id=0x1AB, data=bytes(range(12)),
            fd=True, brs=True)
        encoded = encode_can_frame(frame)
        self.assertEqual(encoded[1], 0x09)
        self.assertEqual(encoded[2], 0x01)
        self.assertEqual(encoded[3:7], bytes.fromhex('80 00 01 AB'))
        self.assertEqual(len(encoded), 8 + 12)

    def test_fd_dlc_64(self) -> None:
        frame = CanFrame(channel=0, can_id=0x123, data=bytes(64), fd=True)
        encoded = encode_can_frame(frame)
        self.assertEqual(encoded[1], 0x0F)
        self.assertEqual(len(encoded), 8 + 64)

    def test_remote_frame(self) -> None:
        frame = CanFrame(channel=1, can_id=0x321, data=b'', remote=True)
        encoded = encode_can_frame(frame)
        self.assertEqual(encoded[1], 0x80)
        self.assertEqual(encoded[2], 0x02)

    def test_invalid_frame_arguments(self) -> None:
        with self.assertRaises(ValueError):
            encode_can_frame(CanFrame(channel=8, can_id=0x1, data=b''))
        with self.assertRaises(ValueError):
            encode_can_frame(CanFrame(channel=0, can_id=0x800, data=b''))
        with self.assertRaises(ValueError):
            encode_can_frame(CanFrame(
                channel=0, can_id=0x2000_0000, data=b'', extended=True))
        with self.assertRaises(ValueError):
            encode_can_frame(CanFrame(channel=0, can_id=0x1, data=bytes(9)))
        with self.assertRaises(ValueError):
            encode_can_frame(CanFrame(
                channel=0, can_id=0x1, data=bytes(9), fd=True))


class FrameParserTest(unittest.TestCase):
    def _parse(self, data: bytes):
        parser = FrameParser()
        return parser.feed(data)

    def test_config_response(self) -> None:
        data = bytes.fromhex('53 29 5B 2C 61 89 9A A6')
        encoded = encode_config_request(FUNC_SERIAL_NUMBER, data, write=False)
        messages = self._parse(encoded)
        self.assertEqual(
            messages,
            [ConfigResponse(function=FUNC_SERIAL_NUMBER, write=False,
                            data=_config_payload(data))])

    def test_config_write_response(self) -> None:
        data = bytes((0, 0, 0x45))
        encoded = encode_config_request(
            FUNC_CAN_PARAMETERS, data, write=True)
        messages = self._parse(encoded)
        self.assertEqual(
            messages,
            [ConfigResponse(function=FUNC_CAN_PARAMETERS, write=True,
                            data=_config_payload(data))])

    def test_config_payload_may_contain_frame_markers(self) -> None:
        data = b'\x5a\x49\x3b\xa5\x00\x00\x00\x00'
        encoded = encode_config_request(FUNC_DEVICE_INFO, data)
        messages = self._parse(encoded)
        self.assertEqual(len(messages), 1)
        self.assertIsInstance(messages[0], ConfigResponse)

    def test_heartbeat_2_channel(self) -> None:
        raw = bytes((0x5A, FUNC_HEARTBEAT)) + bytes(14) + bytes((0xA5,))
        self.assertEqual(len(raw), 17)
        messages = self._parse(raw)
        self.assertEqual(len(messages), 1)
        self.assertIsInstance(messages[0], Heartbeat)
        self.assertEqual(messages[0].channels, 2)
        self.assertEqual(messages[0].raw, raw)

    def test_heartbeat_4_channel(self) -> None:
        raw = bytes((0x5A, FUNC_HEARTBEAT_4CH)) + bytes(29) + bytes((0xA5,))
        self.assertEqual(len(raw), 32)
        messages = self._parse(raw)
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0].channels, 4)

    def test_heartbeat_status_decoding(self) -> None:
        raw = (
            bytes((0x5A, FUNC_HEARTBEAT))
            + bytes(8)
            + bytes((0x20,))
            + bytes(5)
            + bytes((0xA5,)))
        self.assertEqual(len(raw), 17)
        heartbeat = self._parse(raw)[0]
        self.assertEqual(heartbeat.status, 0x20)
        self.assertTrue(heartbeat.channel_enabled(0))
        self.assertFalse(heartbeat.channel_enabled(1))
        self.assertEqual(heartbeat.bus_state(0), 'ok')
        self.assertEqual(heartbeat.bus_state(1), 'ok')

    def test_heartbeat_status_errors(self) -> None:
        raw = bytes((0x5A, FUNC_HEARTBEAT)) + bytes(8) + bytes((0x2A,)) + bytes(5) + bytes((0xA5,))
        heartbeat = self._parse(raw)[0]
        self.assertEqual(heartbeat.bus_state(0), 'passive_error')
        self.assertEqual(heartbeat.bus_state(1), 'passive_error')

    def test_heartbeat_status_requires_2_channels(self) -> None:
        raw = bytes((0x5A, FUNC_HEARTBEAT_4CH)) + bytes(29) + bytes((0xA5,))
        heartbeat = self._parse(raw)[0]
        with self.assertRaises(ValueError):
            _ = heartbeat.status

    def test_classic_frame_round_trip(self) -> None:
        frame = CanFrame(channel=2, can_id=0x7FF, data=b'\xde\xad\xbe\xef')
        self.assertEqual(self._parse(encode_can_frame(frame)), [frame])

    def test_extended_frame_round_trip(self) -> None:
        frame = CanFrame(
            channel=1, can_id=0x1FFFFFFF, data=bytes(range(8)),
            extended=True)
        self.assertEqual(self._parse(encode_can_frame(frame)), [frame])

    def test_fd_frame_round_trip(self) -> None:
        for length in (0, 12, 16, 20, 24, 32, 48, 64):
            frame = CanFrame(
                channel=0, can_id=0x123, data=bytes(length),
                fd=True, brs=length > 8)
            with self.subTest(length=length):
                self.assertEqual(
                    self._parse(encode_can_frame(frame)), [frame])

    def test_remote_frame_round_trip(self) -> None:
        frame = CanFrame(channel=0, can_id=0x456, data=b'', remote=True)
        self.assertEqual(self._parse(encode_can_frame(frame)), [frame])

    def test_avoids_start_byte_inside_heartbeat(self) -> None:
        heartbeat = bytes((0x5A, FUNC_HEARTBEAT)) + bytes(14) + bytes((0xA5,))
        frame = CanFrame(channel=0, can_id=0x55, data=b'\x49\x3b')
        messages = self._parse(heartbeat + encode_can_frame(frame))
        self.assertEqual(len(messages), 2)
        self.assertIsInstance(messages[0], Heartbeat)
        self.assertEqual(messages[1], frame)

    def test_partial_frame_is_buffered(self) -> None:
        encoded = encode_can_frame(
            CanFrame(channel=0, can_id=0x123, data=bytes(range(8))))
        parser = FrameParser()
        self.assertEqual(parser.feed(encoded[:6]), [])
        self.assertEqual(
            parser.feed(encoded[6:]),
            [CanFrame(channel=0, can_id=0x123, data=bytes(range(8)))])

    def test_partial_config_is_buffered(self) -> None:
        encoded = encode_config_request(FUNC_DEVICE_INFO, b'\x01\x02')
        parser = FrameParser()
        self.assertEqual(parser.feed(encoded[:10]), [])
        messages = parser.feed(encoded[10:])
        self.assertEqual(len(messages), 1)
        self.assertIsInstance(messages[0], ConfigResponse)

    def test_leading_noise_is_discarded(self) -> None:
        frame = CanFrame(channel=0, can_id=0x100, data=b'\x01')
        messages = self._parse(b'\x00\x11\x22\x33' + encode_can_frame(frame))
        self.assertEqual(messages, [frame])

    def test_invalid_config_tail_resynchronizes(self) -> None:
        bad_config = (
            bytes.fromhex('49 3B 40 52') + bytes(16) + bytes.fromhex('00 00'))
        frame = CanFrame(channel=0, can_id=0x101, data=b'\x02')
        messages = self._parse(bad_config + encode_can_frame(frame))
        self.assertEqual(messages, [frame])

    def test_invalid_access_flag_resynchronizes(self) -> None:
        bad_config = (
            bytes.fromhex('49 3B 40 00') + bytes(16) + CONFIG_TAIL)
        frame = CanFrame(channel=0, can_id=0x102, data=b'\x03')
        messages = self._parse(bad_config + encode_can_frame(frame))
        self.assertEqual(messages, [frame])

    def test_invalid_heartbeat_tail_resynchronizes(self) -> None:
        bad_heartbeat = bytes((0x5A, FUNC_HEARTBEAT)) + bytes(15)
        frame = CanFrame(channel=0, can_id=0x103, data=b'\x04')
        messages = self._parse(bad_heartbeat + encode_can_frame(frame))
        self.assertEqual(messages, [frame])

    def test_classic_frame_with_fd_dlc_is_rejected(self) -> None:
        bad_frame = (
            bytes((0x5A, 0x09, 0x00))
            + bytes.fromhex('00 00 01 23')
            + bytes(12)
            + bytes((0xA5,)))
        frame = CanFrame(channel=0, can_id=0x105, data=b'\x05')
        messages = self._parse(bad_frame + encode_can_frame(frame))
        self.assertEqual(messages, [frame])

    def test_zero_length_fd_frame_is_not_dropped(self) -> None:
        frame = CanFrame(channel=0, can_id=0x104, data=b'', fd=True)
        messages = self._parse(encode_can_frame(frame))
        self.assertEqual(messages, [frame])
        self.assertEqual(messages[0].data, b'')

    def test_multiple_messages_in_one_chunk(self) -> None:
        frames = [
            CanFrame(channel=0, can_id=0x110, data=b'\x01'),
            CanFrame(channel=1, can_id=0x111, data=b'\x02\x03'),
        ]
        messages = self._parse(b''.join(map(encode_can_frame, frames)))
        self.assertEqual(messages, frames)


class HelpersTest(unittest.TestCase):
    def test_decode_serial_number(self) -> None:
        self.assertEqual(decode_serial_number(b'ABC123\x00\x00'), 'ABC123')

    def test_decode_serial_number_replaces_non_printable_bytes(self) -> None:
        self.assertEqual(decode_serial_number(b'A\x89B'), 'A.B')

    def test_can_frame_format(self) -> None:
        frame = CanFrame(
            channel=2, can_id=0x18FF50E5, data=b'\x01\x02',
            extended=True, fd=True, brs=True)
        self.assertEqual(
            frame.format(),
            'can2 18FF50E5#0102 [extended fd brs]')


class ZqwlCanFdTransportTest(unittest.TestCase):
    def test_read_device_info_over_pty(self) -> None:
        info = bytes.fromhex('12 34 56 78')

        def handler(master_fd: int) -> None:
            request = _read_exact(master_fd, 22)
            self.assertEqual(
                request, encode_config_request(FUNC_DEVICE_INFO))
            os.write(
                master_fd,
                encode_config_request(FUNC_DEVICE_INFO, info))

        adapter = PtyAdapter(handler)
        try:
            with ZqwlCanFd(port=adapter.path, timeout=2.0) as client:
                self.assertEqual(
                    client.read_device_info(), _config_payload(info))
        finally:
            adapter.close()

    def test_configure_and_disable_can_over_pty(self) -> None:
        requests = []

        def handler(master_fd: int) -> None:
            for function in (
                    FUNC_CAN_PARAMETERS, FUNC_SYSTEM_CONTROL,
                    FUNC_SYSTEM_CONTROL):
                request = _read_exact(master_fd, 22)
                self.assertEqual(request[2], function)
                self.assertEqual(request[3], 0x57)
                requests.append(request)
                os.write(
                    master_fd,
                    encode_config_request(function, write=True))

        adapter = PtyAdapter(handler)
        try:
            with ZqwlCanFd(port=adapter.path, timeout=2.0) as client:
                client.configure_can(
                    channel=0, bitrate=1_000_000,
                    data_bitrate=1_000_000, fd=True, save=False)
                client.disable_can(channel=0)
        finally:
            adapter.close()

        self.assertEqual(len(requests), 3)
        self.assertEqual(requests[0][2], FUNC_CAN_PARAMETERS)
        self.assertEqual(requests[0][4], 0)
        self.assertEqual(requests[0][6], 0x03)
        self.assertEqual(requests[1][2], FUNC_SYSTEM_CONTROL)
        self.assertEqual(requests[1][6], 1)
        self.assertEqual(requests[2][2], FUNC_SYSTEM_CONTROL)
        self.assertEqual(requests[2][6], 0)

    def test_configure_tolerates_missing_write_ack(self) -> None:
        requests = []

        def handler(master_fd: int) -> None:
            for _ in range(3):
                request = _read_exact(master_fd, 22)
                self.assertEqual(request[3], 0x57)
                requests.append(request)

        adapter = PtyAdapter(handler)
        try:
            with ZqwlCanFd(port=adapter.path, timeout=0.1) as client:
                client.configure_can(
                    channel=0, bitrate=1_000_000, fd=False)
                client.disable_can(channel=0)
        finally:
            adapter.close()

        self.assertEqual(len(requests), 3)
        self.assertEqual(requests[0][2], FUNC_CAN_PARAMETERS)
        self.assertEqual(requests[1][2], FUNC_SYSTEM_CONTROL)
        self.assertEqual(requests[2][2], FUNC_SYSTEM_CONTROL)

    def test_send_frame_over_pty(self) -> None:
        frame = CanFrame(channel=0, can_id=0x123, data=b'\x01\x02')
        encoded = encode_can_frame(frame)
        received = []

        def handler(master_fd: int) -> None:
            received.append(_read_exact(master_fd, len(encoded)))

        adapter = PtyAdapter(handler)
        try:
            with ZqwlCanFd(port=adapter.path, timeout=2.0) as client:
                client.send(frame)
        finally:
            adapter.close()

        self.assertEqual(received, [encoded])

    def test_receive_frames_over_pty(self) -> None:
        frames = [
            CanFrame(channel=0, can_id=0x123, data=b'\x01'),
            CanFrame(channel=1, can_id=0x456, data=b'\x02\x03', extended=False),
        ]

        def handler(master_fd: int) -> None:
            time.sleep(0.05)
            for frame in frames:
                os.write(master_fd, encode_can_frame(frame))

        adapter = PtyAdapter(handler)
        try:
            with ZqwlCanFd(port=adapter.path, timeout=2.0) as client:
                self.assertEqual(client.receive(0.5), frames)
        finally:
            adapter.close()


if __name__ == '__main__':
    unittest.main()
