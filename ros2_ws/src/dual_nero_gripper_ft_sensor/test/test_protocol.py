import pytest

from dual_nero_gripper_ft_sensor.protocol import (
    CMD_GET_FIRMWARE_VERSION,
    CMD_GET_SERIAL_NUMBER,
    CMD_SET_BAUD_RATE,
    CMD_SET_SAMPLE_RATE,
    CMD_START_AUTO_UPLOAD,
    CMD_STOP_AUTO_UPLOAD,
    FrameParser,
    ProtocolError,
    build_frame,
    crc16_modbus,
    decode_measurements,
    decode_serial_number,
    format_firmware_version,
)


MEASUREMENT_FRAME = bytes.fromhex(
    "AA 55 01 18 69 13 2B 42 A7 34 02 43 CF 1C CB C2 "
    "EB 46 40 40 A2 79 D4 BE C5 6D 61 41 95 17")

EXPECTED_MEASUREMENTS = (
    42.76895523071289,
    130.20567321777344,
    -101.55626678466797,
    3.004328489303589,
    -0.41499048471450806,
    14.089299201965332,
)


@pytest.mark.parametrize(
    ("message_type", "payload", "expected"),
    [
        (CMD_START_AUTO_UPLOAD, b"", "AA 55 09 00 37 BC"),
        (CMD_STOP_AUTO_UPLOAD, b"", "AA 55 0A 00 37 4C"),
        (CMD_GET_FIRMWARE_VERSION, b"", "AA 55 03 00 31 1C"),
        (CMD_GET_SERIAL_NUMBER, b"", "AA 55 04 00 33 2C"),
        (CMD_SET_BAUD_RATE, b"\x00", "AA 55 02 01 00 8D 84"),
        (CMD_SET_BAUD_RATE, b"\x03", "AA 55 02 01 03 CD 85"),
        (CMD_SET_SAMPLE_RATE, b"\x04", "AA 55 0F 01 04 1D 84"),
        (CMD_SET_SAMPLE_RATE, b"\x05", "AA 55 0F 01 05 DC 44"),
    ],
)
def test_documented_command_frames(message_type, payload, expected):
    assert build_frame(message_type, payload).hex(" ").upper() == expected


def test_documented_measurement_frame_crc():
    assert crc16_modbus(MEASUREMENT_FRAME[:-2]) == 0x1795


def test_measurement_frame_decodes_six_little_endian_floats():
    frames = FrameParser().feed(MEASUREMENT_FRAME)
    assert len(frames) == 1
    assert frames[0].message_type == 0x01
    assert decode_measurements(frames[0].data) == pytest.approx(
        EXPECTED_MEASUREMENTS)


def test_parser_accepts_fragmented_stream():
    parser = FrameParser()
    frames = []
    for byte in MEASUREMENT_FRAME:
        frames.extend(parser.feed(bytes((byte,))))
    assert len(frames) == 1
    assert frames[0].message_type == 0x01


def test_parser_resynchronises_after_junk_and_bad_crc():
    corrupted = bytearray(MEASUREMENT_FRAME)
    corrupted[10] ^= 0xFF
    good_frame = build_frame(CMD_START_AUTO_UPLOAD)
    parser = FrameParser()

    frames = parser.feed(b"junk" + corrupted + good_frame)

    assert parser.stats.crc_errors == 1
    assert len(frames) == 1
    assert frames[0].message_type == CMD_START_AUTO_UPLOAD
    assert frames[0].data == b""


def test_parser_rejects_oversized_length_without_losing_next_frame():
    parser = FrameParser()
    frames = parser.feed(b"\xAA\x55\x01\xFF" + build_frame(CMD_STOP_AUTO_UPLOAD))
    assert len(frames) == 1
    assert frames[0].message_type == CMD_STOP_AUTO_UPLOAD


def test_decodes_documented_version_and_serial_number():
    version = bytes.fromhex("AA 55 03 03 00 01 00 58 DB")
    serial_frame = bytes.fromhex(
        "AA 55 04 20 46 36 2D 30 32 30 2D 30 35 30 2D 46 "
        "54 4A 31 2D 32 38 00 00 00 00 00 00 00 00 00 00 "
        "00 00 00 00 AC 6F")

    parser = FrameParser()
    frames = parser.feed(version + serial_frame)

    assert format_firmware_version(frames[0].data) == "0.1.0"
    assert decode_serial_number(frames[1].data) == "F6-020-050-FTJ1-28"


def test_rejects_payload_over_128_bytes():
    with pytest.raises(ProtocolError):
        build_frame(CMD_START_AUTO_UPLOAD, bytes(129))
