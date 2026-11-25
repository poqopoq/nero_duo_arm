import struct

from dual_nero_gripper_ft_sensor.op3_rs485 import (
    SUB_FACE_A,
    SUB_FACE_B,
    _extract,
    build_can_id,
    crc16,
    decode_all,
    decode_face,
    wire_frame,
)


def test_crc16_known_frame():
    body = bytes.fromhex('EE 55 01 01 13 01 00')
    assert crc16(body) == 0x62D5


def test_build_can_id_position():
    # Matches the vendor SDK captured on the wire: EE55 01 01 13 01 ...
    assert build_can_id(1, 0x13, 0x01, False) == 0x01130101
    assert build_can_id(1, 0x13, 0x01, True) == 0x01130181


def test_build_can_id_tactile():
    assert build_can_id(1, 0x06, 0x00, False) == 0x00060101
    assert build_can_id(1, 0x06, SUB_FACE_A, False) == 0x01060101
    assert build_can_id(1, 0x06, SUB_FACE_B, False) == 0x02060101


def test_wire_frame_matches_sdk_capture():
    frame = wire_frame(0x01130101, b'')
    assert frame == bytes.fromhex('EE 55 01 01 13 01 00 D5 62')


def test_decode_face():
    values = (1.5, 2.5, 3.5, 4.5, 5.5, 6.5)
    payload = b'\x01\x00' + struct.pack('<6f', *values)
    assert decode_face(payload) == values


def test_decode_all():
    face_a = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    face_b = (7.0, 8.0, 9.0, 10.0, 11.0, 12.0)
    payload = b'\x01\x00' + struct.pack('<12f', *(face_a + face_b))
    assert decode_all(payload) == (face_a, face_b)


def test_extract_waits_for_split_sof():
    can_id = 0x01060101
    frame = wire_frame(can_id, b'\x00')
    payload, rest = _extract(frame[:1], can_id)
    assert payload is None
    assert rest == b'\xee'
    payload, rest = _extract(rest + frame[1:], can_id)
    assert payload == b'\x00'
    assert rest == b''
