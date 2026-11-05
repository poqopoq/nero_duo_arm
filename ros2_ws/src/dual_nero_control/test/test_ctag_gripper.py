from __future__ import annotations

import unittest

from dual_nero_control.ctag_gripper import (
    alarm_reset_frame,
    decode_feedback,
    decode_register_u16,
    enable_frame,
    feedback_request_frame,
    joint_from_position,
    motion_config_frames,
    motion_frames,
    opening_from_joint_rad,
    opening_from_position,
    position_from_joint,
    position_payload,
    register_can_id,
    register_read_request_frame,
)
from dual_nero_control.zqwl_canfd import encode_can_frame


class RegisterProtocolTest(unittest.TestCase):
    def test_register_ids_match_vendor_layout(self) -> None:
        self.assertEqual(register_can_id(0x0102, True, 1), 0x00010281)
        self.assertEqual(register_can_id(0x0100, True, 1), 0x00010081)
        self.assertEqual(register_can_id(0x0108, True, 1), 0x00010881)
        self.assertEqual(register_can_id(0x0418, False, 1), 0x00041801)

    def test_position_payload_uses_high_then_low_word(self) -> None:
        self.assertEqual(position_payload(1000), bytes.fromhex('00 00 e8 03'))
        self.assertEqual(position_payload(0x00012345), bytes.fromhex('01 00 45 23'))

    def test_feedback_decode_is_inverse_of_payload(self) -> None:
        for value in (0, 1, 500, 1000, 65536, 0x12345678):
            self.assertEqual(decode_feedback(position_payload(value)), value)

    def test_feedback_rejects_short_payload(self) -> None:
        with self.assertRaises(ValueError):
            decode_feedback(b'\x00\x01\x02')

    def test_register_u16_decode(self) -> None:
        self.assertEqual(decode_register_u16(bytes.fromhex('18 80')), 0x8018)
        self.assertEqual(decode_register_u16(bytes.fromhex('00 00 ff')), 0)
        with self.assertRaises(ValueError):
            decode_register_u16(b'\x01')

    def test_register_ranges(self) -> None:
        with self.assertRaises(ValueError):
            register_can_id(0x10000, False)
        with self.assertRaises(ValueError):
            register_can_id(0x0100, False, 0)
        with self.assertRaises(ValueError):
            register_can_id(0x0100, False, 128)


class FrameProtocolTest(unittest.TestCase):
    def test_alarm_reset_frame(self) -> None:
        frame = alarm_reset_frame()
        self.assertEqual(frame.can_id, 0x00011081)
        self.assertEqual(frame.data, b'\x01\x00')
        self.assertTrue(frame.extended)
        self.assertFalse(frame.fd)

    def test_motion_produces_position_then_trigger(self) -> None:
        position, trigger = motion_frames(500)
        self.assertTrue(position.extended)
        self.assertEqual(position.can_id, 0x00010281)
        self.assertEqual(position.data, bytes.fromhex('00 00 f4 01'))
        self.assertEqual(trigger.can_id, 0x00010881)
        self.assertEqual(trigger.data, b'\x01\x00')

    def test_enable_frame(self) -> None:
        self.assertEqual(enable_frame(True).data, b'\x01\x00')
        self.assertEqual(enable_frame(False).data, b'\x00\x00')

    def test_feedback_request_is_read_only(self) -> None:
        frame = feedback_request_frame()
        self.assertEqual(frame.can_id, 0x00041801)
        self.assertEqual(frame.data, bytes(4))
        self.assertTrue(frame.extended)
        self.assertFalse(frame.fd)

    def test_status_read_request_is_read_only(self) -> None:
        frame = register_read_request_frame(0x0401)
        self.assertEqual(frame.can_id, 0x00040101)
        self.assertEqual(frame.data, bytes(2))
        self.assertTrue(frame.extended)
        self.assertFalse(frame.fd)
        with self.assertRaises(ValueError):
            register_read_request_frame(0x0401, payload_size=65)

    def test_motion_config_uses_vendor_scalar_registers(self) -> None:
        speed, torque, ramp = motion_config_frames(25, 60, 200)
        self.assertEqual(speed.can_id, 0x00010481)
        self.assertEqual(speed.data, bytes.fromhex('19 00'))
        self.assertEqual(torque.can_id, 0x00010581)
        self.assertEqual(torque.data, bytes.fromhex('3c 00'))
        self.assertEqual(ramp.can_id, 0x00010681)
        self.assertEqual(ramp.data, bytes.fromhex('c8 00'))
        self.assertTrue(all(frame.extended for frame in (speed, torque, ramp)))

    def test_motion_config_ranges(self) -> None:
        with self.assertRaises(ValueError):
            motion_config_frames(0, 100, 200)
        with self.assertRaises(ValueError):
            motion_config_frames(101, 100, 200)
        with self.assertRaises(ValueError):
            motion_config_frames(25, 0, 200)
        with self.assertRaises(ValueError):
            motion_config_frames(25, 101, 200)
        with self.assertRaises(ValueError):
            motion_config_frames(25, 100, 1001)

    def test_classic_and_fd_encoding(self) -> None:
        classic = encode_can_frame(motion_frames(1000)[0])
        self.assertEqual(classic[2], 0x04)
        self.assertEqual(classic[3:7], bytes.fromhex('00 01 02 81'))

        fd_frame = motion_frames(1000, fd=True, brs=True)[0]
        encoded = encode_can_frame(fd_frame)
        self.assertEqual(encoded[2], 0x05)
        self.assertEqual(encoded[3:7], bytes.fromhex('80 01 02 81'))

    def test_invalid_motion_position(self) -> None:
        with self.assertRaises(ValueError):
            motion_frames(-1)
        with self.assertRaises(ValueError):
            motion_frames(1001)
        with self.assertRaises(ValueError):
            motion_frames(0, brs=True)


class JawMappingTest(unittest.TestCase):
    def test_endpoints(self) -> None:
        self.assertAlmostEqual(joint_from_position(0), 0.0, places=5)
        self.assertAlmostEqual(joint_from_position(1000), -0.90838, places=4)
        self.assertAlmostEqual(opening_from_position(0), 90.0, places=5)
        self.assertAlmostEqual(opening_from_position(1000), 0.0, places=5)

    def test_round_trip_position_to_joint_and_back(self) -> None:
        for position in (0, 100, 250, 500, 750, 900, 1000):
            joint = joint_from_position(position)
            recovered = position_from_joint(joint)
            self.assertLessEqual(abs(recovered - position), 1)

    def test_opening_matches_vendor_kinematics(self) -> None:
        self.assertAlmostEqual(opening_from_joint_rad(0.0), 90.0, places=5)
        self.assertAlmostEqual(
            opening_from_joint_rad(joint_from_position(500)), 45.0, places=4)

    def test_invalid_mapping_range(self) -> None:
        with self.assertRaises(ValueError):
            opening_from_position(500, position_min=1000, position_max=0)


if __name__ == '__main__':
    unittest.main()
