"""Tests for ecu_cli.py framing/parsing (no hardware needed): python3 -m unittest tools/test_ecu_cli.py"""
import io
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import ecu_cli  # noqa: E402


class FramingTest(unittest.TestCase):
    def test_crc_matches_firmware(self):
        # Same check value as firmware/test/test_main.c
        import zlib
        self.assertEqual(zlib.crc32(b"123456789"), 0xCBF43926)

    def test_encode_matches_firmware_layout(self):
        f = ecu_cli.encode_frame(b"Q")
        self.assertEqual(f[:3], b"\x00\x01Q")
        self.assertEqual(len(f), 7)

    def test_roundtrip_and_bad_crc(self):
        f = ecu_cli.encode_frame(b"\x00hello")
        self.assertEqual(ecu_cli.read_frame(io.BytesIO(f).read), b"\x00hello")
        bad = f[:-1] + bytes([f[-1] ^ 1])
        with self.assertRaises(IOError):
            ecu_cli.read_frame(io.BytesIO(bad).read)
        with self.assertRaises(TimeoutError):
            ecu_cli.read_frame(io.BytesIO(b"").read)

    def test_realtime_layout(self):
        self.assertEqual(struct.calcsize(ecu_cli.RT_FORMAT), 46)   # sizeof(realtime_t)
        raw = bytearray(46)
        struct.pack_into("<I", raw, 0, 1234)
        struct.pack_into("<H", raw, 32, 7200)
        raw[36] = 3
        raw[37] = 20
        struct.pack_into("<H", raw, 44, 2)
        rt = ecu_cli.parse_realtime(bytes(raw))
        self.assertEqual((rt["uptime_ms"], rt["rpm"], rt["sync"], rt["cam_pos"], rt["cam_errors"]),
                         (1234, 7200, 3, 20, 2))

    def test_teeth(self):
        data = bytes([2]) + struct.pack("<HBB", 104, 0, 0x31) + struct.pack("<HBB", 35, 0x81, 0x32)
        t = ecu_cli.parse_teeth(data)
        self.assertEqual(t[0], {"interval_us": 416, "tooth": 0, "rev": 0, "gap": True, "cam": False,
                                "noise": False, "sync": 3})
        self.assertEqual((t[1]["tooth"], t[1]["rev"], t[1]["cam"]), (1, 1, True))


if __name__ == "__main__":
    unittest.main()
