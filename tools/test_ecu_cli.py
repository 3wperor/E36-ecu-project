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
        self.assertEqual(struct.calcsize(ecu_cli.RT_FORMAT), 50)   # sizeof(realtime_t)
        raw = bytearray(50)
        struct.pack_into("<I", raw, 0, 1234)
        struct.pack_into("<H", raw, 32, 7200)
        raw[36] = 3
        raw[37] = 20
        struct.pack_into("<H", raw, 44, 2)
        raw[39] = 3
        struct.pack_into("<H", raw, 46, 5)
        rt = ecu_cli.parse_realtime(bytes(raw))
        self.assertEqual((rt["uptime_ms"], rt["rpm"], rt["sync"], rt["cam_pos"], rt["cam_errors"],
                          rt["out_enable"], rt["sched_late"]), (1234, 7200, 3, 20, 2, 3, 5))

    def test_cfg(self):
        self.assertEqual(struct.calcsize(ecu_cli.CFG_FORMAT), 12)     # sizeof(sched_cfg_t)
        # same bytes as the firmware host test (test_main.c): enable 3, advance 10.0°,
        # dwell 3000 us, pw 2500 us, end of injection 380.0°, TDC offset 1.6°
        raw = bytes([3, 0, 0x10, 0x00, 100, 0, 0xB8, 0x0B, 0xC4, 0x09, 0xD8, 0x0E])
        c = ecu_cli.parse_cfg(raw)
        self.assertEqual(c, {"enable": 3, "spare": 0, "tdc_offset": 16, "advance": 100,
                             "dwell_us": 3000, "inj_pw_us": 2500, "inj_end": 3800})
        self.assertEqual(ecu_cli.pack_cfg(c), raw)
        c2 = ecu_cli.apply_settings(c, ["ign=off", "advance=-2.5", "dwell=2.8", "pw=4", "tdc=60", "eoi=400"])
        self.assertEqual((c2["enable"], c2["advance"], c2["dwell_us"], c2["inj_pw_us"], c2["tdc_offset"],
                          c2["inj_end"]), (2, -25, 2800, 4000, 600, 4000))
        self.assertEqual(ecu_cli.apply_settings(c, ["off"])["enable"], 0)
        with self.assertRaises(ValueError):
            ecu_cli.apply_settings(c, ["boost=2"])

    def test_teeth(self):
        data = bytes([2]) + struct.pack("<HBB", 104, 0, 0x31) + struct.pack("<HBB", 35, 0x81, 0x32)
        t = ecu_cli.parse_teeth(data)
        self.assertEqual(t[0], {"interval_us": 416, "tooth": 0, "rev": 0, "gap": True, "cam": False,
                                "noise": False, "sync": 3})
        self.assertEqual((t[1]["tooth"], t[1]["rev"], t[1]["cam"]), (1, 1, True))


if __name__ == "__main__":
    unittest.main()
