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
        import zlib
        self.assertEqual(zlib.crc32(b"123456789"), 0xCBF43926)   # same check value as firmware tests

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


class RealtimeTest(unittest.TestCase):
    def test_layout_from_definition(self):
        d = ecu_cli.DEF
        raw = bytearray(d["realtime_size"])
        off = lambda n: ecu_cli.channel(n)["offset"]
        struct.pack_into("<I", raw, off("uptime_ms"), 1234)
        struct.pack_into("<H", raw, off("rpm"), 7200)
        struct.pack_into("<H", raw, off("adc") + 2 * 3, 512)          # adcClt
        struct.pack_into("<h", raw, off("clt"), -105)                 # -10.5 C
        raw[off("sync")] = 3
        rt = ecu_cli.parse_realtime(bytes(raw))
        self.assertEqual((rt["uptime_ms"], rt["rpm"], rt["sync"], rt["adcClt"], rt["clt"]),
                         (1234, 7200, 3, 512, -105))
        self.assertAlmostEqual(ecu_cli.shown(ecu_cli.channel("clt"), rt["clt"]), -10.5)
        self.assertIn("7200 rpm", ecu_cli.describe_realtime(rt))


class KnockViewTest(unittest.TestCase):
    def test_knock_view(self):
        raw = bytearray(ecu_cli.DEF["realtime_size"])
        off = lambda n: ecu_cli.channel(n)["offset"]
        struct.pack_into("<H", raw, off("knockLevel") + 2 * 2, 300)     # cylinder 3
        struct.pack_into("<H", raw, off("knockNoise") + 2 * 2, 120)
        raw[off("knockRetard") + 2] = 25                                  # 2.5 deg
        raw[off("knockState")] = 0x07
        text = ecu_cli.describe_knock(ecu_cli.parse_realtime(bytes(raw)))
        self.assertIn("enabled active KNOCK", text)
        self.assertRegex(text, r"\n\s+3\s+300\s+120\s+2\.50\s+2\.5°")


class TuneFieldTest(unittest.TestCase):
    def page(self, name):
        p, f = ecu_cli.find_field(name)
        return p, f, bytearray(p["size"])

    def test_scalar_roundtrip(self):
        p, f, pg = self.page("revLimit")                  # U08 x100 rpm
        b = ecu_cli.encode_field(f, "6500", pg[f["offset"]:f["offset"] + 1])
        self.assertEqual(b, bytes([65]))
        pg[f["offset"]] = b[0]
        self.assertEqual(ecu_cli.decode_field(f, pg), 6500)

    def test_translate_and_array(self):
        p, f, pg = self.page("cylIgnTrim")                # U08 x0.5, translate -20
        b = ecu_cli.encode_field(f, "0,1.5,-2,0,0,10", b"")
        self.assertEqual(list(b), [20, 23, 16, 20, 20, 40])
        pg[f["offset"]:f["offset"] + 6] = b
        self.assertEqual(ecu_cli.decode_field(f, pg), [0, 1.5, -2, 0, 0, 10])
        self.assertEqual(ecu_cli.encode_field(f, "1", b""), bytes([22] * 6))   # one value for all

    def test_limits_and_options(self):
        p, f, pg = self.page("revLimit")
        with self.assertRaises(ValueError):
            ecu_cli.encode_field(f, "12000", b"\x00")
        p, f, pg = self.page("loadSource")
        self.assertEqual(ecu_cli.encode_field(f, "alpha-n", b"\x00"), b"\x01")
        pg[f["offset"]] = 2
        self.assertEqual(ecu_cli.decode_field(f, pg), "Blend (table)")
        with self.assertRaises(ValueError):
            ecu_cli.find_field("boost")


class BenchCfgTest(unittest.TestCase):
    def test_cfg(self):
        self.assertEqual(struct.calcsize(ecu_cli.CFG_FORMAT), 12)     # sizeof(sched_cfg_t)
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
