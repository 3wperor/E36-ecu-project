"""Tuner core tests, no GUI needed:  python3 -m unittest discover -s tuner/tests -t tuner"""
import json
import os
import tempfile
import unittest

from e36tuner import ops
from e36tuner.definition import Definition
from e36tuner.fake_ecu import FakeEcu
from e36tuner.link import EcuLink, LinkError, SignatureError, encode_frame, read_frame
from e36tuner.tune import Tune, TuneFileError, diff, read_tune_file

D = Definition.load()


class DefinitionTest(unittest.TestCase):
    def test_encoding(self):
        f = D.fields["cylIgnTrim"]                       # U08, x0.5, translate -20, -10..10
        self.assertEqual((f.to_raw(1.5), f.to_shown(23), f.raw_min, f.raw_max), (23, 1.5, 0, 40))
        self.assertEqual(f.to_raw(99), 40)               # limited to the allowed range
        self.assertFalse(f.in_range(10.5))
        f = D.fields["revLimit"]                         # U08, x100
        self.assertEqual((f.to_raw(6500), f.min, f.max, f.step), (65, 1000, 9000, 100))

    def test_every_field_has_an_editor(self):
        seen = set()
        for items in D.menus().values():
            for _kind, it in items:
                seen.update(it.get("fields", []))
                seen.update(it[k] for k in ("x", "y", "z") if k in it)
        self.assertEqual(seen, set(D.fields))

    def test_defaults_match_page_sizes(self):
        for p, size in D.page_sizes.items():
            self.assertEqual(len(D.defaults[p]), size)

    def test_labels(self):
        from e36tuner.definition import label
        self.assertEqual([label(n) for n in ("injDeadV", "sync_losses", "rpm", "air")],
                         ["Inj Dead V", "Sync losses", "RPM", "Air"])


class TuneTest(unittest.TestCase):
    def setUp(self):
        self.t = Tune(D)
        self.changes = []
        self.t.listeners.append(self.changes.extend)

    def test_set_get_and_listener(self):
        self.assertNotEqual(self.t.get("revLimit"), 6400)
        self.assertTrue(self.t.set("revLimit", 6400))
        self.assertEqual(self.t.get("revLimit"), 6400)
        f = D.fields["revLimit"]
        self.assertEqual(self.changes, [(f.page, f.offset, bytes([64]))])
        self.assertFalse(self.t.set("revLimit", 6400))   # no change: nothing sent, no undo step
        self.assertEqual(len(self.t.undo_stack), 1)
        with self.assertRaises(ValueError):
            self.t.set("revLimit", 12000)
        with self.assertRaises(ValueError):
            self.t.set("injDead", [1, 2])
        self.t.set("loadSource", "alpha-n")
        self.assertEqual(self.t.get("loadSource"), "Alpha-N")

    def test_array_edit_is_one_span(self):
        f = D.fields["ignTable"]
        self.t.set_raw("ignTable", {3: 100, 20: 101})
        (page, off, data), = self.changes
        self.assertEqual((page, off, len(data)), (f.page, f.offset + 3, 18))
        self.assertEqual((self.t.raw("ignTable", 3), self.t.raw("ignTable", 20)), (100, 101))
        self.t.set_raw("loadBins", {2: 123})             # 16-bit elements
        self.assertEqual(self.changes[-1][1:], (D.fields["loadBins"].offset + 4, bytes([123, 0])))

    def test_undo_redo_and_groups(self):
        before, baro = self.t.snapshot(), self.t.get("baro")
        with self.t.group("two edits"):
            self.t.set("revLimit", 6000)
            self.t.set("injFlow", 300)
        self.t.set("baro", 95)
        self.assertEqual([d for d, _ in self.t.undo_stack], ["two edits", "change Baro"])
        self.assertEqual(self.t.undo(), "change Baro")
        self.assertEqual(self.t.undo(), "two edits")
        self.assertEqual(self.t.snapshot(), before)
        self.assertIsNone(self.t.undo())
        self.assertEqual(self.t.redo(), "two edits")
        self.assertEqual((self.t.get("revLimit"), self.t.get("injFlow"), self.t.get("baro")), (6000, 300, baro))
        self.t.set("baro", 90)                            # a new edit drops the redo history
        self.assertFalse(self.t.can_redo())
        # replaying everything the listener saw on a copy gives the same tune (what the ECU receives)
        mirror = {p: bytearray(b) for p, b in D.defaults.items()}
        self.assertNotIn([], self.changes)
        for page, off, data in self.changes:
            mirror[page][off:off + len(data)] = data
        self.assertEqual({p: bytes(b) for p, b in mirror.items()}, self.t.snapshot())

    def test_bits_keep_other_bits(self):
        f = D.fields["loadSource"]
        self.t.pages[f.page][f.offset] = 0xF0
        self.t.set("loadSource", 2)
        self.assertEqual(self.t.pages[f.page][f.offset], 0xF2)

    def test_file_roundtrip_and_modified(self):
        self.t.set("ignTable", [10.0] * 256)
        self.assertTrue(self.t.modified)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "a.tune")
            self.t.save(path)
            self.assertFalse(self.t.modified)
            pages, notes = read_tune_file(D, path)
            self.assertEqual((pages, notes), (self.t.snapshot(), []))
            # the command-line tool's files have the same format
            saved = json.load(open(path))
            self.assertEqual(set(saved), {"signature", "layout_hash", "pages", "values"})
            self.assertEqual(saved["values"]["ignTable"][0], 10.0)

    def test_import_from_another_layout(self):
        self.t.set("revLimit", 6200)
        self.t.set("loadSource", "Blend (table)")
        self.t.set("veTable", [50.0] * 256)
        d = self.t.to_file_dict()
        d["layout_hash"] += 1
        del d["values"]["baro"]                           # not in the old firmware
        d["values"]["boostTarget"] = 1.2                  # gone from this firmware
        d["values"]["injFlow"] = 9999                     # beyond today's limit
        d["values"]["injDead"] = [1, 2, 3]                # different length
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "old.tune")
            json.dump(d, open(path, "w"))
            pages, notes = read_tune_file(D, path)
            t = Tune(D, pages)
            self.assertEqual((t.get("revLimit"), t.get("loadSource"), t.get("veTable")[5]),
                             (6200, "Blend (table)", 50.0))
            self.assertEqual((t.get("injFlow"), t.get("baro")), (2000, Tune(D).get("baro")))
            self.assertEqual(t.get("injDead"), Tune(D).get("injDead"))
            text = "\n".join(notes)
            for word in ("Baro", "boostTarget", "Inj Flow: limited", "Inj Dead: the file has 3"):
                self.assertIn(word, text)
            self.assertEqual(len(notes), 4)
            # damaged / foreign files are refused with a clear error
            for bad in ("{}", "not json", json.dumps({"layout_hash": D.layout_hash, "pages": {"0": "zz"}}),
                        json.dumps({"layout_hash": D.layout_hash, "pages": {"0": "00"}}),
                        json.dumps({"layout_hash": 1})):
                open(path, "w").write(bad)
                with self.assertRaises(TuneFileError):
                    read_tune_file(D, path)

    def test_diff(self):
        other = Tune(D)
        other.set("revLimit", 7000)
        other.set_raw("afrTable", {0: 100, 17: 101})
        other.set("ignEnable", "On")
        got = {e["field"].name: e for e in diff(D, self.t.snapshot(), other.snapshot())}
        self.assertEqual(set(got), {"revLimit", "afrTable", "ignEnable"})
        self.assertEqual(got["afrTable"]["indices"], [0, 17])
        self.assertEqual((got["ignEnable"]["a"], got["ignEnable"]["b"]), (["Off"], ["On"]))
        self.assertEqual(diff(D, self.t.snapshot(), self.t.snapshot()), [])


class OpsTest(unittest.TestCase):
    def test_interpolate_row_and_block(self):
        row = {(0, c): v for c, v in enumerate([10, 0, 0, 0, 50])}
        self.assertEqual([ops.interpolate(row)[(0, c)] for c in range(5)], [10, 20, 30, 40, 50])
        # uneven bins: the value follows the axis, not the cell count
        got = ops.interpolate(row, cols_axis=[0, 1, 2, 5, 10])
        self.assertEqual([got[(0, c)] for c in range(5)], [10, 14, 18, 30, 50])
        block = {(r, c): 0.0 for r in range(3) for c in range(3)}
        block.update({(0, 0): 0, (0, 2): 20, (2, 0): 10, (2, 2): 30})
        got = ops.interpolate(block)
        self.assertEqual((got[(1, 1)], got[(0, 1)], got[(1, 0)], got[(2, 1)]), (15, 10, 5, 20))
        with self.assertRaises(ValueError):
            ops.interpolate({(0, 0): 1, (1, 1): 2})

    def test_smooth(self):
        grid = {(r, c): 0.0 for r in range(3) for c in range(3)}
        grid[(1, 1)] = 8.0
        self.assertEqual(ops.smooth(grid, [(1, 1)]), {(1, 1): 4.0})          # (4x8 + 0) / 8
        self.assertEqual(ops.smooth(grid, [(0, 1)]), {(0, 1): 8 / 7})        # edge: three neighbours
        flat = {k: 5.0 for k in grid}
        self.assertEqual(ops.smooth(flat, list(flat)), flat)                 # a flat table stays flat

    def test_clipboard(self):
        self.assertEqual(ops.to_clipboard([[1, 2.5], [3, 4]]), "1\t2.5\n3\t4")
        self.assertEqual(ops.from_clipboard("1\t2.5\r\n3\t4\n"), [[1, 2.5], [3, 4]])
        self.assertEqual(ops.from_clipboard("1,5;2,5"), [[1.5, 2.5]])        # decimal commas from Excel
        for bad in ("", "abc"):
            with self.assertRaises(ValueError):
                ops.from_clipboard(bad)


class LinkTest(unittest.TestCase):
    def setUp(self):
        self.ecu = FakeEcu(D, engine=False)
        self.link = EcuLink(self.ecu, D)

    def test_framing(self):
        import io
        f = encode_frame(b"Q")
        self.assertEqual((f[:3], len(f)), (b"\x00\x01Q", 7))
        self.assertEqual(read_frame(io.BytesIO(encode_frame(b"\x00hi")).read), b"\x00hi")
        with self.assertRaises(LinkError):
            read_frame(io.BytesIO(f[:-1] + bytes([f[-1] ^ 1])).read)

    def test_pages_write_verify(self):
        self.assertEqual(self.link.check_signature(), D.signature)
        self.assertEqual(self.link.read_pages(), D.defaults)
        f = D.fields["veTable"]
        self.link.write(f.page, f.offset, bytes(range(256)))                 # more than one frame
        self.assertEqual(bytes(self.ecu.ram[f.page][f.offset:f.offset + 256]), bytes(range(256)))
        self.assertTrue(all(len(r) <= 7 + 240 for r in self.ecu.requests))
        self.assertTrue(self.link.verify(self.ecu.ram))
        self.assertFalse(self.link.verify(D.defaults))

    def test_burn_and_power_cycle(self):
        f = D.fields["revLimit"]
        self.link.write(f.page, f.offset, bytes([61]))
        self.assertTrue(int(self.link.realtime()["faults"]) & 0x80)          # default tune until burned
        self.link.burn(poll=0.01)
        self.assertFalse(int(self.link.realtime()["faults"]) & 0x80)
        self.link.write(f.page, f.offset, bytes([70]))                       # not burned
        self.ecu.power_cycle()
        self.assertEqual(self.link.read_page(f.page)[f.offset], 61)

    def test_retries_and_errors(self):
        self.ecu.drop_replies = 1
        self.ecu.corrupt_replies = 1                                         # timeout, then bad CRC, then OK
        self.assertEqual(self.link.signature(), D.signature)
        self.assertEqual(self.link.retried, 2)
        self.ecu.dead = True
        with self.assertRaises(LinkError):
            self.link.signature()
        self.ecu.dead = False
        with self.assertRaises(LinkError) as cm:                             # refused by the ECU: no retry
            self.link.request(b"R" + bytes([9, 0, 0, 0, 1, 0]))
        self.assertIn("range error", str(cm.exception))
        with self.assertRaises(SignatureError):
            EcuLink(FakeEcu(D, signature="E36ECU-dev-p3"), D).check_signature()

    def test_demo_engine_follows_the_tune(self):
        t = [100.0]
        ecu = FakeEcu(D, clock=lambda: t[0])
        link = EcuLink(ecu, D)
        t[0] += 12.0                                                          # mid pull
        rt = link.realtime()
        self.assertTrue(2000 < rt["rpm"] < 7000 and rt["sync"] == 3 and rt["pw"] == 0)   # injection is off
        f = D.fields["injEnable"]
        link.write(f.page, f.offset, b"\x01")
        self.assertGreater(link.realtime()["pw"], 1.0)
        f = D.fields["ignTable"]
        link.write(f.page, f.offset, bytes([f.to_raw(20.0)] * 256))
        self.assertAlmostEqual(link.realtime()["advance"], 20.0, places=1)


if __name__ == "__main__":
    unittest.main()
