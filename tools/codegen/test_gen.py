"""Tests for the definition generator: python3 -m unittest tools/codegen/test_gen.py"""
import copy
import json
import os
import re
import sys
import tomllib
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import gen  # noqa: E402


class GeneratedFilesTest(unittest.TestCase):
    def test_up_to_date(self):
        self.assertEqual(gen.main(["gen.py", "--check"]), 0, "run: python3 tools/codegen/gen.py")

    def test_layout_is_contiguous(self):
        d = json.load(open(gen.OUT_JSON))
        for p in d["pages"]:
            off = 0
            for f in p["fields"]:
                self.assertEqual(f["offset"], off, f["name"])
                off += f["size"]
            self.assertEqual(off, p["size"])
        off = 0
        for c in d["channels"]:
            self.assertEqual(c["offset"], off, c["name"])
            off += gen.TYPES[c["type"]][0] * (c.get("shape", [1])[0])
        self.assertEqual(off, d["realtime_size"])

    def test_ini_matches_layout(self):
        ini = open(gen.OUT_INI).read()
        d = json.load(open(gen.OUT_JSON))
        consts = ini[ini.index("[Constants]"):ini.index("[SettingContextHelp]")]
        for p in d["pages"]:
            for f in p["fields"]:
                m = re.search(rf"^\s+{f['name']}\s+= \w+,\s+{f['type']}, (\d+),", consts, re.M)
                self.assertIsNotNone(m, f["name"])
                self.assertEqual(int(m.group(1)), f["offset"], f["name"])
        self.assertIn(f"ochBlockSize   = {d['realtime_size']}", ini)
        self.assertIn(f'signature      = "{d["signature"]}"', ini)
        sizes = re.search(r"pageSize\s+= (.*)", ini).group(1)
        self.assertEqual([int(x) for x in sizes.split(",")], [p["size"] for p in d["pages"]])
        # every name used by a table / curve / dialog / gauge exists
        names = {f["name"] for p in d["pages"] for f in p["fields"]}
        chans = set()
        for c in d["channels"]:
            chans.update(c.get("names") or [c["name"]])
        for m in re.finditer(r"^\s+[xyz]Bins\s+= (\w+)(?:, (\w+))?", ini, re.M):
            self.assertIn(m.group(1), names)
            if m.group(2):
                self.assertIn(m.group(2), chans)
        for m in re.finditer(r'^\s+field = "[^"]*", (\w+)', ini, re.M):
            self.assertIn(m.group(1), names)

    def test_c_header_matches_layout(self):
        h = open(gen.OUT_TUNE_H).read()
        d = json.load(open(gen.OUT_JSON))
        for p in d["pages"]:
            self.assertIn(f"#define TUNE_PAGE{p['id']}_SIZE {p['size']}u", h)
            for f in p["fields"]:
                self.assertIn(f"#define TU_{gen.c_ident(f['name'])}_OFF {f['offset']}u", h)
        self.assertIn(f"0x{d['layout_hash']:08X}UL", h)


class DefinitionErrorsTest(unittest.TestCase):
    def setUp(self):
        with open(gen.DEF, "rb") as fh:
            self.d = tomllib.load(fh)

    def check_error(self, d, text):
        with self.assertRaises(gen.DefError) as cm:
            gen.layout(d)
        self.assertIn(text, str(cm.exception))

    def test_duplicate_name(self):
        d = copy.deepcopy(self.d)
        d["channels"][0]["name"] = "injFlow"
        self.check_error(d, "duplicate")

    def test_default_out_of_range(self):
        d = copy.deepcopy(self.d)
        f = [f for f in d["pages"][0]["fields"] if f["name"] == "revLimit"][0]
        f["default"] = 20000
        self.check_error(d, "revLimit")

    def test_table_shape_mismatch(self):
        d = copy.deepcopy(self.d)
        t = [t for t in d["tables"] if t["name"] == "eoiTbl"][0]
        t["x"] = "rpmBins"      # 16 bins for an 8x8 table
        self.check_error(d, "eoiTbl")

    def test_unknown_reference(self):
        d = copy.deepcopy(self.d)
        d["dialogs"][0]["fields"].append("noSuchField")
        self.check_error(d, "noSuchField")


if __name__ == "__main__":
    unittest.main()
