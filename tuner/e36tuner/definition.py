"""
The ECU definition (tuner/ecu_def.json, generated from definitions/ecu.toml):
tune fields and their encoding, live-data channels, and the editor layout.

Value encoding follows TunerStudio:  shown value = (raw + translate) * scale
"""
import json
import os
import struct

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ecu_def.json")

TYPE_FMT = {"U08": "B", "S08": "b", "U16": "H", "S16": "h", "U32": "I", "S32": "i"}
TYPE_SIZE = {"U08": 1, "S08": 1, "U16": 2, "S16": 2, "U32": 4, "S32": 4}
TYPE_SIGNED = {"U08": False, "S08": True, "U16": False, "S16": True, "U32": False, "S32": True}


def label(name: str) -> str:
    """injDeadV -> 'Inj Dead V', sync_losses -> 'Sync losses', rpm -> 'RPM'."""
    if name.isupper() or "_" in name:
        return name.replace("_", " ").capitalize()
    if name in ("rpm", "tps", "clt", "iat", "afr", "ve", "pw", "eoi"):
        return name.upper()
    out = ""
    for i, ch in enumerate(name):
        if ch.isupper() and i and not name[i - 1].isupper():
            out += " "
        out += ch
    return out[0].upper() + out[1:]


class Field:
    """One tune setting: a scalar, an option list (bits), a 1D array or a 2D table."""

    def __init__(self, page: int, d: dict):
        self.page = page
        self.name = d["name"]
        self.type = d["type"]
        self.offset = d["offset"]
        self.size = d["size"]
        self.shape = d.get("shape")
        self.units = d.get("units", "")
        self.scale = d.get("scale", 1)
        self.translate = d.get("translate", 0)
        self.digits = d.get("digits", 0)
        self.bits = d.get("bits")
        self.options = d.get("options")
        self.help = d.get("help", "")
        self.label = label(self.name)
        self.item_size = TYPE_SIZE[self.type]
        self.count = self.size // self.item_size
        self._fmt = "<" + TYPE_FMT[self.type]
        if self.bits:
            lo, hi = self.bits
            self.raw_min, self.raw_max = 0, (1 << (hi - lo + 1)) - 1
        else:
            bits = 8 * self.item_size
            lo = -(1 << (bits - 1)) if TYPE_SIGNED[self.type] else 0
            hi = (1 << (bits - 1)) - 1 if TYPE_SIGNED[self.type] else (1 << bits) - 1
            # the allowed range is the definition's min/max, never more than the type holds
            if "min" in d:
                lo = max(lo, self._to_raw_unclamped(d["min"]))
            if "max" in d:
                hi = min(hi, self._to_raw_unclamped(d["max"]))
            self.raw_min, self.raw_max = lo, hi
        self.min, self.max = self.to_shown(self.raw_min), self.to_shown(self.raw_max)

    # ---- raw <-> shown
    def _to_raw_unclamped(self, shown: float) -> int:
        return round(shown / self.scale - self.translate)

    def to_shown(self, raw: int) -> float:
        return round((raw + self.translate) * self.scale, 6)

    def to_raw(self, shown: float) -> int:
        """Nearest raw value, clamped to the allowed range."""
        return min(self.raw_max, max(self.raw_min, self._to_raw_unclamped(shown)))

    def in_range(self, shown: float) -> bool:
        return self.min - 1e-9 <= shown <= self.max + 1e-9

    def format(self, shown) -> str:
        if self.bits:
            return str(shown)
        return f"{shown:.{self.digits}f}"

    @property
    def step(self) -> float:
        """Smallest change that can be stored."""
        return abs(self.scale)

    # ---- bytes
    def read_raw(self, page_bytes, index: int = 0) -> int:
        (v,) = struct.unpack_from(self._fmt, page_bytes, self.offset + index * self.item_size)
        if self.bits:
            lo, hi = self.bits
            v = (v >> lo) & ((1 << (hi - lo + 1)) - 1)
        return v

    def pack_raw(self, page_bytes, index: int, raw: int) -> bytes:
        """Bytes of element `index` with `raw` stored (bit fields keep their other bits)."""
        if self.bits:
            lo, hi = self.bits
            mask = ((1 << (hi - lo + 1)) - 1) << lo
            (cur,) = struct.unpack_from(self._fmt, page_bytes, self.offset)
            raw = (cur & ~mask) | ((raw << lo) & mask)
        return struct.pack(self._fmt, raw)

    def element_offset(self, index: int) -> int:
        return self.offset + index * self.item_size


class Channel:
    """One live-data value."""

    def __init__(self, name, ctype, offset, d):
        self.name = name
        self.type = ctype
        self.offset = offset
        self.units = d.get("units", "")
        self.scale = d.get("scale", 1)
        self.translate = d.get("translate", 0)
        self.digits = d.get("digits", 0)
        self.gauge = d.get("gauge")          # [min, max, low danger, low warn, high warn, high danger]
        self.log = bool(d.get("log"))
        self.label = label(name)
        self._fmt = "<" + TYPE_FMT[ctype]

    def read(self, data: bytes) -> float:
        (raw,) = struct.unpack_from(self._fmt, data, self.offset)
        if self.scale == 1 and not self.translate:
            return raw
        return (raw + self.translate) * self.scale

    def format(self, v) -> str:
        return f"{v:.{self.digits}f}"


class Definition:
    def __init__(self, d: dict):
        self.raw = d
        self.signature = d["signature"]
        self.version = d.get("version", "")
        self.layout_hash = d["layout_hash"]
        self.page_sizes = {p["id"]: p["size"] for p in d["pages"]}
        self.page_names = {p["id"]: p["name"] for p in d["pages"]}
        self.fields = {}
        for p in d["pages"]:
            for f in p["fields"]:
                self.fields[f["name"]] = Field(p["id"], f)
        self.realtime_size = d["realtime_size"]
        self.channels = {}
        for c in d["channels"]:
            n = c.get("shape", [1])[0]
            names = c.get("names") or ([c["name"]] if n == 1 else [f"{c['name']}{i}" for i in range(n)])
            for i, name in enumerate(names):
                self.channels[name] = Channel(name, c["type"], c["offset"] + i * TYPE_SIZE[c["type"]],
                                              c if n == 1 else {"units": c.get("units", "")})
        self.defaults = {int(k): bytes.fromhex(v) for k, v in d["defaults"].items()}
        self.tables = d.get("tables", [])
        self.curves = d.get("curves", [])
        self.dialogs = d.get("dialogs", [])

    @classmethod
    def load(cls, path: str = DEFAULT_PATH) -> "Definition":
        with open(path, encoding="utf-8") as fh:
            return cls(json.load(fh))

    def parse_realtime(self, data: bytes) -> dict:
        """Shown values of every live-data channel by name."""
        if len(data) < self.realtime_size:
            raise ValueError(f"live data is {len(data)} bytes, expected {self.realtime_size}")
        return {name: c.read(data) for name, c in self.channels.items()}

    def menus(self) -> dict:
        """{menu: [(kind, item)]} in definition order; settings no editor shows go under 'Other'."""
        out = {}
        used = set()
        for kind, items in (("dialog", self.dialogs), ("table", self.tables), ("curve", self.curves)):
            for it in items:
                out.setdefault(it["menu"], []).append((kind, it))
                used.update(it.get("fields", []))
                used.update(it[k] for k in ("x", "y", "z") if k in it)
        rest = [n for n in self.fields if n not in used]
        if rest:
            out.setdefault("Other", []).append(
                ("dialog", {"name": "otherDlg", "title": "Other settings", "menu": "Other", "fields": rest}))
        return out
