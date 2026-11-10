#!/usr/bin/env python3
"""
Command-line link to the ECU (the first building block of the tuner).

    pip install pyserial
    python3 tools/ecu_cli.py PORT info                 signature + version
    python3 tools/ecu_cli.py PORT watch                live engine data (Ctrl+C to stop)
    python3 tools/ecu_cli.py PORT teeth                one tooth-log capture
    python3 tools/ecu_cli.py PORT knock                live knock level / noise / retard per cylinder

  Tune (names and units: definitions/ecu.toml, or `tune list`):
    python3 tools/ecu_cli.py PORT tune list            every setting with its value
    python3 tools/ecu_cli.py PORT tune get revLimit
    python3 tools/ecu_cli.py PORT tune set revLimit 6000
    python3 tools/ecu_cli.py PORT tune set injDead 1500,1100,850,700,600,520
    python3 tools/ecu_cli.py PORT tune burn            store in EEPROM (otherwise lost at power-off)
    python3 tools/ecu_cli.py PORT tune save my.tune    save to a file
    python3 tools/ecu_cli.py PORT tune load my.tune    load a file into the ECU (then `tune burn`)
    python3 tools/ecu_cli.py PORT tune defaults        restore the default tune (then `tune burn`)

  Bench output test (overrides the engine calculation while on):
    python3 tools/ecu_cli.py PORT outputs
    python3 tools/ecu_cli.py PORT set ign=on inj=on advance=10 dwell=3 pw=2.5 tdc=60 eoi=380
    python3 tools/ecu_cli.py PORT set off

PORT: Linux /dev/ttyACM0 or /dev/ttyUSB0, macOS /dev/cu.usbmodem..., Windows COM5.
Opening the port resets an Arduino Mega, so the tool waits 2 s for it to boot.
BENCH USE: with real coils or injectors connected, `set` and enabling
ignEnable / injEnable fire them.

Protocol: [u16 BE length][payload][u32 BE CRC32(payload)]; see
firmware/src/comms/frame.h. Layouts come from tuner/ecu_def.json, generated
from definitions/ecu.toml.
"""
import json
import os
import struct
import sys
import time
import zlib

DEF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tuner", "ecu_def.json")
DEF = json.load(open(DEF_PATH))

RC_NAMES = {0x00: "OK", 0x82: "CRC error", 0x83: "unknown command", 0x84: "range error", 0x85: "busy"}
SYNC_NAMES = {0: "none", 1: "pending", 2: "crank 360°", 3: "full 720°"}
LOSS_NAMES = {0: "-", 1: "tooth count", 2: "no gap", 3: "engine stopped"}
ENGINE_BITS = ["running", "cranking", "REV LIMIT", "accel", "after-start", "TEST MODE", "burning", "pump"]
FAULT_BITS = ["TPS1", "TPS2", "TPS disagree", "CLT", "IAT", "MAF", "battery", "DEFAULT TUNE"]
TYPE_FMT = {"U08": "B", "S08": "b", "U16": "H", "S16": "h", "U32": "I", "S32": "i"}
TYPE_SIZE = {"U08": 1, "S08": 1, "U16": 2, "S16": 2, "U32": 4, "S32": 4}
CHUNK = 240                         # bytes per page read/write (fits one frame)

# sched_cfg_t, firmware/src/sched/sched_types.h (12 bytes)
CFG_FORMAT = "<BBHhHHH"
CFG_FIELDS = ["enable", "spare", "tdc_offset", "advance", "dwell_us", "inj_pw_us", "inj_end"]


# ---------------------------------------------------------------- framing --

def encode_frame(payload: bytes) -> bytes:
    return struct.pack(">H", len(payload)) + payload + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF)


def read_frame(read) -> bytes:
    """read(n) -> bytes. Returns the payload; raises on timeout or bad CRC."""
    hdr = read(2)
    if len(hdr) < 2:
        raise TimeoutError("no response from ECU")
    (n,) = struct.unpack(">H", hdr)
    body = read(n + 4)
    if len(body) < n + 4:
        raise TimeoutError("incomplete response")
    payload, (crc,) = body[:n], struct.unpack(">I", body[n:])
    if zlib.crc32(payload) & 0xFFFFFFFF != crc:
        raise IOError("response CRC mismatch")
    return payload


# ------------------------------------------------------------ live data --

def parse_realtime(data: bytes, d=DEF) -> dict:
    """Raw channel values by name (arrays expanded to their `names`)."""
    out = {}
    for c in d["channels"]:
        fmt, size = TYPE_FMT[c["type"]], TYPE_SIZE[c["type"]]
        n = c.get("shape", [1])[0]
        names = c.get("names") or ([c["name"]] if n == 1 else [f"{c['name']}{i}" for i in range(n)])
        for i, name in enumerate(names):
            (out[name],) = struct.unpack_from("<" + fmt, data, c["offset"] + i * size)
    return out


def shown(c, raw):
    return (raw + c.get("translate", 0)) * c.get("scale", 1)


def channel(name, d=DEF):
    return [c for c in d["channels"] if c["name"] == name][0]


# engine2 bits: idle, overrun cut, VANOS, O2 heaters, closed loop, wideband fault, NB A / B rich
ENGINE2_BITS = ["IDLE", "DFCO", "VANOS", "HEAT", "CL", "WB-BAD", "NB-A-rich", "NB-B-rich"]


def describe_realtime(rt: dict) -> str:
    v = lambda n: shown(channel(n), rt[n])
    eng = [b for i, b in enumerate(ENGINE_BITS) if rt["engine"] & (1 << i)]
    flt = [b for i, b in enumerate(FAULT_BITS) if rt["faults"] & (1 << i)]
    eng2 = [b for i, b in enumerate(ENGINE2_BITS) if rt.get("engine2", 0) & (1 << i)]
    out = "".join(("IGN " if rt["out_enable"] & 1 else "", "INJ " if rt["out_enable"] & 2 else "")) or "off "
    return (f"{rt['rpm']:5d} rpm  sync {SYNC_NAMES.get(rt['sync'], '?'):10s} "
            f"TPS {v('tps'):5.1f}%  air {v('air'):5.1f}mg  AFR {v('afr'):5.2f}/{v('afrTarget'):5.2f}  "
            f"pw {v('pw'):6.3f}ms  adv {v('advance'):5.1f}°  CLT {v('clt'):5.1f}C  {v('vbat'):5.2f}V  "
            f"trim {v('lambdaTrimA'):+5.1f}/{v('lambdaTrimB'):+5.1f}%  IAC {rt['iacDuty']:3d}%  "
            f"out {out} {' '.join(eng + eng2)}  {('FAULT: ' + ' '.join(flt)) if flt else ''}")


# ------------------------------------------------------------ tune values --

def find_field(name, d=DEF):
    for p in d["pages"]:
        for f in p["fields"]:
            if f["name"] == name:
                return p, f
    raise ValueError(f"unknown setting {name!r} (see `tune list`)")


def field_count(f):
    n = 1
    for s in f.get("shape", [1]):
        n *= s
    return n


def decode_field(f, page_bytes: bytes):
    """Shown values of a field from its page (list for arrays, value or option name for scalars)."""
    fmt, size = TYPE_FMT[f["type"]], TYPE_SIZE[f["type"]]
    raws = [struct.unpack_from("<" + fmt, page_bytes, f["offset"] + i * size)[0] for i in range(field_count(f))]
    if "bits" in f:
        lo, hi = f["bits"]
        return f["options"][(raws[0] >> lo) & ((1 << (hi - lo + 1)) - 1)]
    vals = [round((r + f.get("translate", 0)) * f.get("scale", 1), 6) for r in raws]
    return vals if "shape" in f else vals[0]


def encode_field(f, text: str, old: bytes) -> bytes:
    """Bytes for a field from user text: a number, a comma list, or an option name."""
    fmt, size = TYPE_FMT[f["type"]], TYPE_SIZE[f["type"]]
    if "bits" in f:
        opts = [o.lower() for o in f["options"]]
        t = text.lower()
        idx = opts.index(t) if t in opts else int(text)
        if not 0 <= idx < len(f["options"]):
            raise ValueError(f"{f['name']}: choose one of {f['options']}")
        lo, hi = f["bits"]
        mask = ((1 << (hi - lo + 1)) - 1) << lo
        cur = old[0]
        return bytes([(cur & ~mask) | (idx << lo)])
    vals = [float(x) for x in text.split(",")]
    n = field_count(f)
    if len(vals) == 1 and n > 1:
        vals = vals * n
    if len(vals) != n:
        raise ValueError(f"{f['name']}: needs {n} values, got {len(vals)}")
    out = b""
    for v in vals:
        if not f.get("min", -1e18) - 1e-9 <= v <= f.get("max", 1e18) + 1e-9:
            raise ValueError(f"{f['name']}: {v} outside {f.get('min')} .. {f.get('max')} {f.get('units', '')}")
        raw = round(v / f.get("scale", 1) - f.get("translate", 0))
        out += struct.pack("<" + fmt, raw)
    return out


# ------------------------------------------------------------ bench test --

def parse_cfg(data: bytes) -> dict:
    return dict(zip(CFG_FIELDS, struct.unpack(CFG_FORMAT, data[:12])))


def pack_cfg(cfg: dict) -> bytes:
    return struct.pack(CFG_FORMAT, *(int(cfg[k]) for k in CFG_FIELDS))


def apply_settings(cfg: dict, args) -> dict:
    """Apply `key=value` settings (user units) to a config dict."""
    cfg = dict(cfg)
    for a in args:
        if a == "off":
            cfg["enable"] = 0
            continue
        if "=" not in a:
            raise ValueError(f"expected key=value, got {a!r}")
        k, v = a.split("=", 1)
        if k in ("ign", "inj"):
            bit = 1 if k == "ign" else 2
            on = v.lower() in ("on", "1", "yes", "true")
            cfg["enable"] = (cfg["enable"] | bit) if on else (cfg["enable"] & ~bit)
        elif k == "advance":
            cfg["advance"] = round(float(v) * 10)
        elif k == "tdc":
            cfg["tdc_offset"] = round(float(v) * 10) % 7200
        elif k == "eoi":
            cfg["inj_end"] = round(float(v) * 10) % 7200
        elif k == "dwell":
            cfg["dwell_us"] = round(float(v) * 1000)
        elif k == "pw":
            cfg["inj_pw_us"] = round(float(v) * 1000)
        else:
            raise ValueError(f"unknown setting {k!r}")
    return cfg


def describe_cfg(c: dict) -> str:
    on = lambda b: "ON" if c["enable"] & b else "off"
    return (f"test outputs: ignition {on(1)}, injection {on(2)}\n"
            f"  cyl 1 TDC {c['tdc_offset'] / 10:.1f}° after tooth 0, advance {c['advance'] / 10:.1f}° BTDC, "
            f"dwell {c['dwell_us'] / 1000:.2f} ms\n"
            f"  pulse width {c['inj_pw_us'] / 1000:.2f} ms per cycle, injection ends {c['inj_end'] / 10:.1f}° BTDC")


def parse_teeth(data: bytes):
    count = data[0]
    out = []
    for i in range(count):
        interval, pos, flags = struct.unpack_from("<HBB", data, 1 + 4 * i)
        out.append({"interval_us": interval * 4, "tooth": pos & 0x7F, "rev": pos >> 7,
                    "gap": bool(flags & 1), "cam": bool(flags & 2), "noise": bool(flags & 4),
                    "sync": flags >> 4})
    return out


# ---------------------------------------------------------------- the ECU --

class Ecu:
    def __init__(self, port: str):
        import serial  # pyserial
        self.ser = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(2.0)                     # Arduino resets when the port opens
        self.ser.reset_input_buffer()

    def request(self, payload: bytes) -> bytes:
        self.ser.write(encode_frame(payload))
        resp = read_frame(self.ser.read)
        if not resp or resp[0] != 0:
            code = resp[0] if resp else -1
            raise IOError(f"ECU replied {RC_NAMES.get(code, hex(code))}")
        return resp[1:]

    def read_page(self, page: int, size: int) -> bytes:
        out = b""
        while len(out) < size:
            n = min(CHUNK, size - len(out))
            out += self.request(b"R" + struct.pack("<HHH", page, len(out), n))
        return out

    def write_page(self, page: int, offset: int, data: bytes):
        for k in range(0, len(data), CHUNK):
            part = data[k:k + CHUNK]
            self.request(b"M" + struct.pack("<HHH", page, offset + k, len(part)) + part)

    def realtime(self) -> dict:
        return parse_realtime(self.request(b"A"))

    def check_signature(self):
        sig = self.request(b"Q").decode()
        if sig != DEF["signature"]:
            raise IOError(f"ECU firmware {sig!r} does not match tuner/ecu_def.json ({DEF['signature']!r}); "
                          "use the tools from the same version as the firmware")


# --------------------------------------------------------------- commands --

def cmd_info(ecu, args):
    print("signature:", ecu.request(b"Q").decode())
    print("version:  ", ecu.request(b"S").decode())


def cmd_watch(ecu, args):
    while True:
        print("\r" + describe_realtime(ecu.realtime()) + "   ", end="", flush=True)
        time.sleep(0.2)


def cmd_teeth(ecu, args):
    ecu.request(b"T")                       # starts a fresh capture
    time.sleep(0.5)
    teeth = parse_teeth(ecu.request(b"T"))
    print(f"{len(teeth)} teeth")
    for t in teeth:
        marks = " ".join(m for m, on in (("GAP", t["gap"]), ("CAM", t["cam"]), ("noise", t["noise"])) if on)
        print(f"  rev {t['rev']} tooth {t['tooth']:2d}  {t['interval_us']:6d} us  sync {t['sync']}  {marks}")


KNOCK_BITS = ["enabled", "active", "KNOCK", "CHIP FAULT"]


def describe_knock(rt: dict) -> str:
    st = [b for i, b in enumerate(KNOCK_BITS) if rt["knockState"] & (1 << i)]
    lines = [f"{rt['rpm']:5d} rpm  knock {' '.join(st) or 'off'}  events {rt['knockCount']}",
             "  cyl   level   noise   ratio   retard"]
    for c in range(1, 7):
        lvl, noi, ret = rt[f"knockLevel{c}"], rt[f"knockNoise{c}"], rt[f"knockRetard{c}"] / 10
        ratio = f"{lvl / noi:5.2f}" if noi else "    -"
        lines.append(f"  {c:3d}  {lvl:6d}  {noi:6d}   {ratio}  {ret:5.1f}°")
    return "\n".join(lines)


def cmd_knock(ecu, args):
    while True:
        text = describe_knock(ecu.realtime())
        print("\x1b[H\x1b[J" + text, flush=True)          # clear screen, redraw
        time.sleep(0.3)


def cmd_outputs(ecu, args):
    print(describe_cfg(parse_cfg(ecu.request(b"p"))))


def cmd_set(ecu, args):
    cfg = apply_settings(parse_cfg(ecu.request(b"p")), args)
    ecu.request(b"P" + pack_cfg(cfg))
    print(describe_cfg(parse_cfg(ecu.request(b"p"))))


def cmd_tune(ecu, args):
    ecu.check_signature()
    sub = args[0]
    pages = {p["id"]: ecu.read_page(p["id"], p["size"]) for p in DEF["pages"]} if sub in ("list", "get", "set", "save") else {}
    if sub == "list":
        for p in DEF["pages"]:
            for f in p["fields"]:
                if f.get("shape") and len(f["shape"]) == 2:
                    print(f"  {f['name']:<16} table {f['shape'][0]}x{f['shape'][1]} {f.get('units', '')}")
                else:
                    print(f"  {f['name']:<16} {decode_field(f, pages[p['id']])} {f.get('units', '')}")
    elif sub == "get":
        p, f = find_field(args[1])
        v = decode_field(f, pages[p["id"]])
        if f.get("shape") and len(f["shape"]) == 2:
            x, y = f["shape"]
            for r in range(y - 1, -1, -1):
                print("  " + " ".join(f"{v[r * x + c]:6g}" for c in range(x)))
        else:
            print(f"{f['name']} = {v} {f.get('units', '')}")
        if f.get("help"):
            print(" ", f["help"])
    elif sub == "set":
        p, f = find_field(args[1])
        old = pages[p["id"]][f["offset"]:f["offset"] + f["size"]]
        ecu.write_page(p["id"], f["offset"], encode_field(f, args[2], old))
        print(f"{f['name']} = {decode_field(f, ecu.read_page(p['id'], p['size']))} (in RAM; `tune burn` to keep)")
    elif sub == "burn":
        for p in DEF["pages"]:
            ecu.request(b"B" + struct.pack("<H", p["id"]))
        t0 = time.time()
        while ecu.realtime()["engine"] & 0x40:
            time.sleep(0.2)
            if time.time() - t0 > 30:
                raise IOError("burn did not finish")
        print(f"burned ({time.time() - t0:.1f} s)")
    elif sub == "save":
        out = {"signature": DEF["signature"], "layout_hash": DEF["layout_hash"],
               "pages": {str(i): b.hex() for i, b in pages.items()},
               "values": {f["name"]: decode_field(f, pages[p["id"]]) for p in DEF["pages"] for f in p["fields"]}}
        with open(args[1], "w") as fh:
            json.dump(out, fh, indent=1)
        print("saved", args[1])
    elif sub == "load":
        t = json.load(open(args[1]))
        if t.get("layout_hash") != DEF["layout_hash"]:
            raise ValueError("tune file is for a different firmware layout")
        for p in DEF["pages"]:
            ecu.write_page(p["id"], 0, bytes.fromhex(t["pages"][str(p["id"])]))
        print("loaded into the ECU's RAM; `tune burn` to keep")
    elif sub == "defaults":
        for p in DEF["pages"]:
            ecu.request(b"D" + struct.pack("<H", p["id"]))
        print("defaults restored in RAM; `tune burn` to keep")
    else:
        raise ValueError(f"unknown tune command {sub!r}")


COMMANDS = {"info": (cmd_info, 0), "watch": (cmd_watch, 0), "teeth": (cmd_teeth, 0), "knock": (cmd_knock, 0),
            "outputs": (cmd_outputs, 0), "set": (cmd_set, 1), "tune": (cmd_tune, 1)}
TUNE_ARGS = {"list": 1, "get": 2, "set": 3, "burn": 1, "save": 2, "load": 2, "defaults": 1}


def main(argv):
    if len(argv) < 3 or argv[2] not in COMMANDS or len(argv) - 3 < COMMANDS[argv[2]][1]:
        print(__doc__)
        return 2
    args = argv[3:]
    try:
        if argv[2] == "set":
            apply_settings(dict.fromkeys(CFG_FIELDS, 0), args)       # validate before connecting
        if argv[2] == "tune" and (args[0] not in TUNE_ARGS or len(args) != TUNE_ARGS[args[0]]):
            print(__doc__)
            return 2
        COMMANDS[argv[2]][0](Ecu(argv[1]), args)
    except KeyboardInterrupt:
        print()
    except (ValueError, IOError) as e:
        print("error:", e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
