#!/usr/bin/env python3
"""
Minimal command-line link to the ECU (the first building block of the tuner).

    pip install pyserial
    python3 tools/ecu_cli.py /dev/ttyUSB0 info     # signature + version
    python3 tools/ecu_cli.py /dev/ttyUSB0 watch    # live RPM, sync, sensors (Ctrl+C to stop)
    python3 tools/ecu_cli.py /dev/ttyUSB0 teeth    # one tooth-log capture
    python3 tools/ecu_cli.py /dev/ttyUSB0 outputs  # injector/coil test settings
    python3 tools/ecu_cli.py /dev/ttyUSB0 set ign=on inj=on advance=10 dwell=3 pw=2.5 tdc=60 eoi=380
    python3 tools/ecu_cli.py /dev/ttyUSB0 set off  # all injectors and coils off

`set` units: advance/tdc/eoi in degrees, dwell/pw in milliseconds. Outputs are
OFF at every power-up. BENCH USE ONLY: with real coils or injectors connected,
`set` fires them.

On Windows the port is e.g. COM5. Opening the port resets an Arduino Mega, so
the tool waits 2 s for it to boot.

Protocol: [u16 BE length][payload][u32 BE CRC32(payload)]; see
firmware/src/comms/frame.h.
"""
import struct
import sys
import time
import zlib

RC_NAMES = {0x00: "OK", 0x82: "CRC error", 0x83: "unknown command", 0x84: "range error", 0x85: "busy"}
SYNC_NAMES = {0: "none", 1: "pending", 2: "crank 360°", 3: "full 720°"}
LOSS_NAMES = {0: "-", 1: "tooth count", 2: "no gap", 3: "engine stopped"}
ADC_NAMES = ["MAF", "TPS1", "TPS2", "CLT", "IAT", "WBO2", "NBO2a", "NBO2b", "VBAT", "MAP"]

# realtime_t, firmware/src/status.h (50 bytes, little-endian)
RT_FORMAT = "<IHBB10HHHHHBBBBHHHHH"
RT_FIELDS = (["uptime_ms", "loops_per_sec", "reset_cause", "flags"]
             + [f"adc_{n}" for n in ADC_NAMES]
             + ["uart_overflows", "frame_errors", "rpm", "rpm_inst", "sync", "cam_pos",
                "loss_reason", "out_enable", "sync_losses", "noise_teeth", "cam_errors",
                "sched_late", "sched_overlap"])

# sched_cfg_t, firmware/src/sched/sched_core.h (12 bytes)
CFG_FORMAT = "<BBHhHHH"
CFG_FIELDS = ["enable", "spare", "tdc_offset", "advance", "dwell_us", "inj_pw_us", "inj_end"]


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


def parse_realtime(data: bytes) -> dict:
    return dict(zip(RT_FIELDS, struct.unpack(RT_FORMAT, data[:struct.calcsize(RT_FORMAT)])))


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
    return (f"ignition {on(1)}, injection {on(2)}\n"
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


def cmd_info(ecu):
    print("signature:", ecu.request(b"Q").decode())
    print("version:  ", ecu.request(b"S").decode())


def cmd_watch(ecu):
    while True:
        rt = parse_realtime(ecu.request(b"A"))
        adc = " ".join(f"{n}={rt['adc_' + n] * 5.0 / 1023:.2f}V" for n in ADC_NAMES[:5])
        print(f"\r{rt['rpm']:5d} rpm  sync {SYNC_NAMES.get(rt['sync'], '?'):10s} "
              f"cam@{rt['cam_pos']:3d}  losses {rt['sync_losses']} (last: {LOSS_NAMES.get(rt['loss_reason'], '?')})  "
              f"noise {rt['noise_teeth']}  cam err {rt['cam_errors']}  "
              f"out {'IGN ' if rt['out_enable'] & 1 else ''}{'INJ ' if rt['out_enable'] & 2 else ''}"
              f"{'' if rt['out_enable'] else 'off '}late {rt['sched_late']}  {adc}   ", end="", flush=True)
        time.sleep(0.2)


def cmd_teeth(ecu):
    ecu.request(b"T")                       # starts a fresh capture
    time.sleep(0.5)
    teeth = parse_teeth(ecu.request(b"T"))
    print(f"{len(teeth)} teeth")
    for t in teeth:
        marks = " ".join(m for m, on in (("GAP", t["gap"]), ("CAM", t["cam"]), ("noise", t["noise"])) if on)
        print(f"  rev {t['rev']} tooth {t['tooth']:2d}  {t['interval_us']:6d} us  sync {t['sync']}  {marks}")


def cmd_outputs(ecu, args=()):
    print(describe_cfg(parse_cfg(ecu.request(b"p"))))


def cmd_set(ecu, args):
    cfg = apply_settings(parse_cfg(ecu.request(b"p")), args)
    ecu.request(b"P" + pack_cfg(cfg))
    print(describe_cfg(parse_cfg(ecu.request(b"p"))))


def main(argv):
    cmds = {"info": cmd_info, "watch": cmd_watch, "teeth": cmd_teeth}
    cmds_args = {"outputs": cmd_outputs, "set": cmd_set}
    if len(argv) < 3 or (argv[2] not in cmds and argv[2] not in cmds_args) \
            or (argv[2] in cmds and len(argv) != 3) or (argv[2] == "set" and len(argv) < 4):
        print(__doc__)
        return 2
    try:
        if argv[2] in cmds:
            cmds[argv[2]](Ecu(argv[1]))
        else:
            if argv[2] == "set":
                apply_settings(dict.fromkeys(CFG_FIELDS, 0), argv[3:])   # validate before connecting
            cmds_args[argv[2]](Ecu(argv[1]), argv[3:])
    except KeyboardInterrupt:
        print()
    except (ValueError, IOError) as e:
        print("error:", e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
