#!/usr/bin/env python3
"""
Minimal command-line link to the ECU (the first building block of the tuner).

    pip install pyserial
    python3 tools/ecu_cli.py /dev/ttyUSB0 info     # signature + version
    python3 tools/ecu_cli.py /dev/ttyUSB0 watch    # live RPM, sync, sensors (Ctrl+C to stop)
    python3 tools/ecu_cli.py /dev/ttyUSB0 teeth    # one tooth-log capture

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

# realtime_t, firmware/src/status.h (46 bytes, little-endian)
RT_FORMAT = "<IHBB10HHHHHBBBBHHH"
RT_FIELDS = (["uptime_ms", "loops_per_sec", "reset_cause", "flags"]
             + [f"adc_{n}" for n in ADC_NAMES]
             + ["uart_overflows", "frame_errors", "rpm", "rpm_inst", "sync", "cam_pos",
                "loss_reason", "spare", "sync_losses", "noise_teeth", "cam_errors"])


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
              f"noise {rt['noise_teeth']}  cam err {rt['cam_errors']}  {adc}   ", end="", flush=True)
        time.sleep(0.2)


def cmd_teeth(ecu):
    ecu.request(b"T")                       # starts a fresh capture
    time.sleep(0.5)
    teeth = parse_teeth(ecu.request(b"T"))
    print(f"{len(teeth)} teeth")
    for t in teeth:
        marks = " ".join(m for m, on in (("GAP", t["gap"]), ("CAM", t["cam"]), ("noise", t["noise"])) if on)
        print(f"  rev {t['rev']} tooth {t['tooth']:2d}  {t['interval_us']:6d} us  sync {t['sync']}  {marks}")


def main(argv):
    cmds = {"info": cmd_info, "watch": cmd_watch, "teeth": cmd_teeth}
    if len(argv) != 3 or argv[2] not in cmds:
        print(__doc__)
        return 2
    try:
        cmds[argv[2]](Ecu(argv[1]))
    except KeyboardInterrupt:
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
