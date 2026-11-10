"""
A pretend ECU that speaks the real protocol, for the tuner's demo mode and its
tests. It keeps tune pages in "RAM" and "EEPROM" like the firmware, and a toy
engine whose live data follows the tune (so edits visibly change something).

It is NOT the firmware's engine maths: numbers are plausible, not exact. The
real firmware in the simulator (`make sim-pty` in firmware/) is the reference.
"""
import math
import struct
import time
import zlib

from .definition import TYPE_FMT
from .link import encode_frame


def _interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            if xs[i] == xs[i - 1]:
                return ys[i]
            return ys[i - 1] + (x - xs[i - 1]) / (xs[i] - xs[i - 1]) * (ys[i] - ys[i - 1])
    return ys[-1]


class FakeEcu:
    """Use as the transport of an EcuLink: write() takes request frames, read() returns the replies."""
    BURN_SECONDS = 0.4

    def __init__(self, definition, signature=None, clock=time.monotonic, engine=True):
        self.d = definition
        self.signature = signature or definition.signature
        self.clock = clock
        self.engine = engine
        self.ram = {p: bytearray(b) for p, b in definition.defaults.items()}
        self.eeprom = None                      # nothing burned yet -> DEFAULT TUNE fault
        self.burn_until = 0.0
        self.t0 = clock()
        self._rx = bytearray()
        self._tx = bytearray()
        self.closed = False
        # test hooks
        self.requests = []                      # every request payload received
        self.drop_replies = 0                   # swallow the next N replies (timeout)
        self.corrupt_replies = 0                # break the CRC of the next N replies
        self.dead = False                       # never answer (unplugged cable)

    # ---- transport
    def write(self, data):
        if self.closed:
            raise OSError("port is closed")
        self._rx += data
        while len(self._rx) >= 2:
            (n,) = struct.unpack_from(">H", self._rx, 0)
            if len(self._rx) < n + 6:
                break
            payload, (crc,) = bytes(self._rx[2:2 + n]), struct.unpack_from(">I", self._rx, 2 + n)
            del self._rx[:n + 6]
            if self.dead:
                continue
            reply = self.handle(payload) if zlib.crc32(payload) & 0xFFFFFFFF == crc else b"\x82"
            frame = bytearray(encode_frame(reply))
            if self.drop_replies:
                self.drop_replies -= 1
                continue
            if self.corrupt_replies:
                self.corrupt_replies -= 1
                frame[-1] ^= 0xFF
            self._tx += frame
        return len(data)

    def read(self, n):
        out = bytes(self._tx[:n])
        del self._tx[:n]
        return out                              # short = "timeout", like pyserial

    def reset_input_buffer(self):
        self._tx.clear()

    def close(self):
        self.closed = True

    # ---- protocol
    def handle(self, req: bytes) -> bytes:
        self.requests.append(req)
        cmd, n = req[:1], len(req)
        sizes = self.d.page_sizes
        if cmd == b"Q":
            return b"\x00" + self.signature.encode()
        if cmd == b"S":
            return b"\x00" + (self.d.version or "demo ECU").encode()
        if cmd == b"A":
            return b"\x00" + self.realtime_bytes()
        if cmd == b"O":
            if n != 5:
                return b"\x84"
            off, cnt = struct.unpack_from("<HH", req, 1)
            if off + cnt > self.d.realtime_size:
                return b"\x84"
            return b"\x00" + self.realtime_bytes()[off:off + cnt]
        if cmd in (b"R", b"M"):
            if n < 7:
                return b"\x84"
            page, off, cnt = struct.unpack_from("<HHH", req, 1)
            if page not in sizes or off + cnt > sizes[page] or cnt > 255:
                return b"\x84"
            if cmd == b"R":
                return (b"\x00" + bytes(self.ram[page][off:off + cnt])) if n == 7 else b"\x84"
            if n != 7 + cnt:
                return b"\x84"
            self.ram[page][off:off + cnt] = req[7:]
            return b"\x00"
        if cmd in (b"B", b"D"):
            if n != 3 or struct.unpack_from("<H", req, 1)[0] not in sizes:
                return b"\x84"
            (page,) = struct.unpack_from("<H", req, 1)
            if cmd == b"B":
                self.eeprom = self.eeprom or {p: bytes(b) for p, b in self.d.defaults.items()}
                self.eeprom[page] = bytes(self.ram[page])
                self.burn_until = self.clock() + self.BURN_SECONDS
            else:
                self.ram[page][:] = self.d.defaults[page]
            return b"\x00"
        return b"\x83"

    def power_cycle(self):
        """RAM is lost; the last burned tune (or the defaults) comes back."""
        src = self.eeprom or self.d.defaults
        self.ram = {p: bytearray(b) for p, b in src.items()}
        self.burn_until = 0.0

    # ---- toy engine
    def _raws(self, name):
        f = self.d.fields[name]
        return [f.read_raw(self.ram[f.page], i) for i in range(f.count)]

    def _vals(self, name):
        f = self.d.fields[name]
        return [f.to_shown(r) for r in self._raws(name)]

    def _table(self, t, x, y):
        tb = next(tb for tb in self.d.tables if tb["z"] == t)
        xs, ys, z = self._vals(tb["x"]), self._vals(tb["y"]), self._vals(t)
        nx = len(xs)
        rows = [_interp(x, xs, z[r * nx:(r + 1) * nx]) for r in range(len(ys))]
        return _interp(y, ys, rows)

    def realtime_values(self) -> dict:
        t = self.clock() - self.t0
        v = dict.fromkeys(self.d.channels, 0)
        v.update(uptime_ms=int(t * 1000) & 0xFFFFFFFF, loops_per_sec=4200, flags=1, vbat=13.9,
                 adcVbat=570, clt=min(88.0, 20 + t * 1.5), iat=27.0)
        burning = self.clock() < self.burn_until
        faults = 0 if self.eeprom else 0x80
        engine = 0x40 if burning else 0
        ign = self._raws("ignEnable")[0]
        inj = self._raws("injEnable")[0]
        v["out_enable"] = ign | (inj << 1)
        if self.engine:
            # a slow drive cycle: idle, pull through the revs, lift off
            phase = (t % 24.0) / 24.0
            tps = 0.0 if phase < 0.25 else (min(100.0, (phase - 0.25) * 260) if phase < 0.75 else 3.0)
            rpm = 850 + 25 * math.sin(t * 5) if phase < 0.25 else (
                850 + (phase - 0.25) * 2 * 5900 if phase < 0.75 else 6750 - (phase - 0.75) * 4 * 5900)
            limit = self._vals("revLimit")[0]
            if rpm > limit:
                rpm, engine = limit, engine | 0x04
            ve = self._table("veTable", rpm, tps)
            disp = self._vals("displacement")[0]
            air = ve / 100 * disp / 6 * 1.16                      # mg per cylinder at ~1.16 g/l
            afr_t = self._table("afrTable", rpm, air)
            flow = self._vals("injFlow")[0] * self._vals("fuelDensity")[0] / 60     # mg per ms
            warm = _interp(v["clt"], self._vals("warmupT"), self._vals("warmup"))
            dead = _interp(v["vbat"], self._vals("injDeadV"), self._vals("injDead")) / 1000
            pw = air / afr_t / flow * warm / 100 + dead
            if engine & 0x04 or not inj:
                pw = 0.0
            adv = self._table("ignTable", rpm, air)
            cycle_ms = 120000 / max(rpm, 1)
            v.update(rpm=int(rpm), rpm_inst=int(rpm), sync=3, cam_pos=41, tps=tps, air=air, ve=ve,
                     afrTarget=afr_t, afr=afr_t + 0.25 * math.sin(t * 2.3), mafGs=air * rpm / 20000,
                     mafVolts=min(4.9, 0.6 + (air * rpm / 20000) ** 0.4 * 0.72), warmupCor=warm,
                     pw=pw, deadTime=dead, duty=min(100, pw / cycle_ms * 100), advance=adv,
                     dwellUs=_interp(v["vbat"], self._vals("dwellV"), self._vals("dwell")) / 1000,
                     eoi=self._table("eoiTable", rpm, air), nbo2a=450 + 350 * math.sin(t * 6),
                     nbo2b=450 + 350 * math.cos(t * 6), blend=self._table("blendTable", rpm, tps),
                     adcTps1=int(100 + tps * 8), adcTps2=int(100 + tps * 8), adcMaf=300, adcClt=400,
                     adcIat=520)
            engine |= 0x01 | 0x80
        v["engine"], v["faults"] = engine, faults
        return v

    def realtime_bytes(self) -> bytes:
        out = bytearray(self.d.realtime_size)
        for name, val in self.realtime_values().items():
            c = self.d.channels[name]
            raw = round(val / c.scale - c.translate)
            fmt = "<" + TYPE_FMT[c.type]
            size = struct.calcsize(fmt)
            lo = -(1 << (8 * size - 1)) if fmt[1].islower() else 0
            hi = (1 << (8 * size - 1)) - 1 if fmt[1].islower() else (1 << (8 * size)) - 1
            struct.pack_into(fmt, out, c.offset, min(hi, max(lo, raw)))
        return bytes(out)
