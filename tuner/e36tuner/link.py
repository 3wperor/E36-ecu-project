"""
Serial link to the ECU (no Qt here, so it is tested on its own).

Framing (firmware/src/comms/frame.h, TunerStudio "msEnvelope"):
    [u16 BE length][payload][u32 BE CRC32(payload)]
Requests start with a command character; responses with a status byte.
"""
import struct
import time
import zlib

RC_NAMES = {0x00: "OK", 0x82: "CRC error", 0x83: "unknown command", 0x84: "range error", 0x85: "busy"}
CHUNK = 240                 # bytes per page read/write (fits one frame)
ENGINE_BURNING = 0x40       # live-data `engine` bit: an EEPROM burn is pending or running


class LinkError(IOError):
    """The ECU did not answer, or answered with an error."""


class SignatureError(LinkError):
    """The firmware's tune layout is not the one this tuner was built for."""


def encode_frame(payload: bytes) -> bytes:
    return struct.pack(">H", len(payload)) + payload + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF)


def read_frame(read) -> bytes:
    """read(n) -> bytes. Returns the payload; raises LinkError on timeout or bad CRC."""
    hdr = read(2)
    if len(hdr) < 2:
        raise LinkError("no response from the ECU")
    (n,) = struct.unpack(">H", hdr)
    body = read(n + 4)
    if len(body) < n + 4:
        raise LinkError("incomplete response from the ECU")
    payload, (crc,) = body[:n], struct.unpack(">I", body[n:])
    if zlib.crc32(payload) & 0xFFFFFFFF != crc:
        raise LinkError("response CRC mismatch")
    return payload


class EcuLink:
    """
    Request/response over a transport with read(n) (blocking, with a timeout),
    write(bytes), reset_input_buffer() and close() — a pyserial port, or FakeEcu.
    """
    RETRIES = 2
    BURN_FLAG_DELAY = 0.05

    def __init__(self, transport, definition):
        self.t = transport
        self.d = definition
        self.retried = 0            # requests that needed a retry (shown as link quality)

    def close(self):
        self.t.close()

    def request(self, payload: bytes) -> bytes:
        last = None
        for attempt in range(self.RETRIES + 1):
            if attempt:
                self.retried += 1
                time.sleep(0.05)                # let a half-sent reply finish, then drop it
                self.t.reset_input_buffer()
            self.t.write(encode_frame(payload))
            try:
                resp = read_frame(self.t.read)
            except LinkError as e:
                last = e
                continue
            if not resp:
                last = LinkError("empty response from the ECU")
                continue
            if resp[0] == 0x82:                 # the ECU saw a corrupted request: send it again
                last = LinkError("the ECU reported a CRC error")
                continue
            if resp[0] != 0:
                raise LinkError(f"the ECU replied: {RC_NAMES.get(resp[0], hex(resp[0]))}")
            return resp[1:]
        raise last

    # ---- commands
    def signature(self) -> str:
        return self.request(b"Q").decode(errors="replace")

    def version(self) -> str:
        return self.request(b"S").decode(errors="replace")

    def check_signature(self) -> str:
        sig = self.signature()
        if sig != self.d.signature:
            raise SignatureError(
                f"The ECU firmware is '{sig}', but this tuner is built for '{self.d.signature}'. "
                "Use the tuner from the same version as the firmware.")
        return sig

    def realtime_bytes(self) -> bytes:
        return self.request(b"A")

    def realtime(self) -> dict:
        return self.d.parse_realtime(self.realtime_bytes())

    def read_page(self, page: int) -> bytes:
        size, out = self.d.page_sizes[page], b""
        while len(out) < size:
            n = min(CHUNK, size - len(out))
            part = self.request(b"R" + struct.pack("<HHH", page, len(out), n))
            if len(part) != n:
                raise LinkError(f"page read returned {len(part)} bytes instead of {n}")
            out += part
        return out

    def read_pages(self) -> dict:
        return {p: self.read_page(p) for p in self.d.page_sizes}

    def write(self, page: int, offset: int, data: bytes):
        for k in range(0, len(data), CHUNK):
            part = data[k:k + CHUNK]
            self.request(b"M" + struct.pack("<HHH", page, offset + k, len(part)) + part)

    def write_pages(self, pages: dict):
        for p, data in pages.items():
            self.write(p, 0, bytes(data))

    def verify(self, pages: dict) -> bool:
        """True if the ECU's RAM holds exactly these pages."""
        return all(self.read_page(p) == bytes(data) for p, data in pages.items())

    def restore_defaults(self):
        for p in self.d.page_sizes:
            self.request(b"D" + struct.pack("<H", p))

    def burn(self, timeout: float = 30.0, poll: float = 0.1) -> float:
        """Store every page in EEPROM and wait until the ECU has finished. Returns seconds taken."""
        t0 = time.monotonic()
        for p in self.d.page_sizes:
            self.request(b"B" + struct.pack("<H", p))
        # The firmware refreshes its "burning" flag every 10 ms: give it time to show this request,
        # so a stale "not burning" is never taken for "finished".
        time.sleep(self.BURN_FLAG_DELAY)
        while True:
            if not int(self.realtime()["engine"]) & ENGINE_BURNING:
                return time.monotonic() - t0
            if time.monotonic() - t0 > timeout:
                raise LinkError("the burn did not finish")
            time.sleep(poll)


def open_serial(port: str, baud: int = 115200, boot_wait: float = 2.0):
    """Open a serial port. An Arduino resets when its port opens, so wait for it to boot."""
    import serial  # pyserial
    try:
        ser = serial.Serial(port, baud, timeout=0.5, write_timeout=1.0)
    except (serial.SerialException, OSError) as e:
        raise LinkError(f"cannot open {port}: {e}") from e
    time.sleep(boot_wait)
    ser.reset_input_buffer()
    return ser


def list_ports():
    """[(device, description)] of the serial ports on this computer."""
    try:
        from serial.tools import list_ports as lp
        return [(p.device, p.description) for p in sorted(lp.comports())]
    except Exception:
        return []
