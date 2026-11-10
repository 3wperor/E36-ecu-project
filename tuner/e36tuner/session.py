"""
The connection to the ECU as the GUI sees it. All serial traffic happens on
one background thread (so the window never freezes); results come back as Qt
signals on the GUI thread.
"""
import queue
import threading
import time

from PySide6.QtCore import QObject, Signal

from .fake_ecu import FakeEcu
from .link import EcuLink, LinkError, open_serial

OFFLINE, CONNECTING, ONLINE = "offline", "connecting", "online"


class EcuSession(QObject):
    stateChanged = Signal(str)              # OFFLINE / CONNECTING / ONLINE
    connected = Signal(str, str, object)    # signature, version, pages read from the ECU
    realtime = Signal(object)               # {channel: shown value}
    failed = Signal(str)                    # the link was lost or could not be opened (now offline)
    burned = Signal(float)                  # burn finished, seconds taken
    synced = Signal(bool)                   # verify(): the ECU's RAM equals the pages given
    busyChanged = Signal(int)               # commands waiting to be sent

    POLL_HZ = 20

    def __init__(self, definition, parent=None):
        super().__init__(parent)
        self.d = definition
        self.state = OFFLINE
        self.what = ""                      # port name or "demo"
        self._jobs = queue.Queue()
        self._thread = None
        self._stop = threading.Event()

    # ------------------------------------------------------- GUI thread --
    def connect_serial(self, port, boot_wait=2.0):
        self._start(lambda: open_serial(port, boot_wait=boot_wait), port)

    def connect_demo(self):
        self._start(lambda: FakeEcu(self.d), "demo ECU")

    def connect_transport(self, transport, what="test"):
        self._start(lambda: transport, what)

    def disconnect(self):
        if self._thread:
            self._stop.set()
            self._jobs.put(None)
            self._thread.join(timeout=5)
            self._thread = None
        self._set_state(OFFLINE)

    @property
    def online(self):
        return self.state == ONLINE

    def write(self, page, offset, data):
        self._post(lambda link: link.write(page, offset, bytes(data)))

    def send_pages(self, pages):
        pages = {p: bytes(b) for p, b in pages.items()}
        self._post(lambda link: link.write_pages(pages))

    def burn(self, pages=None):
        """Burn. With `pages`, first check the ECU's RAM holds exactly them; if not, synced(False), no burn."""
        pages = pages and {p: bytes(b) for p, b in pages.items()}

        def job(link):
            if pages and not link.verify(pages):
                self.synced.emit(False)
            else:
                self.burned.emit(link.burn())
        self._post(job)

    def verify(self, pages):
        pages = {p: bytes(b) for p, b in pages.items()}
        self._post(lambda link: self.synced.emit(link.verify(pages)))

    def pending(self):
        return self._jobs.qsize()

    # ----------------------------------------------------------- worker --
    def _post(self, job):
        if self.state == ONLINE:
            self._jobs.put(job)
            self.busyChanged.emit(self._jobs.qsize())

    def _set_state(self, state):
        if state != self.state:
            self.state = state
            self.stateChanged.emit(state)

    def _start(self, opener, what):
        self.disconnect()
        self.what = what
        self._jobs = queue.Queue()
        self._stop = threading.Event()
        self._set_state(CONNECTING)
        self._thread = threading.Thread(target=self._run, args=(opener, self._jobs, self._stop),
                                        name="ecu-link", daemon=True)
        self._thread.start()

    def _run(self, opener, jobs, stop):
        link = None
        try:
            link = EcuLink(opener(), self.d)
            if stop.is_set():
                return
            sig = link.check_signature()
            version = link.version()
            pages = link.read_pages()
            self.state = ONLINE                 # jobs may be queued from now on
            self.connected.emit(sig, version, pages)
            self.stateChanged.emit(ONLINE)
            period = 1.0 / self.POLL_HZ
            next_poll = time.monotonic()
            while not stop.is_set():
                try:
                    job = jobs.get(timeout=max(0.0, next_poll - time.monotonic()))
                except queue.Empty:
                    job = False
                if job is None:
                    break
                if job:
                    job(link)
                    self.busyChanged.emit(jobs.qsize())
                    continue                    # send everything waiting before the next poll
                self.realtime.emit(link.realtime())
                next_poll = max(next_poll + period, time.monotonic())
        except (LinkError, OSError, ValueError) as e:
            if not stop.is_set():
                self.state = OFFLINE
                self.failed.emit(str(e))
                self.stateChanged.emit(OFFLINE)
        finally:
            if link:
                try:
                    link.close()
                except OSError:
                    pass
