"""Live data: status lights, gauges and a numeric list (tuner milestone T1)."""
from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (QFrame, QGridLayout, QHBoxLayout, QLabel, QScrollArea, QSizePolicy, QVBoxLayout,
                               QWidget)

SYNC_NAMES = {0: "No sync", 1: "Sync pending", 2: "Crank sync (360°)", 3: "Full sync (720°)"}
LOSS_NAMES = {0: "-", 1: "tooth count", 2: "no gap", 3: "engine stopped"}
ENGINE_BITS = ["Running", "Cranking", "REV LIMIT", "Accel", "After-start", "TEST MODE", "Burning", "Fuel pump"]
FAULT_BITS = ["TPS 1", "TPS 2", "TPS disagree", "Coolant sensor", "Air temp sensor", "MAF", "Battery",
              "DEFAULT TUNE"]
COUNTERS = [("sync_losses", "Sync losses"), ("noise_teeth", "Noise teeth"), ("cam_errors", "Cam errors"),
            ("sched_late", "Late events"), ("sched_overlap", "Overlaps"), ("frame_errors", "Frame errors"),
            ("uart_overflows", "UART overflows")]

GOOD, WARN, BAD, IDLE = "#2e9e5b", "#d9a400", "#d64545", "#7d8590"


class Gauge(QWidget):
    """A 270° arc gauge with warning and danger zones from the definition."""
    START, SPAN = 225.0, 270.0              # degrees, clockwise from the lower left

    def __init__(self, channel, parent=None):
        super().__init__(parent)
        self.ch = channel
        self.lo, self.hi, self.lo_danger, self.lo_warn, self.hi_warn, self.hi_danger = channel.gauge
        self.value = None
        self.setMinimumSize(120, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def sizeHint(self):
        return QSize(150, 135)

    def set_value(self, v):
        if v != self.value:
            self.value = v
            self.update()

    def zone(self, v):
        if v is None:
            return IDLE
        if v <= self.lo_danger or v >= self.hi_danger:
            return BAD
        if v <= self.lo_warn or v >= self.hi_warn:
            return WARN
        return GOOD

    def _frac(self, v):
        return min(1.0, max(0.0, (v - self.lo) / (self.hi - self.lo)))

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        caption = 15                            # room under the dial for the name
        side = max(40.0, min(w, h - caption))
        thick = max(6.0, side * 0.075)
        pad = thick * 1.15                      # the zone marks sit outside the track
        r = QRectF((w - side) / 2 + pad, (h - caption - side) / 2 + pad, side - 2 * pad, side - 2 * pad)
        text = self.palette().windowText().color()
        track = QColor(text)
        track.setAlpha(40)

        def arc(f0, f1, color, width):
            pen = QPen(QColor(color), width)
            pen.setCapStyle(Qt.FlatCap)
            p.setPen(pen)
            p.drawArc(r, int((self.START - f0 * self.SPAN) * 16), int(-(f1 - f0) * self.SPAN * 16))

        arc(0, 1, track, thick)
        # zone marks just outside the track
        outer = r.adjusted(-thick * 0.8, -thick * 0.8, thick * 0.8, thick * 0.8)
        r_keep, r = r, outer
        for a, b, col in ((self.lo, self.lo_danger, BAD), (self.lo_danger, self.lo_warn, WARN),
                          (self.hi_warn, self.hi_danger, WARN), (self.hi_danger, self.hi, BAD)):
            if b > a and b > self.lo and a < self.hi:
                arc(self._frac(a), self._frac(b), col, max(2.0, thick * 0.25))
        r = r_keep
        if self.value is not None:
            arc(0, self._frac(self.value), self.zone(self.value), thick)

        p.setPen(text)
        f = QFont(self.font())
        f.setPixelSize(max(12, int(side * 0.19)))
        f.setBold(True)
        p.setFont(f)
        p.drawText(r, Qt.AlignCenter, "–" if self.value is None else self.ch.format(self.value))
        f.setPixelSize(max(9, int(side * 0.085)))
        f.setBold(False)
        p.setFont(f)
        units = self.ch.units if self.ch.units.lower() != self.ch.name.lower() else ""
        label = self.ch.label + (f" ({units})" if units else "")
        # the dial is open at the bottom: the name sits in that opening
        p.drawText(QRectF(0, r.bottom() - r.height() * 0.12, w, h), Qt.AlignHCenter | Qt.AlignTop, label)


class Light(QLabel):
    """A status pill: grey when off, coloured when on."""

    def __init__(self, text, color=GOOD, parent=None):
        super().__init__(text, parent)
        self.color = color
        self.setAlignment(Qt.AlignCenter)
        self.set_on(False)

    def set_on(self, on, text=None, color=None):
        if text is not None:
            self.setText(text)
        c = (color or self.color) if on else None
        self.setStyleSheet(
            "border-radius: 9px; padding: 2px 9px; font-weight: 600;"
            + (f"background: {c}; color: white;" if c else f"border: 1px solid {IDLE}; color: {IDLE};"))


class Dashboard(QWidget):
    def __init__(self, definition, parent=None):
        super().__init__(parent)
        self.d = definition
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        # status lights
        self.sync = Light("No sync")
        self.lights = {name: Light(name, color) for name, color in (
            ("Running", GOOD), ("Cranking", WARN), ("REV LIMIT", BAD), ("Ignition", BAD), ("Injection", BAD),
            ("TEST MODE", WARN), ("Burning", WARN), ("Fuel pump", GOOD))}
        row = QGridLayout()
        row.setSpacing(5)
        row.addWidget(self.sync, 0, 0, 1, 3)
        for i, w in enumerate(self.lights.values()):
            row.addWidget(w, 1 + i // 3, i % 3)
        root.addLayout(row)
        self.faults = QLabel()
        self.faults.setWordWrap(True)
        root.addWidget(self.faults)

        # gauges
        self.gauges = {}
        grid = QGridLayout()
        grid.setSpacing(2)
        for i, c in enumerate(c for c in definition.channels.values() if c.gauge):
            g = Gauge(c)
            self.gauges[c.name] = g
            grid.addWidget(g, i // 3, i % 3)
        root.addLayout(grid, 1)

        # numbers
        self.numbers = {}
        box = QWidget()
        ngrid = QGridLayout(box)
        ngrid.setContentsMargins(0, 0, 0, 0)
        ngrid.setHorizontalSpacing(10)
        ngrid.setVerticalSpacing(1)
        lights = {"sync", "engine", "faults", "out_enable"} | {n for n, _ in COUNTERS}    # shown elsewhere
        shown = [(c.name, c.label, c.units) for c in definition.channels.values()
                 if c.log and not c.gauge and c.name not in lights]
        shown += [(n, t, "") for n, t in COUNTERS if n in definition.channels]
        shown.append(("loss_reason", "Last sync loss", ""))
        for i, (name, text, units) in enumerate(shown):
            r, col = i // 2, (i % 2) * 2
            lab = QLabel(text + (f" ({units})" if units else ""))
            lab.setStyleSheet(f"color: {IDLE};")
            val = QLabel("–")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setStyleSheet("font-family: monospace;")
            ngrid.addWidget(lab, r, col)
            ngrid.addWidget(val, r, col + 1)
            self.numbers[name] = val
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(box)
        scroll.setMinimumHeight(90)
        root.addWidget(scroll, 1)
        self.clear()

    def clear(self):
        """No live data (offline)."""
        for g in self.gauges.values():
            g.set_value(None)
        for v in self.numbers.values():
            v.setText("–")
        self.sync.set_on(False, "Not connected")
        for w in self.lights.values():
            w.set_on(False)
        self.faults.setText("")

    def update_values(self, rt):
        for name, g in self.gauges.items():
            g.set_value(rt[name])
        for name, lab in self.numbers.items():
            if name == "loss_reason":
                lab.setText(LOSS_NAMES.get(int(rt[name]), "?"))
            else:
                lab.setText(self.d.channels[name].format(rt[name]))
        sync = int(rt["sync"])
        self.sync.set_on(sync >= 2, SYNC_NAMES.get(sync, "?"), GOOD if sync == 3 else WARN)
        eng, out = int(rt["engine"]), int(rt["out_enable"])
        for i, name in enumerate(ENGINE_BITS):
            if name in self.lights:
                self.lights[name].set_on(bool(eng & (1 << i)))
        self.lights["Ignition"].set_on(bool(out & 1), "Ignition ON" if out & 1 else "Ignition off")
        self.lights["Injection"].set_on(bool(out & 2), "Injection ON" if out & 2 else "Injection off")
        faults = [n for i, n in enumerate(FAULT_BITS) if int(rt["faults"]) & (1 << i)]
        self.faults.setText(f"<span style='color:{BAD}; font-weight:600'>Faults: " + ", ".join(faults) + "</span>"
                            if faults else f"<span style='color:{IDLE}'>No faults</span>")
