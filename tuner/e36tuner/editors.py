"""
Tune editors (tuner milestone T2): 2D table editor with heat-map colours and
block operations, curve editor with a plot, and settings forms. All of them
edit the shared Tune, which gives undo/redo and the live write to the ECU.
"""
from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QGuiApplication, QKeySequence, QPen
from PySide6.QtWidgets import (QAbstractItemView, QComboBox, QDoubleSpinBox, QFormLayout, QHBoxLayout,
                               QHeaderView, QInputDialog, QLabel, QLineEdit, QPushButton, QScrollArea,
                               QSplitter, QStyledItemDelegate, QTableView, QVBoxLayout, QWidget)

from . import ops

# blue -> cyan -> green -> yellow -> red, pale enough for black text
HEAT = [(0.0, (120, 160, 255)), (0.25, (120, 220, 235)), (0.5, (140, 225, 140)), (0.75, (245, 230, 120)),
        (1.0, (245, 130, 115))]
AXIS_BG = QColor(125, 133, 144, 70)


def heat_color(f):
    f = min(1.0, max(0.0, f))
    for (f0, c0), (f1, c1) in zip(HEAT, HEAT[1:]):
        if f <= f1:
            k = (f - f0) / (f1 - f0)
            return QColor(*(round(a + (b - a) * k) for a, b in zip(c0, c1)))
    return QColor(*HEAT[-1][1])


def parse_number(text):
    text = text.strip()
    if "," in text and "." not in text:
        text = text.replace(",", ".")
    return float(text)                          # ValueError if it is not a number


class GridModel(QAbstractTableModel):
    """
    A grid of tune values. `cells` maps (row, col) -> (field name, element index);
    `heat` names the field whose cells get heat-map colours.
    """
    message = Signal(str)

    def __init__(self, tune, rows, cols, cells, heat=None, headers=None, parent=None):
        super().__init__(parent)
        self.tune, self.rows, self.cols, self.cells = tune, rows, cols, cells
        self.heat, self.headers = heat, headers
        self.fields = {name for name, _ in cells.values()}
        self.live = None                        # (row, col) the engine is operating in
        self._range = None
        tune.listeners.append(self._tune_changed)

    def detach(self):
        if self._tune_changed in self.tune.listeners:
            self.tune.listeners.remove(self._tune_changed)

    def _tune_changed(self, changes):
        d = self.tune.d
        for page, off, data in changes:
            for name in self.fields:
                f = d.fields[name]
                if f.page == page and off < f.offset + f.size and f.offset < off + len(data):
                    self._range = None
                    self.dataChanged.emit(self.index(0, 0), self.index(self.rows - 1, self.cols - 1))
                    return

    # ---- Qt model
    def rowCount(self, parent=QModelIndex()):
        return 0 if parent.isValid() else self.rows

    def columnCount(self, parent=QModelIndex()):
        return 0 if parent.isValid() else self.cols

    def cell(self, index):
        return self.cells.get((index.row(), index.column()))

    def value(self, rc):
        name, i = self.cells[rc]
        return self.tune.shown(name, i)

    def heat_range(self):
        if self._range is None:
            vals = [self.tune.shown(n, i) for n, i in self.cells.values() if n == self.heat]
            self._range = (min(vals), max(vals)) if vals else (0, 0)
        return self._range

    def data(self, index, role=Qt.DisplayRole):
        c = self.cell(index)
        if c is None:
            return None
        name, i = c
        f = self.tune.d.fields[name]
        if role in (Qt.DisplayRole, Qt.EditRole):
            return f.format(self.tune.shown(name, i))
        if role == Qt.TextAlignmentRole:
            return int(Qt.AlignCenter)
        if role == Qt.BackgroundRole:
            if name != self.heat:
                return QBrush(AXIS_BG)
            lo, hi = self.heat_range()
            return QBrush(heat_color((self.tune.shown(name, i) - lo) / (hi - lo) if hi > lo else 0.5))
        if role == Qt.ForegroundRole and name == self.heat:
            return QBrush(QColor(20, 20, 20))
        if role == Qt.FontRole and name != self.heat and self.heat:
            font = QFont()
            font.setBold(True)
            return font
        if role == Qt.ToolTipRole:
            return f"{f.label}: {f.format(f.min)} .. {f.format(f.max)} {f.units}".rstrip()
        return None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and self.headers and orientation == Qt.Horizontal:
            return self.headers[section]
        return super().headerData(section, orientation, role)

    def flags(self, index):
        if self.cell(index) is None:
            return Qt.NoItemFlags
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsEditable

    def setData(self, index, text, role=Qt.EditRole):
        if role != Qt.EditRole or self.cell(index) is None:
            return False
        return self.apply({(index.row(), index.column()): text}, "edit")

    # ---- edits
    def apply(self, values, what):
        """values: {(row, col): shown value or text}. One undo step. Out-of-range values are limited."""
        by_field, limited = {}, False
        for rc, v in values.items():
            if rc not in self.cells:
                continue
            name, i = self.cells[rc]
            f = self.tune.d.fields[name]
            try:
                v = parse_number(v) if isinstance(v, str) else float(v)
            except ValueError:
                self.message.emit(f"'{v}' is not a number")
                return False
            limited |= not f.in_range(v)
            by_field.setdefault(name, {})[i] = f.to_raw(v)
        if not by_field:
            return False
        names = ", ".join(self.tune.d.fields[n].label for n in by_field)
        with self.tune.group(f"{what} {names}"):
            for name, raws in by_field.items():
                self.tune.set_raw(name, raws)
        if limited:
            f = self.tune.d.fields[next(iter(by_field))]
            self.message.emit(f"Limited to the allowed range {f.format(f.min)} .. {f.format(f.max)} {f.units}")
        return True

    def step(self, cells, n):
        """Move cells by n of the smallest storable step (exact, no rounding drift)."""
        by_field = {}
        for rc in cells:
            name, i = self.cells[rc]
            by_field.setdefault(name, {})[i] = self.tune.raw(name, i) + n
        with self.tune.group("adjust " + ", ".join(self.tune.d.fields[n_].label for n_ in by_field)):
            for name, raws in by_field.items():
                self.tune.set_raw(name, raws)

    def set_live(self, rc):
        if rc != self.live:
            old, self.live = self.live, rc
            for c in (old, rc):
                if c:
                    self.dataChanged.emit(self.index(*c), self.index(*c))


class _Delegate(QStyledItemDelegate):
    """Typing a value with several cells selected sets all of them; draws the live-cell marker."""

    def __init__(self, view):
        super().__init__(view)
        self.view = view

    def createEditor(self, parent, option, index):
        e = QLineEdit(parent)
        e.setAlignment(Qt.AlignCenter)
        return e

    def setModelData(self, editor, model, index):
        cells = self.view.selected_cells(index)
        model.apply(dict.fromkeys(cells, editor.text()), "set")

    def paint(self, painter, option, index):
        super().paint(painter, option, index)
        if index.model().live == (index.row(), index.column()):
            painter.save()
            painter.setPen(QPen(QColor(20, 20, 20), 2))
            painter.drawRect(option.rect.adjusted(1, 1, -2, -2))
            painter.setPen(QPen(QColor(255, 255, 255), 1, Qt.DotLine))
            painter.drawRect(option.rect.adjusted(1, 1, -2, -2))
            painter.restore()


class GridView(QTableView):
    """
    Keys:  + / -  one step      Ctrl with + / -  ten steps      * scale by %
           I interpolate        S smooth        Ctrl+C / Ctrl+V copy / paste
           type a number: sets every selected cell (Enter opens a cell, e.g. for a negative number)
    Block operations work on the selected cells of the same kind as the current one.
    """

    def __init__(self, model, rows_axis=None, cols_axis=None, parent=None):
        super().__init__(parent)
        self.setModel(model)
        self.rows_axis, self.cols_axis = rows_axis, cols_axis   # callables -> axis values for interpolation
        self.setItemDelegate(_Delegate(self))
        self.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.AnyKeyPressed
                             | QAbstractItemView.EditKeyPressed)
        self.verticalHeader().hide()
        self.horizontalHeader().setVisible(bool(model.headers))
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.verticalHeader().setMinimumSectionSize(20)
        self.horizontalHeader().setMinimumSectionSize(34)

    def selected_cells(self, current=None):
        """Selected (row, col) of the same field as the current cell."""
        m = self.model()
        current = current if current is not None and current.isValid() else self.currentIndex()
        cur = m.cell(current) if current.isValid() else None
        if cur is None:
            return []
        cells = [(i.row(), i.column()) for i in self.selectedIndexes() if (m.cell(i) or (None,))[0] == cur[0]]
        return cells or [(current.row(), current.column())]

    # ---- operations (also on the editor's buttons)
    def step(self, n):
        cells = self.selected_cells()
        if cells:
            self.model().step(cells, n)

    def set_value(self):
        cells = self.selected_cells()
        if not cells:
            return
        f = self.model().tune.d.fields[self.model().cells[cells[0]][0]]
        v, ok = QInputDialog.getDouble(self, "Set value", f"{f.label} ({f.units})" if f.units else f.label,
                                       self.model().value(cells[0]), f.min, f.max, max(f.digits, 1))
        if ok:
            self.model().apply(dict.fromkeys(cells, v), "set")

    def scale(self):
        cells = self.selected_cells()
        if not cells:
            return
        pct, ok = QInputDialog.getDouble(self, "Scale", "Multiply the selected cells by (%):", 100.0, 0.0, 1000.0, 1)
        if ok:
            self.scale_by(pct)

    def scale_by(self, pct):
        m = self.model()
        m.apply({rc: m.value(rc) * pct / 100.0 for rc in self.selected_cells()}, f"scale {pct:g}%")

    def interpolate(self):
        m = self.model()
        cells = self.selected_cells()
        try:
            new = ops.interpolate({rc: m.value(rc) for rc in cells},
                                  self.rows_axis() if self.rows_axis else None,
                                  self.cols_axis() if self.cols_axis else None)
        except ValueError as e:
            m.message.emit(str(e))
            return
        if len(new) < 3:
            m.message.emit("Select at least three cells in a row, column or block to interpolate")
            return
        m.apply(new, "interpolate")

    def smooth(self):
        m = self.model()
        cells = self.selected_cells()
        if not cells:
            return
        name = m.cells[cells[0]][0]
        grid = {rc: m.value(rc) for rc, (n, _) in m.cells.items() if n == name}
        m.apply(ops.smooth(grid, cells), "smooth")

    def copy(self):
        idx = [i for i in self.selectedIndexes() if self.model().cell(i)]
        if not idx:
            return
        r0, r1 = min(i.row() for i in idx), max(i.row() for i in idx)
        c0, c1 = min(i.column() for i in idx), max(i.column() for i in idx)
        m = self.model()
        rows = [[m.value((r, c)) for c in range(c0, c1 + 1) if (r, c) in m.cells] for r in range(r0, r1 + 1)]
        QGuiApplication.clipboard().setText(ops.to_clipboard([r for r in rows if r]))

    def paste(self):
        m = self.model()
        try:
            rows = ops.from_clipboard(QGuiApplication.clipboard().text())
        except ValueError:
            m.message.emit("The clipboard does not hold numbers")
            return
        cells = self.selected_cells()
        if not cells:
            return
        if len(rows) == 1 and len(rows[0]) == 1:            # one number: fill the selection
            m.apply(dict.fromkeys(cells, rows[0][0]), "paste")
            return
        name = m.cells[cells[0]][0]
        r0, c0 = min(r for r, _ in cells), min(c for _, c in cells)
        new = {(r0 + r, c0 + c): v for r, row in enumerate(rows) for c, v in enumerate(row)}
        new = {rc: v for rc, v in new.items() if m.cells.get(rc, (None,))[0] == name}
        if m.apply(new, "paste"):
            m.message.emit(f"Pasted {len(new)} cells")

    def keyPressEvent(self, e):
        if self.state() != QAbstractItemView.EditingState:
            ctrl = bool(e.modifiers() & Qt.ControlModifier)
            key, text = e.key(), e.text()
            if e.matches(QKeySequence.Copy):
                return self.copy()
            if e.matches(QKeySequence.Paste):
                return self.paste()
            if key in (Qt.Key_Plus, Qt.Key_Equal, Qt.Key_Greater):
                return self.step(10 if ctrl else 1)
            if key in (Qt.Key_Minus, Qt.Key_Underscore, Qt.Key_Less):
                # "-" adjusts; a negative number is entered with Set… or by opening the cell (Enter)
                return self.step(-10 if ctrl else -1)
            if not ctrl and key == Qt.Key_Asterisk:
                return self.scale()
            if not ctrl and key == Qt.Key_I:
                return self.interpolate()
            if not ctrl and key == Qt.Key_S:
                return self.smooth()
            if key in (Qt.Key_Return, Qt.Key_Enter) and self.currentIndex().isValid():
                return self.edit(self.currentIndex())
        super().keyPressEvent(e)


def _button(text, tip, slot):
    b = QPushButton(text)
    b.setToolTip(tip)
    b.setFocusPolicy(Qt.NoFocus)                # keep the keyboard in the grid
    b.clicked.connect(slot)
    return b


def _toolbar(view, block_ops=True):
    bar = QHBoxLayout()
    bar.addWidget(_button("−", "Decrease the selected cells by one step  (key: -, Ctrl for ten steps)",
                          lambda: view.step(-1)))
    bar.addWidget(_button("+", "Increase the selected cells by one step  (key: +, Ctrl for ten steps)",
                          lambda: view.step(1)))
    bar.addWidget(_button("Set…", "Set the selected cells to a value  (or just type a number)", view.set_value))
    bar.addWidget(_button("Scale %…", "Multiply the selected cells by a percentage  (key: *)", view.scale))
    bar.addWidget(_button("Interpolate", "Fill between the ends / corners of the selection  (key: I)",
                          view.interpolate))
    if block_ops:
        bar.addWidget(_button("Smooth", "Blend the selected cells with their neighbours  (key: S)", view.smooth))
    bar.addWidget(_button("Copy", "Copy the selection (pastes into a spreadsheet)  (Ctrl+C)", view.copy))
    bar.addWidget(_button("Paste", "Paste from a spreadsheet, starting at the selection  (Ctrl+V)", view.paste))
    bar.addStretch(1)
    return bar


class TableEditor(QWidget):
    """
    A 2D table. The top row is the highest load, as on a map. The left column
    and bottom row are the axis bins (editable); the outlined cell is where the
    engine is running right now.
    """
    message = Signal(str)

    def __init__(self, tune, table, parent=None):
        super().__init__(parent)
        d = tune.d
        self.tune, self.table = tune, table
        fx, fy, fz = d.fields[table["x"]], d.fields[table["y"]], d.fields[table["z"]]
        self.fx, self.fy, self.fz = fx, fy, fz
        nx, ny = fz.shape
        cells = {}
        for r in range(ny):
            y = ny - 1 - r                              # highest load at the top
            cells[(r, 0)] = (fy.name, y)
            for x in range(nx):
                cells[(r, x + 1)] = (fz.name, y * nx + x)
        for x in range(nx):
            cells[(ny, x + 1)] = (fx.name, x)
        self.model = GridModel(tune, ny + 1, nx + 1, cells, heat=fz.name, parent=self)
        self.model.message.connect(self.message)
        self.view = GridView(self.model,
                             rows_axis=lambda: [tune.shown(fy.name, ny - 1 - r) for r in range(ny)] + [0],
                             cols_axis=lambda: [0] + [tune.shown(fx.name, x) for x in range(nx)])
        self.title = QLabel(f"<b>{table['title']}</b> ({fz.units})  —  across: {fx.label} ({fx.units}),  "
                            f"up: {fy.label} ({fy.units})")
        self.live = QLabel("")
        self.live.setStyleSheet("color: #7d8590;")
        lay = QVBoxLayout(self)
        lay.addWidget(self.title)
        lay.addLayout(_toolbar(self.view))
        lay.addWidget(self.view, 1)
        lay.addWidget(self.live)
        if fz.help:
            lay.addWidget(_help(fz.help))

    def update_live(self, rt):
        """Mark the cell nearest to the engine's operating point."""
        if rt is None:
            self.model.set_live(None)
            self.live.setText("")
            return
        x, y = rt[self.table["xchan"]], rt[self.table["ychan"]]
        nx, ny = self.fz.shape
        xs = [self.tune.shown(self.fx.name, i) for i in range(nx)]
        ys = [self.tune.shown(self.fy.name, i) for i in range(ny)]
        xi = min(range(nx), key=lambda i: abs(xs[i] - x))
        yi = min(range(ny), key=lambda i: abs(ys[i] - y))
        self.model.set_live((ny - 1 - yi, xi + 1))
        d = self.tune.d
        cx, cy = d.channels[self.table["xchan"]], d.channels[self.table["ychan"]]
        self.live.setText(f"Engine now: {cx.format(x)} {cx.units}, {cy.format(y)} {cy.units}  →  cell value "
                          f"{self.fz.format(self.tune.shown(self.fz.name, yi * nx + xi))} {self.fz.units}")


class CurveEditor(QWidget):
    """A curve: two columns of numbers and a plot of them."""
    message = Signal(str)

    def __init__(self, tune, curve, parent=None):
        super().__init__(parent)
        d = tune.d
        self.tune = tune
        self.fx, self.fy = d.fields[curve["x"]], d.fields[curve["y"]]
        n = self.fx.count
        cells = {}
        for i in range(n):
            cells[(i, 0)] = (self.fx.name, i)
            cells[(i, 1)] = (self.fy.name, i)
        head = [f"{f.label} ({f.units})" if f.units else f.label for f in (self.fx, self.fy)]
        self.model = GridModel(tune, n, 2, cells, headers=head, parent=self)
        self.model.message.connect(self.message)
        self.view = GridView(self.model)
        self.view.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.addLayout(_toolbar(self.view, block_ops=False))
        ll.addWidget(self.view, 1)

        import pyqtgraph as pg
        self.plot = pg.PlotWidget()
        self.plot.setBackground(None)
        self.plot.setMenuEnabled(False)
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", head[0])
        self.plot.setLabel("left", head[1])
        self.line = self.plot.plot([], [], pen=pg.mkPen("#2f7fd6", width=2), symbol="o", symbolSize=7,
                                   symbolBrush="#2f7fd6", symbolPen=None)
        split = QSplitter(Qt.Horizontal)
        split.addWidget(left)
        split.addWidget(self.plot)
        left.setMaximumWidth(460)
        self.plot.setMinimumWidth(260)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([430, 640])
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel(f"<b>{curve['title']}</b>"))
        lay.addWidget(split, 1)
        for f in (self.fx, self.fy):
            if f.help:
                lay.addWidget(_help(f.help))
        self.model.dataChanged.connect(lambda *_: self.replot())
        self.replot()

    def replot(self):
        self.line.setData([self.tune.shown(self.fx.name, i) for i in range(self.fx.count)],
                          [self.tune.shown(self.fy.name, i) for i in range(self.fy.count)])

    def update_live(self, rt):
        pass


def _help(text):
    lab = QLabel(text)
    lab.setWordWrap(True)
    lab.setStyleSheet("color: #7d8590;")
    return lab


class SettingsForm(QWidget):
    """Scalars, option lists and short arrays. `confirm(field, raw)` may veto a change."""
    message = Signal(str)

    def __init__(self, tune, dialog, confirm=None, parent=None):
        super().__init__(parent)
        self.tune, self.confirm = tune, confirm
        self.widgets = {}                       # (field name, index) -> editor
        inner = QWidget()
        form = QFormLayout(inner)
        form.setLabelAlignment(Qt.AlignRight)
        for name in dialog["fields"]:
            f = tune.d.fields[name]
            text = f.label + (f" ({f.units})" if f.units else "")
            if f.bits:
                w = QComboBox()
                w.addItems([o for o in f.options if o != "INVALID"])
                w.activated.connect(lambda i, f=f: self._changed(f, 0, i))
                self.widgets[(name, 0)] = w
                row = w
            else:
                row = QWidget()
                rl = QHBoxLayout(row)
                rl.setContentsMargins(0, 0, 0, 0)
                for i in range(f.count):
                    w = QDoubleSpinBox()
                    w.setDecimals(f.digits)
                    w.setRange(f.min, f.max)
                    w.setSingleStep(f.step)
                    w.setKeyboardTracking(False)        # commit on Enter / leaving the box, not per key
                    w.setMinimumWidth(80)
                    w.valueChanged.connect(lambda v, f=f, i=i: self._changed(f, i, f.to_raw(v)))
                    self.widgets[(name, i)] = w
                    rl.addWidget(w)
                rl.addStretch(1)
            row.setToolTip(f.help)
            form.addRow(text, row)
            if f.help:
                form.addRow("", _help(f.help))
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(inner)
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel(f"<b>{dialog['title']}</b>"))
        lay.addWidget(scroll, 1)
        self.fields = {n for n, _ in self.widgets}
        tune.listeners.append(self._tune_changed)
        self.refresh()

    def detach(self):
        if self._tune_changed in self.tune.listeners:
            self.tune.listeners.remove(self._tune_changed)

    def _tune_changed(self, _changes):
        self.refresh()

    def refresh(self):
        for (name, i), w in self.widgets.items():
            w.blockSignals(True)
            if isinstance(w, QComboBox):
                w.setCurrentIndex(min(self.tune.raw(name), w.count() - 1))
            else:
                w.setValue(self.tune.shown(name, i))
            w.blockSignals(False)

    def _changed(self, f, index, raw):
        if raw == self.tune.raw(f.name, index):
            return
        if self.confirm and not self.confirm(f, raw):
            self.refresh()
            return
        self.tune.set_raw(f.name, {index: raw})
        self.refresh()                          # show what was actually stored

    def update_live(self, rt):
        pass
