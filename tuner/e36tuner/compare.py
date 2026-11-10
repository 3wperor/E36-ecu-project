"""Compare the tune being edited with another one (tuner milestone T3)."""
from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QHBoxLayout, QLabel, QPushButton, QSplitter,
                               QTableWidget, QTableWidgetItem, QVBoxLayout)

from .tune import diff


def _summary(entry):
    f = entry["field"]
    if f.bits:
        return entry["a"][0], entry["b"][0]
    if f.count == 1:
        return f.format(entry["a"][0]), f.format(entry["b"][0])
    deltas = [entry["b"][i] - entry["a"][i] for i in entry["indices"]]
    big = max(deltas, key=abs)
    return f"{len(deltas)} of {f.count} values differ", f"largest change {big:+.{f.digits}f}"


class CompareDialog(QDialog):
    """Lists every setting that differs; arrays show a cell-by-cell difference grid."""

    def __init__(self, tune, other_pages, other_name, parent=None):
        super().__init__(parent)
        self.tune, self.other = tune, other_pages
        self.setWindowTitle(f"Compare with {other_name}")
        self.resize(980, 620)
        self.entries = diff(tune.d, tune.snapshot(), other_pages)

        self.list = QTableWidget(len(self.entries), 4)
        self.list.setHorizontalHeaderLabels(["Setting", "Units", "This tune", other_name])
        self.list.setSelectionBehavior(QTableWidget.SelectRows)
        self.list.setEditTriggers(QTableWidget.NoEditTriggers)
        self.list.verticalHeader().hide()
        for r, e in enumerate(self.entries):
            a, b = _summary(e)
            for c, text in enumerate((e["field"].label, e["field"].units, str(a), str(b))):
                self.list.setItem(r, c, QTableWidgetItem(text))
        self.list.resizeColumnsToContents()
        self.list.horizontalHeader().setStretchLastSection(True)
        self.list.itemSelectionChanged.connect(self._show)

        self.detail_title = QLabel("")
        self.detail = QTableWidget(0, 0)
        self.detail.setEditTriggers(QTableWidget.NoEditTriggers)
        self.detail.horizontalHeader().hide()
        self.detail.verticalHeader().hide()

        self.take = QPushButton("Copy the selected settings into this tune")
        self.take.setToolTip("Takes the other tune's values for the selected rows. Can be undone.")
        self.take.clicked.connect(self._take)
        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(self.reject)

        split = QSplitter(Qt.Vertical)
        split.addWidget(self.list)
        split.addWidget(self.detail)
        lay = QVBoxLayout(self)
        n = len(self.entries)
        lay.addWidget(QLabel("The tunes are identical." if not n else
                             f"{n} setting{'s' if n != 1 else ''} differ. Select one to see the details."))
        lay.addWidget(split, 1)
        lay.addWidget(self.detail_title)
        row = QHBoxLayout()
        row.addWidget(self.take)
        row.addStretch(1)
        row.addWidget(buttons)
        lay.addLayout(row)
        self.take.setEnabled(False)
        if n:
            self.list.selectRow(0)

    def _rows(self):
        return sorted({i.row() for i in self.list.selectedIndexes()})

    def _show(self):
        rows = self._rows()
        self.take.setEnabled(bool(rows))
        self.detail.clear()
        if not rows:
            return
        e = self.entries[rows[0]]
        f = e["field"]
        if f.bits or f.count == 1:
            self.detail.setRowCount(0)
            self.detail_title.setText("")
            return
        nx = f.shape[0] if len(f.shape) == 2 else f.count
        ny = f.shape[1] if len(f.shape) == 2 else 1
        self.detail.setRowCount(ny)
        self.detail.setColumnCount(nx)
        self.detail_title.setText(f"{f.label}: other − this ({f.units}); blank cells are equal"
                                  + (", top row = highest load" if ny > 1 else ""))
        changed = set(e["indices"])
        for y in range(ny):
            for x in range(nx):
                i = y * nx + x
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                if i in changed:
                    delta = e["b"][i] - e["a"][i]
                    item.setText(f"{delta:+.{f.digits}f}")
                    item.setToolTip(f"this {f.format(e['a'][i])}, other {f.format(e['b'][i])}")
                    item.setBackground(QBrush(QColor(245, 130, 115) if delta > 0 else QColor(120, 160, 255)))
                    item.setForeground(QBrush(QColor(20, 20, 20)))
                self.detail.setItem(ny - 1 - y, x, item)
        self.detail.resizeColumnsToContents()

    def _take(self):
        rows = self._rows()
        with self.tune.group("copy settings from the other tune"):
            for r in rows:
                f = self.entries[r]["field"]
                self.tune.set_raw(f.name, {i: f.read_raw(self.other[f.page], i) for i in range(f.count)})
        self.accept()
