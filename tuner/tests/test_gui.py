"""
Tuner GUI tests against the built-in pretend ECU. They run without a display
(Qt's offscreen platform) and are skipped if PySide6 is not installed.
"""
import os
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtCore import QItemSelection, QItemSelectionModel, Qt
    from PySide6.QtWidgets import QApplication, QDoubleSpinBox, QLineEdit, QMessageBox
    HAVE_QT = True
except ImportError:                                     # pragma: no cover
    HAVE_QT = False

if HAVE_QT:
    from e36tuner import app as app_mod
    from e36tuner.compare import CompareDialog
    from e36tuner.definition import Definition
    from e36tuner.fake_ecu import FakeEcu
    from e36tuner.tune import Tune

    D = Definition.load()


def pump(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.002)
    return False


@unittest.skipUnless(HAVE_QT, "PySide6 not installed")
class GuiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.asked = []                                 # message boxes that would have been shown
        self.answer = QMessageBox.Yes
        self._orig = {n: getattr(QMessageBox, n) for n in ("warning", "question", "information")}
        for n in self._orig:
            setattr(QMessageBox, n, staticmethod(self._box))
        self.win = app_mod.MainWindow(D)
        self.win.settings.clear()
        self.ecu = FakeEcu(D)

    def tearDown(self):
        self.win.session.disconnect()
        self.win.tune.mark_saved()
        self.win.close()
        for n, f in self._orig.items():
            setattr(QMessageBox, n, f)

    def _box(self, _parent, title, text, *a, **k):
        self.asked.append((title, text))
        return self.answer

    def connect(self, ecu=None):
        self.win.session.connect_transport(ecu or self.ecu)
        self.assertTrue(pump(lambda: self.win.session.online), "did not connect")
        pump(lambda: False, 0.05)

    def sent(self):
        """Wait until everything queued has reached the ECU."""
        self.assertTrue(pump(lambda: self.win.session.pending() == 0))
        pump(lambda: False, 0.08)

    def ecu_matches(self):
        self.sent()
        return {p: bytes(b) for p, b in self.ecu.ram.items()} == self.win.tune.snapshot()

    def select(self, view, r0, c0, r1, c1):
        m = view.model()
        view.setCurrentIndex(m.index(r0, c0))
        view.selectionModel().select(QItemSelection(m.index(r0, c0), m.index(r1, c1)),
                                     QItemSelectionModel.ClearAndSelect)

    # ------------------------------------------------------------------
    def test_connect_loads_the_ecu_tune_and_shows_live_data(self):
        f = D.fields["revLimit"]
        self.ecu.ram[f.page][f.offset] = 58             # the ECU holds its own tune
        self.connect()
        self.assertEqual(self.win.tune.get("revLimit"), 5800)
        self.assertFalse(self.win.tune.modified or self.win.tune.can_undo() or self.win.unburned)
        self.assertEqual(self.asked, [])
        self.assertTrue(pump(lambda: self.win.dashboard.gauges["rpm"].value is not None))
        self.assertIn("Full sync", self.win.dashboard.sync.text())
        self.assertIn("DEFAULT TUNE", self.win.dashboard.faults.text())
        ed = self.win.show_editor("ignTbl")
        self.assertTrue(pump(lambda: ed.model.live is not None))
        self.assertIn("Engine now", ed.live.text())
        self.win.disconnect_ecu()
        self.assertIsNone(self.win.dashboard.gauges["rpm"].value)
        self.assertIsNone(ed.model.live)
        self.assertEqual(self.win.connect_btn.text(), "Connect")

    def test_edit_goes_to_ecu_then_burn(self):
        self.connect()
        ed = self.win.show_editor("ignTbl")
        m, f = ed.model, D.fields["ignTable"]
        self.assertEqual(m.cells[(0, 1)], ("ignTable", 15 * 16))           # top row = highest load
        self.assertTrue(m.setData(m.index(0, 1), "12.5"))
        self.assertEqual(self.win.tune.shown("ignTable", 240), 12.5)
        self.assertTrue(self.ecu_matches())
        self.assertEqual(self.ecu.ram[f.page][f.offset + 240], f.to_raw(12.5))
        self.assertTrue(self.win.unburned)
        self.assertIn("not burned", self.win.burn_label.text())
        self.assertIsNone(self.ecu.eeprom)

        self.win.burn()
        self.assertTrue(pump(lambda: not self.win.unburned))
        self.assertEqual(self.ecu.eeprom[f.page][f.offset + 240], f.to_raw(12.5))

        self.win.undo()                                                    # undo is sent too
        self.assertTrue(self.ecu_matches())
        self.assertNotEqual(self.ecu.ram[f.page][f.offset + 240], f.to_raw(12.5))
        self.assertTrue(self.win.unburned)
        self.win.redo()
        self.assertTrue(self.ecu_matches())

    def test_table_operations(self):
        ed = self.win.show_editor("veTbl")
        v, m, t = ed.view, ed.model, self.win.tune
        get = lambda r, c: m.value((r, c))
        # typing one number with a block selected sets the whole block, as one undo step
        self.select(v, 2, 3, 4, 6)
        editor = QLineEdit()
        editor.setText("40")
        v.itemDelegate().setModelData(editor, m, m.index(2, 3))
        self.assertEqual({get(r, c) for r in range(2, 5) for c in range(3, 7)}, {40.0})
        self.assertEqual(len(t.undo_stack), 1)
        self.assertTrue(self.win.a_undo.isEnabled())
        self.assertIn("Ve Table", self.win.a_undo.text())
        # steps are exact multiples of the smallest step (0.5 % for this table)
        v.step(1)
        v.step(10)
        self.assertEqual(get(3, 4), 45.5)
        v.step(-1)
        self.assertEqual(get(3, 4), 45.0)
        v.scale_by(110)
        self.assertEqual(get(3, 4), 49.5)
        # interpolate a row between its ends, using the rpm axis spacing
        self.select(v, 2, 3, 2, 6)
        m.apply({(2, 3): 10, (2, 6): 40}, "set")
        v.interpolate()
        xs = [t.shown("rpmBins", c - 1) for c in range(3, 7)]
        for c, x in zip(range(3, 7), xs):
            want = 10 + 30 * (x - xs[0]) / (xs[-1] - xs[0])
            self.assertAlmostEqual(get(2, c), want, delta=0.26)            # stored in 0.5 % steps
        # smooth pulls a spike toward its neighbours
        m.apply({(8, 8): 100}, "set")
        around = [get(7, 8), get(9, 8), get(8, 7), get(8, 9)]
        self.select(v, 8, 8, 8, 8)
        v.smooth()
        self.assertAlmostEqual(get(8, 8), (400 + sum(around)) / 8, delta=0.26)
        # copy a block, paste it elsewhere; out-of-table cells are ignored, axes untouched
        self.select(v, 2, 3, 4, 6)
        v.copy()
        text = QApplication.clipboard().text()
        self.assertEqual((len(text.splitlines()), len(text.splitlines()[0].split("\t"))), (3, 4))
        rpm = t.raws("rpmBins")
        self.select(v, 14, 14, 14, 14)
        v.paste()
        self.assertEqual([get(14, 14), get(14, 15), get(14, 16)], [get(2, 3), get(2, 4), get(2, 5)])
        self.assertEqual(get(15, 16), get(3, 5))
        self.assertEqual(t.raws("rpmBins"), rpm)
        # out-of-range input is limited and reported
        msgs = []
        m.message.connect(msgs.append)
        m.setData(m.index(0, 1), "500")
        self.assertEqual(get(0, 1), D.fields["veTable"].max)
        self.assertIn("Limited", msgs[-1])
        self.assertFalse(m.setData(m.index(0, 1), "abc"))
        # axis cells are editable too, and separate from the table values
        self.assertTrue(m.setData(m.index(16, 1), "600"))
        self.assertEqual(t.shown("rpmBins", 0), 600)
        # every step can be undone back to the defaults
        while t.can_undo():
            t.undo()
        self.assertEqual(t.snapshot(), D.defaults)

    def test_curve_and_settings_editors(self):
        t = self.win.tune
        ed = self.win.show_editor("dwellCurve")
        ed.model.setData(ed.model.index(2, 1), "3300")
        self.assertEqual(t.shown("dwell", 2), 3300)
        self.assertEqual(list(ed.line.getData()[1])[2], 3300)                # the plot follows
        form = self.win.show_editor("engineDlg")
        box = form.widgets[("revLimit", 0)]
        self.assertIsInstance(box, QDoubleSpinBox)
        self.assertEqual((box.minimum(), box.maximum(), box.singleStep()), (1000, 9000, 100))
        box.setValue(6240)                                                   # stored in 100 rpm steps
        self.assertEqual((t.get("revLimit"), box.value()), (6200, 6200))
        t.undo()
        self.assertEqual(box.value(), t.get("revLimit"))                     # forms follow undo
        combo = form.widgets[("loadSource", 0)]
        self.assertNotIn("INVALID", [combo.itemText(i) for i in range(combo.count())])
        combo.setCurrentIndex(1)
        combo.activated.emit(1)
        self.assertEqual(t.get("loadSource"), "Alpha-N")

    def test_enabling_outputs_needs_confirmation_when_connected(self):
        form = self.win.show_editor("ignDlg")
        combo = form.widgets[("ignEnable", 0)]
        combo.activated.emit(1)                                              # offline: no question
        self.assertEqual((self.win.tune.get("ignEnable"), self.asked), ("On", []))
        self.win.tune.undo()
        self.connect()
        self.answer = QMessageBox.Cancel
        combo.activated.emit(1)
        self.assertEqual(self.win.tune.get("ignEnable"), "Off")
        self.assertEqual((len(self.asked), combo.currentIndex()), (1, 0))
        self.assertIn("ignition coils", self.asked[0][1])
        self.answer = QMessageBox.Yes
        combo.activated.emit(1)
        self.assertEqual(self.win.tune.get("ignEnable"), "On")
        self.assertTrue(self.ecu_matches())

    def test_connect_with_a_different_local_tune(self):
        self.win.tune.set("revLimit", 6100)
        f = D.fields["revLimit"]
        for choice, want_local, want_ecu in (("mine", 6100, 61), ("ecu", 6500, 65), (None, 6100, 65)):
            with self.subTest(choice=choice):
                self.win.session.disconnect()
                self.win.tune.replace(D.defaults)
                self.win.tune.set("revLimit", 6100)
                self.ecu = FakeEcu(D)
                self.win._ask_which_tune = lambda pages, c=choice: c
                self.win.session.connect_transport(self.ecu)
                pump(lambda: self.win.session.online)
                pump(lambda: False, 0.2)
                self.assertEqual(self.win.tune.get("revLimit"), want_local)
                self.assertEqual(self.ecu.ram[f.page][f.offset], want_ecu)
                self.assertEqual(self.win.session.online, choice is not None)
                if choice == "mine":
                    self.assertTrue(self.win.unburned)
                if choice == "ecu":                                          # the local tune comes back with undo
                    self.win.undo()
                    self.assertEqual(self.win.tune.get("revLimit"), 6100)
                    self.assertTrue(self.ecu_matches())

    def test_wrong_firmware_and_lost_link(self):
        self.win.session.connect_transport(FakeEcu(D, signature="E36ECU-dev-p3"))
        self.assertTrue(pump(lambda: self.asked))
        self.assertIn("E36ECU-dev-p3", self.asked[0][1])
        self.assertFalse(self.win.session.online)
        self.asked.clear()
        self.connect()
        self.win.tune.set("revLimit", 6000)
        self.sent()
        self.ecu.dead = True                                                 # cable pulled
        self.assertTrue(pump(lambda: self.asked))
        self.assertIn("offline", self.asked[0][1])
        self.assertEqual((self.win.session.state, self.win.connect_btn.text()), ("offline", "Connect"))
        self.assertEqual(self.win.tune.get("revLimit"), 6000)                # the tune is kept
        self.assertFalse(self.win.a_burn.isEnabled())

    def test_burn_refused_when_the_ecu_holds_something_else(self):
        self.connect()
        f = D.fields["revLimit"]
        self.ecu.ram[f.page][f.offset] = 40                                  # e.g. the ECU reset meanwhile
        self.answer = QMessageBox.Yes                                        # "send this tune now?"
        self.win.burn()
        self.assertTrue(pump(lambda: self.asked))
        self.assertIn("nothing was burned", self.asked[0][1])
        self.assertIsNone(self.ecu.eeprom)
        self.assertTrue(self.ecu_matches())
        self.win.burn()
        self.assertTrue(pump(lambda: self.ecu.eeprom is not None))

    def test_files_and_compare(self):
        w, t = self.win, self.win.tune
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "base.tune")
            t.set("revLimit", 6000)
            self.assertIn("*", w.windowTitle())
            w.path = path
            self.assertTrue(w.save_tune())
            self.assertEqual(w.windowTitle().split(" ")[0], "base.tune")
            t.set("revLimit", 7000)
            t.set_raw("afrTable", {0: 120, 255: 121})
            # compare with the saved file, then take one setting back from it
            other = Tune(D, app_mod.read_tune_file(D, path)[0])
            dlg = CompareDialog(t, other.snapshot(), "base.tune")
            self.assertEqual([e["field"].name for e in dlg.entries], ["revLimit", "afrTable"])
            self.assertEqual((dlg.list.item(0, 2).text(), dlg.list.item(0, 3).text()), ("7000", "6000"))
            dlg.list.selectRow(1)
            self.assertEqual((dlg.detail.rowCount(), dlg.detail.columnCount()), (16, 16))
            self.assertTrue(dlg.detail.item(15, 0).text().startswith(("+", "-")))     # index 0: bottom left
            self.assertEqual(dlg.detail.item(5, 5).text(), "")
            dlg.list.selectRow(0)
            dlg._take()
            self.assertEqual((t.get("revLimit"), t.raw("afrTable", 0)), (6000, 120))
            # opening a file asks about unsaved changes (answer: discard) and loads it exactly
            self.answer = QMessageBox.Discard
            self.assertTrue(w.open_tune(path))
            self.assertEqual((t.snapshot(), t.modified, w.path), (other.snapshot(), False, path))
            self.assertIn("Unsaved changes", self.asked[0][0])
            self.assertFalse(w.open_tune(os.path.join(tmp, "missing.tune")))

    def test_opening_a_file_while_connected_sends_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "hot.tune")
            other = Tune(D)
            other.set("injEnable", "On")
            other.set("veTable", [60.0] * 256)
            other.save(path)
            self.connect()
            self.answer = QMessageBox.Cancel
            self.assertFalse(self.win.open_tune(path))
            self.assertIn("switches ON the injectors", self.asked[-1][1])
            self.assertEqual(self.win.tune.snapshot(), D.defaults)
            self.answer = QMessageBox.Yes
            self.assertTrue(self.win.open_tune(path))
            self.assertTrue(self.ecu_matches())
            self.assertEqual(self.win.tune.get("injEnable"), "On")
            self.assertTrue(self.win.unburned)


if __name__ == "__main__":
    unittest.main()
