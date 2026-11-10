"""The tuner's main window: connect, live data, editors, burn, tune files."""
import argparse
import os
import sys

from PySide6.QtCore import QSettings, Qt
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (QApplication, QComboBox, QDockWidget, QFileDialog, QLabel, QMainWindow,
                               QMessageBox, QPushButton, QStackedWidget, QToolBar, QTreeWidget,
                               QTreeWidgetItem)

from . import __version__
from .compare import CompareDialog
from .dashboard import Dashboard
from .definition import Definition
from .editors import CurveEditor, GridView, SettingsForm, TableEditor
from .link import list_ports
from .session import CONNECTING, ONLINE, EcuSession
from .tune import Tune, TuneFileError, read_tune_file

FILE_FILTER = "Tune files (*.tune *.json);;All files (*)"
OUTPUT_ENABLES = {"ignEnable": "ignition coils", "injEnable": "injectors"}

KEYS_HELP = """<h3>Table and curve keys</h3>
<table cellpadding="3">
<tr><td><b>type a number</b></td><td>sets every selected cell</td></tr>
<tr><td><b>Enter</b></td><td>open the cell for editing (use this for negative numbers)</td></tr>
<tr><td><b>+</b> / <b>-</b></td><td>one step up / down (hold Ctrl for ten steps)</td></tr>
<tr><td><b>*</b></td><td>scale the selection by a percentage</td></tr>
<tr><td><b>I</b></td><td>interpolate between the ends or corners of the selection</td></tr>
<tr><td><b>S</b></td><td>smooth the selection</td></tr>
<tr><td><b>Ctrl+C</b> / <b>Ctrl+V</b></td><td>copy / paste (works with spreadsheets)</td></tr>
<tr><td><b>Ctrl+Z</b> / <b>Ctrl+Shift+Z</b></td><td>undo / redo</td></tr>
</table>
<p>Edits go to the ECU straight away but only into its working memory.
<b>Burn</b> stores them; without a burn they are lost when the ECU loses power.</p>"""


class MainWindow(QMainWindow):
    def __init__(self, definition=None):
        super().__init__()
        self.d = definition or Definition.load()
        self.tune = Tune(self.d)
        self.session = EcuSession(self.d, self)
        self.path = None                # tune file being edited
        self.unburned = False           # the ECU's RAM holds changes that are not in its EEPROM
        self._adopting = False          # the tune is being replaced by what the ECU holds: don't send it back
        self.ecu_version = ""
        self.settings = QSettings("e36ecu", "tuner")
        self.resize(1360, 820)

        self._build_actions()
        self._build_toolbar()
        self._build_editors()
        self._build_dashboard()
        self._build_statusbar()

        self.tune.listeners.append(self._tune_changed)
        s = self.session
        s.stateChanged.connect(self._state_changed)
        s.connected.connect(self._connected)
        s.realtime.connect(self._realtime)
        s.failed.connect(self._failed)
        s.burned.connect(self._burned)
        s.synced.connect(self._synced)
        self._refresh()
        self._state_changed(self.session.state)

    # ------------------------------------------------------------ building --
    def _action(self, text, slot, shortcut=None, tip=None):
        a = QAction(text, self)
        a.triggered.connect(lambda *_: slot())
        if shortcut:
            a.setShortcut(shortcut)
        if tip:
            a.setToolTip(tip)
            a.setStatusTip(tip)
        return a

    def _build_actions(self):
        A = self._action
        self.a_new = A("&New tune (defaults)", self.new_tune, QKeySequence.New)
        self.a_open = A("&Open tune…", self.open_tune, QKeySequence.Open)
        self.a_save = A("&Save tune", self.save_tune, QKeySequence.Save)
        self.a_save_as = A("Save tune &as…", self.save_tune_as, QKeySequence.SaveAs)
        self.a_compare = A("&Compare with a tune file…", self.compare_file)
        self.a_quit = A("&Quit", self.close, QKeySequence.Quit)
        self.a_undo = A("&Undo", self.undo, QKeySequence.Undo)
        self.a_redo = A("&Redo", self.redo, QKeySequence.Redo)
        self.a_defaults = A("Restore &default tune", self.restore_defaults)
        self.a_connect = A("&Connect", self.connect_port)
        self.a_disconnect = A("&Disconnect", self.disconnect_ecu)
        self.a_demo = A("Connect to the &demo ECU", self.connect_demo,
                        tip="A pretend ECU and engine inside this program, to try the tuner without hardware")
        self.a_burn = A("&Burn to ECU", self.burn, "Ctrl+B",
                        "Store the tune in the ECU permanently (otherwise it is lost at power-off)")
        self.a_keys = A("&Keys and how editing works", self.show_keys, QKeySequence.HelpContents)
        self.a_about = A("&About", self.show_about)

        mb = self.menuBar()
        m = mb.addMenu("&File")
        m.addActions([self.a_new, self.a_open, self.a_save, self.a_save_as])
        m.addSeparator()
        m.addAction(self.a_compare)
        m.addSeparator()
        m.addAction(self.a_quit)
        m = mb.addMenu("&Edit")
        m.addActions([self.a_undo, self.a_redo])
        m.addSeparator()
        m.addAction(self.a_defaults)
        m = mb.addMenu("E&CU")
        m.addActions([self.a_connect, self.a_demo, self.a_disconnect])
        m.addSeparator()
        m.addAction(self.a_burn)
        m = mb.addMenu("&Help")
        m.addActions([self.a_keys, self.a_about])

    def _build_toolbar(self):
        tb = QToolBar("Main")
        tb.setObjectName("main")
        tb.setMovable(False)
        self.addToolBar(tb)
        self.port_box = QComboBox()
        self.port_box.setEditable(True)
        self.port_box.setMinimumWidth(230)
        self.port_box.setToolTip("Serial port of the ECU (Linux /dev/ttyACM0 or /dev/ttyUSB0, Windows COM5, …)")
        tb.addWidget(QLabel(" Port: "))
        tb.addWidget(self.port_box)
        refresh = QPushButton("↻")
        refresh.setFixedWidth(34)
        refresh.setToolTip("Look for serial ports again")
        refresh.clicked.connect(self.refresh_ports)
        tb.addWidget(refresh)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._connect_clicked)
        tb.addWidget(self.connect_btn)
        tb.addAction(self.a_demo)
        self.a_demo.setIconText("Demo ECU")
        tb.addSeparator()
        tb.addAction(self.a_burn)
        self.a_burn.setIconText("Burn")
        tb.addSeparator()
        tb.addActions([self.a_undo, self.a_redo])
        tb.addSeparator()
        tb.addActions([self.a_open, self.a_save])
        self.a_open.setIconText("Open")
        self.a_save.setIconText("Save")
        self.refresh_ports()

    def _build_editors(self):
        self.stack = QStackedWidget()
        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.editors = {}
        welcome = QLabel(
            "<h2>E36 ECU tuner</h2>"
            "<p>Choose a setting on the left. Live data is on the right once connected.</p>"
            "<p><b>No ECU yet?</b> Press <b>Demo ECU</b> to try everything with a pretend engine, "
            "or just edit and save a tune file offline.</p>"
            "<p>Edits go to the ECU's working memory straight away. Press <b>Burn</b> to keep them.</p>"
            "<p style='color:#d64545'><b>The default tune is a safe starting point, not a tune.</b> "
            "Ignition and injection are off, and the injector, MAF and thermistor values are placeholders "
            "to be set from the real parts.</p>")
        welcome.setWordWrap(True)
        welcome.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        welcome.setMargin(20)
        self.stack.addWidget(welcome)
        for menu, items in self.d.menus().items():
            top = QTreeWidgetItem(self.tree, [menu])
            top.setExpanded(True)
            font = top.font(0)
            font.setBold(True)
            top.setFont(0, font)
            for kind, item in items:
                if kind == "table":
                    w = TableEditor(self.tune, item)
                elif kind == "curve":
                    w = CurveEditor(self.tune, item)
                else:
                    w = SettingsForm(self.tune, item, confirm=self._confirm_change)
                w.message.connect(lambda text: self.statusBar().showMessage(text, 6000))
                self.editors[item["name"]] = w
                self.stack.addWidget(w)
                node = QTreeWidgetItem(top, [item["title"]])
                node.setData(0, Qt.UserRole, item["name"])
        self.tree.currentItemChanged.connect(self._tree_selected)
        dock = QDockWidget("Settings", self)
        dock.setObjectName("settings")
        dock.setFeatures(QDockWidget.DockWidgetMovable)
        self.tree.setMinimumWidth(215)
        dock.setWidget(self.tree)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)
        self.setCentralWidget(self.stack)

    def _build_dashboard(self):
        self.dashboard = Dashboard(self.d)
        dock = QDockWidget("Live data", self)
        dock.setObjectName("live")
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        dock.setWidget(self.dashboard)
        dock.setMinimumWidth(430)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

    def _build_statusbar(self):
        self.link_label = QLabel()
        self.burn_label = QLabel()
        self.statusBar().addPermanentWidget(self.burn_label)
        self.statusBar().addPermanentWidget(self.link_label)

    # --------------------------------------------------------------- state --
    def _refresh(self):
        online = self.session.online
        name = os.path.basename(self.path) if self.path else "untitled"
        self.setWindowTitle(f"{name}{'*' if self.tune.modified else ''} — E36 ECU tuner")
        self.a_undo.setEnabled(self.tune.can_undo())
        self.a_redo.setEnabled(self.tune.can_redo())
        self.a_undo.setText("&Undo" + (f" {self.tune.undo_stack[-1][0]}" if self.tune.can_undo() else ""))
        self.a_redo.setText("&Redo" + (f" {self.tune.redo_stack[-1][0]}" if self.tune.can_redo() else ""))
        self.a_undo.setIconText("Undo")
        self.a_redo.setIconText("Redo")
        self.a_burn.setEnabled(online)
        self.a_disconnect.setEnabled(self.session.state != "offline")
        if online and self.unburned:
            self.burn_label.setText("<span style='color:#d9a400; font-weight:600'>"
                                    "Changes not burned: lost when the ECU loses power</span>")
        elif online:
            self.burn_label.setText("<span style='color:#7d8590'>ECU matches this tune</span>")
        else:
            self.burn_label.setText("")

    def _state_changed(self, state):
        if state == ONLINE:
            self.link_label.setText(f"<span style='color:#2e9e5b; font-weight:600'>Connected: "
                                    f"{self.session.what}</span>")
        elif state == CONNECTING:
            self.link_label.setText(f"Connecting to {self.session.what}…")
        else:
            self.link_label.setText("<span style='color:#7d8590'>Offline</span>")
            self.unburned = False
            self.dashboard.clear()
            for w in self.editors.values():
                w.update_live(None)
        self.connect_btn.setText("Connect" if state == "offline" else "Disconnect")
        self.port_box.setEnabled(state == "offline")
        self._refresh()

    def _tree_selected(self, item, _prev=None):
        name = item.data(0, Qt.UserRole) if item else None
        if name:
            self.stack.setCurrentWidget(self.editors[name])
            view = self.editors[name].findChild(GridView)
            if view:
                view.setFocus()

    def show_editor(self, name):
        """Open an editor by its definition name (also selects it in the tree)."""
        for i in range(self.tree.topLevelItemCount()):
            top = self.tree.topLevelItem(i)
            for k in range(top.childCount()):
                if top.child(k).data(0, Qt.UserRole) == name:
                    self.tree.setCurrentItem(top.child(k))
                    return self.editors[name]
        raise KeyError(name)

    # ---------------------------------------------------------- connection --
    def refresh_ports(self):
        current = self.port_box.currentText()
        self.port_box.clear()
        for dev, desc in list_ports():
            self.port_box.addItem(f"{dev}  ({desc})" if desc and desc != "n/a" else dev, dev)
        last = current or self.settings.value("port", "")
        if last:
            i = self.port_box.findData(last)
            if i >= 0:
                self.port_box.setCurrentIndex(i)
            else:
                self.port_box.setEditText(last)

    def _port(self):
        i = self.port_box.currentIndex()
        if i >= 0 and self.port_box.itemText(i) == self.port_box.currentText():
            return self.port_box.itemData(i)
        return self.port_box.currentText().split("  (")[0].strip()

    def _connect_clicked(self):
        if self.session.state == "offline":
            self.connect_port()
        else:
            self.disconnect_ecu()

    def connect_port(self):
        port = self._port()
        if not port:
            QMessageBox.information(self, "Connect", "Choose or type the ECU's serial port first, "
                                    "or press Demo ECU to try the tuner without hardware.")
            return
        self.settings.setValue("port", port)
        self.statusBar().showMessage("Opening the port (the ECU board restarts, about 2 seconds)…", 4000)
        self.session.connect_serial(port)

    def connect_demo(self):
        self.session.connect_demo()

    def disconnect_ecu(self):
        self.session.disconnect()

    def _connected(self, sig, version, pages):
        self.ecu_version = version
        local = self.tune.snapshot()
        if pages != local:
            pristine = self.path is None and not self.tune.modified and not self.tune.can_undo()
            choice = "ecu" if pristine else self._ask_which_tune(pages)
            if choice == "ecu":
                self._adopting = True
                try:
                    if pristine:
                        self.tune.replace(pages)
                        self.tune.mark_saved()
                    else:
                        self.tune.replace(pages, "load the tune from the ECU")
                finally:
                    self._adopting = False
            elif choice == "mine":
                self.session.send_pages(local)
                self.unburned = True
            else:
                self.session.disconnect()
                return
        self.statusBar().showMessage(f"Connected: {version}", 8000)
        self._refresh()

    def _ask_which_tune(self, ecu_pages):
        from .tune import diff
        n = len(diff(self.d, self.tune.snapshot(), ecu_pages))
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Question)
        box.setWindowTitle("Which tune?")
        box.setText(f"The tune in the ECU differs from the one open here ({n} setting{'s' if n != 1 else ''}).")
        box.setInformativeText("Load the ECU's tune here (your current one can be brought back with Undo), "
                               "or send the tune open here to the ECU?")
        ecu = box.addButton("Load from ECU", QMessageBox.AcceptRole)
        mine = box.addButton("Send mine to ECU", QMessageBox.DestructiveRole)
        box.addButton("Disconnect", QMessageBox.RejectRole)
        box.exec()
        return "ecu" if box.clickedButton() is ecu else "mine" if box.clickedButton() is mine else None

    def _failed(self, message):
        self.session.disconnect()
        QMessageBox.warning(self, "ECU connection", message + "\n\nThe tuner is now offline. "
                            "Your tune is still open here; reconnect to continue.")

    def _realtime(self, rt):
        self.dashboard.update_values(rt)
        w = self.stack.currentWidget()
        if hasattr(w, "update_live"):
            w.update_live(rt)

    # --------------------------------------------------------------- edits --
    def _tune_changed(self, changes):
        if changes and self.session.online and not self._adopting:
            for page, offset, data in changes:
                self.session.write(page, offset, data)
            self.unburned = True
        self._refresh()

    def _confirm_change(self, field, raw):
        """Switching an output on while connected can fire real coils / injectors."""
        if field.name in OUTPUT_ENABLES and raw == 1 and self.session.online:
            what = OUTPUT_ENABLES[field.name]
            return QMessageBox.warning(
                self, "Enable outputs",
                f"This switches the {what} ON in the connected ECU.\n\n"
                f"If real {what} are wired up they will be driven as soon as the engine turns. Continue?",
                QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel) == QMessageBox.Yes
        return True

    def undo(self):
        what = self.tune.undo()
        if what:
            self.statusBar().showMessage(f"Undone: {what}", 4000)

    def redo(self):
        what = self.tune.redo()
        if what:
            self.statusBar().showMessage(f"Redone: {what}", 4000)

    def restore_defaults(self):
        if QMessageBox.question(self, "Restore defaults", "Replace every setting with the default tune?"
                                + ("\n\nThis also goes to the connected ECU (not burned)."
                                   if self.session.online else "") + "\n\nIt can be undone.") == QMessageBox.Yes:
            self.tune.replace(self.d.defaults, "restore the default tune")

    def burn(self):
        if self.session.online:
            self.statusBar().showMessage("Burning…")
            self.session.burn(self.tune.snapshot())

    def _burned(self, seconds):
        self.unburned = self.session.pending() > 0
        self.statusBar().showMessage(f"Burned to the ECU ({seconds:.1f} s)", 8000)
        self._refresh()

    def _synced(self, same):
        if not same:
            self.statusBar().clearMessage()
            if QMessageBox.warning(self, "Burn", "The ECU's working memory does not match the tune open here, "
                                   "so nothing was burned.\n\nSend this tune to the ECU now?",
                                   QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.session.send_pages(self.tune.snapshot())
                self.unburned = True
                self._refresh()

    # --------------------------------------------------------------- files --
    def _keep_changes(self):
        """Before the open tune is replaced. False = the user cancelled."""
        if not self.tune.modified:
            return True
        r = QMessageBox.question(self, "Unsaved changes", "Save the changes to this tune first?",
                                 QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
        if r == QMessageBox.Save:
            return self.save_tune()
        return r == QMessageBox.Discard

    def _confirm_send(self, pages, what):
        """Loading a tune while connected sends it to the ECU: say so, and warn about outputs."""
        if not self.session.online:
            return True
        on = [label for name, label in OUTPUT_ENABLES.items()
              if self.d.fields[name].read_raw(pages[self.d.fields[name].page])]
        text = f"{what} will also be sent to the connected ECU (not burned)."
        if on:
            text += f"\n\nIt switches ON the {' and '.join(on)}."
        return QMessageBox.warning(self, "Send to ECU", text + "\n\nContinue?",
                                   QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel) == QMessageBox.Yes

    def new_tune(self):
        if self._keep_changes() and self._confirm_send(self.d.defaults, "The default tune"):
            self.tune.replace(self.d.defaults)
            self.tune.mark_saved()
            self.path = None
            self._refresh()

    def open_tune(self, path=None):
        if not self._keep_changes():
            return False
        if not path:
            path, _ = QFileDialog.getOpenFileName(self, "Open tune", self._dir(), FILE_FILTER)
            if not path:
                return False
        try:
            pages, notes = read_tune_file(self.d, path)
        except TuneFileError as e:
            QMessageBox.warning(self, "Open tune", str(e))
            return False
        if notes and not self._show_import_notes(path, notes):
            return False
        if not self._confirm_send(pages, "This tune"):
            return False
        self.tune.replace(pages)
        self.settings.setValue("dir", os.path.dirname(os.path.abspath(path)))
        if notes:                       # imported from another layout: a new, unsaved tune
            self.path = None
        else:
            self.path = path
            self.tune.mark_saved()
        self._refresh()
        self.statusBar().showMessage(f"Opened {path}", 6000)
        return True

    def _show_import_notes(self, path, notes):
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Information)
        box.setWindowTitle("Import tune")
        box.setText(f"{os.path.basename(path)} was saved by a different firmware version. "
                    "Its settings were imported by name. Check the tune before using it.")
        box.setDetailedText("\n".join(notes))
        box.setInformativeText(f"{len(notes)} note{'s' if len(notes) != 1 else ''} (see Details).")
        box.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        return box.exec() == QMessageBox.Ok

    def save_tune(self):
        if not self.path:
            return self.save_tune_as()
        try:
            self.tune.save(self.path)
        except OSError as e:
            QMessageBox.warning(self, "Save tune", f"Cannot save {self.path}: {e}")
            return False
        self._refresh()
        self.statusBar().showMessage(f"Saved {self.path}", 6000)
        return True

    def save_tune_as(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save tune", self.path or os.path.join(self._dir(), "my.tune"),
                                              FILE_FILTER)
        if not path:
            return False
        if not os.path.splitext(path)[1]:
            path += ".tune"
        self.path = path
        self.settings.setValue("dir", os.path.dirname(os.path.abspath(path)))
        return self.save_tune()

    def compare_file(self, path=None):
        if not path:
            path, _ = QFileDialog.getOpenFileName(self, "Compare with tune", self._dir(), FILE_FILTER)
            if not path:
                return
        try:
            pages, notes = read_tune_file(self.d, path)
        except TuneFileError as e:
            QMessageBox.warning(self, "Compare", str(e))
            return
        if notes and not self._show_import_notes(path, notes):
            return
        CompareDialog(self.tune, pages, os.path.basename(path), self).exec()

    def _dir(self):
        return self.settings.value("dir", os.path.expanduser("~"))

    # ---------------------------------------------------------------- misc --
    def show_keys(self):
        QMessageBox.information(self, "Keys", KEYS_HELP)

    def show_about(self):
        QMessageBox.about(self, "About", f"<b>E36 ECU tuner {__version__}</b><p>For firmware "
                          f"<code>{self.d.signature}</code><br>{self.ecu_version or self.d.version}</p>")

    def closeEvent(self, e):
        if not self._keep_changes():
            return e.ignore()
        if self.session.online and self.unburned and QMessageBox.warning(
                self, "Not burned", "The ECU holds changes that are not burned. They will be lost when it "
                "loses power.\n\nClose anyway?", QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel) != QMessageBox.Yes:
            return e.ignore()
        self.session.disconnect()
        e.accept()


def main(argv=None):
    ap = argparse.ArgumentParser(prog="e36tuner", description="E36 ECU tuner")
    ap.add_argument("tune", nargs="?", help="tune file to open")
    ap.add_argument("--port", help="connect to this serial port at start")
    ap.add_argument("--demo", action="store_true", help="connect to the built-in demo ECU at start")
    args = ap.parse_args(argv)
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    if args.tune:
        win.open_tune(args.tune)
    if args.demo:
        win.connect_demo()
    elif args.port:
        win.port_box.setEditText(args.port)
        win.connect_port()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
