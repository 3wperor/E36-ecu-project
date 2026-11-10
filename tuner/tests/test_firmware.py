"""
The tuner against the REAL firmware running in the simavr simulator, over a
pseudo-terminal exactly like a serial port. Skipped unless the simulator has
been built:   cd firmware && make build/ecu.elf build/sim_pty
"""
import os
import subprocess
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from e36tuner.definition import Definition
from e36tuner.link import ENGINE_BURNING, EcuLink, open_serial

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
SIM = os.path.join(ROOT, "firmware", "build", "sim_pty")
ELF = os.path.join(ROOT, "firmware", "build", "ecu.elf")
PTY = "/tmp/simavr-uart0"
D = Definition.load()


@unittest.skipUnless(os.path.exists(SIM) and os.path.exists(ELF), "simulated ECU not built")
class FirmwareTest(unittest.TestCase):
    def setUp(self):
        if os.path.lexists(PTY):
            os.unlink(PTY)
        self.sim = subprocess.Popen([SIM, ELF, "3000", "120"], stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL)
        end = time.time() + 10
        while not os.path.exists(PTY) and time.time() < end:
            time.sleep(0.05)
        self.assertTrue(os.path.exists(PTY), "the simulator did not start")

    def tearDown(self):
        self.sim.kill()
        self.sim.wait()

    def test_link(self):
        link = EcuLink(open_serial(PTY, boot_wait=1.0), D)
        try:
            self.assertEqual(link.check_signature(), D.signature)
            self.assertEqual(link.version(), D.version)
            # a new ECU holds exactly the default tune this tuner starts from
            self.assertEqual(link.read_pages(), D.defaults)
            rt = link.realtime()
            self.assertEqual(rt["sync"], 3)
            self.assertAlmostEqual(rt["rpm"], 3000, delta=15)
            self.assertTrue(int(rt["faults"]) & 0x80)                      # DEFAULT TUNE
            # write a whole table (two frames) and single values; read them back
            f = D.fields["ignTable"]
            data = bytes(f.to_raw(10 + (i % 16)) for i in range(256))
            link.write(f.page, f.offset, data)
            g = D.fields["revLimit"]
            link.write(g.page, g.offset, bytes([g.to_raw(6100)]))
            pages = link.read_pages()
            self.assertEqual(pages[f.page][f.offset:f.offset + 256], data)
            self.assertEqual(pages[g.page][g.offset], 61)
            self.assertTrue(link.verify(pages))
            # the firmware refuses what the tuner must never send
            for bad in (b"R" + bytes([9, 0, 0, 0, 1, 0]), b"M" + bytes([0, 0, 255, 255, 2, 0, 1, 2])):
                with self.assertRaises(IOError):
                    link.request(bad)
            # burn: the busy flag rises and falls, and the default-tune fault clears
            link.request(b"B\x01\x00")                                     # 256 changed bytes: ~1 s
            time.sleep(link.BURN_FLAG_DELAY)
            self.assertTrue(int(link.realtime()["engine"]) & ENGINE_BURNING)
            took = link.burn()
            self.assertTrue(0.1 < took < 30, took)
            self.assertFalse(int(link.realtime()["engine"]) & ENGINE_BURNING)
            self.assertFalse(int(link.realtime()["faults"]) & 0x80)
            self.assertEqual(link.retried, 0)
        finally:
            link.close()

    def test_gui_session(self):
        try:
            from PySide6.QtWidgets import QApplication
        except ImportError:                                 # pragma: no cover
            self.skipTest("PySide6 not installed")
        from e36tuner.app import MainWindow
        from tests.test_gui import pump
        app = QApplication.instance() or QApplication([])   # noqa: F841
        win = MainWindow(D)
        try:
            win.session.connect_serial(PTY, boot_wait=1.0)
            self.assertTrue(pump(lambda: win.session.online, 15), "did not connect")
            self.assertTrue(pump(lambda: win.dashboard.gauges["rpm"].value is not None, 5))
            self.assertAlmostEqual(win.dashboard.gauges["rpm"].value, 3000, delta=15)
            ed = win.show_editor("ignTbl")
            m = ed.model
            self.assertTrue(pump(lambda: m.live is not None, 5))
            # 3000 rpm is the column of the 3000 rpm bin (column 0 is the load axis)
            self.assertEqual(win.tune.shown("rpmBins", m.live[1] - 1), 3000)
            # edit the table: the firmware's ignition advance follows it (plus its own coolant trim)
            name = m.cells[m.live][0]
            cells = [rc for rc, (n, _) in m.cells.items() if n == name]
            got = []
            win.session.realtime.connect(lambda rt: got.append(rt["advance"]))

            def advance():
                self.assertTrue(pump(lambda: win.session.pending() == 0, 5))
                del got[:]
                self.assertTrue(pump(lambda: len(got) > 8, 5))
                return got[-1]

            m.apply(dict.fromkeys(cells, 20.0), "set")
            flat20 = advance()
            m.apply(dict.fromkeys(cells, 14.0), "set")
            self.assertAlmostEqual(flat20 - advance(), 6.0, delta=0.15)
            win.undo()
            self.assertAlmostEqual(advance(), flat20, delta=0.15)
            # burn through the GUI
            win.tune.set("revLimit", 6000)
            self.assertTrue(win.unburned)
            win.burn()
            self.assertTrue(pump(lambda: not win.unburned, 30), "burn did not finish")
        finally:
            win.session.disconnect()
            win.tune.mark_saved()
            win.close()


if __name__ == "__main__":
    unittest.main()
