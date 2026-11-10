# E36 ECU Project

A custom, bare-metal standalone engine management system for a 1997 UK BMW E36 323i (M52B25),
built to run a plenum-ITB intake on the stock MAF:
ATmega1280 @ 16 MHz, full 6-cylinder sequential injection and ignition, knock control,
and a custom desktop tuning application.

- Project plan: [docs/PLAN.md](docs/PLAN.md)
- Car profile, sensor inventory and Phase 0 checklist: [docs/CAR.md](docs/CAR.md)

- Firmware build/test/flash and trigger simulator wiring: [firmware/README.md](firmware/README.md)

Status: Phases 1–5 (foundations, trigger simulator, crank/cam decoder, injector/coil scheduling, engine maths and tune storage, knock control), 4b (our own tuner: live data, table editing, burn, tune files) and 9a (closed-loop lambda with narrowband and wideband, idle valve and idle timing control, VANOS, overrun fuel cut, tach output) done in simulation. The knock chip's settings still need confirming on a real TPIC8101; the next steps need hardware (bench test on a Mega, then the Phase 0 checks on the car).

- Tuning (our tuner, TunerStudio and the command-line tool): [tuner/README.md](tuner/README.md)
- Tune layout (single source of truth): [definitions/ecu.toml](definitions/ecu.toml)
