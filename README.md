# E36 ECU Project

A custom, bare-metal standalone engine management system for a 1997 UK BMW E36 323i (M52B25),
built to run a plenum-ITB intake on the stock MAF:
ATmega1280 @ 16 MHz, full 6-cylinder sequential injection and ignition, knock control,
and a custom desktop tuning application.

- Project plan: [docs/PLAN.md](docs/PLAN.md)
- Car profile, sensor inventory and Phase 0 checklist: [docs/CAR.md](docs/CAR.md)

- Firmware build/test/flash and trigger simulator wiring: [firmware/README.md](firmware/README.md)

Status: Phases 1–3 (firmware foundations, trigger simulator, crank/cam decoder, injector/coil scheduling) done in simulation.
