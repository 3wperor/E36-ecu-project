# Tuning

Two ways to tune the ECU today; our own GUI (Phase 4b) comes next.

Both files in this folder are **generated** from `definitions/ecu.toml` by
`tools/codegen/gen.py`. Don't edit them by hand.

| File | Used by |
|------|---------|
| `e36ecu.ini` | TunerStudio |
| `ecu_def.json` | `tools/ecu_cli.py` and our own tuner |

## TunerStudio

1. Install TunerStudio MS (the free "Lite" version is enough).
2. **File → New Project**, name it, and when asked for the firmware choose **Other / Browse** and
   select `tuner/e36ecu.ini` from this repository.
3. Connection: the ECU's serial port at **115200** baud.
4. The ECU's signature must match the ini (`E36ECU-dev-p4` for this version). If the firmware and
   the ini come from different versions, TunerStudio will refuse to connect, which is intended.

Menus: **Fuel** (engine and injectors, cranking, acceleration, AFR / VE / blend / MAF-trim /
end-of-injection tables, dead-time / cranking / warm-up curves), **Ignition** (enable, TDC offset,
cam mode, advance table, dwell and coolant-trim curves), **Sensors** (TPS calibration, battery,
wideband, MAF and thermistor curves). Changes are live in the ECU's RAM; **Burn** stores them.

**Not yet tested with TunerStudio itself** (it can't run where this was built). The ini follows
the TunerStudio format and is checked for internal consistency (every offset, size and name
matches the firmware), but if TunerStudio reports an error, note the message and the line and
the generator can be fixed.

## Command line (`tools/ecu_cli.py`)

Works today and was tested against the simulated ECU:

```sh
pip install pyserial
python3 tools/ecu_cli.py PORT watch                 # live data
python3 tools/ecu_cli.py PORT tune list             # every setting
python3 tools/ecu_cli.py PORT tune get ignTable     # a table, printed as a grid (top row = highest load)
python3 tools/ecu_cli.py PORT tune set injFlow 240
python3 tools/ecu_cli.py PORT tune set loadSource alpha-n
python3 tools/ecu_cli.py PORT tune burn             # store in EEPROM
python3 tools/ecu_cli.py PORT tune save base.tune   # back up to a file
python3 tools/ecu_cli.py PORT tune load base.tune   # restore (then `tune burn`)
```

Values are in real units (rpm, ms, degrees, %); out-of-range values are refused before they are
sent. A tune file records the firmware layout, so a file from a different layout is refused.

## Before anything runs an engine

The default tune is a **safe starting point, not a tune**: ignition and injection outputs are
**off** (`ignEnable`, `injEnable`), timing is conservative, mixtures are rich, and several values are
placeholders that must be set from the real parts first:

- `injFlow` and `injDead`: from the motorcycle injectors' data (or a flow test).
- `mafV` / `mafFlow`: the transfer curve of the actual HFM part number.
- `thermAdc` / `thermC`: check against the real coolant and air-temperature sensors.
- `tdcOffset`: measured with a timing light (Phase 8).
- `crankPw`, `warmup`: starting points only.

The live data shows **DEFAULT TUNE** (fault bit 7) until a tune has been burned.
