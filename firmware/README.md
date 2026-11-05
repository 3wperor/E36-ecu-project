# Firmware

Bare-metal C for the ATmega1280 (also builds for the pin-compatible ATmega2560).
No Arduino core. Design: [../docs/PLAN.md](../docs/PLAN.md) §3.

## Status — Phases 1 and 2 done in simulation

| Part | State |
|------|-------|
| Safe output init (all injectors/coils/aux off before anything runs) | done |
| Synchronised 16-bit timers 1/3/4/5 at 4 µs/tick + 1 kHz system tick | done, sync verified at runtime (`flags` bit 0) |
| UART0 interrupt ring buffers, 115200 baud | done |
| Background ADC scan of 10 channels (~960 Hz each) | done |
| Watchdog (60 ms) + reset-cause capture | done |
| TunerStudio-style framing (length + payload + CRC32), commands `Q` `S` `A` | done |
| Cooperative main loop: 1 kHz / 100 Hz / 10 Hz / 1 Hz tasks | done (tasks are placeholders) |
| Crank/cam decoder: 60-2 gap sync, cam phase (single pulse or half-moon), RPM, noise rejection, stall detection | done — see below |
| Tooth logger (`T` command, one-shot 64-tooth capture) | done |
| Injector/coil scheduling | Phase 3 |

Current size: ~4.7 KB flash, ~1.3 KB RAM.

### Decoder test results

`make test` runs the decoder against a simulated engine (same pattern code as the trigger simulator):

| Scenario | Result |
|----------|--------|
| Steady 100 … 9,000 RPM | FULL sync after 3 revolutions, RPM error ≤ 0.02 % |
| Start from rest, every crank starting position | FULL sync after ≤ 3 revolutions, no wrong positions |
| Cranking at 200 RPM with ±10/25/35 % compression ripple | No sync loss |
| 800→7,200 RPM in 2 s, 1,000↔7,000 RPM in 0.4 s | No sync loss |
| Dropped tooth | Sync dropped, re-acquired, never a wrong position |
| Noise spike at 10 % / 50 % of a tooth | Rejected, sync kept |
| Noise spike at 70 % of a tooth | Sync dropped and re-acquired safely |
| No cam / cam disabled | Runs on CRANK (360°) sync |
| Spurious or vanished cam edge | Drops to CRANK, recovers with the correct phase |
| Engine stops, restarts | Detected within 200 ms, re-syncs |

`make sim-test` runs the real firmware in simavr with a simulated engine on the ICP pins: full sync and
1,000 / 7,200 RPM read back over the serial protocol, tooth log, stall detection, and the crank ISR timed
at **13.5 µs average / 15.8 µs worst case** (fails above 17 µs).

## Build, test, flash

Needs `gcc-avr avr-libc binutils-avr avrdude`; the simulator test also needs `simavr libsimavr-dev`.

```sh
cd firmware
make                         # build/ecu.hex for ATmega1280
make MCU=atmega2560          # for a Mega 2560 board
make test                    # host unit tests (CRC32, framing, commands)
make sim-test                # boots the real firmware in simavr and talks to it over UART
make flash PORT=/dev/ttyUSB0 # upload via the Arduino bootloader
```

## Trigger simulator (`../tools/stim`)

A second Arduino (Uno/Nano) generates the M52 60-2 crank and cam signals.

```sh
cd tools/stim
make && make flash PORT=/dev/ttyUSB1
make test                    # pattern unit test + simavr timing test
```

Wiring:

| Stim (Uno/Nano) | ECU dev board (Mega) |
|-----------------|----------------------|
| D9  (crank)     | D48 (ICP5)           |
| D10 (cam)       | D49 (ICP4)           |
| GND             | GND                  |

Once both boards are wired, watch the ECU with the command-line tool (`pip install pyserial`):

```sh
python3 tools/ecu_cli.py /dev/ttyACM0 info     # firmware signature and version
python3 tools/ecu_cli.py /dev/ttyACM0 watch    # live RPM, sync state, sensor voltages
python3 tools/ecu_cli.py /dev/ttyACM0 teeth    # tooth-log capture
```

With the simulator at `r3000`, `watch` should show 3000 rpm and sync `full 720°`.
(Port names: Linux `/dev/ttyACM0` or `/dev/ttyUSB0`, macOS `/dev/cu.usbmodem…`, Windows `COM5` etc.)

Serial console at 115200 baud: `r3000` (set rpm, `r0` stops), `s800,7200,5000`
(sweep 800→7200 rpm over 5 s), `c0|c1|c2` (cam none / single pulse / half-moon),
`o40` (cam position in 3° steps), `d` (drop one tooth), `?` (status).

### Trying the PC tools without hardware

```sh
cd firmware && make sim-pty RPM=3000 SECONDS=60 &      # simulated ECU + engine on /tmp/simavr-uart0
python3 tools/ecu_cli.py /tmp/simavr-uart0 watch
```

## Testing notes

- Portable modules (`comms/`, `decoder/`, later the engine maths) are unit-tested on the PC.
- simavr 1.6 does **not** model output-compare pins correctly in Normal timer mode
  (it also drives them at overflow). Tests that check hardware-timed outputs
  therefore capture the firmware's `TCCRnA`/`OCRnx` register writes, which define
  exactly what the real timer hardware does (see `tools/stim/sim_stim.c`). The ECU's
  injector/coil scheduler tests in Phase 3 use the same method, and a logic
  analyser on real hardware is the final check.
