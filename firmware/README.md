# Firmware

Bare-metal C for the ATmega1280 (also builds for the pin-compatible ATmega2560).
No Arduino core. Design: [../docs/PLAN.md](../docs/PLAN.md) §3.

## Status — Phase 1 (foundations)

| Part | State |
|------|-------|
| Safe output init (all injectors/coils/aux off before anything runs) | done |
| Synchronised 16-bit timers 1/3/4/5 at 4 µs/tick + 1 kHz system tick | done, sync verified at runtime (`flags` bit 0) |
| UART0 interrupt ring buffers, 115200 baud | done |
| Background ADC scan of 10 channels (~960 Hz each) | done |
| Watchdog (60 ms) + reset-cause capture | done |
| TunerStudio-style framing (length + payload + CRC32), commands `Q` `S` `A` | done |
| Cooperative main loop: 1 kHz / 100 Hz / 10 Hz / 1 Hz tasks | done (tasks are placeholders) |
| Crank/cam decoder | Phase 2 |

Current size: ~2.5 KB flash, ~1 KB RAM.

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

Serial console at 115200 baud: `r3000` (set rpm, `r0` stops), `s800,7200,5000`
(sweep 800→7200 rpm over 5 s), `c0|c1|c2` (cam none / single pulse / half-moon),
`o40` (cam position in 3° steps), `d` (drop one tooth), `?` (status).

## Testing notes

- Portable modules (`comms/`, later the decoder and engine maths) are unit-tested on the PC.
- simavr 1.6 does **not** model output-compare pins correctly in Normal timer mode
  (it also drives them at overflow). Tests that check hardware-timed outputs
  therefore capture the firmware's `TCCRnA`/`OCRnx` register writes, which define
  exactly what the real timer hardware does (see `tools/stim/sim_stim.c`). The ECU's
  injector/coil scheduler tests in Phase 3 use the same method, and a logic
  analyser on real hardware is the final check.
