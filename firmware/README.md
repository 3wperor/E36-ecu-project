# Firmware

Bare-metal C for the ATmega1280 (also builds for the pin-compatible ATmega2560).
No Arduino core. Design: [../docs/PLAN.md](../docs/PLAN.md) §3.

## Status — Phases 1 to 5 and 9a done in simulation

| Part | State |
|------|-------|
| Safe output init (all injectors/coils/aux off before anything runs) | done |
| Synchronised 16-bit timers 1/3/4/5 at 4 µs/tick + 1 kHz system tick | done, sync verified at runtime (`flags` bit 0) |
| UART0 interrupt ring buffers, 115200 baud | done |
| Background ADC scan of 10 channels (~960 Hz each) | done |
| Watchdog (60 ms) + reset-cause capture | done |
| TunerStudio-style framing (length + payload + CRC32), commands `Q` `S` `A` `O` `R` `M` `B` `D` `T` `P` `p` | done |
| Cooperative main loop: 1 kHz / 100 Hz / 10 Hz / 1 Hz tasks | done |
| Crank/cam decoder: 60-2 gap sync, cam phase (single pulse or half-moon), RPM, noise rejection, stall detection | done — see below |
| Tooth logger (`T` command, one-shot 64-tooth capture) | done |
| Injector/coil scheduling: 6 injectors + 6 coils on output-compare hardware, sequential / wasted spark, spark refinement, safety rules | done — see below |
| Bench output test (`P`/`p`): fixed advance, dwell, pulse width; overrides the engine calculation while on; **off at power-up** | done |
| Tune layout generated from `definitions/ecu.toml` (pages, accessors, defaults, live data, TunerStudio ini) | done |
| Tune storage: two pages in EEPROM, A/B slots with CRC, background burn, power-cut safe, defaults fallback | done |
| Sensors: MAF curve, two TPS, CLT/IAT thermistor, battery, wideband, narrowbands, fault detection | done |
| Engine maths: MAF / Alpha-N / blend load, fuel (AFR target, flow, warm-up, accel and after-start enrichment, cranking, dead time, cylinder trims, rev limiter), ignition (table, coolant and cylinder trims, cranking advance, dwell), end of injection, fuel pump | done — see below |
| Knock control: TPIC8101 on SPI, per-cylinder windows, learned noise, per-cylinder retard and recovery, chip-fault retard | done in simulation; chip settings to confirm on the bench (below) |
| Overrun fuel cut (closed throttle, above an rpm, warm, after a delay; resumes below a lower rpm) | done (Phase 9a) |
| Closed-loop lambda: off / narrowband per bank / wideband / wideband with narrowband fallback; trim limits; wideband fault detection | done in simulation (Phase 9a) |
| Idle: target rpm and base valve opening vs coolant, PI on rpm, cranking opening, dashpot, ignition idle control | done in simulation (Phase 9a) |
| Idle valve driver: complementary PWM on Timer2 for the two-coil rotary valve (122 or 245 Hz) | done (Phase 9a); check the valve on the bench |
| VANOS on/off by rpm window, throttle and coolant, with hysteresis; narrowband heaters after a delay | done (Phase 9a) |
| Tach output to the instrument cluster (3, 2 or 1 pulses per revolution) | done (Phase 9a) |
| Fan, A/C idle-up, long-term fuel trim learning, VANOS by rpm × load table | later (Phase 9b) |

Current size: ~28 KB flash (22 %), ~4.5 KB RAM (55 %) — RAM is the resource to watch.

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

### Scheduler test results

`make test` also runs the decoder **and** scheduler against a simulated engine and a model of the AVR
output-compare hardware (pins switch at exact ticks, ISRs run with realistic latency). Every pulse is
converted back to a true crank angle:

| Scenario | Result |
|----------|--------|
| Sequential at 200 … 9,000 rpm | Every cylinder exactly once per cycle, firing order 1-5-3-6-2-4; spark/injection angle within 0.25° (≤ 3,000 rpm) and 2.5 timer ticks above (≈ 0.4° at 7,200); dwell and pulse width exact |
| Cam-less (CRANK) sync | Wasted spark every 360°, injectors every revolution at half pulse width |
| Start from rest | Nothing fires before sync; never two sparks within 180° |
| 1,000 → 7,000 rpm in 0.5 s | Spark within 0.3°, dwell within 8 %, no missed or extra events |
| Big advance / end-of-injection changes while running | No missed or double events |
| Sync loss (dropped tooth) | No new outputs until re-sync; running dwells finish normally |
| Engine stops | All channels idle, nothing left dwelling |
| Outputs disabled while running | Stop within one cycle |
| 30 ms pulse requested at 7,200 rpm / 8 ms dwell at 9,000 rpm | Capped at 90 % duty / half the period |

`make sim-test` runs the real firmware in simavr with a simulated engine on the input-capture pins:

- full sync and 1,000 / 7,200 rpm read back over the serial protocol, tooth log, stall detection;
- outputs enabled over the protocol; at 7,200 rpm the injector/coil schedule is checked from the
  output-compare register writes: 396 sparks + 396 injections in 1.1 s, every dwell 3.0 ms, every pulse
  2.5 ms, firing order correct, sparks 120° apart within 1.6 ticks, no late edges;
- every ISR timed: crank ISR **20 µs on a plain tooth, 24 µs average, 56 µs worst** (when several outputs
  are armed on one tooth); output-compare ISRs 5–6 µs; **total interrupt load 23 % at 7,200 rpm with all
  12 outputs running**. The test fails if the crank ISR averages over 25 µs or exceeds 60 µs;
- after the engine stops, all 12 channels are idle (read from the firmware's RAM).

## Build, test, flash

Needs `gcc-avr avr-libc binutils-avr avrdude`; the simulator test also needs `simavr libsimavr-dev`.

```sh
cd firmware
make                         # build/ecu.hex for ATmega1280
make MCU=atmega2560          # for a Mega 2560 board
make test                    # host unit tests (CRC32, framing, commands)
make sim-test                # boots the real firmware in simavr and talks to it over UART
make tuner-test              # the tuner GUI (headless) against the firmware in simavr
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

### Engine maths and storage test results (Phase 4)

`make test`:

| Suite | What it checks |
|-------|----------------|
| `test_engine` | Sensors vs an NTC model (thermistor within 1.1 °C from −30 to 120 °C), two-TPS logic, faults and fallbacks; air mass vs a floating-point calculation (MAF 93.1 vs 93.3 mg, Alpha-N within 0.2 %); pulse width within 0.03 %; warm-up, blend, MAF trim, cylinder trims; cranking, after-start fade, rev-limit hysteresis, acceleration enrichment rise and fade; ignition table interpolation within 0.04°, coolant and cylinder trims, dwell, end of injection |
| `test_storage` | Blank EEPROM → defaults; burn and reload; slot alternation; only changed bytes written; **power cut after each of the 239 writes of a burn → always the complete old or new tune**; corrupted slot → previous copy; different layout → defaults; edits during a burn |
| `test_main` | Page read/write/burn/defaults and live-data commands, including every range error |
| `tools/codegen/test_gen.py` | Generated files up to date; offsets contiguous; ini and C header match the layout; bad definitions rejected |

`make sim-test` (real firmware in simavr): default tune at first boot; tune switched on over the
protocol; at 3000 rpm the calculated pulse width matches an independent calculation within 0.1 % and the
injectors and coils fire exactly the calculated pulse and dwell; `engine_update` costs ~0.9 ms per 10 ms
(Phase 4; ~1.1 ms with the Phase 9a controls, incl. interrupts); burn → power cycle (new chip, saved EEPROM) → the tune is still there.

### Knock control test results (Phase 5)

`make test` (`test_knock`): window placement for 5,047 combinations of TDC offset, window start and
length (always 6 opens and 6 closes, never colliding, at least 2 teeth to read the chip and switch
sensor before the next window); sensor alternation with the firing order; noise learning; detection,
single and double steps, cap, recovery rate; inactive conditions; chip fault and its retard; threshold
curve; both result byte orders.

`make sim-test` emulates a TPIC8101 on the simulated SPI bus and INT/HOLD pin: at 3000 rpm, 218
windows in 1.5 s, evenly over the 6 cylinders, opening 12° ATDC (first tooth after 10°) for 60°, every
one on the correct sensor; background noise learned (120 vs 120 injected); knock injected on cylinder
3 retards only cylinder 3, and its spark, measured in the timer registers, moves 9.8° against a 9.7°
retard; timing comes back at ~1°/s; a chip returning 0 puts 3° on every cylinder; disabling stops the
windows. Crank ISR with knock running: 25.5 µs average, 66 µs worst.

### Lambda, idle, VANOS, overrun and tach test results (Phase 9a)

`make test` (`test_control`, the control logic against simple engine models): overrun cut after 510 ms
with a 0.5 s delay and back on below the resume speed; wideband loop on an engine 8 % lean settles at
+7.9 % within 2 s (AFR 14.71 vs 14.7), and a 25 % rich engine stops at the 15 % limit; narrowband loop
with a 100 ms sensor delay, bank A 5 % lean and bank B 3 % rich → average trims +4.1 % / −2.1 % (each
bank on its own); wideband at a rail → narrowband fallback in mode 3, open loop in mode 2; idle on a
valve/speed model holds 797 rpm against an 800 target, also after an A/C load step; idle timing pushes
the right way and is limited; dashpot; VANOS hysteresis; heater delay.

`make sim-test` (real firmware in simavr): tach 150 pulses in 1 s at 3000 rpm (3 per revolution) and
held low when the engine stops; VANOS output on at 3000 rpm / 25 % throttle and off on a closed
throttle; idle valve PWM duty in the Timer2 registers matches the live value (OC2A normal, OC2B
inverted); closed throttle at 3000 rpm → no injections at all; wideband reading 16.0 against a 14.7
target → +15 % trim (the limit) and a longer measured injector pulse; narrowband bank A rich / bank B
lean → −8.4 % / +6.3 % after 3 s and measured bank A pulses shorter than bank B's.

Pins on the Mega for a bench check: idle valve open coil D10 (OC2A), close coil D9 (OC2B), VANOS D24,
narrowband heaters D25/D26, tach D27. These are 5 V logic signals: the valve, solenoid, heaters and
cluster need driver stages (low-side MOSFETs; open-collector with a pull-up for the tach), which come
with the PCB in Phase 6.

**Before closed loop on the car:** leave `lambdaMode` Off for the first start, check the wideband reads
the same as its own controller's display, then enable wideband mode. Narrowband mode only acts at
λ = 1 targets (14.4–15.0 AFR).

### Knock chip bench test (Phase 5 hardware step)

The firmware's view of the TPIC8101 rests on assumptions that can only be checked with the chip:
its command encoding (`knock.c`, one marked block), the byte order of its result (tune setting
`knockReadOrder`) and its codes for prescaler, band-pass, gain and integrator time (tune settings,
from the datasheet tables). Bench procedure:

1. Wire a TPIC8101 module to the Mega: SCLK → D52, SI → D51, SO → D50, CS → D53, INT/HOLD → D35,
   5 V, GND, and the crystal/clock the module uses (set `knockPrescaler` for it from the datasheet).
2. Feed sensor input 1 from a signal generator through the module's input network: a 6.7 kHz sine,
   a few hundred mV. Run the trigger simulator at 3000 rpm, set `knockEnable` on.
3. `python3 tools/ecu_cli.py PORT knock`: the levels should be steady and rise in proportion to the
   generator amplitude. If they jump around or look like garbage, flip `knockReadOrder`.
4. Sweep `knockBand`: the level peaks at the code nearest 6.7 kHz. Set `knockGain` so the normal
   level sits around 100–200 of 1023.
5. Pulse the generator only during some windows (or swap sensor inputs) and check that the right
   cylinders show knock and retard.

On the car (Phase 7, sniffer mode) the band is checked again against the real engine's knock sound.

### Bench-testing the outputs (Phase 3)

With the trigger simulator driving the Mega, a logic analyser on the output pins shows the schedule:

| Channel | Mega pins |
|---------|-----------|
| Injectors 1–6 | D11, D12, D13, D5, D2, D3 |
| Coils 1–6 | D6, D7, D8, D46, D45, D44 |

```sh
python3 tools/ecu_cli.py /dev/ttyACM0 set ign=on inj=on advance=10 dwell=3 pw=2.5 tdc=60 eoi=380
python3 tools/ecu_cli.py /dev/ttyACM0 watch
python3 tools/ecu_cli.py /dev/ttyACM0 set off
```

Outputs are off at every power-up and must be enabled like this. **Bench only:** do not connect real
coils or injectors to the Mega's pins (they are 5 V logic outputs, and with real drivers attached `set`
fires them). The `tdc` value is the calibration found on the car with a timing light; any value works on
the bench.

### Trying the PC tools without hardware

```sh
cd firmware && make sim-pty RPM=3000 SECONDS=60 &      # simulated ECU + engine on /tmp/simavr-uart0
python3 tools/ecu_cli.py /tmp/simavr-uart0 watch
python3 tuner/run_tuner.py --port /tmp/simavr-uart0   # or the tuner (see ../tuner/README.md)
```

## Testing notes

- Portable modules (`comms/`, `decoder/`, `sched/`, `engine/`, `tune/`, `knock/`) are unit-tested on the PC.
- simavr does not model the ATmega1280's ADC channels 8–15 (selected with MUX5), so the battery input
  (ADC8) reads 0 V in simulation and the battery fault is expected there. Real silicon reads them.
- simavr 1.6 does **not** model output-compare pins correctly in Normal timer mode
  (it also drives them at overflow). Tests that check hardware-timed outputs
  therefore capture the firmware's `TCCRnA`/`OCRnx` register writes, which define
  exactly what the real timer hardware does (see `tools/stim/sim_stim.c`). The ECU's
  injector/coil scheduler tests in Phase 3 use the same method, and a logic
  analyser on real hardware is the final check.
