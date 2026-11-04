# E36 323i (M52) Custom Bare-Metal ECU — Project Plan

Status: **planning** · Target: BMW E36 323i, M52B25, full sequential fuel + spark, knock control, custom tuning GUI.

This document turns the original project brief into an executable plan. It records
what we are building, how the firmware, hardware and tuning software fit together,
which parts of the brief need correcting, and the order we build and test them.

---

## 0. Corrections and clarifications to the original brief

These came out of checking the brief's numbers and the ATmega1280's datasheet behaviour.
None of them threaten the project; several change the design.

| # | Brief says | Reality | Impact |
|---|-----------|---------|--------|
| 1 | Teeth pass every **277 µs** at 7,200 RPM | 7,200 RPM = 120 rev/s × 60 tooth positions = 7,200 teeth/s → **138.9 µs per tooth** (the 16.67 ms cycle figure is correct) | CPU budget per tooth is ~2,220 clock cycles, not ~4,400. Still comfortable (see §3.6). |
| 2 | Crank ISR must take **< 1–2 µs** | ISR entry/exit alone on AVR with avr-gcc is ~1.5–3 µs. Realistic target: **≤ 10 µs** worst case. | Fine: 120 teeth × 10 µs = 1.2 ms per 16.67 ms cycle ≈ 7 % CPU. |
| 3 | "High-priority" Input Capture interrupt | **AVR has no interrupt priorities and no nesting by default.** All ISRs are equal; a long ISR delays every other ISR. | Every ISR (UART, ADC, SPI, timers) must be short. Precision comes from hardware: Input Capture **latches the timestamp in hardware** regardless of ISR latency, and Output Compare **toggles the pin in hardware**. ISR latency only affects *arming* the next event, not when it fires. |
| 4 | Crank ISR *only* logs a timestamp and counts teeth | Events must be scheduled relative to a recent tooth to stay accurate during acceleration, so the ISR also does a cheap table lookup and arms an Output Compare when "this tooth is the reference tooth for an event". | All heavy math stays in the main loop, which pre-computes `(reference_tooth, delay_ticks)` pairs; the ISR does only a compare + add + register write. |
| 5 | Main loop writes pulse width to the timer registers | On AVR, all 16-bit timer registers share **one TEMP byte**. If the main loop and an ISR both access 16-bit timer registers, values get corrupted. Also, in Normal mode OCR writes take effect immediately, so a late write can miss and wait a full timer wrap. | **Only ISRs touch timer registers.** Main loop hands schedules to ISRs through a double-buffered struct. ISR checks "is the target time already passed or too close?" and fires immediately (Force Output Compare) or skips instead of waiting 262 ms with a coil dwelling. |
| 6 | 6 ignition drivers "or logic-level triggers for smart coils" | The M52's stock coil-on-plug coils are **"dumb" coils** (no built-in igniter); the stock MS41 drives them with IGBTs inside the DME. | Board needs 6 ignition IGBTs (400 V clamped, logic-level gate) unless we swap to smart coils. Dwell control is the firmware's job. |
| 7 | MAP **or** MAF | Stock M52/MS41 is MAF-based (HFM hot-film). | Decision needed (§7). Recommended: speed-density (MAP + VE table) as primary, MAF input kept as a spare ADC channel. |
| 8 | Brief lists fuel + spark only | The M52 also needs: **single VANOS** solenoid (on/off), **dual-coil idle control valve** (two PWM outputs), fuel pump relay, main relay, tach output, check-engine light, optionally A/C compressor cut. | Extra outputs on the PCB (§2.3). |

---

## 1. System overview

```
              ┌────────────────────────── Car ───────────────────────────┐
 Crank VR  ──►│ MAX9926 ch1 ─► ICP5 (PL1)                                │
 Cam Hall  ──►│ RC + clamp  ─► INT / PCINT                               │
 Knock 1,2 ──►│ TPIC8101 ◄─SPI─►                                         │
 TPS, MAP, CLT, IAT, WBO2, Vbatt ─► ADC (ratiometric to sensor 5 V)      │
              │                                                          │
              │          ATmega1280 @ 16 MHz                             │
              │   Timer1 OC1A/B/C ─► Injectors 1-3  (low-side drivers)   │
              │   Timer3 OC3A/B/C ─► Injectors 4-6                       │
              │   Timer4 OC4A/B/C ─► Coils 1-3      (ignition IGBTs)     │
              │   Timer5 OC5A/B/C ─► Coils 4-6                           │
              │   Timer2 OC2A/OC2B ─► Idle valve (open/close coils)      │
              │   GPIO ─► VANOS, fuel pump, main relay, CEL, tach        │
              │   UART0 ─► USB-serial ─► Tuning software (PC)            │
              └──────────────────────────────────────────────────────────┘
```

Three deliverables, developed in parallel after the foundations are in place:

1. **Firmware** (`firmware/`) — bare-metal C, avr-gcc, no Arduino core.
2. **Hardware** (`hardware/`) — KiCad schematic + 4-layer PCB.
3. **Tuning software** (`tuner/`) — desktop GUI, TunerPro-style table editing plus live data and logging.

A fourth, small deliverable makes all of this testable on a desk:

4. **Trigger simulator** (`tools/stim/`) — a second MCU (any Arduino) that generates the M52 60-2 crank and cam patterns at a commanded RPM.

---

## 2. Hardware architecture

### 2.1 Timer and pin allocation (the core design decision)

The ATmega1280 has exactly **12 hardware Output Compare channels** on its four 16-bit
timers — one per injector and coil. Each fires a pin edge in hardware with zero jitter.

| Function | Timer / channel | MCU pin | Arduino Mega pin (dev board) |
|----------|-----------------|---------|------------------------------|
| Injector 1 / 2 / 3 | OC1A / OC1B / OC1C | PB5 / PB6 / PB7 | D11 / D12 / D13 |
| Injector 4 / 5 / 6 | OC3A / OC3B / OC3C | PE3 / PE4 / PE5 | D5 / D2 / D3 |
| Coil 1 / 2 / 3 | OC4A / OC4B / OC4C | PH3 / PH4 / PH5 | D6 / D7 / D8 |
| Coil 4 / 5 / 6 | OC5A / OC5B / OC5C | PL3 / PL4 / PL5 | D46 / D45 / D44 |
| Crank (timestamp) | ICP5 | PL1 | D48 |
| Cam | ICP4 (or INTn) | PL0 | D49 |
| Knock IC (TPIC8101) | SPI + GPIO (INT/HOLD, CS) | PB0–PB3 + GPIO | D50–D53 |
| Idle valve open / close | OC2A / OC2B | PB4 / PH6 | D10 / D9 |
| Tuning link | UART0 | PE0 / PE1 | D0 / D1 (USB) |
| System tick (1 kHz) | Timer0 (ISR only, no pin) | — | — |

Outputs are numbered by **cylinder**, not by firing order; the firing order
1-5-3-6-2-4 lives in firmware as a per-cylinder TDC angle table (0°, 480°, 240°, 600°, 120°, 360°
for cylinders 1–6). This keeps wiring and the tuning GUI intuitive.

**Shared time base.** Timers 0, 1, 3, 4 and 5 share one prescaler. At start-up we halt them
(`GTCCR = TSM | PSRSYNC`), zero all `TCNTn`, then release them together. From then on all four
16-bit counters hold the same value, so a crank timestamp captured in `ICR5` can be used directly
to schedule an event on `OCR1A`. We never write `TCNTn` again.

**Prescaler:** /64 → **4 µs per tick**, 262 ms wrap. 4 µs = 0.17° at 7,200 RPM and 0.16 % of a
2.5 ms idle pulse — well below anything measurable on an engine — and the long wrap means no
overflow-extension logic in the scheduler. (If finer RPM resolution is needed later, /8 = 0.5 µs is
the alternative; it needs software overflow extension for cranking-speed events.)

**Development hardware:** every pin above is broken out on an Arduino Mega (2560 or 1280) board.
All firmware development up to the first engine start can happen on a stock Mega plus the trigger
simulator and a logic analyser, before any custom PCB exists. (The ATmega2560 is pin- and
register-compatible with the 1280 and has twice the flash; SRAM and EEPROM are the same. It is a
drop-in upgrade if flash ever gets tight — it shouldn't.)

### 2.2 Inputs and signal conditioning

| Input | M52 sensor | Conditioning |
|-------|-----------|--------------|
| Crank position | Inductive (VR) on the 60-2 wheel *(verify on car: MS41-era is VR, M52TU/MS42+ is Hall)* | MAX9926 channel 1, adaptive threshold, differential input, shielded twisted pair |
| Cam position | Hall effect on intake cam *(verify)* | Pull-up, RC filter, Schmitt input, clamp. MAX9926 channel 2 footprint as a jumper option in case it's VR. |
| Knock ×2 | Piezo, sensor 1 = cyl 1–3, sensor 2 = cyl 4–6 | **TI TPIC8101** dual-channel knock processor (programmable bandpass, gain, integrator; SPI). Check stock/lifecycle before committing; fallback below. |
| TPS | Potentiometer | RC filter, clamp, ratiometric |
| MAP | External 1-bar (NA) or 2-bar (future-proof) sensor | RC filter, clamp, ratiometric |
| MAF (optional) | Stock HFM | Spare ADC channel |
| CLT / IAT | NTC thermistors (CLT sensor is dual-element; ECU uses one element) | 2.49 kΩ pull-up to sensor 5 V, RC filter |
| Wideband O2 | Bosch LSU 4.9 | External controller with 0–5 V output for v1 (on-board CJ125 is a possible rev B feature) |
| Battery voltage | — | Divider + filter (used for injector dead-time and dwell compensation) |

ADC reference = the same 5 V rail that powers the sensors, so supply drift cancels (ratiometric).

### 2.3 Outputs and drivers

| Output | Driver | Notes |
|--------|--------|-------|
| 6 × injector | Protected low-side switch (e.g. Infineon HITFET family) or logic-level MOSFET + zener flyback clamp | M52 injectors are high-impedance (~12–16 Ω, verify) → saturated drive, no peak-and-hold needed. Zener clamp (~36–40 V) closes the injector faster than a plain diode. |
| 6 × ignition | Logic-level ignition IGBT with ~400 V clamp (e.g. ISL9V3040 class) | Gate pull-downs so coils are **off** while the MCU is in reset or unprogrammed. Firmware enforces max dwell. |
| Idle valve (2 coils) | 2 × low-side MOSFET + flyback diode | Driven as a complementary PWM pair from Timer2. |
| VANOS solenoid | Low-side MOSFET + flyback diode | On/off. |
| Fuel pump relay, main relay, CEL, A/C cut | Low-side MOSFET / protected switch | Relay coils need flyback diodes. |
| Tach out | Open-collector | Feeds the cluster tach input. |

### 2.4 Power, protection, EMI

- Reverse-battery protection (P-MOSFET ideal diode) and load-dump TVS (SMCJ class) at the 12 V input.
- Pre-regulator (automotive buck or LM2940-class LDO) → clean 5 V logic rail; **separate, protected 5 V sensor supply** (sensor wiring shorts to ground must not reset the MCU).
- Star ground: separate return paths for power ground (injectors/coils/relays, heavy copper, straight to engine ground) and signal/sensor ground.
- 4-layer PCB with solid ground plane; ignition drivers physically far from VR, knock and ADC inputs.
- Every external input: series resistor + RC + clamp. Every output: flyback handled locally.
- Hardware **watchdog** enabled in firmware; on reset all outputs fall to the safe state via pull-downs.
- Brown-out detector enabled (4.3 V) so cranking voltage dips reset cleanly rather than corrupting state.

### 2.5 Connection to the car

Decision needed (§7): either a **plug-and-play board** that fits the stock DME connector (connector
harvested from a dead MS41), or an **adapter harness** from the stock DME connector to the custom ECU's
connector. Either way, swapping back to the stock DME must take minutes — this is the safety net for
the whole project.

---

## 3. Firmware architecture

### 3.1 Toolchain and layout

- avr-gcc + avr-libc, CMake (or plain Make), avrdude. No Arduino core, no `digitalWrite`/`analogRead`.
- Host build of all engine-math modules with the native gcc for unit tests (`firmware/test/`).
- Bootloader: stk500v2-compatible (as on the Mega) so the tuning software can flash over USB.

```
firmware/
  src/
    hal/        timers, gpio, adc, uart, spi, eeprom, watchdog — the only code that touches registers
    decoder/    60-2 + cam decoder, sync state machine, RPM, angle ↔ time
    sched/      ignition + injection event scheduler (ISR side + main-loop side)
    fuel/       VE lookup, pulse-width equation, corrections, accel enrichment, closed loop
    ign/        advance lookup, dwell, corrections, rev limiter
    knock/      TPIC8101 driver, windowing, per-cylinder retard/recover
    sensors/    ADC scaling, filtering, thermistor tables, fault detection
    aux/        idle valve, VANOS, fuel pump, fan, tach, CEL
    comms/      framed binary protocol, realtime packet, page read/write/burn, loggers
    storage/    tune pages in RAM, EEPROM burn with CRC, versioning
    safety/     limp-home, sensor fallbacks, overdwell, watchdog
    main.c      init + cooperative main loop
  test/         host-side unit tests (decoder replay, interpolation, fuel math)
```

### 3.2 Crank / cam decoding

**Input Capture ISR (ICP5, every tooth):**

1. Read `ICR5` (hardware-latched edge time).
2. `gap = now - last`. If `gap > 1.5 × previous_gap` → this is the tooth after the missing gap → tooth index = 0, revolution toggles.
3. Validate tooth count (must be 58 between gaps; otherwise lose sync and count the error).
4. If cam sync is pending, sample cam state at the gap to set phase (0–360° vs 360–720°).
5. Check the event table: for each output whose reference tooth == this tooth, arm its Output Compare (§3.3).
6. Knock window: open/close the TPIC8101 integrator on window-start/end teeth (6° resolution is fine for windows).
7. Store tooth time for the main loop and optional tooth logger.

No divisions, no 32-bit multiplies, no floats in the ISR.

**Sync state machine:** `NO_SYNC → CRANK_SYNC (360°, found gap) → FULL_SYNC (720°, cam phase known)`.
- In CRANK_SYNC the engine already runs: **wasted spark** on companion pairs (1/6, 5/2, 3/4 — 360° apart
  in this firing order) and **paired/batch injection**. This gives fast starts.
- On FULL_SYNC switch to full sequential. Losing cam sync drops back to wasted-spark/batch, not to engine off.

**Cam pattern:** the decoder will support both a single cam pulse (edge counted relative to the gap) and a
"half-moon" pattern (level sampled at the gap). Which one the M52 intake cam has — and how far single
VANOS moves the edge relative to the gap — gets measured on the car with a scope in Phase 0. Sampling
point must keep margin from the edge across the full VANOS range.

**TDC offset:** angle from the missing-tooth gap to cylinder 1 TDC is a calibration value, measured on the
car and confirmed with a timing light (fixed-timing mode in the tuning software).

### 3.3 Output scheduling (the hard part)

**Main loop** (once per revolution or faster, per cylinder) computes:
- Ignition: `spark_angle = TDC_cyl − advance`, `dwell_start_angle = spark_angle − dwell_time × degrees_per_µs`.
- Injection: `end_of_injection_angle` from table, `start_angle = EOI − PW × degrees_per_µs`.
- Converts each event angle into `(reference_tooth, delay_ticks)` where the reference tooth is the last
  physical tooth before the event (taking the missing-tooth gap into account).
- Writes the results into the *inactive* half of a double buffer and flips a flag (atomic single-byte write).

**ISR side** (crank ISR at the reference tooth):
- `OCRnx = ICR5 + delay_ticks`, compare mode = *set pin on match* (start dwell / open injector).
- Guard: if the target is already in the past or within a few ticks of `TCNTn`, use Force Output Compare to
  act now (or skip spark this cycle) — never let a compare wait for the next 262 ms wrap.

**Output Compare ISR** (on the start edge):
- `OCRnx += duration_ticks` (dwell time or injector pulse width), compare mode = *clear pin on match*.
- The closing edge is then also hardware-timed.

**Safety:**
- Every start edge always has its end edge armed in the same ISR → a coil can't be left dwelling.
- Max-dwell clamp; dwell/injection disabled when RPM = 0 or sync is lost; fuel pump off 1 s after last tooth.
- Injector pulses longer than the cycle (duty > ~85 %) are clamped and flagged.

### 3.4 Engine math (main loop, fixed-point integers)

**Fuel (speed density):**
```
PW = REQ_FUEL × VE(rpm, map) × MAP × corrections + dead_time(vbatt)
corrections = warmup(clt) × iat_density(iat) × accel_enrich(dTPS, dMAP) × closed_loop(afr_target − afr) × baro
```
- `REQ_FUEL` derived from 2,494 cc / 6 cyl and measured injector flow.
- AFR target table (rpm × load) drives the closed loop and the tuner's VE auto-tune.
- Cranking fuel and after-start enrichment tables.
- Injection timing table: end-of-injection angle (rpm × load).

**Ignition:**
- Advance table (rpm × load) + CLT/IAT trims + per-cylinder knock retard + idle timing control.
- Dwell vs battery voltage table, reduced at very high RPM so dwell + spark fits the cycle.
- Rev limiter: soft (spark retard) → hard (fuel cut / spark cut, staggered per cylinder).

**Tables:** 16 × 16 bilinear interpolation, 8-bit values with per-table scale/offset, 16-bit axes.

**Aux:** idle valve closed-loop on RPM target vs CLT; VANOS on/off by RPM × load table; fan control; A/C idle-up.

### 3.5 Knock control

- M52 bore 84 mm → first circumferential knock mode ≈ **6.6–6.8 kHz** as a starting bandpass; verify by
  logging the sensor spectrum on the car.
- Per firing cylinder: select sensor (cyl 1–3 → sensor 1, 4–6 → sensor 2), open integration window
  (e.g. 10°–70° ATDC, tunable), close it, read the integrator over SPI in the main loop after the window.
- Per-cylinder background noise model (learned vs RPM); knock = level > noise × threshold(rpm).
- Response: per-cylinder retard step proportional to intensity, slow recovery per cycle, global
  safety enrichment if multiple cylinders knock.
- Knock logger channel streamed to the tuner (per-cylinder level, noise, retard).
- **Fallback if TPIC8101 is unobtainable:** analogue bandpass + rectifier + integrator with a gated window,
  read by the MCU's ADC. Works, less flexible.

### 3.6 Timing and resource budget (7,200 RPM)

| Item | Per engine cycle (16.67 ms) |
|------|-----------------------------|
| 120 crank ISRs × ≤ 10 µs | ≤ 1.2 ms |
| 24 output-compare ISRs × ~3 µs | ~0.07 ms |
| Knock window + ADC + UART ISRs | ~0.3 ms |
| **Interrupt load** | **≈ 9 %** |
| Main loop available | > 85 % |

Main loop target: full fuel/ignition recompute for all six cylinders at ≥ once per revolution at redline (8.3 ms).
Fixed-point math, no floats, 32-bit divides limited to once-per-revolution RPM calculation.

Memory (estimate): flash 30–50 KB of 128 KB; SRAM ~2 KB working tune + ~1 KB buffers + stack of 8 KB;
EEPROM ~2–3 KB of 4 KB for tune pages + CRC. EEPROM is written only on "burn" (100k write endurance).

### 3.7 Main loop scheduling

Cooperative, driven by the 1 kHz Timer0 tick:

| Rate | Tasks |
|------|-------|
| Every pass | Comms parsing, recompute event schedules when a new revolution completed |
| 1 kHz | ADC fast channels (TPS, MAP), accel enrichment |
| 100 Hz | Fuel + ignition corrections, closed-loop AFR, idle valve |
| 10 Hz | CLT, IAT, battery, VANOS, fan, fuel pump, sensor fault checks |
| 1 Hz | Housekeeping, error counters |

---

## 4. Communication protocol (ECU ↔ tuner)

- UART0 over USB at **500,000 baud** (0 % error at 16 MHz; 115,200 has 2.1 % error on AVR).
- Binary framing: `[0xA5][len][cmd][payload…][crc16]`, response mirrors command with status byte.
- Commands: `HELLO/VERSION` (firmware + definition signature), `GET_REALTIME`, `READ_PAGE(page, off, len)`,
  `WRITE_PAGE(page, off, data)` (live, RAM only), `BURN(page)` (to EEPROM), `START/STOP_STREAM(rate)`,
  `TOOTH_LOGGER`, `COMPOSITE_LOGGER` (crank+cam edges), `KNOCK_LOGGER`, `SET_TEST_MODE` (fixed timing,
  injector/coil output tests), `REBOOT_TO_BOOTLOADER`.
- Realtime packet ~64 bytes at 50 Hz ≈ 3.2 KB/s, well within the link.

**Single source of truth:** one definition file (`definitions/ecu.yaml`) describes every page, table, axis,
scalar, scale, unit and realtime channel. A code generator produces:
- the C structs + offsets for the firmware (`firmware/src/generated/`),
- the tuner's table/gauge definitions,
so firmware and GUI can never disagree about memory layout. (This plays the role TunerPro's XDF plays —
but generated, not hand-written.) Firmware reports the definition's hash; the tuner refuses to write to a
mismatched ECU.

---

## 5. Tuning software (`tuner/`)

**Stack:** Python 3.12 + PySide6 (Qt) + pyqtgraph (2D/3D plotting) + pyserial, packaged with PyInstaller for
Windows/Linux/macOS. Chosen for speed of development and good table/plot widgets; the protocol is plain
enough that a later rewrite (C++/Qt, C#) would be straightforward if ever needed.

**Feature roadmap:**

| Milestone | Features |
|-----------|----------|
| T1 — Connect & view | Serial connect, version/definition check, realtime gauges + numeric dashboard, sync/error status |
| T2 — Edit | Definition-driven tree of tables/scalars; 2D/3D table editor (heat-map colours, select ranges, +/−, set, scale %, interpolate, smooth, copy/paste to Excel); live write to RAM; burn; undo/redo |
| T3 — Tune files | Save/load tune file (JSON with definition hash), offline editing, compare two tunes (diff view), import older tunes across definition versions |
| T4 — Logging | Record datalogs, log viewer with time-series graphs, scrub/playback, table cursor trace showing where the engine is operating |
| T5 — Diagnostics | Tooth logger + composite logger (crank/cam edges) view, knock per-cylinder view, output test panel (fire individual injector/coil), fixed-timing mode for timing-light calibration |
| T6 — Auto-tune | VE analyser: from logs, filter steady-state cells and compute VE corrections from AFR error; proposes changes cell-by-cell for approval |
| T7 — Firmware | Flash firmware via bootloader (avrdude) from inside the app |

**Optional risk reducer:** the same definition file could also generate a TunerStudio-style `.ini`, giving a
mature fallback GUI for the first engine start while our tuner is still young. Worth deciding before Phase 4.

---

## 6. Development phases and milestones

Each phase ends with a concrete, testable result. Phases 1–5 need no car and no custom PCB.

| Phase | Goal | Done when |
|-------|------|-----------|
| **0. Car reconnaissance** | Measure the real engine | Pinout of the stock DME connector documented; crank/cam waveforms scoped at cranking and idle (VR vs Hall, cam pattern, VANOS shift); gap-to-TDC offset estimated; injector and coil primary resistance measured; knock sensor signals captured; car options checked (manual/auto, ASC, EWS — see §8) |
| **1. Foundations** | Firmware skeleton on an Arduino Mega | Build system, HAL, UART protocol skeleton, 1 kHz tick, ADC, watchdog; trigger simulator generates 60-2 + cam from 100 to 8,000 RPM |
| **2. Decoder** | Sync on the simulated engine | Correct tooth index, sync, RPM; sync survives acceleration ramps and lost/extra-tooth injection; tooth logger streams to PC; host unit tests replay captured car waveforms from Phase 0 |
| **3. Scheduler** | Hardware-timed outputs | Logic analyser shows all 12 outputs at the right angle and duration across RPM sweep; jitter < 1 tick; wasted-spark → sequential transition; overdwell protection proven by yanking the trigger signal mid-dwell |
| **4. Engine math + tuner MVP** | Tables drive outputs | Fuel and ignition from tables and sensors (pots on the bench); tuner T1–T3: live gauges, edit tables, burn, save tune |
| **5. Knock bench** | Knock chain works | TPIC8101 on a breadboard; injected 6.7 kHz bursts in the window are detected for the right cylinder, out-of-window ones ignored; knock logger in the tuner |
| **6. PCB rev A** | Real hardware | Schematic + layout, ordered, assembled; bring-up checklist: power, protection, each input, each driver on resistive/real loads, real coils into spark gaps on the bench |
| **7. Sniffer mode on the car** | Validate decoder on the real engine with zero risk | Custom ECU wired **in parallel** to stock crank/cam/knock/sensors (outputs disconnected) while the stock DME runs the car; logs show solid sync, correct RPM and sensible sensor values through drives |
| **8. First start** | Engine runs on the custom ECU | Base tune derived from displacement/injectors; timing confirmed with timing light; starts, idles, revs; failsafes verified |
| **9. Tuning & refinement** | Drives properly | Closed-loop AFR, idle control, VANOS, warm-up/accel enrichment, knock calibration (noise learning, thresholds), VE auto-tune on the road/dyno, tuner T4–T7 |

Suggested parallelisation: hardware design (Phase 6) can start right after Phase 0 measurements, while
firmware Phases 1–5 progress on the dev board.

---

## 7. Open decisions

1. **Load sensing:** speed-density with a MAP sensor (recommended) or stock MAF (HFM)? Or both, blendable?
2. **Car connection:** plug-and-play board in the stock DME connector, or adapter harness?
3. **Ignition:** keep the stock dumb coils with on-board IGBTs (recommended), or swap to smart coils?
4. **Wideband:** external controller with 0–5 V output for v1 (recommended), or on-board CJ125?
5. **Tuning GUI stack:** Python + PySide6 (recommended), or C++/Qt / C# WPF?
6. **TunerStudio `.ini` as a fallback** for the first start — yes or no?
7. **Car specifics:** model year / DME (MS41?), manual or automatic, ASC/traction control?

## 8. Risks and things to verify early

| Risk | Mitigation |
|------|-----------|
| Automatic gearbox (EGS) or ASC expect data from the DME | Check in Phase 0. If present, may need an extra interface (possibly CAN via an MCP2515 on SPI) or accept loss of that function. |
| EWS immobiliser interaction with starter / DME | Verify the starter still works with the stock DME unplugged; the custom ECU itself ignores EWS. |
| EMI resets or false teeth from ignition noise | Hardware filtering (§2.4), decoder tooth-validity checks, error counters in realtime data, sniffer-mode testing before driving. |
| TPIC8101 availability | Confirm stock early; analogue fallback design (§3.5). |
| Coil/injector damage during development | Gate pull-downs, max-dwell clamp, end edge always armed with start edge, watchdog, bench tests on real coils before the car. |
| Lean condition / engine damage during early tuning | Conservative base tune (rich, retarded), wideband from day one, rev limit low until tuned, knock control validated in sniffer mode first. |
| Being unable to drive the car mid-project | Stock DME swap-back in minutes via plug-and-play board or adapter harness. |

---

## 9. Proposed repository layout

```
docs/          plan, hardware notes, Phase 0 measurements, bring-up checklists
definitions/   ecu.yaml — single source of truth for tune layout + realtime channels
firmware/      bare-metal ATmega1280 firmware + host unit tests
tuner/         PySide6 tuning software
tools/stim/    trigger simulator firmware
tools/codegen/ definition → C header + tuner definitions
hardware/      KiCad project
```

## 10. Immediate next steps

1. Answer the open decisions in §7.
2. Phase 0: borrow/buy a cheap USB scope or logic analyser; capture crank, cam and knock signals with the
   stock DME running; record the DME connector pinout.
3. Phase 1 in parallel: set up the firmware build, HAL timers with synchronised start, and the trigger simulator.
