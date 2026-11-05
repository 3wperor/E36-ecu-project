# E36 323i (M52) Custom Bare-Metal ECU — Project Plan

Status: **planning** · Target: BMW E36 323i, M52B25, full sequential fuel + spark, knock control, custom tuning GUI.

**The car:** 1997 UK-market E36 323i — M52B25 (pre-TU), Siemens MS41 DME, single (intake) VANOS,
ASC traction control, two knock sensors, no OBD-II check-engine light, **manual gearbox**.
**The reason:** a plenum-ITB intake built from **motorcycle throttle bodies** inside an airbox, with the
injectors mounted in the ITBs aimed at the intake valves, metered by the **stock MAF**, with full tuning
control. Both **narrowband and wideband** lambda control are supported. Sensor inventory and what to measure: [CAR.md](CAR.md).

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
| 2 | Crank ISR must take **< 1–2 µs** | ISR entry/exit alone on AVR with avr-gcc is ~1.5–3 µs. **Measured (Phase 2, simavr): 13.5 µs average, 15.8 µs worst case** for the complete decoder (gap detection, noise rejection, sync, cam phase). | Fine: ~10 % CPU at 7,200 RPM. A regression test fails above 17 µs. |
| 3 | "High-priority" Input Capture interrupt | **AVR has no interrupt priorities and no nesting by default.** All ISRs are equal; a long ISR delays every other ISR. | Every ISR (UART, ADC, SPI, timers) must be short. Precision comes from hardware: Input Capture **latches the timestamp in hardware** regardless of ISR latency, and Output Compare **toggles the pin in hardware**. ISR latency only affects *arming* the next event, not when it fires. |
| 4 | Crank ISR *only* logs a timestamp and counts teeth | Events must be scheduled relative to a recent tooth to stay accurate during acceleration, so the ISR also does a cheap table lookup and arms an Output Compare when "this tooth is the reference tooth for an event". | All heavy math stays in the main loop, which pre-computes `(reference_tooth, delay_ticks)` pairs; the ISR does only a compare + add + register write. |
| 5 | Main loop writes pulse width to the timer registers | On AVR, all 16-bit timer registers share **one TEMP byte**. If the main loop and an ISR both access 16-bit timer registers, values get corrupted. Also, in Normal mode OCR writes take effect immediately, so a late write can miss and wait a full timer wrap. | **Only ISRs touch timer registers.** Main loop hands schedules to ISRs through a double-buffered struct. ISR checks "is the target time already passed or too close?" and fires immediately (Force Output Compare) or skips instead of waiting 262 ms with a coil dwelling. |
| 6 | 6 ignition drivers "or logic-level triggers for smart coils" | The M52's stock coil-on-plug coils are **"dumb" coils** (no built-in igniter); the stock MS41 drives them with IGBTs inside the DME. | Board needs 6 ignition IGBTs (400 V clamped, logic-level gate) unless we swap to smart coils. Dwell control is the firmware's job. |
| 7 | MAP **or** MAF | Stock M52/MS41 is MAF-based (HFM hot-film). | **Decided: stock MAF.** This suits a plenum-ITB build better than MAP: ITBs give a weak, noisy manifold vacuum signal, while a MAF on the airbox inlet sees the real airflow. It is blended with an Alpha-N (throttle × RPM) model for fast transients and low-RPM pulsation (§3.4, §3.8). |
| 8 | Brief lists fuel + spark only | The M52 also needs: **single VANOS** solenoid (on/off), **dual-coil idle control valve** (two PWM outputs), fuel pump relay, main relay, tach output, check-engine light, optionally A/C compressor cut. | Extra outputs on the PCB (§2.3). |

---

## 1. System overview

```
              ┌────────────────────────── Car ───────────────────────────┐
 Crank VR  ──►│ MAX9926 ch1 ─► ICP5 (PL1)                                │
 Cam Hall  ──►│ RC + clamp  ─► INT / PCINT                               │
 Knock 1,2 ──►│ TPIC8101 ◄─SPI─►                                         │
 MAF, TPS, CLT, IAT, WBO2, NBO2, Vbatt ─► ADC (ratiometric to sensor 5V) │
 Road speed (from ABS/ASC) ─► GPIO                                        │
 CAN (optional MCP2515) ◄─SPI─► ASC / gearbox, future                     │
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

Remaining I/O (dev-board mapping; the single source in code is `firmware/src/board.h`):

| Function | MCU pin | Mega pin | | Function | ADC | Mega pin |
|----------|---------|----------|-|----------|-----|----------|
| Fuel pump relay | PA0 | D22 | | MAF | ADC0 | A0 |
| Main relay | PA1 | D23 | | TPS 1 | ADC1 | A1 |
| VANOS solenoid | PA2 | D24 | | TPS 2 | ADC2 | A2 |
| Narrowband heater 1 | PA3 | D25 | | Coolant temp | ADC3 | A3 |
| Narrowband heater 2 | PA4 | D26 | | Intake air temp | ADC4 | A4 |
| Tach out | PA5 | D27 | | Wideband (0–5 V) | ADC5 | A5 |
| Warning lamp | PA6 | D28 | | Narrowband bank 1 | ADC6 | A6 |
| A/C compressor cut | PA7 | D29 | | Narrowband bank 2 | ADC7 | A7 |
| Purge valve | PC0 | D37 | | Battery voltage | ADC8 | A8 |
| Fuel consumption (TI) out | PC1 | D36 | | MAP (balance rail, optional) | ADC9 | A9 |
| Knock IC INT/HOLD | PC2 | D35 | | Spare | ADC10–15 | A10–A15 |
| Knock IC chip select | PB0 | D53 | | | | |
| Road speed in | PD0 (INT0) | D21 | | | | |

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
| **MAF (primary load)** | Stock Siemens HFM hot-film, 0–5 V | RC filter (light: keep the response fast), clamp, ratiometric. Sampled several times per intake event and averaged per cylinder (§3.8). |
| TPS ×2 | Stock potentiometer now; motorcycle ITB TPS later (one per bank if two ITB sets are used) | RC filter, clamp, ratiometric. Becomes a primary input with ITBs (Alpha-N, acceleration enrichment). Two inputs let both banks be monitored: average them, and flag a fault if they disagree (linkage or sensor failure). |
| MAP (optional, spare) | Not fitted | Spare ADC channel; useful later as a barometer or for an ITB vacuum balance rail |
| CLT / IAT | NTC thermistors (CLT sensor is dual-element; ECU uses one element, the gauge the other) | 2.49 kΩ pull-up to sensor 5 V, RC filter. The IAT has to be relocated into the ITB airbox. |
| Wideband O2 | Bosch LSU 4.9 via an external controller (not yet bought) | 0–5 V analogue input. **Needed before the first start** (Phase 8), not before. |
| Narrowband O2 ×2 | Stock pre-cat lambda sensors, one per bank (verify count) | High-impedance input (~1 MΩ) + filter, 0–1 V range. Heaters switched by low-side PWM outputs. Both lambda modes are supported (§3.9). |
| Road speed | Square-wave from the ABS/ASC module | Input-capture/PCINT; used for idle logic and launch/limiter features |
| Battery voltage | — | Divider + filter (used for injector dead-time and dwell compensation) |

ADC reference = the same 5 V rail that powers the sensors, so supply drift cancels (ratiometric).

### 2.3 Outputs and drivers

| Output | Driver | Notes |
|--------|--------|-------|
| 6 × injector | Protected low-side switch (e.g. Infineon HITFET family) or logic-level MOSFET + zener flyback clamp | M52 injectors are high-impedance (~12–16 Ω, verify) → saturated drive, no peak-and-hold needed. Zener clamp (~36–40 V) closes the injector faster than a plain diode. |
| 6 × ignition | Logic-level ignition IGBT with ~400 V clamp (e.g. ISL9V3040 class) | **Decided: keep stock-type M52 coil-on-plug coils** (see §2.6). Gate pull-downs so coils are **off** while the MCU is in reset or unprogrammed. Firmware enforces max dwell. Each channel also has a jumper/assembly option to output a 5 V logic signal instead, so smart coils can be fitted later without a new board. |
| 2 × narrowband heater | Low-side MOSFET, PWM | Soft warm-up ramp to avoid cracking a wet sensor |
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

**Decided: plug-and-play through a home-made adapter harness.**

```
 car harness ──► [DME-side connector from a donor MS41] ──► adapter harness ──► [sealed connector] ──► custom ECU
  (untouched)         (on a small breakout PCB)               (you build it)      (e.g. TE Superseal/DTM)
```

- Buy a used/dead **Siemens MS41** DME (cheap at breakers/eBay; must match the car's connector). Its
  connector header is removed and mounted on a small breakout board, so the car's own harness plugs in
  exactly as it does into the stock DME. **No cutting of the car's loom.**
- The adapter harness runs from the breakout to the custom ECU's own sealed automotive connector. The ITB
  conversion's extra wiring (new TPS, relocated IAT, idle valve plumbing) joins at this harness too.
- Swapping back to the stock DME is: unplug adapter, plug stock DME back in — minutes. This is the safety net
  for the whole project and for getting home if anything goes wrong.
- Exact pin assignments come from the E36 Electrical Troubleshooting Manual (ETM) for the car's build date,
  confirmed with a multimeter in Phase 0 and recorded in `docs/CAR.md`.

### 2.6 Coil choice

You asked for the cheapest *and* most reliable option. That is the **stock M52 coil-on-plug coils**, driven
by IGBTs on our board, exactly as the factory did it:

- **Cheapest:** the ignition transistors cost a few pounds for all six. A replacement M52 coil from an
  OE supplier (Bremi, Bosch, Beru) is inexpensive, and coils that are healthy now can simply be kept.
- **Most reliable:** factory fit, factory plug wells, factory connectors, no brackets, adaptors or new leads.
- Replace any coil that's cracked or shows a misfire, and fit new plugs, before the first start.

Smart coils (built-in igniter, such as VAG "R8" coils on adaptor kits) only make sense later if the stock
coils can't deliver enough spark at high RPM. The board keeps that option open (§2.3).

---

## 3. Firmware architecture

### 3.1 Toolchain and layout

- avr-gcc + avr-libc, CMake (or plain Make), avrdude. No Arduino core, no `digitalWrite`/`analogRead`.
- Host build of all engine-math modules with the native gcc for unit tests (`firmware/test/`).
- Whole-firmware tests in the simavr simulator (`make sim-test`). Hardware-timed outputs are checked via
  the timer register writes, because simavr mis-models output-compare pins in Normal mode.
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

### 3.2 Crank / cam decoding (implemented in Phase 2: `firmware/src/decoder/`)

**Input Capture ISR (ICP5, every crank tooth):**

1. Read `ICR5` (hardware-latched edge time, 4 µs ticks).
2. Reject edges that arrive impossibly early as noise: under ¼ of the previous interval always, and once
   synced also under ⅝ of the tooth period (a running engine can't speed up that much in 6°).
3. Gap test: interval > 1.5 × previous interval → this edge is tooth 0. The next tooth is compared against
   the gap itself (comparing against the pre-gap tooth can lock up after a false gap).
4. Count teeth: exactly 58 between gaps, otherwise sync is dropped and re-acquired.
5. Cam phase at each gap (below). Phase 3 adds: arm Output Compares whose reference tooth this is (§3.3),
   and Phase 5 the knock window open/close.
6. Optional one-shot tooth logger (64 teeth) for the tuner.

No divisions, no floats in the ISR. RPM (one division) is computed in the main loop.

**Sync state machine:** `NONE → PENDING (first gap) → CRANK (next gap exactly 58 teeth later: 360° known)
→ FULL (cam phase confirmed: 720° known)`.
- Waiting for a second, correctly counted gap before claiming sync costs one revolution of cranking but
  means a noise pulse can never make the engine fire at a wrong angle. Simulated worst case from rest:
  FULL sync after 3 revolutions.
- In CRANK the engine can already run: **wasted spark** on companion pairs (1/6, 5/2, 3/4 — 360° apart
  in this firing order) and **paired injection**.
- On FULL switch to full sequential. Cam faults drop back to CRANK (engine keeps running), never to off.

**Cam pattern:** one rule covers both a single-pulse and a half-moon cam wheel, because both give one
rising edge per 720°: the revolution that starts at the gap *after* the cam edge is rev 0. FULL needs two
consecutive revolutions that alternate (cam / no cam); in FULL every revolution must keep alternating or it
drops to CRANK. The cam edge must stay well away from the gap across the full VANOS range. The decoder
reports where it sees the cam edge (`cam_pos`, in teeth) so this can be checked on the car.

**TDC offset:** angle from the missing-tooth gap to cylinder 1 TDC is a calibration value, measured on the
car and confirmed with a timing light (fixed-timing mode in the tuning software).

**Verified (simulation):** steady 100–9,000 RPM (RPM within 0.02 %), starts from rest from every crank
position, cranking with ±35 % compression ripple, 800→7,200 RPM in 2 s and 1,000↔7,000 RPM in 0.4 s,
dropped tooth, noise spikes, missing/spurious cam edges, stall and restart. Worst-case ISR 15.8 µs.

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

**Load = air mass per cylinder (mg/stroke), from the MAF blended with Alpha-N:**
```
maf_gps      = MAF_curve(volts) × MAF_trim(rpm, tps)                 // stock HFM curve + correction table
air_maf      = maf_gps × 20000 / rpm                                  // mg per cylinder per intake event (6 cyl, 4-stroke)
air_alphaN   = VE_alphaN(rpm, tps) × cyl_volume × air_density(iat, baro)
w            = blend(rpm, tps)                                        // 0…1 table: how much to trust the MAF
air_per_cyl  = w × air_maf + (1 − w) × air_alphaN
```

**Fuel:**
```
fuel_mg = air_per_cyl / AFR_target(rpm, load)
PW      = fuel_mg / injector_flow × warmup(clt) × accel_enrich(dTPS) × closed_loop(afr) + dead_time(vbatt)
```
- Injector flow from the motorcycle injectors' data (measured if unknown), corrected for actual fuel
  pressure; 2,494 cc / 6 cyl ≈ 416 cc per cylinder.
- On the stock intake the blend table is ~100 % MAF everywhere. With ITBs it shifts toward Alpha-N where
  the MAF struggles (§3.8).
- AFR target table (rpm × load) drives the closed loop and the tuner's MAF-trim/VE auto-tune.
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
| 120 crank ISRs × 13.5 µs average (measured, Phase 2; scheduling adds a few µs in Phase 3) | ~1.6 ms |
| 24 output-compare ISRs × ~3 µs | ~0.07 ms |
| Knock window + ADC + UART ISRs | ~0.3 ms |
| **Interrupt load** | **≈ 12 %** (to be re-measured with Phase 3 outputs running) |
| Main loop available | > 80 % |

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

### 3.8 Plenum-ITB specifics

What changes when the stock throttle body and manifold are replaced with ITBs inside an airbox:

- **MAF pulsation.** Each ITB opens almost straight into the airbox, so the airflow the MAF sees pulses
  with every intake stroke, worst at low RPM and wide throttle. A hot-film MAF can't tell forward from
  reverse flow, so it over-reads when air bounces back. Countermeasures: a large airbox, MAF well
  upstream of the trumpets, **crank-synchronous MAF sampling** (several ADC samples per 120° intake event,
  averaged per cylinder), a MAF trim table, and the Alpha-N blend wherever the MAF signal is bad.
- **Throttle response.** With ITBs the throttle directly controls each cylinder, so airflow changes faster
  than the MAF reading. Throttle-rate acceleration enrichment and the Alpha-N blend cover the transient.
- **MAF range.** An ITB engine may breathe beyond what the 2.5 L HFM housing can measure (signal flat-lines
  near 5 V at high RPM). The tuner logs MAF volts against RPM; if it saturates, use a larger housing
  (for example the 2.8's) with its own curve, or let Alpha-N take over at the top end.
- **Idle.** ITBs leave no single throttle to bypass. Options, both supported: route the stock idle valve
  into a vacuum balance rail that joins all six runners after the throttles; and/or ignition-timing idle
  control (fast and effective with ITBs).
- **Sensors to move/add:** TPS onto the ITB linkage; IAT into the airbox; optionally a MAP sensor on the
  balance rail (spare input) for diagnostics and throttle-sync checks.
- **Motorcycle throttle bodies:**
  - *Banks and linkage.* Bike ITBs usually come in sets of 4 (or 3 on triples). Six cylinders means two
    sets (e.g. two 4-throttle sets cut down to 3, or two triples), joined by a linkage and synchronised with
    vacuum gauges. Hence the two TPS inputs.
  - *Secondary butterflies* (fitted to many sports bikes) must be removed or locked fully open.
  - *Spacing.* Bike bore spacing is narrower than the M52's, so the runners/adaptors have to splay out
    to the head ports. This is mechanical work outside the ECU, but runner length affects the VE map.
  - *TPS resolution.* With ITBs nearly all the airflow change happens in the first ~20 % of throttle. The
    TPS axis of every Alpha-N table is therefore bunched at small openings (e.g. 0, 1, 2, 3, 5, 7, 10, 15, 20,
    30 … 100 %), and the TPS signal is filtered lightly so small movements still register.
- **Injectors in the ITBs, aimed at the valves:**
  - *Sizing.* Rule of thumb for ~200 hp at the crank (BSFC 0.5 lb/hp/h, 85 % max duty): ≥ ~210 cc/min
    per injector *at the actual fuel pressure*. Many bike injectors are close to that, so check the part
    number's flow. Flow scales with √pressure: going from a 3 bar rating to the M52's 3.5 bar adds ~8 %.
  - *Impedance.* Our drivers are saturated (high-impedance, ~12 Ω+) drivers. Most modern bike
    injectors are high-impedance, but confirm by measurement. Low-impedance injectors would need ballast
    resistors or peak-and-hold drivers.
  - *Connectors* differ from BMW's; the adapter harness takes care of that.
  - *Injection timing.* The end-of-injection table aims to finish spraying just before the intake valve
    opens at low load (fuel lands on the hot closed valve), moving to open-valve injection where the pulse is
    too long to fit.
  - *Fuel pressure regulator reference.* The stock regulator's vacuum port should go to the airbox
    (constant pressure across the injector), or to the balance rail. With an airbox reference the fuel
    equation needs no pressure correction; a MAP sensor on the balance rail can add one later.
- **Order of work:** the ECU is first brought up and tuned on the **stock intake** (known-good engine,
  MAF-only load), and only then is the ITB plenum fitted. Changing the ECU and the intake at the same time
  would leave two unknowns when something doesn't run right.

---

### 3.9 Lambda (O2) control — narrowband and wideband

Both sensor types are wired permanently; the tune selects how they are used.

| Mode | Uses | Closed loop where | How |
|------|------|-------------------|-----|
| **Off** | — | Never | Open loop from tables only (first start, fault fallback) |
| **Narrowband** | Two stock sensors, one per bank | Idle and light-load cruise, at stoichiometric only (λ = 1) | Classic switching PI controller per bank: ramp fuel lean until the sensor flips rich, then step back. **Bank trims**: bank 1 (cylinders 1–3) and bank 2 (cylinders 4–6) are corrected separately. |
| **Wideband** | LSU 4.9 controller | Anywhere enabled in a rpm × load table, at **any** target AFR (including rich at full load) | PI controller on the AFR error against the AFR target table; applied to all six cylinders. |
| **Both** | Wideband for control and logging; narrowbands as a cross-check | As wideband | If the wideband reading looks wrong (out of range, or disagrees with a switching narrowband at λ = 1), drop to narrowband mode or open loop and flag it. |

Common rules: closed loop only when warm (CLT threshold), after the sensors are ready, not during
acceleration enrichment, overrun fuel cut or rev limiting. Trims are limited (e.g. ±15 %); hitting the limit
sets a warning. Long-term learning (stored trim tables) is a Phase 9 option. The tuner's auto-tune uses the
wideband logs either way.

**Mounting:** the narrowbands stay in their stock positions (one per bank). The wideband is best where
both banks merge, so it sees all six cylinders. On one bank it only sees three.

## 4. Communication protocol (ECU ↔ tuner)

- UART0 over USB, 115,200 baud default (the rate Mega-based ECUs use with TunerStudio); higher rates
  (e.g. 500,000, 0 % error at 16 MHz) selectable for our own tuner.
- **Decided: TunerStudio-compatible framing** — the length-prefixed, CRC32-checked serial protocol that
  Speeduino and rusEFI also implement. Our own tuner speaks the same protocol, so the firmware has one
  protocol and two GUIs can talk to it (§5).
- Commands (mapped onto that protocol's command set): signature/version, realtime data, page read,
  page write (live, RAM only), burn to EEPROM, plus our extensions:
  `TOOTH_LOGGER`, `COMPOSITE_LOGGER` (crank+cam edges), `KNOCK_LOGGER`, `SET_TEST_MODE` (fixed timing,
  injector/coil output tests), `REBOOT_TO_BOOTLOADER`.
- Realtime packet ~64 bytes at 30–50 Hz ≈ 2–3.2 KB/s, within the 11.5 KB/s link.

**Single source of truth:** one definition file (`definitions/ecu.yaml`) describes every page, table, axis,
scalar, scale, unit and realtime channel. A code generator produces:
- the C structs + offsets for the firmware (`firmware/src/generated/`),
- the tuner's table/gauge definitions,
- a TunerStudio `.ini`,
so firmware and GUIs can never disagree about memory layout. (This plays the role TunerPro's XDF plays —
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
| T6 — Auto-tune | From logs, filter steady-state cells and compute MAF-trim and Alpha-N VE corrections from AFR error; proposes changes cell-by-cell for approval |
| T8 — Guided setup | Beginner-friendly wizards: sensor calibration (TPS min/max, thermistors), trigger offset with a timing light, injector setup, base tune generator from engine data |
| T7 — Firmware | Flash firmware via bootloader (avrdude) from inside the app |

**Decided: TunerStudio as a fallback.** Since you're new to tuning, the first engine start shouldn't depend on
brand-new GUI code. The generated `.ini` lets the free TunerStudio MS Lite (a mature, widely documented
tuner) connect to our ECU from day one. Our own tuner is the main tool and grows milestone by milestone.
Anything learned from TunerStudio's tutorials (table editing, logging, autotune) transfers directly.

---

## 6. Development phases and milestones

Each phase ends with a concrete, testable result. Phases 1–5 need no car and no custom PCB.

| Phase | Goal | Done when |
|-------|------|-----------|
| **0. Car reconnaissance** | Know exactly what's on the car | Every item in the [CAR.md](CAR.md) checklist filled in: sensor inventory confirmed, DME connector pinout from the ETM checked with a multimeter, crank/cam waveforms captured at cranking and idle (VR vs Hall, cam pattern, VANOS shift), injector and coil resistance measured, knock signals captured, starter-with-DME-unplugged test done |
| **1. Foundations** ✅ (in simulation) | Firmware skeleton on an Arduino Mega | Build system, HAL, UART protocol skeleton, 1 kHz tick, ADC, watchdog; trigger simulator generates 60-2 + cam from 20 to 9,000 RPM. Verified in simavr; still to verify on real boards (see `firmware/README.md`) |
| **2. Decoder** ✅ (in simulation) | Sync on the simulated engine | Correct tooth index, sync, RPM; sync survives acceleration ramps and lost/extra-tooth injection; tooth logger streams to PC; host unit tests replay captured car waveforms from Phase 0 |
| **3. Scheduler** | Hardware-timed outputs | Logic analyser shows all 12 outputs at the right angle and duration across RPM sweep; jitter < 1 tick; wasted-spark → sequential transition; overdwell protection proven by yanking the trigger signal mid-dwell |
| **4. Engine math + tuner MVP** | Tables drive outputs | Fuel and ignition from tables and sensors (pots on the bench); tuner T1–T3: live gauges, edit tables, burn, save tune |
| **5. Knock bench** | Knock chain works | TPIC8101 on a breadboard; injected 6.7 kHz bursts in the window are detected for the right cylinder, out-of-window ones ignored; knock logger in the tuner |
| **6. PCB rev A** | Real hardware | Schematic + layout, ordered, assembled; bring-up checklist: power, protection, each input, each driver on resistive/real loads, real coils into spark gaps on the bench |
| **7. Sniffer mode on the car** | Validate decoder on the real engine with zero risk | Custom ECU wired **in parallel** to stock crank/cam/knock/sensors (outputs disconnected) while the stock DME runs the car; logs show solid sync, correct RPM and sensible sensor values through drives |
| **8. First start (stock intake)** | Engine runs on the custom ECU | Wideband fitted; base tune from the stock HFM curve and injector data; timing confirmed with timing light; starts, idles, revs; failsafes verified |
| **9. Tuning & refinement (stock intake)** | Drives as well as stock | Closed-loop AFR, idle valve control, VANOS, warm-up/accel enrichment, knock calibration (noise learning, thresholds), MAF-trim auto-tune, tuner T4–T8 |
| **10. ITB conversion** | Plenum ITBs on the known-good ECU | ITB TPS, relocated IAT and idle air plumbed; blend table and Alpha-N VE tuned; MAF pulsation and saturation checked in logs; throttle sync set; knock re-checked; dyno or careful road tuning |

Suggested parallelisation: hardware design (Phase 6) can start right after Phase 0 measurements, while
firmware Phases 1–5 progress on the dev board.

---

## 7. Decisions

| # | Topic | Decision |
|---|-------|----------|
| 1 | Load sensing | Stock HFM MAF, blended with Alpha-N for the ITB build (§3.4, §3.8) |
| 2 | Car connection | Plug-and-play: donor MS41 connector + home-made adapter harness (§2.5) |
| 3 | Coils | Stock-type M52 dumb coils + on-board IGBTs; smart-coil option kept on the board (§2.6) |
| 4 | Wideband | External LSU 4.9 controller with 0–5 V output, bought before Phase 8 |
| 5 | Tuning GUI | Own tuner in Python + PySide6 |
| 6 | Fallback GUI | TunerStudio-compatible protocol + generated `.ini` |
| 7 | Car | 1997 UK E36 323i, M52B25, MS41, single VANOS, ASC, 2 knock sensors |

| 8 | Gearbox | Manual — no gearbox computer to talk to |
| 9 | ITBs | Home-made from motorcycle throttle bodies; injectors in the ITBs, aimed at the intake valves (§3.8) |
| 10 | Lambda | Both: two narrowbands (one per bank) and a wideband, mode selected in the tune (§3.9) |

**Still open (not blocking the firmware):**
- Which motorcycle the ITBs come from, throttle bore, and the injector part number (for flow and impedance).

## 8. Risks and things to verify early

| Risk | Mitigation |
|------|-----------|
| **ASC** expects data from the DME (engine speed/torque, and it asks the DME to cut power when wheels spin) | Without those messages the ASC module will most likely show its warning light and switch traction control off, while ABS keeps working — confirm on the car with the stock DME unplugged. ITBs remove ASC's throttle intervention anyway. The board carries an optional CAN interface (MCP2515 on SPI) so the DME messages can be emulated later if wanted. |
| Motorcycle injectors too small or low-impedance | Check part number and measure resistance before buying/building (§3.8); larger high-impedance injectors fit the same bosses on many bike ITBs. |
| MAF pulsation / saturation with ITBs | Crank-synchronous MAF averaging, MAF trim table, Alpha-N blend, larger housing if it saturates (§3.8). |
| Stock HFM transfer curve unknown | Start from published data for the HFM part number, correct with the wideband using the MAF-trim auto-tune. |
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

1. When the ITB parts are chosen, record bike model, bore and injector data in [CAR.md](CAR.md).
2. Buy the Phase 0/1 kit (shopping list in [CAR.md](CAR.md)): Arduino Mega, a second small Arduino, a USB
   logic analyser, a basic oscilloscope, a donor MS41 DME, and the E36 wiring diagrams (ETM).
3. Phase 0: work through the [CAR.md](CAR.md) checklist on the car.
4. Phase 1 in parallel: firmware build system, HAL timers with synchronised start, and the trigger simulator.
