# Car Profile — 1997 UK E36 323i (M52B25)

This file is the record of what is actually on the car. Everything marked **expected** is what a
1997 UK-market 323i with a Siemens MS41 DME normally has. It still has to be **confirmed on this car**
(Phase 0), because the ECU is designed around these answers.

## 1. Known facts

| Item | Value |
|------|-------|
| Model | BMW E36 323i, UK market, 1997 |
| Engine | M52B25 (pre-TU), 2,494 cc, inline 6, firing order 1-5-3-6-2-4 |
| DME | Siemens MS41 *(read the exact part number and variant from the label on the DME)* |
| VANOS | Single, intake cam, on/off solenoid |
| Traction | ASC |
| Knock control | Yes, two knock sensors expected |
| Check-engine light | None (pre-OBD-II UK car) |
| Immobiliser | EWS expected |
| Gearbox | Manual |
| Planned intake | Home-made ITBs from motorcycle throttle bodies inside a plenum/airbox, stock MAF on the inlet |
| Planned injectors | Motorcycle injectors in the ITBs, aimed at the intake valves |
| Lambda | Stock narrowbands (one per bank) + a wideband to be added |

## 2. Expected sensors and actuators — confirm each one

Tick each row once it has been found on the car and its connector pins traced to the DME connector.

### Inputs (sensors)

| ✓ | Sensor | Expected type | How to identify / test | Our ECU input |
|---|--------|---------------|------------------------|---------------|
| ☐ | Crankshaft position | Inductive (VR) on MS41 — reads the 60-2 wheel | **VR:** two-wire coil (+ shield), measures a few hundred Ω to ~1.5 kΩ across its pins, makes a sine wave that grows with RPM. **Hall:** has a supply pin (5 V or 12 V with ignition on), no coil resistance, makes a clean square wave. | MAX9926 ch1 → ICP5 |
| ☐ | Camshaft position | Hall effect on the intake cam, front of the cylinder head | Supply + ground + signal; square wave. Record the pattern over two crank revolutions and how far it moves when VANOS switches. | Hall input → ICP4 |
| ☐ | Mass air flow (HFM) | Siemens hot-film, 0–5 V output | In the intake duct after the air filter. Note its part number (needed to find its calibration curve) and its pinout (supply, ground, signal, signal ground). | ADC (load) |
| ☐ | Throttle position | Potentiometer on the throttle body | Three wires: 5 V reference, ground, signal sweeping roughly 0.5–4.5 V. | ADC |
| ☐ | Intake air temperature | NTC thermistor in the intake manifold | Two wires; resistance falls as it warms (several kΩ cold). Must be relocated into the ITB airbox later. | ADC |
| ☐ | Coolant temperature | Dual-element NTC (one element for the DME, one for the dash gauge) | Find which pins go to the DME; leave the gauge element alone. | ADC |
| ☐ | Knock sensor 1 | Piezo, cylinders 1–3, on the block under the intake manifold | Two wires + shield. | TPIC8101 ch1 |
| ☐ | Knock sensor 2 | Piezo, cylinders 4–6 | Two wires + shield. | TPIC8101 ch2 |
| ☐ | Lambda (O2) sensors | Narrowband, pre-cat, probably one per bank of three cylinders | Four wires each: signal, signal ground, heater +, heater −. Count how many there are. | ADC + heater outputs (optional) |
| ☐ | Road speed signal | Square wave from the ABS/ASC module | Trace from the DME connector. | GPIO |
| ☐ | A/C request / pressure switch | Switch input, if A/C fitted | Trace from the DME connector. | GPIO |

### Outputs (actuators)

| ✓ | Actuator | Expected type | What to measure | Our ECU output |
|---|----------|---------------|-----------------|----------------|
| ☐ | Injectors ×6 | High-impedance, saturated drive | Resistance of each (expected ~12–16 Ω). Note part number for flow data. | OC1A–C, OC3A–C |
| ☐ | Ignition coils ×6 | Coil-on-plug, no built-in igniter | Primary resistance of each (low, under ~1 Ω); check for cracks. | OC4A–C, OC5A–C via IGBTs |
| ☐ | Idle control valve | Rotary valve with two coils (three pins) | Resistance of each coil. Later plumbed into the ITB balance rail. | OC2A / OC2B |
| ☐ | VANOS solenoid | On/off | Coil resistance. | GPIO low-side |
| ☐ | Fuel pump relay | Relay coil switched by DME | — | GPIO low-side |
| ☐ | Main (DME) relay | Relay coil switched by DME | — | GPIO low-side |
| ☐ | Tank vent / purge valve | Solenoid | — | Spare low-side output |
| ☐ | Tachometer signal (TD) | Square wave to the instrument cluster | — | Tach output |
| ☐ | Fuel consumption signal (TI) | Injector pulse copy to the cluster/OBC | — | Spare output |
| ☐ | A/C compressor cut | Relay | — | Spare low-side output |

## 3. Phase 0 checklist

Do these with the **stock DME still running the car**.

1. ☐ Get the E36 Electrical Troubleshooting Manual (ETM) for the car's build date and copy the DME connector
   pinout into section 4 below.
2. ☐ Read and record the DME label (part number, MS41 variant), HFM part number, injector part number.
3. ☐ Find and tick off every sensor/actuator in section 2; note anything missing or extra.
4. ☐ Battery disconnected: measure and record the resistances listed in section 2.
5. ☐ Scope the crank and cam signals together at cranking, idle and ~3,000 RPM. Save the captures in
   `docs/captures/`; they become test data for the decoder.
6. ☐ Scope both knock sensors at idle and during a few revs (frequency content for the bandpass setting).
7. ☐ Mark cylinder 1 TDC on the crank pulley and note which crank tooth (counted from the gap) it lines up with
   (gives the trigger offset).
8. ☐ With the stock DME **unplugged** and ignition on: does the starter still crank? Which warning lights come on
   (ASC, others)? This tells us what EWS and ASC do without a DME.
9. ☑ Gearbox: manual.

## 4. ITB build data

*(Fill in once the parts are chosen. The ECU base tune is calculated from these.)*

| Item | Value |
|------|-------|
| Donor motorcycle (make, model, year) | |
| Number of ITB sets / throttles per set | |
| Throttle bore (mm) | |
| Secondary butterflies removed/locked? | |
| TPS: type, supply voltage, pinout, one or two fitted | |
| Injector part number | |
| Injector rated flow (cc/min) and test pressure (bar) | |
| Injector resistance (Ω) — measured | |
| Fuel pressure regulator reference (airbox / balance rail) | |
| Idle air: stock idle valve into a balance rail? | |
| Runner length, trumpet length (mm) | |
| Airbox volume (litres, approx.) | |

## 5. DME connector pinout

*(Fill in from the ETM, then confirm each pin with a multimeter.)*

| Module | Pin | Wire colour | Function | Direction | Confirmed |
|--------|-----|-------------|----------|-----------|-----------|
| | | | | | |

## 6. Shopping list

**Phase 0–1 (now):**

| Item | Why |
|------|-----|
| Arduino Mega 2560 (clone is fine) | Firmware development board — every pin we use is broken out |
| Second small Arduino (Uno/Nano) | Trigger simulator: fakes the 60-2 crank + cam signals on the desk |
| 8-channel USB logic analyser (24 MHz class, works with free PulseView/sigrok) | Checks injector/coil timing from the firmware |
| Basic oscilloscope (handheld or USB, ≥ 1 MHz bandwidth, 2 channels) | Sees analogue signals the logic analyser can't: VR crank sine wave, knock sensor, MAF |
| Multimeter | Resistance and pinout checks |
| Used/dead Siemens MS41 DME matching the car's connector | Donor connector for the plug-and-play adapter |
| E36 ETM (wiring diagrams) | DME pinout and wire colours |
| Breadboard, jumper wires, a few LEDs and resistors | Bench testing outputs |

**Before the first start (Phase 8):**

| Item | Why |
|------|-----|
| Wideband controller kit with an LSU 4.9 sensor and 0–5 V analogue output (e.g. 14Point7 Spartan 3, AEM X-Series UEGO, Innovate LC-2) | Essential for safe tuning |
| Exhaust bung welded in (collector, or after the two banks merge) | Mounting point for the wideband sensor |
| Timing light | Sets the trigger offset and checks real ignition timing |
| New spark plugs, plus replacement coils for any faulty ones | Remove ignition faults from the equation before first start |
