# Bench test: build and wiring guide

Everything here runs on a desk from your laptop's USB ports. There is **no 12 V anywhere** and nothing
from the car is connected. You build it in stages and test each stage before the next, so when
something doesn't work you know which part to look at.

| Stage | What you build | What it proves |
|-------|----------------|----------------|
| 1 | Software on the laptop, firmware on both boards | Tools work, the ECU answers |
| 2 | Nano → Mega crank/cam wires | The ECU syncs to a 60-2 + cam signal |
| 3 | Potentiometers as sensors | Sensor readings and fault detection |
| 4 | LEDs on the extra outputs | Pump, VANOS, heaters, idle valve, tach |
| 5 | Logic analyser on the outputs | Injector and coil timing on real hardware |
| 6 | TPIC8101 knock chip on the breadboard | Knock chip setup, sensor switching, knock retard |

```
                          laptop (USB hub if you are short of ports)
                 ┌──────────────┼───────────────────┬───────────────┐
              USB│           USB│                USB│               │ 3.5 mm audio
          ┌──────┴─────┐  ┌─────┴──────────────┐ ┌──┴───────────┐   │ (6.7 kHz test tone)
          │ Arduino    │  │ Arduino Mega 2560  │ │ logic        │   │
          │ Nano       │  │ = the ECU          │ │ analyser     │   │
          │ (crank/cam │  │                    │ │ 8 channels   │   │
          │ simulator) │  │                    │ │              │   │
          └──┬──┬───┬──┘  └┬──┬────┬────┬───┬─┘ └──────┬───────┘   │
          D9 │  │D10│GND   │  │    │    │   │ SPI      │ probes    │
             │  │   │  D48 │  │D49 │A0-A8   │ D50-53   │ on pins   │
             └──┼───┼──────┘  │    │    │   │ D35      │ to watch  │
                └───┼─────────┘  pots  LEDs │          │           │
                    └── GND ──────────────  └── TPIC8101 on breadboard ◄──┘
```

---

## Stage 1 — software and firmware

### 1a. Install the tools on the laptop

**Linux (Ubuntu/Debian):**
```sh
sudo apt install git make gcc-avr avr-libc avrdude python3-pip
sudo usermod -aG dialout $USER            # serial port access; log out and back in
pip install pyserial
pip install -r tuner/requirements.txt     # for the tuner GUI
```

**macOS** (with [Homebrew](https://brew.sh)):
```sh
xcode-select --install                    # make, git
brew tap osx-cross/avr && brew install avr-gcc avrdude
pip3 install pyserial
pip3 install -r tuner/requirements.txt
```

**Windows:** the simplest route is to flash ready-built `.hex` files (ask Claude for them, or build
them in WSL) with **AVRDUDESS** (a free GUI for avrdude) or the `avrdude.exe` that comes with the
Arduino IDE. Install Python from python.org, then `pip install pyserial` and
`pip install -r tuner\requirements.txt`. To build from source, install WSL (Ubuntu) and follow the
Linux steps inside it. Port names are `COM3`, `COM4`, … (Device Manager → Ports).

Also install **PulseView** (from sigrok.org) for the logic analyser. Clone Nanos use a CH340 USB
chip: Windows and older macOS may need its driver.

Get the code:
```sh
git clone https://github.com/3wperor/e36-ecu-project.git
cd e36-ecu-project
git checkout ccr-b7e022de-kdo2ys          # the branch with the latest firmware (until it is merged)
```

### 1b. Flash the Nano (trigger simulator)

Plug in **only the Nano**, so you know which port it is (Linux `/dev/ttyUSB0`, macOS
`/dev/cu.usbserial-…`, Windows `COMx`).

```sh
cd tools/stim
make
make flash PORT=/dev/ttyUSB0              # if it fails: make flash PORT=/dev/ttyUSB0 BAUD=57600
cd ../..
```
Most clone Nanos have the "old bootloader", which needs `BAUD=57600`.

Windows: `avrdude -p m328p -c arduino -P COM4 -b 57600 -D -U flash:w:stim.hex:i`

### 1c. Flash the Mega (the ECU)

Plug in the Mega (Linux `/dev/ttyACM0`, macOS `/dev/cu.usbmodem…`).

```sh
cd firmware
make MCU=atmega2560
make flash MCU=atmega2560 PORT=/dev/ttyACM0
cd ..
```
Windows: `avrdude -p m2560 -c wiring -P COM5 -b 115200 -D -U flash:w:ecu-mega2560.hex:i`

**Check:**
```sh
python3 tools/ecu_cli.py /dev/ttyACM0 info
```
It should print the signature `E36ECU-dev-p9a` and the firmware version. Faults are expected at this
point (nothing is connected to the sensor inputs yet, and the first boot loads the default tune).

> Every time a program opens the Mega's port, the Mega resets and the tool waits 2 s for it to boot.
> That's normal. The `tune set` command stores each change in EEPROM straight away, so changes
> survive the reset.

---

## Stage 2 — crank and cam signals (Nano → Mega)

Three jumper wires, Nano to Mega:

| Nano | Mega | Signal |
|------|------|--------|
| D9 | **D48** | crank, 60-2 |
| D10 | **D49** | cam |
| GND | GND | common ground |

D48 and D49 are on the Mega's double header at the end of the board (the even numbers are on the
row nearer the board edge on most Megas; check the labels printed on the board).

Open a serial terminal to the **Nano** (115200 baud) and set a speed:
```sh
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200
r3000                                    # type this and press Enter: 3000 rpm
```
(The Arduino IDE's Serial Monitor works too: 115200 baud, "Newline" line ending.) Leave the
terminal open, and in a second terminal watch the ECU:

```sh
python3 tools/ecu_cli.py /dev/ttyACM0 watch
```

**Check:** `3000 rpm  sync full 720°`. Then try these on the Nano:

| Nano command | Expected on the ECU |
|--------------|---------------------|
| `r800` / `r7000` | rpm follows, sync stays `full 720°` |
| `s800,7000,3000` | 3 s sweep up, sync never drops |
| `d` | drops one tooth: sync drops for a moment and comes back, no wrong position |
| `c0` | cam off: sync falls back to 360° (wasted spark) |
| `c2` | cam back on: `full 720°` again |
| `r0` | engine stopped: rpm 0, sync none |

`python3 tools/ecu_cli.py /dev/ttyACM0 teeth` prints a tooth log: 58 teeth at an even interval plus
one long gap per revolution.

---

## Stage 3 — potentiometers as sensors

Each pot has three legs. The **outer legs go to 5 V and GND** (use the breadboard's power rails, fed
from the Mega's 5V and GND pins). The **middle leg (wiper) goes to the analogue pin**. Turning the
knob sweeps that input from 0 to 5 V.

```
   Mega 5V ──── breadboard + rail ──┬─────────┬─── …
                                    │         │
                                 ┌──┴──┐   ┌──┴──┐
                                 │ pot │   │ pot │    10 kΩ linear (B10K)
                                 └┬─┬──┘   └┬─┬──┘
                         wiper ───┘ │       │ └──── …
                                    │       │
   Mega GND ─── breadboard − rail ──┴───────┴─── …
```

| Pot | Mega pin | Set it to | Shows in `watch` as |
|-----|----------|-----------|---------------------|
| MAF | **A0** | ~1.5–3.2 V (more volts = more air) | `air … mg` |
| Throttle (TPS) | **A1** | 0.5 V closed … 4.5 V wide open | `TPS …%` |
| Coolant (CLT) | **A3** | ~0.5 V = warm (~85 °C); higher volts = colder | `CLT …C` |
| Intake air (IAT) | **A4** | ~2.5 V | (in the tuner) |
| Wideband | **A5** | 2.35 V = 14.7 AFR (0 V = 10, 5 V = 20 AFR) | `AFR …/…` |
| Narrowband bank 1 | **A6** | 0–1 V (see below) | (in the tuner) |
| Narrowband bank 2 | **A7** | 0–1 V (see below) | (in the tuner) |
| Battery | **A8** | ~3.45 V = 13.8 V (the ECU multiplies by 4) | `…V` |

A2 (second TPS) and A9 (MAP) are not used by the default tune; you can leave them unconnected.

**Narrowband pots:** a real narrowband only gives 0–1 V, so put a **47 kΩ resistor** between the
5 V rail and the top leg of those two pots (bottom leg to GND). The pot then covers about 0–0.9 V,
which makes it easy to cross the 0.45 V rich/lean switching point.

**"Closed throttle" means the TPS pot at about 0.5 V (0 %), not 0 V:** at 0 V the ECU sees a shorted
sensor and flags a fault, as a real one would.

**Check:** with the pots set as above, `watch` shows no `FAULT:` text. Turn the coolant pot to
either end (0 V or 5 V): `FAULT: CLT` appears (that's a shorted or open sensor). Turn it back and the
fault clears. Same for MAF and TPS.

---

## Stage 4 — LEDs on the extra outputs

Each LED gets a **1 kΩ resistor** in series. The long leg of the LED goes towards the Mega pin, and
the resistor and short leg go to GND.

```
   Mega pin ──── LED (long leg) ──|>|── (short leg) ──── 1 kΩ ──── GND
```

| Mega pin | Output | What you should see |
|----------|--------|---------------------|
| **D22** | Fuel pump relay | On for the prime time at power-up, on while the "engine" turns, off 1 s after `r0` |
| **D24** | VANOS | On between 1500 and 5000 rpm with the throttle above 10 % and a warm engine |
| **D25**, **D26** | O2 sensor heaters | On 10 s after the engine starts |
| **D27** | Tach | Flickers with rpm: try `r100` (about 5 flashes per second); above ~300 rpm it looks steady |
| **D10** | Idle valve, open coil | Brightness = valve opening |
| **D9** | Idle valve, close coil | The opposite of D10 (bright when D10 is dim) |

These only draw about 3 mA each, which is well within the Mega's limits. Never connect a solenoid,
relay coil or motor straight to a Mega pin. Those need driver transistors, which come with the PCB.

**Tests:**

1. **VANOS:** `r3000`, throttle pot at ~25 %, coolant warm → D24 on. Close the throttle (TPS 0 %) →
   off. Run `r6000` → off above 5000 rpm.
2. **Overrun fuel cut:** `r3000`, close the throttle → after 0.5 s `watch` shows `DFCO` and `pw` drops
   to 0. Open the throttle → fuel is back.
3. **Idle:** `r800`, throttle closed, coolant warm → `watch` shows `IDLE`, and `IAC` is the valve
   opening. Change the Nano speed: `r700` (too slow) → `IAC` rises and D10 gets brighter; `r950`
   (too fast) → `IAC` falls. Turn the coolant pot colder → the idle target rises and the valve opens
   more.
4. **Lambda, narrowband:** `python3 tools/ecu_cli.py /dev/ttyACM0 tune set lambdaMode 1` (then
   start `watch` again). `r2500`, light load (MAF ~2 V), throttle ~15 %, coolant warm. After 30 s
   `watch` shows `CL` (closed loop). Turn the bank 1 pot above 0.45 V (rich): the bank 1 `trim`
   ramps negative. Below 0.45 V (lean): it ramps positive. Bank 2 works the same with its own pot.
5. **Lambda, wideband:** `tune set lambdaMode 2`. Turn the wideband pot leaner (higher voltage) →
   trim goes positive, up to +15 %; richer → negative. Set it back to 0 afterwards with
   `tune set lambdaMode 0`: **the first start on the car must be open loop.**

---

## Stage 5 — logic analyser (injector and coil timing)

Connect the analyser's **GND to the Mega's GND first**, then the channel probes to the pins below.
With 8 channels, do it in groups. Keep CH0 on the crank (D48) every time as the reference.

| Analyser channel | Group A: injectors | Group B: coils | Group C: extras |
|------------------|--------------------|----------------|-----------------|
| CH0 | D48 crank | D48 crank | D48 crank |
| CH1 | D49 cam | D49 cam | D27 tach |
| CH2 | D11 injector 1 | D6 coil 1 | D10 idle open |
| CH3 | D12 injector 2 | D7 coil 2 | D9 idle close |
| CH4 | D13 injector 3 | D8 coil 3 | D35 knock INT/HOLD |
| CH5 | D5 injector 4 | D46 coil 4 | D53 knock CS |
| CH6 | D2 injector 5 | D45 coil 5 | D52 knock SCK |
| CH7 | D3 injector 6 | D44 coil 6 | D50 knock MISO |

The outputs are **off by default**. Turn them on with the bench test (fixed values). It runs while
the command is open, shows live data, and **Ctrl+C turns the outputs off again**:

```sh
python3 tools/ecu_cli.py /dev/ttyACM0 set ign=on inj=on advance=10 dwell=3 pw=2.5 tdc=60 eoi=380
```

In PulseView pick the analyser (fx2lafw driver), **2 MHz** sample rate and **2 s** capture, then
press Run.

**What correct looks like** (Nano at `r3000` and cam on: one engine cycle = 40 ms):

- Each injector pin goes high for **2.5 ms**, once every 40 ms. The six fire in the firing order
  **1-5-3-6-2-4**, evenly spaced 120° (6.7 ms) apart.
- Each coil pin goes high for **3 ms** (the dwell). The falling edge is the spark: **10° before** that
  cylinder's top dead centre, also in order 1-5-3-6-2-4.
- Change the advance (e.g. `advance=30`) → the falling edges move earlier. Change `dwell=` → the high
  time changes but the falling edge stays put.
- `c0` on the Nano (cam off) → coils switch to wasted spark (pairs fire together every revolution).
- Pull the D48 crank wire out mid-run → every coil pin goes low within ~10 ms (a coil is never left
  charging), and the injectors stop.

Then turn the real calculation on instead of the bench test (outputs follow the tune and pots):
```sh
python3 tools/ecu_cli.py /dev/ttyACM0 tune set injEnable 1
python3 tools/ecu_cli.py /dev/ttyACM0 tune set ignEnable 1
```
Turn the MAF pot up: the injector pulses get longer. **Set both back to 0 before anything real is
ever connected:** `tune set injEnable 0`, `tune set ignEnable 0`.

> **Bench only.** Never connect real coils or injectors to the Mega's pins. They are 5 V logic
> signals for the driver transistors on the PCB.

---

## Stage 6 — TPIC8101 knock chip

### 6a. Solder the chip onto the adapter

Solder the TPIC8101 (SOIC-20) onto the SOIC-20 → DIP adapter board. **Pin 1 is marked by a dot or
notch on the chip; match it to pin 1 on the adapter.** Tack two opposite corner pins first, check
alignment, then solder the rest. Use plenty of flux. Check with a magnifier and the multimeter's
continuity mode that no neighbouring pins are bridged. Solder the header pins on and press it into
the breadboard across the centre gap.

### 6b. Chip pinout (top view)

```
                 ┌────────┬┬────────┐
         VDD   1 │●       └┘        │ 20  CH1P
         GND   2 │                  │ 19  CH1N
        Vref   3 │                  │ 18  CH1FB
         OUT   4 │    TPIC8101      │ 17  CH2FB
          NC   5 │                  │ 16  CH2N
          NC   6 │                  │ 15  CH2P
    INT/HOLD   7 │                  │ 14  TEST
          CS   8 │                  │ 13  SCLK
         XIN   9 │                  │ 12  SDI
        XOUT  10 │                  │ 11  SDO
                 └──────────────────┘
```

### 6c. Wiring

**Power and reference**

| Pin | Connect to |
|-----|------------|
| 1 VDD | Mega **5V**. Also a **4.7 µF** and a **100 nF** capacitor from pin 1 to GND, right next to the chip (if the 4.7 µF is electrolytic, its − leg goes to GND) |
| 2 GND | Mega **GND** |
| 3 Vref | **100 nF** to GND. This is the chip's 2.5 V mid-point; it also feeds pins 15 and 20 (below) |
| 4 OUT | **2.2 nF** to GND (optional: we read the result over SPI) |
| 5, 6 NC | nothing |
| 14 TEST | nothing (internal pull-up = normal mode) |

**Clock (8 MHz crystal)**

| Part | Between |
|------|---------|
| 8 MHz crystal | pin 9 XIN and pin 10 XOUT |
| 1 MΩ resistor | pin 9 and pin 10 (across the crystal) |
| 22 pF capacitor | pin 9 and GND |
| 22 pF capacitor | pin 10 and GND |

The firmware's default `knockPrescaler` is 3, which tells the chip it has an 8 MHz clock. If you use
a different crystal, see the setting's help for the code.

**SPI and control to the Mega**

| TPIC8101 pin | Mega pin |
|--------------|----------|
| 13 SCLK | **D52** (SCK) |
| 12 SDI | **D51** (MOSI) |
| 11 SDO | **D50** (MISO) |
| 8 CS | **D53** |
| 7 INT/HOLD | **D35** |

**Sensor inputs (both channels the same)**

```
  input 1 ──||────/\/\/───┬──── pin 19 CH1N          pin 20 CH1P ──── pin 3 Vref
           3.3 nF  47 kΩ  │
                          └─/\/\/──── pin 18 CH1FB
                            47 kΩ

  input 2 ──||────/\/\/───┬──── pin 16 CH2N          pin 15 CH2P ──── pin 3 Vref
           3.3 nF  47 kΩ  │
                          └─/\/\/──── pin 17 CH2FB
                            47 kΩ
```

The firmware calls **channel 1 the bank 1 sensor (cylinders 1–3)** and **channel 2 the bank 2
sensor (cylinders 4–6)**, the same as the M52's two knock sensors.

**Test tone:** cut a 3.5 mm audio cable, or use a breakout. The **tip (left channel) goes to
"input 1"** (the free end of the 3.3 nF capacitor), and the **sleeve (ground) goes to GND**.
Connect "input 2" to GND for now. Play a tone from a phone or PC tone-generator (web or app): sine
wave, **6.7 kHz**, volume low (start around 20 %).

### 6d. Before powering up

- Multimeter on resistance: Mega 5V to GND should **not** read near 0 Ω (no short).
- Every wire checked against the tables above (SDI/SDO are the easiest to swap).
- Plug the Mega in, then check chip pin 1 reads ~5 V and pin 3 (Vref) ~2.5 V.

### 6e. Test

Turn knock control on, and set the pots so the "engine" is warm and loaded (knock control is only
active above 1500 rpm, 200 mg of air and 60 °C):

```sh
python3 tools/ecu_cli.py /dev/ttyACM0 tune set knockEnable 1
```
Nano at `r3000`, MAF pot ~3.2 V (`air` above 200 mg in `watch`), coolant warm. Then:

```sh
python3 tools/ecu_cli.py /dev/ttyACM0 knock
```

It shows per-cylinder level, learned noise, ratio and retard, and the state.

1. **Chip answers.** The state shows `enabled active`, and **not** `CHIP FAULT` once the tone is
   playing. (With no tone at all the level can sit at 0, which the ECU treats as a dead chip, so
   `CHIP FAULT` with no tone is expected.) If `CHIP FAULT` stays with the tone on, check the SPI
   wires, the crystal and the 5 V supply; also try group C on the logic analyser and PulseView's SPI
   decoder (mode 1, CS active low). Right after power-up you should see the byte `71` go out first.
2. **Right sensor per cylinder.** With the tone on input 1, **cylinders 1–3 read high and 4–6 low**.
   Move the tone wire to input 2 (and input 1 to GND): now **4–6 high and 1–3 low**. This proves the
   sensor switching between windows.
3. **Level follows the volume.** Raise the volume slowly: the level rises smoothly. If it jumps
   about or drops back down as you turn up, the result byte layout is different from the datasheet:
   try `tune set knockReadOrder 1`, then `2`, and keep whichever follows the volume smoothly. Tell
   Claude which one worked.
4. **Gain.** Set the volume to a "normal" level and adjust `tune set knockGain N` (higher number =
   less gain) until the level sits around **100–200** (of 1023).
5. **Band-pass filter.** Keep the volume fixed and change the tone frequency: 4, 5, 6, 6.7, 8,
   10 kHz. The level should **peak around 6.6–6.7 kHz** (`knockBand` 40 = 6.64 kHz). Note the
   readings at each frequency.
6. **Knock detection.** Hold a steady volume for ~10 s (the ECU learns this as background noise),
   then turn the volume up sharply. Cylinders 1–3 show `KNOCK`, their **retard** jumps, and 4–6 stay at
   0. Turn it back down: the retard comes back at about 1° per second.
7. **Chip failure.** Pull the SDO wire (chip pin 11 → D50): `CHIP FAULT` and 3° retard on every
   cylinder. Push it back in: the fault clears by itself. (Don't pull the chip's 5 V wire while the
   SPI wires are connected: it can half-power the chip through its input pins. If the chip ever
   really resets, the ECU notices the missing `71` reply and sends the setup again.)

When you're done, `tune set knockEnable 0` (knock control gets switched on for real on the car,
in sniffer mode).

---

## Troubleshooting

| Problem | Likely cause |
|---------|--------------|
| `avrdude: stk500_recv(): programmer is not responding` (Nano) | Old bootloader: add `BAUD=57600`. Wrong port. Another program (serial monitor) has the port open |
| Mega flash fails | Wrong port; close every terminal/tuner using the port; use `-c wiring -b 115200` |
| Mega stops answering until it is unplugged | Some old Mega bootloaders hang after a watchdog reset. Unplug and replug it; if it keeps happening, note when it happens |
| `info` times out | Wrong port, or the Mega is still booting (the tool waits 2 s) |
| rpm shows but sync stays `crank 360°` | Cam wire (D10 → D49) loose, or Nano set to `c0` |
| No rpm at all | Crank wire (D9 → D48) loose; GND not shared; Nano not running (`?` in its terminal shows settings) |
| Sensor reads wrong/jumps | Pot wiper not on the right pin; the pot's outer legs not on 5 V/GND |
| `FAULT: battery` | A8 pot not connected or not at ~3.45 V |
| LED never lights | LED backwards (flat side = short leg = towards GND) |
| `CHIP FAULT` with the tone on | SPI wires swapped (SDI↔SDO), crystal not oscillating (check the 22 pF caps and 1 MΩ), no 5 V on pin 1 |
| Knock level 1023 (full scale) | Volume far too high, or gain too high (raise `knockGain` number) |

## What to send back

Write down (or screenshot) the results so the next firmware changes can be based on the real
hardware:

- `info` output, and whether sync held through the sweeps and the dropped tooth.
- PulseView captures of group A and group B at 3000 rpm.
- Knock: which `knockReadOrder` follows the volume, the level at each test frequency (step 5), the
  `knockGain` you ended up with, and whether steps 2, 6 and 7 behaved as described.
