# Smart Garage Door Controller
## Craftsman 045DCT · ESP32 · Security+ 2.0 Wireline · Apple HomeKit

**Firmware v1.3.0**

---

## Overview

This project adds native Apple HomeKit support to a **Craftsman 045DCT** (2015, yellow learn button) garage door opener by tapping directly into the Security+ 2.0 wired serial bus — the same bus the wall panel uses.  No RF radio is needed, and **no Learn-button enrolment is required**: the opener accepts wired commands from any device on the bus.  The ESP32 connects to WiFi and appears in the Apple Home app as a **Garage Door** accessory with three controls — door, opener light, and remote lockout — plus obstruction reporting.  No hub or cloud service is involved.

**HomeKit features:**

| Control | HomeKit service | How it works |
|---------|-----------------|--------------|
| Garage door open/close | GarageDoorOpener | Explicit `DOOR_ACTION` open/close commands with verified result |
| Opener work light | LightBulb ("Opener Light") | `LIGHT` command (0x281), state mirrored from opener status |
| Remote lockout | LockMechanism ("Remote Lock") | `LOCK` command (0x18C) — blocks wireless remotes; wired bus (HomeKit + wall console) keeps working |
| Obstruction detected | ObstructionDetected flag | Hardware safety-beam sensing on GPIO34 **or** the opener's status flag |

**Key facts:**
- Opener communicates at 9600 baud 8N1 on a single active-LOW wire (RED terminal); every packet is preceded by a ~1.3 ms bus-LOW preamble
- Protocol is Security+ 2.0 wireline, encoded/decoded with the `secplus` library; command codes and payload layout verified against the [ratgdo](https://github.com/ratgdo/esphome-ratgdo) project and live bus captures
- HomeKit is implemented natively on the ESP32 with the `HomeSpan` library — no Homebridge or extra hardware
- Two magnetic reed switches give definitive hardware signals for the "Closed" and "Fully Open" end-stops; bus status reports cover the in-between travel.  For this opener the magnets are mounted on the carriage.  The reeds are located at either end of travel on the carrier rail.  (See Project Photos)
- The opener does **not** broadcast status periodically — the firmware polls `GET_STATUS` while commands are in flight and every 60 s when idle
- Rolling codes are persisted in NVS flash with a wear-limiting policy (saved every 16 increments, +64 boot bump so the counter never falls behind the opener's view)

---

## Bill of Materials

| Ref  | Part                          | Package        | Key Specs                    | Source / Notes                              |
|------|-------------------------------|----------------|------------------------------|---------------------------------------------|
| U1   | ESP32 DevKit (38-pin)         | DIP/module     | Dual-core, WiFi, BT          | Any ESP32-WROOM-32 board                    |
| Q1   | 2N7000 N-channel MOSFET       | TO-92          | Vgs(th) 1–3 V, Vds 60 V      | RX interface — level-shifts 12 V→3.3 V (inverted) |
| Q2   | AO3400A N-channel MOSFET      | SOT-23         | Vgs(th) 0.5–1.2 V, Vds 30 V  | TX interface — pulls 12 V bus LOW; mount on SOT-23→DIP adapter |
| R1   | 10 kΩ resistor                | 1/4 W          | ±5%                          | 2N7000 gate series resistor from RED        |
| R2   | 10 kΩ resistor                | 1/4 W          | ±5%                          | 2N7000 drain pull-up to 3.3 V               |
| R3   | 10 kΩ resistor                | 1/4 W          | ±5%                          | AO3400A gate drive from GPIO21              |
| R4   | 10 kΩ resistor                | 1/4 W          | ±5%                          | SW1 (CLOSED reed) pull-up to 3.3 V         |
| R5   | 10 kΩ resistor                | 1/4 W          | ±5%                          | SW2 (OPEN reed) pull-up to 3.3 V           |
| R6   | 10 kΩ resistor                | 1/4 W          | ±5%                          | Obstruction divider — top (BLACK wire side) |
| R7   | 10 kΩ resistor                | 1/4 W          | ±5%                          | Obstruction divider — bottom (GND side)     |
| R8   | 100 kΩ resistor               | 1/4 W          | ±5%                          | AO3400A gate → GND pull-down; holds the bus released while the ESP32 is in reset/boot. 
| C1   | 100 nF ceramic capacitor      | 0805 or THT    | 50 V                         | Noise filter across RED/GND at the screw terminal |
| SW1  | Magnetic reed switch (NC)     | varies         | normally-closed              | **CLOSED** end-stop — door frame at bottom; magnet on door |
| SW2  | Magnetic reed switch (NC)     | varies         | normally-closed              | **OPEN** end-stop — track at fully-open position; magnet on door |
| J1   | Screw terminal block          | 5 mm pitch     | 300 V / 10 A                 | RED, WHITE, BLACK connections to opener     |
| —    | SOT-23 → DIP adapter board    | PCB            | 3-pin                        | Lets Q2 sit in a breadboard / perfboard     |
| —    | USB 5 V supply                | wall plug      | ≥500 mA                      | Powers ESP32; standard phone charger works  |
| —    | Wire, 22–26 AWG               | stranded       | —                            | Low current; thin wire is fine              |

**Optional improvements:**

| Part | Notes |
|------|-------|
| MP1584 / LM2596 buck converter + 100 µF cap | Power from the opener's 12 V (RED) instead of USB |

---

## Circuit Description

The full schematic is in [`schematics/ESP32_Security+_2.0_Garage_Door_Opener.png`](schematics/ESP32_Security+_2.0_Garage_Door_Opener.png) (source JSON alongside it).  Note: the schematic uses a generic dev-board symbol, so its pin labels (D13 etc.) are representational — the actual GPIO assignments are listed below.

### Opener terminals

The Craftsman 045DCT exposes three relevant wires at the motor head unit:

| Terminal color | Signal |
|----------------|--------|
| **RED**   | +12 V DC supply + serial data (active LOW) |
| **WHITE** | Ground (0 V reference) |
| **BLACK** | Safety-beam (obstruction) sensor line |

The serial line idles at +12 V.  During transmission any device on the bus pulls the line momentarily to GND for each bit, preceded by a ~1.3 ms LOW frame preamble.

### RX sub-circuit (reading the bus)

```
RED ──[R1 10kΩ]──┬── GATE (Q1 / 2N7000)
                  │   SOURCE → GND
                  │   DRAIN ──[R2 10kΩ]── 3.3 V
                  │         └─────────── GPIO22 (ESP32 UART2 RX)
```

- Bus HIGH (12 V): Q1 gate driven → MOSFET ON → GPIO22 pulled LOW
- Bus LOW (0 V): Q1 OFF → GPIO22 pulled HIGH by R2
- Result: GPIO22 is **inverted** relative to the bus

### TX sub-circuit (writing to the bus)

```
GPIO21 (ESP32 UART2 TX) ──[R3 10kΩ]──┬── GATE (Q2 / AO3400A)
                                      │     SOURCE → GND
                                   [R8 100kΩ]  DRAIN ─────────── RED
                                      │
                                     GND
```

- GPIO21 HIGH: Q2 ON → pulls RED to GND (a data bit, or the frame preamble)
- GPIO21 LOW: Q2 OFF → RED floats to 12 V (idle)
- Result: TX logic is **inverted** relative to the bus
- R8 holds the gate (and therefore the bus) released while the ESP32 is in
  reset or being flashed, when GPIO21 floats.  The R3/R8 divider still puts
  ~3.0 V on the gate when driven — well above the AO3400A threshold.
  **R8 must be gate→GND, never in series with R3** — 100K in series with the
  ~1 nF gate capacitance gives a ~100 µs RC, a full bit time at 9600 baud,
  which destroys the TX signal.

**UART inversion:** both Q1 and Q2 invert.  UART2 is configured with `invert=true` in `GDOBus::begin()`, which cancels both inversions.  The frame preamble is generated by briefly detaching GPIO21 from the UART (GPIO matrix) and driving it directly.

### Obstruction sensor input

```
BLACK ──[R6 10kΩ]──┬──[R7 10kΩ]── GND
                    │
                   GPIO34 (input-only)
```

The opener's safety-beam line has three states (ratgdo research): **clear** = HIGH with a brief LOW pulse every ~7 ms, **obstructed** = steady HIGH, **asleep** = steady LOW.  The divider halves the line voltage for the ESP32; a falling-edge interrupt counts the pulses and a 50 ms window classifier decides the state.  Obstruction is only reported after pulses have been seen at least once since boot, so an unconnected wire can never raise a false alarm (R7 parks the pin LOW = "asleep").

> **Measured on this opener:** BLACK line = 5.95 V, divider midpoint = 2.95 V — comfortably within spec (below the 3.3 V rail, above the ESP32's ~2.48 V V_IH logic-high threshold).  The 10K/10K divider is correct as-built.  If a different opener's sensor line runs near 7 V the midpoint would reach ~3.5 V; use 6.8 kΩ for R7 in that case.

### Reed switches (dual end-stop sensors)

Both switches are normally-closed (NC): the contact shorts to GND when the magnet is **absent** and opens when the magnet is **present**.  External pull-ups (reinforced by `INPUT_PULLUP`) hold the pin HIGH while the contact is open.

```
3.3 V ──[R4 10kΩ]───┬── GPIO25  (CLOSED end-stop)
                    │
                   [SW1 — fully-closed position on overhead track]
                    │
                   GND

3.3 V ──[R5 10kΩ]───┬── GPIO26  (OPEN end-stop)
                    │
                   [SW2 — fully-open position on overhead track]
                    │
                   GND
```

**State truth table** (HIGH = magnet present = door at that end-stop):

| SW1 (GPIO25) | SW2 (GPIO26) | Door state                                    |
|--------------|--------------|-----------------------------------------------|
| HIGH         | LOW          | **Closed**                                    |
| LOW          | HIGH         | **Open**                                      |
| LOW          | LOW          | **In between** — Opening, Closing, or Stopped |
| HIGH         | HIGH         | Error (mechanically impossible; logged)       |

---

## GPIO Assignments

| GPIO | Direction | Signal                       | Connected to                          |
|------|-----------|------------------------------|---------------------------------------|
| 21   | Output    | UART2 TX (inverted) + preamble | R3 → Q2 (AO3400A) gate              |
| 22   | Input     | UART2 RX (inverted)          | Q1 (2N7000) drain + R2 pull-up        |
| 25   | Input     | Reed switch — CLOSED end     | SW1 + R4 pull-up to 3.3 V            |
| 26   | Input     | Reed switch — OPEN end       | SW2 + R5 pull-up to 3.3 V            |
| 34   | Input     | Obstruction sensor (input-only pin) | R6/R7 divider midpoint from BLACK wire |
| 2    | Output    | On-board status LED (heartbeat blink) | —                            |

---

## Protocol Notes (Security+ 2.0 wireline)

Verified against [ratgdo/esphome-ratgdo](https://github.com/ratgdo/esphome-ratgdo) (`secplus2.cpp`) and live captures from this opener and wall unit.

**Packet:** 19 bytes — sync `55 01 00` + two 8-byte encoded halves (`secplus` `encode_wireline_command` / `decode_wireline_command`).  Every packet is preceded by a **preamble**: bus held LOW ~1300 µs, released ~130 µs.  Without the preamble the opener ignores the packet (it shows up in RX captures as one `0x00` byte before each packet).

**Payload layout:** `nibble = (payload >> 16) & 0xF`, `byte1 = (payload >> 8) & 0xFF`, `byte2 = payload & 0xFF`.

**Commands used:**

| Code  | Name | Notes |
|-------|------|-------|
| 0x080 | GET_STATUS | Opener replies with STATUS |
| 0x081 | STATUS | nibble = door state (0=unknown 1=open 2=closed 3=stopped 4=opening 5=closing); byte1 bit6 = obstruction (inverted: 0=obstructed); byte2 bit0 = locked, bit1 = light, bit5 = learn active |
| 0x280 | DOOR_ACTION | nibble = action (0=close 1=open 2=toggle 3=stop); sent as PRESS (byte1=1, byte2=1) then ~150 ms later RELEASE (byte1=0, byte2=1); both share one rolling code, incremented after the pair |
| 0x281 | LIGHT | nibble: 0=off 1=on 2=toggle; single packet |
| 0x18C | LOCK | nibble: 0=unlock 1=lock 2=toggle; single packet (remote lockout) |

The opener **does not** broadcast STATUS periodically — only in reply to GET_STATUS or when its state changes.  The firmware polls every 2 s while a door command is being verified and every 60 s when idle.

---

## Software Architecture

```
HomeSpan (HAP)  ←→  GarageDoorService ─┬─→  GDOBus (secplus)  ←→ UART2 ←→ RED bus
                    LightService      ─┤
                    LockService       ─┘
                         ↕
        SW1 (closed)  SW2 (open)  GPIO34 (obstruction)
```

| File | Role |
|------|------|
| `src/GDOBus.h/.cpp` | Wireline driver: packet RX/TX, preamble, rolling-code management, status cache (door/light/lock/obstruction), state-change callback, diagnostics (`verify()`, stats), learn-mode tooling |
| `src/GarageDoorService.h` | HomeKit door: reed debouncing, goal-seeking command sequencer (explicit OPEN/CLOSE with evidence-based retry, max 4 attempts), travel timeout, hardware obstruction pulse detection, background status refresh |
| `src/GarageDoorService.cpp` | Out-of-line definition of the obstruction ISR (IRAM functions must not be defined inline in a header — Xtensa linker literal-pool limitation) |
| `src/LightService.h` | HomeKit light tile — commands + state mirroring |
| `src/LockService.h` | HomeKit lock tile (remote lockout) — commands + state mirroring |
| `src/main.cpp` | Pin definitions, HomeSpan setup, accessory definition, serial 't' learn trigger |

**State priority for the door (highest first):** reed switches (hardware truth at the end-stops) → bus status events → optimistic command state → 18 s travel timeout.

**Command verification:** when HomeKit commands the door, the sequencer sends the explicit action, then watches reed and status evidence.  If the door isn't moving the right way within 4 s it re-sends (explicit actions are idempotent), polling status every 2 s, and gives up after 4 attempts.  A CLOSE goal is abandoned immediately if an obstruction is reported by either source.

---

## Software Setup (VS Code + PlatformIO)

### 1. Install VS Code and PlatformIO

1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Extensions panel → search **PlatformIO IDE** → install, restart VS Code

### 2. Open the project

**File → Open Folder** → select the `SmartGarageDoor` folder (the one containing `platformio.ini`).

### 3. Libraries (automatic)

On first build PlatformIO clones and compiles automatically:

| Library   | Source                                          |
|-----------|-------------------------------------------------|
| HomeSpan  | `https://github.com/HomeSpan/HomeSpan.git`      |
| secplus   | `https://github.com/argilo/secplus.git`         |

### 4. Environments

| Environment | Use |
|-------------|-----|
| `esp32dev` | USB upload — first flash, or when WiFi is unavailable |
| `esp32dev-ota` | Over-the-air upload once the device is on WiFi (update `upload_port` in `platformio.ini` to your device's IP) |

### 5. Pin assignments

Defaults in `src/main.cpp` match the schematic (`GDO_TX_PIN 21`, `GDO_RX_PIN 22`, `REED_CLOSED_PIN 25`, `REED_OPEN_PIN 26`, `OBST_PIN 34`).

### 6. Build / flash

Build `Ctrl+Alt+B` · Upload `Ctrl+Alt+U` · Serial Monitor `Ctrl+Alt+S` (115200 baud).

### 7. Pair with Apple HomeKit

The pairing code is set in `main.cpp` (`homeSpan.setPairingCode(...)`).  In the **Home app**: **+** → **Add Accessory** → **More Options** → select the Garage Door accessory → enter the code.  The light and lock controls appear grouped with the door; they can be split into separate tiles in the accessory settings.

---

## First Boot — What to Expect

On boot the firmware runs a **bus verification** (1 s listen) and prints a report:

```
[GDO] ── Verification Report ────────────────────────────
[GDO]   Raw bytes received  : 60
[GDO]   Sync headers found  : 3
[GDO]   Complete packets    : 3
[GDO]   Decoded OK          : 3
[GDO]   Result : PASS
```

(Raw bytes ≈ 20 × packets — each packet is 19 bytes plus its preamble arriving as one `0x00`.)  It then queries the opener's status; you should see:

```
[GDO] RX  cmd=0x081  dev=0x...  roll=0x...  payload=0x16062
[GDO]     door=OPEN     light=ON   lock=unlocked  obstruction=no   learn=off
```

When commanding the door from the Home app, look for the press/release pair and the opener's response:

```
[Seq] Door action CLOSE (1 of 4)
[GDO] TX  cmd=0x280  payload=0x00101  roll=0x...   ← press
[GDO] TX  cmd=0x280  payload=0x00001  roll=0x...   ← release
[GDO] RX  cmd=0x081  ...  door=CLOSING ...
```

Block the safety beam and `[Obst] Obstruction → DETECTED` should appear within ~100 ms.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Verify: no bytes received | RX circuit wiring | Check R1, R2, Q1 orientation; RED/WHITE connections; RX pin |
| Verify: bytes but no sync headers | Wrong baud / missing UART inversion | Confirm `GDO_BAUD 9600` and `invert=true` in `GDOBus::begin()` |
| Commands sent but door doesn't move | TX path | Verify Q2 wiring (drain to RED); scope GPIO21 for the 1.3 ms preamble + packet; confirm the wall unit still works (bus not held low) |
| Door state wrong in Home app | Status parsing / reed wiring | Compare `[GDO] door=...` lines against reality; check reed truth table above |
| Obstruction always clear, never detected | Sensor line not pulsing at GPIO34 | Measure divider midpoint: should pulse ~3 V with brief drops when clear; check BLACK wire connection |
| Obstruction false alarms | Divider voltage / noise | Verify midpoint ≤ 3.3 V; keep the divider wiring short |
| Reed switch reads inverted | NO switch fitted instead of NC | Use NC switches, or invert the `digitalRead() == HIGH` tests in `GarageDoorService.h` |
| Rolling code rejected after many reboots | NVS not persisting | The boot bump (+64) covers normal operation; check the NVS partition exists (`min_spiffs.csv`) |
| WiFi provisioning stuck | First boot, no credentials | HomeSpan CLI: type `W` in the serial monitor to set WiFi credentials |

**Learn mode (diagnostic only):** typing `t` in the serial monitor arms a one-shot door command for the next learn-mode window (Learn LED lit, reported in STATUS byte2 bit5).  This is *not* required for operation — wireline control needs no enrolment.

---

## Extending the Project

- **Motion sensor:** the opener broadcasts `MOTION (0x285)` — could feed a HomeKit MotionSensor service
- **Openings counter:** `GET_OPENINGS (0x48B)` returns the lifetime door-cycle count
- **Power from the opener:** MP1584 12 V→5 V buck from RED to VIN eliminates the USB cable
- **PCB:** the circuit maps to a small two-layer board; the schematic JSON is the reference netlist

---

## References

- [ratgdo project](https://paulwieland.github.io/ratgdo/) — original Security+ 2.0 wireline reverse engineering by Paul Wieland
- [ESPHome ratgdo](https://github.com/ratgdo/esphome-ratgdo) — command codes, payload layout, preamble timing, and obstruction sensor behavior (all verified against this source)
- [argilo/secplus](https://github.com/argilo/secplus) — Security+ codec C library (Clayton Smith, GPL-3.0)
- [HomeSpan](https://github.com/HomeSpan/HomeSpan) — ESP32 HomeKit Arduino library
- [rat-ratgdo open-source schematics](https://github.com/Kaldek/rat-ratgdo) — community PCB designs (Kaldek)
- [Craftsman 045DCT at LiftMaster](https://www.liftmaster.com/receiver-logic-board-security-2-0/p/045DCT) — product page / part reference
- [Espressif ESP32 Dev Module](https://docs.platformio.org/en/latest/boards/espressif32/esp32dev.html) — board documentation
- [Espressif Hardware Design Guidelines](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/index.html) — ESP32 hardware design reference
