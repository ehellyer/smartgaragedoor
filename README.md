# Smart Garage Door Controller
## Craftsman 045DCT · ESP32 · Security+ 2.0 Wireline · Apple HomeKit

---

## Overview

This project adds native Apple HomeKit support to a **Craftsman 045DCT** (2015, yellow learn button) garage door opener by tapping directly into the Security+ 2.0 wired serial bus — the same bus the wall panel uses.  No RF radio is needed.  The ESP32 module connects to your WiFi and appears in the Apple Home app as a first-class **Garage Door** accessory, accessible by your whole family without any hub or cloud service.

**Key facts:**
- Opener communicates at 9600 baud on a single active-LOW wire (RED terminal)
- Protocol is Security+ 2.0 wireline, encoded/decoded with the `secplus` library
- HomeKit is implemented natively on the ESP32 using the `HomeSpan` library — no Homebridge or extra hardware required
- Two magnetic reed switches give definitive hardware signals for both "Closed" and "Fully Open" end-stops; when neither is triggered the door is mid-travel
- Rolling codes are persisted in NVS flash; the device survives reboots without needing re-pairing

---

## Bill of Materials

| Ref  | Part                          | Package        | Key Specs                    | Source / Notes                              |
|------|-------------------------------|----------------|------------------------------|---------------------------------------------|
| U1   | ESP32 DevKit (38-pin)         | DIP/module     | Dual-core, WiFi, BT          | Any ESP32-WROOM-32 board; Wemos D1 Mini ESP32 works too |
| Q1   | 2N7000 N-channel MOSFET       | TO-92          | Vgs(th) 1–3 V, Vds 60 V     | RX interface — level-shifts 12 V→3.3 V (inverted) |
| Q2   | AO3400A N-channel MOSFET      | SOT-23         | Vgs(th) 0.5–1.2 V, Vds 30 V | TX interface — pulls 12 V bus LOW; mount on SOT-23→DIP adapter |
| R1   | 10 kΩ resistor                | 1/4 W          | ±5%                          | 2N7000 gate current limit                   |
| R2   | 10 kΩ resistor                | 1/4 W          | ±5%                          | 2N7000 drain pull-up to 3.3 V               |
| R3   | 10 kΩ resistor                | 1/4 W          | ±5%                          | AO3400A gate drive from GPIO21              |
| R4   | 10 kΩ resistor                | 1/4 W          | ±5%                          | SW1 (CLOSED reed) pull-up to 3.3 V         |
| R5   | 10 kΩ resistor                | 1/4 W          | ±5%                          | SW2 (OPEN reed) pull-up to 3.3 V           |
| C1   | 100 nF ceramic capacitor      | 0805 or THT    | 50 V                         | Decoupling across RED/GND at the screw terminal |
| SW1  | Magnetic reed switch (NC)     | varies         | normally-closed              | **CLOSED** end-stop — door frame at bottom; magnet on door |
| SW2  | Magnetic reed switch (NC)     | varies         | normally-closed              | **OPEN** end-stop — track at fully-open position; magnet on door |
| J1   | 3-terminal screw terminal     | 5 mm pitch     | 300 V / 10 A                 | RED, GND, (spare) connections to opener     |
| —    | SOT-23 → DIP adapter board   | PCB            | 3-pin                        | Lets Q2 sit in a breadboard / perfboard     |
| —    | USB 5 V supply                | wall plug       | ≥500 mA                      | Powers ESP32; standard phone charger works  |
| —    | Wire, 22–26 AWG               | stranded        | —                            | Low current; thin wire is fine              |

**Optional — power from the opener's 12 V supply instead of USB:**

| Part                 | Notes                                             |
|----------------------|---------------------------------------------------|
| MP1584 / LM2596 buck converter | 12 V → 5 V; connect 5 V output to ESP32 VIN/USB |
| 100 µF electrolytic capacitor | Output filter for the buck converter |

---

## Circuit Description

### Security+ 2.0 Bus Interface

The Craftsman 045DCT exposes two terminals internally (accessible by opening the light-cover on the motor head unit):

| Terminal color | Signal            |
|----------------|-------------------|
| **RED**        | +12 V DC supply + serial data (active LOW) |
| **WHITE**      | Ground (0 V reference) |

The serial line is held at +12 V when idle. During data transmission any device on the bus (including the opener itself) pulls the line momentarily to GND for each bit.

**RX sub-circuit (reading the bus):**

```
RED ──[R1 10kΩ]──┬── GATE (Q1 / 2N7000)
                  │   SOURCE → GND
                  │   DRAIN ──[R2 10kΩ]── 3.3 V
                  │         └─────────── GPIO22 (ESP32 UART2 RX)
```

- Bus HIGH (12 V): Q1 gate driven → MOSFET ON → GPIO22 pulled LOW by Q1
- Bus LOW  (0 V):  Q1 gate at 0 V → MOSFET OFF → GPIO22 pulled HIGH by R2
- Result: signal on GPIO22 is **inverted** relative to the bus

**TX sub-circuit (writing to the bus):**

```
GPIO21 (ESP32 UART2 TX) ──[R3 10kΩ]──── GATE (Q2 / AO3400A)
                                          SOURCE → GND
                                          DRAIN ──────────────── RED
```

- GPIO21 HIGH (3.3 V): Q2 turns fully on → pulls RED bus to GND (sending a data bit)
- GPIO21 LOW  (0 V):   Q2 OFF → RED bus floats to 12 V (idle)
- Result: ESP32 TX logic is **inverted** relative to the bus

**UART inversion:**  
Both Q1 and Q2 invert their respective signals.  The ESP32 UART2 is configured with `invert=true` in `GDOBus::begin()`, which compensates for both inversions.  From the software's perspective the UART behaves normally.

### Reed Switches (dual end-stop sensors)

Both switches wire identically — one terminal to GND, the other to the GPIO pin with a pull-up resistor. `INPUT_PULLUP` is also enabled in firmware as a belt-and-suspenders measure.

```
3.3 V ──[R4 10kΩ]──┬── GPIO25  (CLOSED end-stop)
                    │
                   [SW1 — mounted at bottom of door frame]
                    │
                   GND

3.3 V ──[R5 10kΩ]──┬── GPIO26  (OPEN end-stop)
                    │
                   [SW2 — mounted at fully-open position on overhead track]
                    │
                   GND
```

**State truth table:**

| SW1 (GPIO25) | SW2 (GPIO26) | Door state |
|---|---|---|
| LOW | HIGH | **Closed** — magnet at bottom |
| HIGH | LOW | **Open** — magnet at top of travel |
| HIGH | HIGH | **In between** — Opening, Closing, or Stopped |
| LOW | LOW | Error (impossible mechanically) |

**Mounting the open-position switch (SW2):**  
For a typical sectional overhead door, the "fully open" position is when the bottom panel has cleared the vertical track and is lying flat along the ceiling rails.  Mount SW2 on the overhead track where the bottom panel comes to rest, and the magnet on the door's bottom panel.  The exact position depends on your door geometry — test manually before final mounting.

---

## GPIO Assignments

| GPIO | Direction | Signal                       | Connected to                          |
|------|-----------|------------------------------|---------------------------------------|
| 21   | Output    | UART2 TX (inverted)          | R3 → Q2 (AO3400A) gate               |
| 22   | Input     | UART2 RX (inverted)          | Q1 (2N7000) drain + R2 pull-up        |
| 25   | Input     | Reed switch — CLOSED end     | SW1 + R4 pull-up to 3.3 V            |
| 26   | Input     | Reed switch — OPEN end       | SW2 + R5 pull-up to 3.3 V            |

---

## Software Setup (VS Code + PlatformIO)

### 1. Install VS Code and PlatformIO

1. Download and install [Visual Studio Code](https://code.visualstudio.com/)
2. Open the Extensions panel (`Ctrl+Shift+X` / `Cmd+Shift+X`), search **PlatformIO IDE**, install it
3. Restart VS Code — PlatformIO will finish installing in the background (takes a minute)

### 2. Open the project

1. In VS Code, choose **File → Open Folder**
2. Navigate to and select the `Make dumb garage door opener smart` folder (the one containing `platformio.ini`)
3. PlatformIO will detect the `platformio.ini` and activate the project automatically

### 3. Install libraries (automatic)

No manual library installation needed.  On the **first build**, PlatformIO reads `platformio.ini` and automatically clones and compiles:

| Library   | Source                                          |
|-----------|-------------------------------------------------|
| HomeSpan  | `https://github.com/HomeSpan/HomeSpan.git`      |
| secplus   | `https://github.com/argilo/secplus.git`         |

Libraries are cached in `~/.platformio/packages/` and reused across projects.

### 4. Select your board

The default environment in `platformio.ini` targets a generic **ESP32 Dev Module** (`esp32dev`).  If you are using a **Wemos D1 Mini ESP32**, open `platformio.ini` and uncomment the `[env:wemos_d1_mini32]` section (and comment out `[env:esp32dev]`).

### 5. Configure pin assignments

Open `src/main.cpp`.  The default pin assignments match the schematic:

```cpp
#define GDO_TX_PIN       21    // AO3400A gate
#define GDO_RX_PIN       22    // 2N7000 drain
#define REED_CLOSED_PIN  25    // SW1 — closed end-stop
#define REED_OPEN_PIN    26    // SW2 — open end-stop
```

Change these if you wire to different GPIO pins.

### 6. Build and flash

| Action        | Keyboard shortcut  | PlatformIO toolbar button |
|---------------|--------------------|---------------------------|
| Build         | `Ctrl+Alt+B`       | ✓ (checkmark)             |
| Upload        | `Ctrl+Alt+U`       | → (arrow)                 |
| Serial Monitor| `Ctrl+Alt+S`       | plug icon                 |
| Clean         | —                  | trash icon                |

On first build PlatformIO fetches the libraries, which may take 1–2 minutes.  Subsequent builds are fast.

### 7. Pair with Apple HomeKit

1. Open **Serial Monitor** at 115200 baud
2. HomeSpan prints a setup code, e.g.:
   ```
   *** WELCOME TO HOMESPAN! ***
   ...
   To pair, enter setup code: 466-37-726
   ```
3. Open the **Apple Home app** → tap **+** → **Add Accessory**
4. Select **More Options** → find **Garage Door** → enter the setup code
5. Assign it to your home and a room

Alternatively, HomeSpan can display a QR code on a connected OLED; see the HomeSpan documentation for details.

---

## How It Works (end-to-end)

1. **Startup:** ESP32 boots, loads rolling code + device_id from NVS flash, connects to WiFi, registers with HomeKit.
2. **Listening:** GDOBus continuously receives Security+ 2.0 status packets broadcast by the opener.  Each packet is decoded with `decode_wireline_command()` and the door state is extracted from the payload.
3. **Reed switches:** Both GPIO25 and GPIO26 are polled and debounced every loop.  SW1 (closed) going LOW immediately sets the state to Closed; SW2 (open) going LOW immediately sets it to Open.  These are the authoritative signals — no bus packet can override them.
4. **Open command:** When the Home app requests "Open", HomeSpan calls `GarageDoorService::update()`.  This calls `GDOBus::sendDoorCommand()` which encodes a Security+ 2.0 wireline command packet (door toggle, command code `0x0280`) using `encode_wireline_command()`, waits for the bus to be idle, then writes the 19-byte packet to UART2.  The rolling counter is incremented and persisted to NVS.
5. **In-between state:** While both reed switches read HIGH (door is mid-travel), bus status packets from the opener are used to distinguish Opening from Closing.  If no bus data is available, the direction is inferred from the last command and current target.  A 30-second travel timeout catches situations where neither end-stop is reached.

---

## First Boot — Verifying the Bus

On first boot with `GDO_DEBUG_RX 1` in `GDOBus.cpp`, open the Serial Monitor and **press the wall button or the existing remote**.  You should see lines like:

```
[GDO] RX  cmd=0x199  dev=0x...  roll=0x...  payload=0x00001
[GDO] Door state → CLOSED
```

If you see decode errors, the baud rate may differ from 9600 on your specific unit.  Try 4800 or 1200 baud by changing `GDO_BAUD` in `GDOBus.h`.

If the command code for status messages differs from `0x0199` or `0x0181`, update the `GDO_CMD_STATUS` / `GDO_CMD_STATUS_2` defines in `GDOBus.h` to match what you observe.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| No RX packets at all | MOSFET circuit wiring error | Check R1, R2, Q1 orientation; verify RED/WHITE connections |
| Decode errors only | Wrong baud rate | Try `#define GDO_BAUD 4800` |
| Commands sent but door doesn't move | Wrong command code | Enable `GDO_DEBUG_RX`, observe what the wall button sends, use that command code |
| HomeKit shows wrong state | Payload bit positions differ | Compare raw payload values when door is known open/closed; adjust bit mask in `processPacket()` |
| WiFi provisioning stuck | Normal on first boot | HomeSpan creates an AP for setup; follow HomeSpan docs for WiFi credential entry |
| Reed switch state inverted | NC vs NO switch | Change `== LOW` to `== HIGH` in the relevant `digitalRead()` call in `GarageDoor.h` constructor |
| SW2 never triggers "Open" | Magnet misaligned at open position | Move magnet/switch until LED on reed switch activates when door is fully up; a strong N52 magnet helps with alignment tolerance |
| Rolling code rejected after reboot | NVS not persisting | Ensure partition scheme includes NVS; `Default 4MB with spiffs` works |

---

## Extending the Project

- **Control the opener's built-in light:** Send `GDO_CMD_LIGHT_ACTION (0x0281)` from a new `SpanButton` or a second HomeKit switch accessory
- **Add OTA updates:** Already enabled via `homeSpan.enableOTA()` — use Arduino IDE's network port after first flash
- **Add a status LED:** Wire an LED+resistor to any free GPIO; set it from `GarageDoorService::loop()`
- **Power from the opener:** Add an MP1584 12V→5V buck converter between the RED wire and ESP32 VIN to eliminate the USB cable
- **PCB design:** The circuit maps directly to a two-layer 50×40 mm PCB — KiCad files can be generated from the schematic above

---

## References

- [ratgdo project](https://paulwieland.github.io/ratgdo/) — original Security+ 2.0 wireline reverse engineering by Paul Wieland
- [rat-ratgdo open-source schematics](https://github.com/Kaldek/rat-ratgdo) — community PCB designs (Kaldek)
- [argilo/secplus](https://github.com/argilo/secplus) — Security+ codec C library (Clayton Smith, GPL-3.0)
- [HomeSpan](https://github.com/HomeSpan/HomeSpan) — ESP32 HomeKit Arduino library
- [ESPHome ratgdo](https://github.com/ratgdo/esphome-ratgdo) — reference for command codes and payload layout
- [Craftsman 045DCT at LiftMaster](https://www.liftmaster.com/receiver-logic-board-security-2-0/p/045DCT) — product page / part reference
- [Espressif ESP32 Dev Module](https://docs.platformio.org/en/latest/boards/espressif32/esp32dev.html) - Datasheet and information about the ESP32 Dev Module 
- [Espressif Hardware Design Guidelines](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/index.html) - This document provides guidelines for the ESP32 SoC