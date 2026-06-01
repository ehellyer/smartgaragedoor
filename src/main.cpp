// =============================================================================
//  Smart Garage Door Controller  v1.1
//  ESP32 · Security+ 2.0 Wireline Bus · Apple HomeKit (HomeSpan)
//
//  Target opener:   Craftsman 045DCT  (LiftMaster/Chamberlain Security+ 2.0)
//                   Yellow learn button · manufactured 07/2015
//
//  Architecture:
//    ┌──────────────────────────────────────────────────────────────────────┐
//    │  HomeSpan (HAP)  ←→  GarageDoorService  ←→  GDOBus (secplus.h)     │
//    │                              ↕                                       │
//    │                   SW1 (closed)   SW2 (open)                         │
//    └──────────────────────────────────────────────────────────────────────┘
//
//  PlatformIO / VS Code project.
//  Libraries are declared in platformio.ini and fetched automatically.
//
//  GPIO Assignments:
//    GPIO 21  →  AO3400A gate  (TX: pulls GDO RED wire LOW when pin is HIGH)
//    GPIO 22  →  2N7000  drain (RX: HIGH when bus is LOW — inverted by MOSFET)
//    GPIO 25  →  Reed switch SW1 — CLOSED end-stop  (INPUT_PULLUP; LOW = closed)
//    GPIO 26  →  Reed switch SW2 — OPEN   end-stop  (INPUT_PULLUP; LOW = open)
//
//  First boot:
//    1. Flash via USB (PlatformIO: click Upload or press Ctrl+Alt+U).
//    2. Open Serial Monitor (Ctrl+Alt+S) at 115200 baud.
//    3. HomeSpan prints a setup code — use it to pair in the Apple Home app.
//    4. A random device_id and starting rolling code are generated on first
//       boot and stored in NVS flash.  Subsequent reboots reload them.
// =============================================================================

#include <Arduino.h>   // Required by PlatformIO — not needed in Arduino IDE .ino files
#include "HomeSpan.h"
#include "GDOBus.h"
#include "GarageDoor.h"

// ── Pin Definitions ──────────────────────────────────────────────────────────
#define GDO_TX_PIN       21    // AO3400A gate  (open-drain pull-down on 12V bus)
#define GDO_RX_PIN       22    // 2N7000  drain (level-shifted, inverted 12V→3.3V)
#define REED_CLOSED_PIN  25    // SW1 — closed end-stop (LOW = door at closed position)
#define REED_OPEN_PIN    26    // SW2 — open   end-stop (LOW = door at open  position)

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println(F("\n\n=== Smart Garage Door Controller v1.1 ==="));

    // Initialise the Security+ 2.0 serial bus driver first so we start
    // receiving status packets as soon as the bus is ready.
    GDOBus::begin(GDO_TX_PIN, GDO_RX_PIN);

    // Configure HomeSpan for native HomeKit (no hub needed).
    homeSpan.enableOTA();                          // OTA firmware updates
    homeSpan.setSketchVersion("1.1.0");
    homeSpan.setQRID("GRAG");                      // 4-char ID used in QR code

    homeSpan.begin(Category::GarageDoorOpeners, "Garage Door");

    // ── Accessory definition ────────────────────────────────────────────────
    new SpanAccessory();

        new Service::AccessoryInformation();
            new Characteristic::Identify();
            new Characteristic::Name("Garage Door");
            new Characteristic::Manufacturer("Hellyer Multimedia");
            new Characteristic::Model("HMGDO001");
            new Characteristic::SerialNumber("GDO-001");
            new Characteristic::FirmwareRevision("1.1.0");

        new GarageDoorService(REED_CLOSED_PIN, REED_OPEN_PIN);
}

// ── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
    homeSpan.poll();   // Drives HAP, WiFi provisioning, and GarageDoorService callbacks
    GDOBus::poll();    // Reads and decodes incoming Security+ 2.0 bus packets
}
