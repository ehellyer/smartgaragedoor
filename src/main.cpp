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
//    GPIO 25  →  Reed switch SW1 — CLOSED end-stop  (NC switch + INPUT_PULLUP;
//                HIGH = magnet present = door closed; LOW = magnet absent)
//    GPIO 26  →  Reed switch SW2 — OPEN   end-stop  (NC switch + INPUT_PULLUP;
//                HIGH = magnet present = door open;   LOW = magnet absent)
//
//  First boot:
//    1. Flash via USB (PlatformIO: click Upload or press Ctrl+Alt+U).
//    2. Open Serial Monitor (Ctrl+Alt+S) at 115200 baud.
//    3. HomeSpan prints a setup code — use it to pair in the Apple Home app.
//    4. A random device_id and starting rolling code are generated on first
//       boot and stored in NVS flash.  Subsequent reboots reload them.
// =============================================================================

#include <Arduino.h> // Required by PlatformIO — not needed in Arduino IDE .ino files
#include <WiFi.h>
#include "GDOBus.h"
#include "GarageDoorService.h"
#include "HomeSpan.h"


// ── Pin Definitions ──────────────────────────────────────────────────────────
static constexpr uint8_t GDO_TX_PIN = 21; // AO3400A gate  (open-drain pull-down on 12V bus)
static constexpr uint8_t GDO_RX_PIN = 22; // 2N7000  drain (level-shifted, inverted 12V→3.3V)
static constexpr uint8_t REED_CLOSED_PIN = 25; // SW1 — closed end-stop (HIGH = door at closed position)
static constexpr uint8_t REED_OPEN_PIN = 26; // SW2 — open end-stop (HIGH = door at open position)
static constexpr uint8_t LED_PIN = 2;        // On-board LED (GPIO2)

// ── Constant Definitions ──────────────────────────────────────────────────────────
static constexpr uint32_t STATUS_LED_INTERVAL_MS = 500; // ms

// ── Status callback (called by HomeSpan each time status changes)
void statusUpdate(HS_STATUS status)
{
  Serial.printf("\n*** HOMESPAN STATUS CHANGE: %s\n", homeSpan.statusString(status));
}


// ── Connection callback (called by HomeSpan each time WiFi
// connects/reconnects) count == 1 → initial connection; count > 1 reconnection after dropout
void onConnection(int count)
{
  Serial.println(count == 1 ? F("\n=== WiFi Connected ===")
                            : F("\n=== WiFi Reconnected ==="));
  Serial.print(F("  SSID       : "));
  Serial.println(WiFi.SSID());
  Serial.print(F("  IP Address : "));
  Serial.println(WiFi.localIP());
  Serial.print(F("  Gateway    : "));
  Serial.println(WiFi.gatewayIP());
  Serial.print(F("  Subnet     : "));
  Serial.println(WiFi.subnetMask());
  Serial.print(F("  MAC Address: "));
  Serial.println(WiFi.macAddress());
  Serial.print(F("  Signal     : "));
  Serial.print(WiFi.RSSI());
  Serial.println(F(" dBm"));
}




// ── Setup ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    delay(200);
    Serial.println(F("\n\n=== Smart Garage Door Controller v1.0.0 ==="));

    // Configure HomeSpan for native HomeKit (no hub needed).
    homeSpan.setLogLevel(1);   // Enable LOG1 output (reed state, HAP events, TX confirmations)
    homeSpan.begin(Category::GarageDoorOpeners, "Garage Door Opener");

    homeSpan.setConnectionCallback(onConnection); // Print WiFi details on connect/reconnect
    homeSpan.setStatusCallback(statusUpdate);     // set callback function for status updates (e.g. to control an LED)
    homeSpan.setPairingCode("41414141");

    // ── Accessory definition ────────────────────────────────────────────────
    new SpanAccessory();
        new Service::AccessoryInformation();
            new Characteristic::Identify();
            new Characteristic::Name("Garage Door Opener");
            new Characteristic::Manufacturer("Hellyer Multimedia Group");
            new Characteristic::Model("HMG-GDO-001");
            new Characteristic::SerialNumber("GDO-001");
            new Characteristic::FirmwareRevision("1.0.0");


    // Initialise the Security+ 2.0 serial bus driver.
    GDOBus::begin(GDO_TX_PIN, GDO_RX_PIN);

    bool busOK = GDOBus::verify(1000);
    if (!busOK) 
    {
        Serial.println(F("[GDO] WARNING: Bus verify failed — check wiring"));
    }

    new GarageDoorService(REED_CLOSED_PIN, REED_OPEN_PIN);
}

// ── Main Loop ────────────────────────────────────────────────────────────────

static unsigned long lastStatusLEDms = 0;
static bool isStatusLEDOn = false;

void loop()
{
  // ── Learn-mode enrol trigger ──────────────────────────────────────────────
  // Type 't' (or 'T') + Enter in the serial monitor to arm auto-enrolment.
  // The arm is also saved to NVS so it survives a reboot.
  //
  // RECOMMENDED ENROLMENT PROCEDURE (wall unit connected):
  //   1. Type 't' + Enter  →  arm is set and saved to NVS
  //   2. Immediately type 'R' + Enter  →  device reboots
  //   3. While rebooting, press the Learn button on the opener
  //      (or press Learn just before step 2 — window is 30 s)
  //   4. During verify() (~t+2.9 s) state=6 is detected and TX fires
  //      automatically — well before the wall unit can respond (~t+4.8 s)
  //   5. Listen for the opener's enrolment beep/click
  //
  // If the ESP32 is already running (loop() active) you can also just:
  //   1. Type 't' + Enter,  2. Press Learn — TX fires within ~20 ms of state=6.
  if (Serial.available() && (Serial.peek() == 't' || Serial.peek() == 'T')) {
    Serial.read();                                    // consume the 't'
    while (Serial.available() && Serial.peek() < 32) // eat trailing CR / LF
      Serial.read();
    GDOBus::armLearnEnroll();
  }

  homeSpan.poll();    // Drives HAP, WiFi provisioning, and GarageDoorService callbacks
  GDOBus::poll();     // Reads and decodes incoming Security+ 2.0 bus packets

  unsigned long now = millis();
  if (now - lastStatusLEDms >= STATUS_LED_INTERVAL_MS) {
    lastStatusLEDms = now;
    isStatusLEDOn = !isStatusLEDOn;
    digitalWrite(LED_PIN, isStatusLEDOn);
  }
}