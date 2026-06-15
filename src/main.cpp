// =============================================================================
//  Smart Garage Door Controller  v1.3.0
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
//    GPIO 34  →  Obstruction sensor (black) wire via 10K/10K divider to GND
//                (input-only; clear = LOW pulse every ~7 ms, obstructed = steady
//                HIGH, asleep = steady LOW)
//
//  First boot:
//    1. Flash via USB (PlatformIO: click Upload or press Ctrl+Alt+U).
//    2. Open Serial Monitor (Ctrl+Alt+S) at 115200 baud.
//    3. HomeSpan prints a setup code — use it to pair in the Apple Home app.
//    4. A random device_id and starting rolling code are generated on first
//       boot and stored in NVS flash.  Subsequent reboots reload them.
// =============================================================================

#include <Arduino.h> // Required by PlatformIO — not needed in Arduino IDE .ino files
#include <ArduinoOTA.h>
#include <WiFi.h>
#include "../.credentials/wifi_credentials.h"
#include "DuaLogger.h"
#include "GDOBus.h"
#include "GarageDoorService.h"
#include "LightService.h"
#include "LockService.h"
#include "HomeSpan.h"


DuaLogger Log;

// ── Pin Definitions ──────────────────────────────────────────────────────────
static constexpr uint8_t GDO_TX_PIN = 21; // AO3400A gate  (open-drain pull-down on 12V bus)
static constexpr uint8_t GDO_RX_PIN = 22; // 2N7000  drain (level-shifted, inverted 12V→3.3V)
static constexpr uint8_t REED_CLOSED_PIN = 25; // SW1 — closed end-stop (HIGH = door at closed position)
static constexpr uint8_t REED_OPEN_PIN = 26; // SW2 — open end-stop (HIGH = door at open position)
static constexpr uint8_t OBST_PIN = 34;      // Safety-sensor (black) wire via 10K/10K divider
                                             // (input-only pin; divider midpoint → GPIO34,
                                             //  bottom leg → GND.  Measured: 5.95 V line →
                                             //  2.95 V at the pin — within spec.)
static constexpr uint8_t LED_PIN = 2;        // On-board LED (GPIO2)


// ── Constant Definitions ──────────────────────────────────────────────────────────
static constexpr uint32_t STATUS_LED_INTERVAL_MS = 500; // ms

// ── Status callback (called by HomeSpan each time status changes)
void statusUpdate(HS_STATUS status)
{
    Log.printf("\n*** HOMESPAN STATUS CHANGE: %s\n", homeSpan.statusString(status));
}





// ── Setup ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    delay(200);
    Log.println(F("\n\n=== Smart Garage Door Controller v1.3.0 ==="));

    // Initialise the Security+ 2.0 serial bus driver.
    GDOBus::begin(GDO_TX_PIN, GDO_RX_PIN);

    bool busOK = GDOBus::verify(1000);
    if (!busOK)
    {
        Log.println(F("[GDO] WARNING: Bus verify failed — check wiring"));
    }

    // Seed the cached door state.  The opener does NOT broadcast status on its
    // own (only on request or when its state changes), so query it once at
    // boot; the 0x081 reply is decoded by GDOBus::poll() in the main loop.
    GDOBus::requestStatus();

    delay(200);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Log.println(F("Connection Failed! Rebooting..."));
        delay(5000);
        ESP.restart();
    }
    
    delay(200);

    Log.begin(23);

    // Configure HomeSpan for native HomeKit (no hub needed).
    homeSpan.setLogLevel(1);   // Enable LOG1 output (reed state, HAP events, TX confirmations)
    homeSpan.setHostNameSuffix("");
    homeSpan.begin(Category::GarageDoorOpeners, "Garage Door Opener", "esp32-garage-door-opener");

    homeSpan.setStatusCallback(statusUpdate);
    homeSpan.setPairingCode("41414141");

    // ── Accessory definition ────────────────────────────────────────────────
    new SpanAccessory();
        new Service::AccessoryInformation();
            new Characteristic::Identify();
            new Characteristic::Name("Garage Door Opener");
            new Characteristic::Manufacturer("Hellyer Multimedia Group");
            new Characteristic::Model("HMG-GDO-001");
            new Characteristic::SerialNumber("GDO-001");
            new Characteristic::FirmwareRevision("1.3.0");

    new GarageDoorService(REED_CLOSED_PIN, REED_OPEN_PIN, OBST_PIN);
    new LightService();   // Opener work light  (LIGHT 0x281)
    new LockService();    // Remote lockout     (LOCK  0x18C)

    // Configure ArduinoOTA
    ArduinoOTA.setHostname("esp32-garage-door-opener"); // Friendly network name

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Log.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Log.println(F("\nEnd"));
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Log.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Log.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)         Log.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)   Log.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Log.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Log.println("Receive Failed");
        else if (error == OTA_END_ERROR)     Log.println("End Failed");
    });

    ArduinoOTA.begin();
    Log.println(F("\n\nOTA Ready"));
    Log.print(F("IP address: "));
    Log.println(WiFi.localIP());
}

// ── Main Loop ────────────────────────────────────────────────────────────────

static unsigned long lastStatusLEDms = 0;
static bool isStatusLEDOn = false;

void loop()
{
  ArduinoOTA.handle();

  // ── Learn-mode enrol trigger ──────────────────────────────────────────────
  // Type 't' (or 'T') + Enter in the serial monitor to arm auto-enrolment.
  // The arm is also saved to NVS so it survives a reboot.
  //
  // RECOMMENDED ENROLMENT PROCEDURE (wall unit connected):
  //   1. Type 't' + Enter  →  arm is set and saved to NVS
  //   2. Immediately type 'R' + Enter  →  device reboots
  //   3. While rebooting, press the Learn button on the opener
  //      (or press Learn just before step 2 — window is 30 s)
  //   4. During verify(), early in boot, state=6 is detected and TX fires
  //      automatically — before the wall unit can respond to the learn window
  //   5. Listen for the opener's enrolment beep/click
  //
  // If the ESP32 is already running (loop() active) you can also just:
  //   1. Type 't' + Enter,  2. Press Learn — TX fires within ~20 ms of state=6.
  if (Serial.available() && (Serial.peek() == 't' || Serial.peek() == 'T')) {
    Serial.read();
    while (Serial.available() && Serial.peek() < 32)
      Serial.read();
    GDOBus::armLearnEnroll();
  }
  if (TelnetStream.available() && (TelnetStream.peek() == 't' || TelnetStream.peek() == 'T')) {
    TelnetStream.read();
    while (TelnetStream.available() && TelnetStream.peek() < 32)
      TelnetStream.read();
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