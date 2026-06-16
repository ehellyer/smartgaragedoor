// =============================================================================
//  GDOBus.h  —  Security+ 2.0 Wireline Serial Bus Driver
//
//  Handles all low-level communication with the Chamberlain / LiftMaster /
//  Craftsman Security+ 2.0 garage door opener over the wired serial bus
//  (the same bus that the wall panel uses).
//
//  Physical layer
//  ──────────────
//  The bus is a two-wire, open-collector, half-duplex serial link:
//    RED   wire  — combined +12 V supply and serial data (active LOW)
//    WHITE wire  — common ground
//
//  The line is pulled to +12 V when idle.  Data bits are transmitted by
//  momentarily pulling the line to GND.  An N-channel MOSFET circuit
//  (see schematic) converts the 12 V signal to ESP32-compatible 3.3 V
//  and inverts it so that idle = HIGH on the ESP32 GPIO, matching normal
//  UART convention.  The ESP32 UART is configured with invert=true to
//  compensate for both MOSFETs' signal inversions.
//
//  Serial parameters:  9600 baud · 8N1 · inverted (handled by UART flag)
//
//  Packet format (Security+ 2.0 wireline — from argilo/secplus):
//    Byte  0     = 0x55  (sync)
//    Byte  1     = 0x01  (sync)
//    Byte  2     = 0x00  (sync)
//    Bytes 3–10  = first  half  (8 bytes, encoded rolling + fixed + data)
//    Bytes 11–18 = second half  (8 bytes)
//    Total       = 19 bytes
//
//  Libraries:  secplus  (argilo/secplus, GPL-3.0)
// =============================================================================

#pragma once
#include <Arduino.h>

// ── Packet length ─────────────────────────────────────────────────────────────
#define GDO_PACKET_LEN      19      // 3 sync bytes + 2 × 8-byte encoded halves
#define GDO_BAUD            9600    // Security+ 2.0 wireline baud rate
#define GDO_SERIAL          Serial2 // ESP32 UART2 (remapped to custom pins)

// ── Debug level ───────────────────────────────────────────────────────────────
//   0 = silent  — no serial output from GDOBus (except errors)
//   1 = normal  — decoded packets, state changes, TX confirmations
//   2 = verbose — everything in level 1 PLUS raw hex dump of every packet
#define GDO_DEBUG_LEVEL     1

// ── Wireline Command Codes ────────────────────────────────────────────────────
// Verified against the ratgdo project source (components/ratgdo/secplus2.cpp)
// and against live bus captures from this opener + wall unit.
// Reference: https://github.com/ratgdo/esphome-ratgdo
#define GDO_CMD_GET_STATUS      0x0080  // Request a status report (opener replies with 0x081)
#define GDO_CMD_STATUS          0x0081  // Status report — sent on request or when state changes (NOT periodic)
#define GDO_CMD_OBST_1          0x0084  // Obstruction sensor broadcast 1 (seen on bus; not parsed)
#define GDO_CMD_OBST_2          0x0085  // Obstruction sensor broadcast 2 (seen on bus; not parsed)
#define GDO_CMD_LOCK            0x018C  // Lock action (nibble: 0=unlock 1=lock 2=toggle)
#define GDO_CMD_DOOR_ACTION     0x0280  // Door action (nibble: 0=close 1=open 2=toggle 3=stop; press+release)
#define GDO_CMD_LIGHT           0x0281  // Light action (nibble: 0=off 1=on 2=toggle)
#define GDO_CMD_MOTOR_ON        0x0284  // Motor-running broadcast from the opener
#define GDO_CMD_MOTION          0x0285  // Motion detected broadcast

// ── Door State ────────────────────────────────────────────────────────────────
// Mirrors the HAP CurrentDoorState values so they can be used directly.
// (Note: the raw nibble in STATUS packets uses DIFFERENT values —
//  0=unknown 1=open 2=closed 3=stopped 4=opening 5=closing — and is
//  translated to this enum in handleDecoded().)
enum class GDODoorState : uint8_t {
    OPEN    = 0,   // Door is fully open
    CLOSED  = 1,   // Door is fully closed
    OPENING = 2,   // Door is in motion, moving to open
    CLOSING = 3,   // Door is in motion, moving to closed
    STOPPED = 4,   // Door stopped mid-travel
    UNKNOWN = 255, // State not yet received from the opener
};

// ── Door Action ───────────────────────────────────────────────────────────────
// Explicit door commands carried in the DOOR_ACTION nibble (ratgdo DoorAction).
// OPEN/CLOSE are idempotent — safe to repeat, no toggle guesswork needed.
enum class GDODoorAction : uint8_t {
    CLOSE  = 0,
    OPEN   = 1,
    TOGGLE = 2,
    STOP   = 3,
};

// Callback signature for door state changes received from the bus
typedef void (*GDOStateCallback)(GDODoorState newState);

// =============================================================================
//  GDOBus — static class (single instance)
// =============================================================================
class GDOBus {
public:
    // ── Lifecycle ─────────────────────────────────────────────────────────────

    // Initialise UART2, load rolling code / device_id from NVS, flush bus.
    // Call once from setup() BEFORE homeSpan.begin().
    static void begin(uint8_t tx_pin, uint8_t rx_pin);

    // Process incoming bytes from the UART.  Call every loop() iteration.
    static void poll();

    // ── Commands ──────────────────────────────────────────────────────────────

    // Transmit an explicit door action (OPEN / CLOSE / TOGGLE / STOP).
    // Per the Security+ 2.0 wireline protocol (verified against ratgdo), each
    // action is sent as a button-PRESS packet followed ~150 ms later by a
    // RELEASE packet; both share one rolling code, incremented after the pair.
    // Each packet is preceded by the required bus preamble (~1.3 ms LOW).
    // Blocks for ~150-300 ms.  OPEN and CLOSE are idempotent.
    //
    // NOTE: wireline control does not require a Learn-button ceremony, but the
    // opener validates rolling codes per device ID.  The device identity
    // (ID + rolling counter) is persisted in NVS and must remain consistent
    // across reboots for the opener to continue accepting commands.
    static void sendDoorAction(GDODoorAction action);

    // Ask the opener to report its status (door/light/lock/obstruction).
    // The opener replies with a 0x081 STATUS packet which updates the cached
    // state below.  It does NOT broadcast status periodically on its own —
    // only on request or when its state changes — so poll this when fresh
    // state is needed (e.g., while verifying a commanded door movement).
    static void requestStatus();

    // Switch the opener's light on/off (LIGHT 0x281; single packet, no
    // press/release pair).  Confirm via requestStatus() + getLightOn().
    static void sendLightAction(bool on);

    // Engage/release the remote lockout — blocks wireless remotes, the wired
    // bus keeps working (LOCK 0x18C; single packet).  Confirm via
    // requestStatus() + getLocked().
    static void sendLockAction(bool lock);

    // ── State ─────────────────────────────────────────────────────────────────

    // Last door state decoded from a bus status packet.
    // Returns UNKNOWN until at least one status packet has been received.
    static GDODoorState getDoorState()           { return _doorState; }

    // Reset cached state (e.g. after a power cycle where bus state is uncertain)
    static void clearState()                     { _doorState = GDODoorState::UNKNOWN; }

    // Register a callback invoked whenever the opener broadcasts a new state.
    // The callback runs inside poll() — keep it short.
    static void onStateChange(GDOStateCallback cb) { _stateCallback = cb; }

    // Last obstruction flag decoded from a STATUS packet (byte1 bit 6,
    // inverted: raw 0 = obstructed).  true = obstruction present.
    static bool getObstruction()                 { return _obstruction; }

    // Last light / remote-lockout states from a STATUS packet (byte2 bits 1/0).
    // Only meaningful once lastStateTimestamp() != 0 (first status received).
    static bool getLightOn()                     { return _lightOn; }
    static bool getLocked()                      { return _locked; }

    // Millisecond timestamp of the last STATUS packet decoded — updated on
    // every status report, even when the state is unchanged.  Use together
    // with requestStatus() to judge whether getDoorState() is fresh.
    static unsigned long lastStateTimestamp()    { return _lastStateTime; }

    // Millisecond timestamp of the last byte received — useful for bus-idle
    // detection from outside this class.
    static unsigned long lastRxTimestamp()       { return _lastRxTime; }

    // ── Diagnostics ───────────────────────────────────────────────────────────

    // Blocking bus health check — call once from setup() after begin().
    // Listens for timeoutMs milliseconds, then prints a full diagnostic report
    // to Serial and returns true if at least one packet decoded successfully.
    //
    // Report differentiates four failure modes:
    //   rawBytes == 0           → no signal  (wiring / MOSFET / pin issue)
    //   rawBytes > 0, sync == 0 → signal but wrong framing  (baud rate?)
    //   sync > 0, decoded == 0  → framing OK but decode fail (protocol mismatch)
    //   decoded > 0             → PASS
    static bool verify(uint32_t timeoutMs = 5000);

    // Running counters — reset at begin(), incremented during normal operation.
    static uint32_t totalBytesRx()     { return _statBytesRx;    }
    static uint32_t totalPacketsRx()   { return _statPacketsRx;  }
    static uint32_t totalDecodeErrors(){ return _statDecodeErr;  }
    static uint32_t totalTxCommands()  { return _statTxCmds;     }

    // Print a one-line running stats summary to Serial.
    static void printStats();


private:
    // ── Internal helpers ──────────────────────────────────────────────────────
    static void processRxByte(uint8_t b);            // Feed one byte through the packet assembler
    static void processPacket(const uint8_t *pkt);   // Decode a complete packet
    static void handleDecoded(uint32_t rolling, uint64_t device_id,
                              uint16_t command, uint32_t payload); // Act on a decoded packet
    static bool txPacket(uint16_t command, uint32_t payload,
                         bool incrementRolling);      // Encode + preamble + transmit one packet
    static void waitBusIdle();                        // Wait for RX_GAP_MS of bus silence (drains RX)
    static void sendPreamble();                       // ~1.3 ms bus-LOW frame preamble before TX
    static void loadIdentity();                        // Load NVS → _deviceId + _rolling (generate on first boot)
    static void saveRolling();                         // Persist _rolling to NVS (called sparsely)
    static void printHex(const uint8_t *buf, size_t len); // Hex dump to Serial

    // ── State ─────────────────────────────────────────────────────────────────
    static GDODoorState     _doorState;
    static bool             _obstruction;     // last obstruction flag from status packet
    static bool             _lightOn;         // last light state from status packet
    static bool             _locked;          // last remote-lockout state from status packet
    static unsigned long    _lastStateTime;   // millis() of last decoded status packet
    static GDOStateCallback _stateCallback;

    // ── Security+ 2.0 identity (persisted in NVS partition) ──────────────────
    static uint32_t         _rolling;    // 28-bit rolling counter
    static uint64_t         _deviceId;  // 40-bit wired-device identifier

    // ── UART RX ring buffer ───────────────────────────────────────────────────
    static uint8_t          _rxBuf[GDO_PACKET_LEN];
    static size_t           _rxIdx;
    static unsigned long    _lastRxTime;
    static uint8_t          _txPin;     // cached for the GPIO-matrix preamble

    // ── Running statistics ────────────────────────────────────────────────────
    static uint32_t         _statBytesRx;
    static uint32_t         _statPacketsRx;
    static uint32_t         _statDecodeErr;
    static uint32_t         _statTxCmds;

    // Gap (ms) between UART bytes that signals end-of-packet / new packet start.
    // At 9600 baud one byte takes ~1 ms; 20 ms gap is conservative.
    static const unsigned long RX_GAP_MS = 20;

    // Maximum time (ms) to wait for bus idle before forcing a TX.
    static const unsigned long TX_WAIT_MS = 500;

    // Gap (ms) between the door-action PRESS and RELEASE packets (ratgdo: 150).
    static const unsigned long DOOR_RELEASE_DELAY_MS = 150;
};
