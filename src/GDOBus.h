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
// Discovered by Paul Wieland (ratgdo project) and the ESPHome ratgdo community.
// Reference: https://github.com/ratgdo/esphome-ratgdo
//
// NOTE: If your opener does not respond, enable GDO_DEBUG_RX in GDOBus.cpp
//       and observe the raw command codes printed to Serial.  You can then
//       substitute the correct values below.
#define GDO_CMD_DOOR_ACTION     0x0280  // Door button press  (toggle open/close/stop)
#define GDO_CMD_LIGHT_ACTION    0x0281  // Light button press (toggle opener light)
#define GDO_CMD_LOCK_ACTION     0x0284  // Lock  button press (toggle lock-out)
#define GDO_CMD_STATUS          0x0199  // Status broadcast from the opener (unconfirmed — not yet seen in log)
#define GDO_CMD_STATUS_2        0x0081  // Secondary status broadcast (confirmed — opener broadcasts this every ~2 s)

// ── Door State ────────────────────────────────────────────────────────────────
// Mirrors the HAP CurrentDoorState values so they can be used directly.
enum class GDODoorState : uint8_t {
    OPEN    = 0,   // Door is fully open
    CLOSED  = 1,   // Door is fully closed
    OPENING = 2,   // Door is in motion, moving to open
    CLOSING = 3,   // Door is in motion, moving to closed
    STOPPED = 4,   // Door stopped mid-travel
    UNKNOWN = 255, // State not yet received from the opener
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

    // Encode and transmit a door-toggle command (open→close, close→open, or
    // stop if the door is mid-travel).  Waits for bus idle before transmitting.
    // Rolling code is incremented and persisted to NVS after each transmission.
    static void sendDoorCommand();

    // ── State ─────────────────────────────────────────────────────────────────

    // Last door state decoded from a bus status packet.
    // Returns UNKNOWN until at least one status packet has been received.
    static GDODoorState getDoorState()           { return _doorState; }

    // Reset cached state (e.g. after a power cycle where bus state is uncertain)
    static void clearState()                     { _doorState = GDODoorState::UNKNOWN; }

    // Register a callback invoked whenever the opener broadcasts a new state.
    // The callback runs inside poll() — keep it short.
    static void onStateChange(GDOStateCallback cb) { _stateCallback = cb; }

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

    // Arm auto-TX for the next learn-mode window (opener broadcasts state=6).
    // Type 't' in the serial monitor.  Safe to call before or during the learn
    // window — fires exactly once when state=6 is first detected after arming.
    static void armLearnEnroll();

private:
    // ── Internal helpers ──────────────────────────────────────────────────────
    static void processPacket(const uint8_t *pkt);   // Decode a complete packet
    static void loadIdentity();                       // Load NVS → _rolling, _deviceId
    static void saveRolling();                        // Persist _rolling to NVS
    static void printHex(const uint8_t *buf, size_t len); // Hex dump to Serial

    // ── State ─────────────────────────────────────────────────────────────────
    static GDODoorState     _doorState;
    static GDOStateCallback _stateCallback;

    // ── Security+ 2.0 identity (persisted in NVS partition) ──────────────────
    static uint32_t         _rolling;    // 28-bit rolling counter
    static uint64_t         _deviceId;  // 40-bit wired-device identifier

    // ── UART RX ring buffer ───────────────────────────────────────────────────
    static uint8_t          _rxBuf[GDO_PACKET_LEN];
    static size_t           _rxIdx;
    static unsigned long    _lastRxTime;

    // ── Learn-mode auto-enrolment ─────────────────────────────────────────────
    static bool             _learnEnrollArmed;  // set by armLearnEnroll()
    static bool             _inLearnMode;       // true while opener broadcasts state=6
    static bool             _pendingLearnTX;    // TX deferred out of processPacket context

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
};
