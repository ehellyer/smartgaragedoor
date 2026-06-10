// =============================================================================
//  GDOBus.cpp  —  Security+ 2.0 Wireline Bus Driver  (implementation)
// =============================================================================

#include "GDOBus.h"
#include "secplus.h"      // argilo/secplus  — GPL-3.0
#include <Preferences.h>  // ESP32 NVS (non-volatile storage)
#include <esp_random.h>   // Hardware RNG

// ── Static member definitions ─────────────────────────────────────────────────
GDODoorState     GDOBus::_doorState      = GDODoorState::UNKNOWN;
GDOStateCallback GDOBus::_stateCallback  = nullptr;
uint32_t         GDOBus::_rolling        = 0;
uint64_t         GDOBus::_deviceId       = 0;
uint8_t          GDOBus::_rxBuf[GDO_PACKET_LEN] = {};
size_t           GDOBus::_rxIdx          = 0;
unsigned long    GDOBus::_lastRxTime     = 0;
bool             GDOBus::_learnEnrollArmed = false;
bool             GDOBus::_inLearnMode      = false;
bool             GDOBus::_pendingLearnTX   = false;
uint32_t         GDOBus::_statBytesRx    = 0;
uint32_t         GDOBus::_statPacketsRx  = 0;
uint32_t         GDOBus::_statDecodeErr  = 0;
uint32_t         GDOBus::_statTxCmds    = 0;

// NVS namespace and key names
static const char *NVS_NS         = "gdo";

// ── Boot-relative timestamp helper ────────────────────────────────────────────
// Prints "[mm:ss.ttt] " to Serial (no newline).
// Call immediately before any [GDO] log line.
static void ts() {
    uint32_t m = millis();
    Serial.printf("[%02u:%02u.%03u] ",
                  (unsigned)(m / 60000UL) % 100,
                  (unsigned)(m / 1000UL)  % 60,
                  (unsigned)(m            % 1000UL));
}
static const char *NVS_ROLLING    = "rolling";
static const char *NVS_DEVICE_ID  = "devId";
static const char *NVS_LEARN_ARM  = "learnArm"; // persistent enrol-arm flag

// =============================================================================
//  begin()  —  Initialise UART and load identity from NVS
// =============================================================================
void GDOBus::begin(uint8_t tx_pin, uint8_t rx_pin) {
    loadIdentity();

    // Configure UART2 for the Security+ 2.0 wireline bus.
    //
    //  invert = true  is ESSENTIAL:
    //    • TX MOSFET (AO3400A): ESP32 HIGH → gate driven → drain pulls bus LOW.
    //      Without inversion UART idle (HIGH) would constantly hold the bus low.
    //    • RX MOSFET (2N7000):  Bus HIGH → MOSFET on → GPIO LOW; Bus LOW → GPIO HIGH.
    //      Without inversion UART sees inverted logic.
    //
    //  With invert=true both MOSFETs' inversions are cancelled transparently.
    GDO_SERIAL.begin(GDO_BAUD, SERIAL_8N1, rx_pin, tx_pin, /*invert=*/true);

    // Flush any stale bytes that arrived during power-on
    delay(100);
    while (GDO_SERIAL.available()) GDO_SERIAL.read();

    // Reset stats
    _statBytesRx   = 0;
    _statPacketsRx = 0;
    _statDecodeErr = 0;
    _statTxCmds    = 0;

    ts(); Serial.println(F("[GDO] ── Bus Initialised ──────────────────────────────────"));
    ts(); Serial.printf( "[GDO]   UART         : Serial2  %d baud  8N1  inverted\n", GDO_BAUD);
    ts(); Serial.printf( "[GDO]   TX pin       : GPIO%d  (AO3400A gate)\n",  tx_pin);
    ts(); Serial.printf( "[GDO]   RX pin       : GPIO%d  (2N7000 drain)\n",  rx_pin);
    ts(); Serial.printf( "[GDO]   Device ID    : 0x%010llX\n", _deviceId);
    ts(); Serial.printf( "[GDO]   Rolling code : 0x%07X\n",    _rolling);
    Serial.println(F("[GDO] ─────────────────────────────────────────────────────\n"));
}

// =============================================================================
//  verify()  —  Blocking bus health check for use in setup().
//               Listens for timeoutMs ms, then prints a diagnostic report.
//               Returns true if at least one packet decoded successfully.
// =============================================================================
bool GDOBus::verify(uint32_t timeoutMs) {
    Serial.println(F("[GDO] ── Bus Verification ────────────────────────────────"));
    Serial.printf( "[GDO]   Listening for %lu ms ...\n", timeoutMs);
    Serial.println(F("[GDO]   (Press the wall button to send a packet immediately)"));

    uint32_t rawBytes    = 0;
    uint32_t syncFound   = 0;
    uint32_t fullPackets = 0;
    uint32_t decodeOK    = 0;
    uint32_t decodeErr   = 0;

    // Use a local buffer so we don't disturb the normal _rxBuf / _rxIdx state
    uint8_t  buf[GDO_PACKET_LEN];
    size_t   idx          = 0;
    unsigned long lastRx  = 0;

    unsigned long deadline = millis() + timeoutMs;

    while (millis() < deadline) {
        while (GDO_SERIAL.available()) {
            uint8_t b        = (uint8_t)GDO_SERIAL.read();
            unsigned long now = millis();
            rawBytes++;

            // Inter-packet gap → reset buffer
            if (idx > 0 && (now - lastRx) > RX_GAP_MS) {
                idx = 0;
            }
            lastRx = now;

            // Sync header validation
            if (idx == 0 && b != 0x55) continue;
            if (idx == 1 && b != 0x01) { idx = 0; continue; }
            if (idx == 2 && b != 0x00) { idx = 0; continue; }
            if (idx == 0) syncFound++;

            buf[idx++] = b;

            if (idx == GDO_PACKET_LEN) {
                fullPackets++;
                idx = 0;

                uint32_t rolling; uint64_t device_id; uint16_t command; uint32_t payload;
                if (decode_wireline_command(buf, &rolling, &device_id, &command, &payload) < 0) {
                    decodeErr++;
                    ts(); Serial.print(F("[GDO]   Decode error — raw bytes: "));
                    printHex(buf, GDO_PACKET_LEN);
                } else {
                    decodeOK++;
                    ts(); Serial.printf("[GDO]   Packet OK  cmd=0x%03X  dev=0x%010llX"
                                        "  roll=0x%07X  payload=0x%05X\n",
                                        command, device_id, rolling, payload);
                    // Feed decoded packet into normal state machine
                    processPacket(buf);
                    // Execute any TX queued by processPacket (e.g., learn-enrol arm was
                    // restored from NVS and state=6 was just detected).  This fires the
                    // enrolment TX at ~[00:02.9s] — before the wall unit responds (~[00:04.8s]).
                    if (_pendingLearnTX) {
                        _pendingLearnTX = false;
                        ts(); Serial.println(F("[GDO] *** Firing enrolment TX during verify() ***"));
                        sendDoorCommand();
                    }
                }
            }
        }
        delay(1);
    }

    // ── Diagnostic report ────────────────────────────────────────────────────
    ts(); Serial.println(F("[GDO] ── Verification Report ────────────────────────────"));
    Serial.printf( "[GDO]   Raw bytes received  : %lu\n", rawBytes);
    Serial.printf( "[GDO]   Sync sequences found: %lu\n", syncFound);
    Serial.printf( "[GDO]   Complete packets    : %lu\n", fullPackets);
    Serial.printf( "[GDO]   Decoded OK          : %lu\n", decodeOK);
    Serial.printf( "[GDO]   Decode errors       : %lu\n", decodeErr);

    // Diagnosis
    if (rawBytes == 0) {
        Serial.println(F("[GDO]   >> No bytes received — check wiring:"));
        Serial.println(F("[GDO]      - RED/WHITE terminals connected?"));
        Serial.println(F("[GDO]      - Q1 (2N7000) gate/drain/source orientation?"));
        Serial.println(F("[GDO]      - R2 pull-up to 3.3V on drain?"));
        Serial.println(F("[GDO]      - Correct RX pin in main.cpp?"));
    } else if (syncFound == 0) {
        Serial.println(F("[GDO]   >> Bytes arriving but no valid sync (0x55 0x01 0x00)"));
        Serial.println(F("[GDO]      - Wrong baud rate? Try GDO_BAUD 4800 in GDOBus.h"));
        Serial.println(F("[GDO]      - UART invert=true present in begin()?"));
    } else if (decodeOK == 0 && fullPackets > 0) {
        Serial.println(F("[GDO]   >> Packets assembled but decode failing"));
        Serial.println(F("[GDO]      - Protocol version mismatch?"));
        Serial.println(F("[GDO]      - Check raw bytes above against secplus.h"));
    } else if (decodeOK == 0) {
        Serial.println(F("[GDO]   >> Some bytes received but no complete packet yet"));
        Serial.println(F("[GDO]      - Opener may be idle — press wall button and re-verify"));
    }

    bool passed = (decodeOK > 0);
    ts(); Serial.printf("[GDO]   Result : %s\n", passed ? "PASS" : "FAIL");
    Serial.println(F("[GDO] ─────────────────────────────────────────────────────\n"));

    // Seed the running stats with what we saw during verify
    _statBytesRx   += rawBytes;
    _statPacketsRx += decodeOK;
    _statDecodeErr += decodeErr;

    return passed;
}

// =============================================================================
//  printStats()  —  One-line running stats summary.
// =============================================================================
void GDOBus::printStats() {
    Serial.printf("[GDO] Stats  rx=%lu pkts  %lu bytes  %lu decodeErr  tx=%lu cmds\n",
                  _statPacketsRx, _statBytesRx, _statDecodeErr, _statTxCmds);
}

// =============================================================================
//  loadIdentity()  —  Restore rolling counter and device_id from NVS.
//                     On first boot a random identity is generated and stored.
// =============================================================================
void GDOBus::loadIdentity() {
    Preferences prefs;
    prefs.begin(NVS_NS, false);

    _deviceId = prefs.getULong64(NVS_DEVICE_ID, 0);
    _rolling  = prefs.getULong(NVS_ROLLING, 0);

    // Restore persistent learn-enrol arm (set by armLearnEnroll()).
    // Consumed here — fires exactly once, during the next verify() or poll() that
    // sees state=6.  This lets enrolment TX fire during verify() at ~[00:02.9s],
    // well before the wall unit can respond (~[00:04.8s]).
    if (prefs.getBool(NVS_LEARN_ARM, false)) {
        _learnEnrollArmed = true;
        prefs.putBool(NVS_LEARN_ARM, false);    // consume — arm fires at most once
        ts(); Serial.println(F("[GDO] Persistent learn-enrol arm restored from NVS"));
    }

    if (_deviceId == 0) {
        // First boot — generate a random 40-bit wired-device identity
        uint32_t rand32;
        esp_fill_random(&rand32, sizeof(rand32));
        _deviceId = 0xF000000000ULL | (uint64_t)rand32;
        _rolling  = 0x1000 + (rand32 & 0x0FFF);

        prefs.putULong64(NVS_DEVICE_ID, _deviceId);
        prefs.putULong(NVS_ROLLING, _rolling);

        ts(); Serial.println(F("[GDO] First boot — new identity generated and stored in NVS"));
    }

    prefs.end();
}

// =============================================================================
//  saveRolling()  —  Persist rolling counter to NVS after each TX.
// =============================================================================
void GDOBus::saveRolling() {
    Preferences prefs;
    prefs.begin(NVS_NS, false);
    prefs.putULong(NVS_ROLLING, _rolling);
    prefs.end();
}

// =============================================================================
//  printHex()  —  Dump a byte buffer as hex to Serial (for debug).
// =============================================================================
void GDOBus::printHex(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.println();
}

// =============================================================================
//  poll()  —  Read UART bytes and assemble + decode complete packets.
//             Must be called every loop() iteration.
// =============================================================================
void GDOBus::poll() {
    while (GDO_SERIAL.available()) {
        uint8_t b        = (uint8_t)GDO_SERIAL.read();
        unsigned long now = millis();
        _statBytesRx++;

        // Inter-packet gap → discard partial buffer
        if (_rxIdx > 0 && (now - _lastRxTime) > RX_GAP_MS) {
#if GDO_DEBUG_LEVEL >= 2
            ts(); Serial.printf("[GDO] RX gap reset (had %u bytes)\n", (unsigned)_rxIdx);
#endif
            _rxIdx = 0;
        }
        _lastRxTime = now;

        // Validate sync header
        if (_rxIdx == 0 && b != 0x55) continue;
        if (_rxIdx == 1 && b != 0x01) { _rxIdx = 0; continue; }
        if (_rxIdx == 2 && b != 0x00) { _rxIdx = 0; continue; }

        _rxBuf[_rxIdx++] = b;

        if (_rxIdx == GDO_PACKET_LEN) {
#if GDO_DEBUG_LEVEL >= 2
            ts(); Serial.print(F("[GDO] RX raw: "));
            printHex(_rxBuf, GDO_PACKET_LEN);
#endif
            processPacket(_rxBuf);
            _rxIdx = 0;
        }
    }

    // Execute any TX deferred from processPacket() — keeps TX outside the
    // serial byte-receive context so the bus is fully idle before we write.
    if (_pendingLearnTX) {
        _pendingLearnTX = false;
        sendDoorCommand();
    }
}

// =============================================================================
//  processPacket()  —  Decode a complete 19-byte packet and update state.
// =============================================================================
void GDOBus::processPacket(const uint8_t *pkt) {
    uint32_t rolling;
    uint64_t device_id;
    uint16_t command;
    uint32_t payload;

    if (decode_wireline_command(pkt, &rolling, &device_id, &command, &payload) < 0) {
        _statDecodeErr++;
        ts(); Serial.print(F("[GDO] RX decode error — raw: "));
        printHex(pkt, GDO_PACKET_LEN);
        return;
    }

    _statPacketsRx++;

#if GDO_DEBUG_LEVEL >= 1
    ts(); Serial.printf("[GDO] RX  cmd=0x%03X  dev=0x%010llX  roll=0x%07X  payload=0x%05X\n",
                        command, device_id, rolling, payload);
#endif

    // ── Status broadcasts from the door controller ────────────────────────────
    //
    //  Payload bit mapping (Security+ 2.0 — ratgdo community research):
    //    bits [2:0] = door state   0=open 1=closed 2=opening 3=closing 4=stopped
    //    bit  [6]   = light        0=off  1=on
    //    bit  [9]   = lock         0=unlocked  1=locked
    //    bit  [12]  = obstruction  0=clear  1=obstructed
    //
    if (command == GDO_CMD_STATUS || command == GDO_CMD_STATUS_2) {
        uint8_t rawDoor = payload & 0x07;

        GDODoorState newState = GDODoorState::UNKNOWN;
        switch (rawDoor) {
            case 0:  newState = GDODoorState::OPEN;    break;
            case 1:  newState = GDODoorState::CLOSED;  break;
            case 2:  newState = GDODoorState::OPENING; break;
            case 3:  newState = GDODoorState::CLOSING; break;
            case 4:  newState = GDODoorState::STOPPED; break;
            default: break;
        }

#if GDO_DEBUG_LEVEL >= 1
        {
            bool lightOn = (payload >> 6)  & 1;
            bool locked  = (payload >> 9)  & 1;
            bool obst    = (payload >> 12) & 1;
            // rawDoor is bits[2:0] of payload → 0-7.  State 6 appears while the
            // opener's Learn LED is lit (learn mode active for this opener model).
            static const char *doorNames[] = {"OPEN","CLOSED","OPENING","CLOSING","STOPPED","?","LEARN?","?"};
            ts(); Serial.printf("[GDO]     door=%-7s  light=%-3s  lock=%-8s  obstruction=%s\n",
                               doorNames[rawDoor],   // rawDoor is 0-7; array has 8 entries
                               lightOn ? "ON"       : "off",
                               locked  ? "LOCKED"   : "unlocked",
                               obst    ? "YES"      : "no");
        }
#endif

        // ── Learn-mode detection (state=6 from opener while Learn LED is lit) ──
        bool nowInLearn = (rawDoor == 6);
        if (nowInLearn && !_inLearnMode) {
            // Just entered learn mode
            _inLearnMode = true;
            ts(); Serial.println(F("[GDO] *** Learn mode ACTIVE — type 't' to enrol this device ***"));
            if (_learnEnrollArmed) {
                _learnEnrollArmed = false;
                _pendingLearnTX   = true;   // executed in poll() after byte loop
                ts(); Serial.println(F("[GDO]     (auto-enrol armed — TX queued)"));
            }
        } else if (!nowInLearn && _inLearnMode) {
            // Learn window closed
            _inLearnMode = false;
            if (_learnEnrollArmed) {
                // Arm expired without ever seeing state=6
                _learnEnrollArmed = false;
                ts(); Serial.println(F("[GDO] Learn mode ended (enrol TX was never sent)"));
            } else {
                ts(); Serial.println(F("[GDO] Learn mode ended"));
            }
        }

        if (newState != GDODoorState::UNKNOWN && newState != _doorState) {
            _doorState = newState;
            static const char *names[] = {"OPEN","CLOSED","OPENING","CLOSING","STOPPED"};
            ts(); Serial.printf("[GDO] Door state → %s\n", names[(int)newState]);
            if (_stateCallback) _stateCallback(newState);
        }
    }
}

// =============================================================================
//  armLearnEnroll()  —  Arm auto-TX for the next learn-mode window.
//
//  When the opener broadcasts state=6 (Learn LED lit), processPacket() will
//  queue a sendDoorCommand() call via _pendingLearnTX.  The TX fires in
//  poll() once the serial byte loop finishes, ensuring bus idle before TX.
//
//  Safe to call before or during the learn window — if already in learn mode,
//  the TX is queued immediately.  Fires exactly once per arm() call.
// =============================================================================
void GDOBus::armLearnEnroll() {
    _learnEnrollArmed = true;

    // Persist the arm to NVS so it survives a reboot.  This enables enrolment TX
    // to fire during verify() (≈t+2.9s) — before the wall unit can respond (≈t+4.8s).
    // The flag is consumed in loadIdentity() on next boot; it fires exactly once.
    {
        Preferences prefs;
        prefs.begin(NVS_NS, false);
        prefs.putBool(NVS_LEARN_ARM, true);
        prefs.end();
    }

    if (_inLearnMode) {
        // Already in learn mode — queue TX immediately
        _learnEnrollArmed = false;
        _pendingLearnTX   = true;
        ts(); Serial.println(F("[GDO] Learn mode already active — TX queued"));
        return;
    }
    // Warn if the door is currently in motion — the opener cannot enter Learn mode
    // while moving.  The arm stays set so the TX fires if Learn is pressed later.
    if (_doorState == GDODoorState::OPENING || _doorState == GDODoorState::CLOSING) {
        ts(); Serial.println(F("[GDO] *** WARNING: Door is in motion — wait for it to stop before pressing Learn ***"));
    }
    ts(); Serial.println(F("[GDO] Learn-enrol armed (persists across reboot) — press Learn or reboot with Learn active"));
}

// =============================================================================
//  sendDoorCommand()  —  Transmit a door-toggle command on the serial bus.
// =============================================================================
void GDOBus::sendDoorCommand() {
    uint8_t pkt[GDO_PACKET_LEN];

    // payload=1: bit 0 = "toggle" flag; matches wall-unit observation (0x20001 lower bits).
    // Sending 0 may cause the opener to treat the command as a no-op even when enrolled.
    if (encode_wireline_command(_rolling, _deviceId, GDO_CMD_DOOR_ACTION, 1, pkt) < 0) {
        ts(); Serial.println(F("[GDO] TX ERROR: encode_wireline_command failed!"));
        return;
    }

    // Wait for bus idle before transmitting (half-duplex)
    unsigned long deadline = millis() + TX_WAIT_MS;
    while ((millis() - _lastRxTime) < RX_GAP_MS && millis() < deadline) {
        delay(1);
    }
    if (millis() >= deadline) {
        ts(); Serial.println(F("[GDO] TX WARN: bus busy timeout — transmitting anyway"));
    }

    GDO_SERIAL.write(pkt, GDO_PACKET_LEN);
    GDO_SERIAL.flush();
    _statTxCmds++;

#if GDO_DEBUG_LEVEL >= 1
    ts(); Serial.printf("[GDO] TX  cmd=0x%03X  dev=0x%010llX  roll=0x%07X\n",
                        GDO_CMD_DOOR_ACTION, _deviceId, _rolling);
#endif
#if GDO_DEBUG_LEVEL >= 2
    ts(); Serial.print(F("[GDO] TX raw: "));
    printHex(pkt, GDO_PACKET_LEN);
#endif

    _rolling = (_rolling + 1) & 0x0FFFFFFFu;
    saveRolling();
}
