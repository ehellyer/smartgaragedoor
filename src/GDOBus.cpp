// =============================================================================
//  GDOBus.cpp  —  Security+ 2.0 Wireline Bus Driver  (implementation)
// =============================================================================

#include "GDOBus.h"
#include "secplus.h"        // argilo/secplus  — GPL-3.0
#include <Preferences.h>    // ESP32 NVS (non-volatile storage)
#include <esp_random.h>     // Hardware RNG
#include <driver/gpio.h>    // gpio_set_level (TX preamble)
#include <esp_rom_gpio.h>   // esp_rom_gpio_connect_out_signal (GPIO-matrix switch)
#include <esp_rom_sys.h>    // esp_rom_delay_us
#include <soc/gpio_sig_map.h> // U2TXD_OUT_IDX / SIG_GPIO_OUT_IDX

// ── Rolling-code persistence policy ───────────────────────────────────────────
// With periodic status polling the rolling code increments often; writing NVS
// on every increment would wear flash (~1.4k writes/day).  Instead the counter
// is saved every ROLLING_SAVE_EVERY increments, and on boot it is bumped by
// ROLLING_BOOT_BUMP (> ROLLING_SAVE_EVERY) so it can never fall behind what
// the opener last saw, even after a crash with unsaved increments.  The opener
// accepts forward jumps in the rolling counter.  (Same strategy as ratgdo.)
static constexpr uint32_t ROLLING_SAVE_EVERY = 16;
static constexpr uint32_t ROLLING_BOOT_BUMP  = 64;
static uint32_t s_unsavedRolling = 0;   // increments since the last NVS save

// ── Security+ 2.0 frame preamble timing (µs) ──────────────────────────────────
// Before every packet the sender holds the bus LOW for ≥1 byte time and then
// releases it for ~1 stop bit (verified against ratgdo ratgdo_uart_esp32.cpp).
// Without this preamble the opener ignores transmitted packets.
static constexpr uint32_t PREAMBLE_LOW_US  = 1300;
static constexpr uint32_t PREAMBLE_HIGH_US = 130;

// ── Static member definitions ─────────────────────────────────────────────────
GDODoorState     GDOBus::_doorState      = GDODoorState::UNKNOWN;
bool             GDOBus::_obstruction    = false;
bool             GDOBus::_lightOn        = false;
bool             GDOBus::_locked         = false;
unsigned long    GDOBus::_lastStateTime  = 0;
GDOStateCallback GDOBus::_stateCallback  = nullptr;
uint32_t         GDOBus::_rolling        = 0;
uint64_t         GDOBus::_deviceId       = 0;
uint8_t          GDOBus::_rxBuf[GDO_PACKET_LEN] = {};
size_t           GDOBus::_rxIdx          = 0;
unsigned long    GDOBus::_lastRxTime     = 0;
uint8_t          GDOBus::_txPin          = 0;
bool             GDOBus::_learnEnrollArmed = false;
bool             GDOBus::_inLearnMode      = false;
bool             GDOBus::_pendingLearnTX   = false;
uint32_t         GDOBus::_statBytesRx    = 0;
uint32_t         GDOBus::_statPacketsRx  = 0;
uint32_t         GDOBus::_statDecodeErr  = 0;
uint32_t         GDOBus::_statTxCmds    = 0;

// NVS namespace and key names
static const char *NVS_NS         = "gdo";
static const char *NVS_ROLLING    = "rolling";
static const char *NVS_DEVICE_ID  = "devId";
static const char *NVS_LEARN_ARM  = "learnArm"; // persistent enrol-arm flag

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

// =============================================================================
//  begin()  —  Initialise UART and load identity from NVS
// =============================================================================
void GDOBus::begin(uint8_t tx_pin, uint8_t rx_pin) {
    _txPin = tx_pin;   // cached for the GPIO-matrix preamble in sendPreamble()
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

    unsigned long start = millis();   // elapsed-time arithmetic is rollover-safe

    while (millis() - start < timeoutMs) {
        while (GDO_SERIAL.available()) {
            uint8_t b        = (uint8_t)GDO_SERIAL.read();
            unsigned long now = millis();
            rawBytes++;

            // Inter-packet gap → reset buffer
            if (idx > 0 && (now - lastRx) > RX_GAP_MS) {
                idx = 0;
            }
            lastRx = now;

            // Sync header validation (0x55 0x01 0x00).  On a mismatch at
            // position 1 or 2 the buffer restarts and the current byte is
            // re-tested as a potential packet start, so 55 55 01 00 resyncs.
            if (idx == 1 && b != 0x01)      idx = 0;
            else if (idx == 2 && b != 0x00) idx = 0;
            if (idx == 0 && b != 0x55) continue;

            buf[idx++] = b;
            if (idx == 3) syncFound++;   // complete 3-byte sync header seen

            if (idx == GDO_PACKET_LEN) {
                fullPackets++;
                idx = 0;

                uint32_t rolling; uint64_t device_id; uint16_t command; uint32_t payload;
                if (decode_wireline_command(buf, &rolling, &device_id, &command, &payload) < 0) {
                    decodeErr++;
                    _statDecodeErr++;
                    ts(); Serial.print(F("[GDO]   Decode error — raw bytes: "));
                    printHex(buf, GDO_PACKET_LEN);
                } else {
                    decodeOK++;
                    _statPacketsRx++;
                    // Feed the decoded packet into the normal state machine
                    // (logs the packet and updates door state / learn mode).
                    handleDecoded(rolling, device_id, command, payload);
                    // Execute any TX queued by handleDecoded (e.g., learn-enrol arm
                    // was restored from NVS and state=6 was just detected).  This
                    // fires the enrolment TX during verify(), early in boot —
                    // before the wall unit responds to the learn window.
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
    Serial.printf( "[GDO]   Sync headers found  : %lu\n", syncFound);
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

    // Seed the running byte counter with what we saw during verify.
    // (Packet and decode-error counters were already incremented inline.)
    _statBytesRx += rawBytes;

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

    // Jump past any increments that were not yet saved when we last powered
    // off (see ROLLING_BOOT_BUMP above) and persist the new baseline.
    if (_deviceId != 0) {
        _rolling = (_rolling + ROLLING_BOOT_BUMP) & 0x0FFFFFFFu;
        prefs.putULong(NVS_ROLLING, _rolling);
    }

    // Restore persistent learn-enrol arm (set by armLearnEnroll()).
    // The NVS flag is cleared here so a reboot consumes it at most once; the
    // in-RAM arm then fires during the next verify() or poll() that sees
    // learn mode active — early in boot, before the wall unit responds to the
    // learn window.  (If the arm instead fires live, without a reboot, the NVS flag
    // is cleared at that point — see clearLearnArmNVS() call sites.)
    if (prefs.getBool(NVS_LEARN_ARM, false)) {
        _learnEnrollArmed = true;
        prefs.putBool(NVS_LEARN_ARM, false);    // consume — arm survives at most one reboot
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
//  saveRolling()  —  Persist the rolling counter to NVS.  Called sparsely
//                    (every ROLLING_SAVE_EVERY increments) — see the
//                    rolling-code persistence policy at the top of this file.
// =============================================================================
void GDOBus::saveRolling() {
    Preferences prefs;
    prefs.begin(NVS_NS, false);
    prefs.putULong(NVS_ROLLING, _rolling);
    prefs.end();
}

// =============================================================================
//  clearLearnArmNVS()  —  Clear the persistent learn-arm flag.
//
//  Must be called at EVERY point the in-RAM arm is consumed without a reboot.
//  A stale NVS flag would silently re-arm on the next boot and could fire an
//  unexpected door command during a future learn window (e.g., while enrolling
//  a different remote) — a physical-safety hazard.
// =============================================================================
void GDOBus::clearLearnArmNVS() {
    Preferences prefs;
    prefs.begin(NVS_NS, false);
    prefs.putBool(NVS_LEARN_ARM, false);
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
//  processRxByte()  —  Feed one received byte through the packet assembler.
//                      Shared by poll() and the bus-idle wait in
//                      sendDoorCommand(), so bytes arriving while we wait to
//                      transmit are still assembled and keep _lastRxTime fresh.
// =============================================================================
void GDOBus::processRxByte(uint8_t b) {
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

    // Validate the 3-byte sync header (0x55 0x01 0x00).  On a mismatch at
    // position 1 or 2 the buffer restarts and the current byte is re-tested
    // as a potential packet start, so a sequence like 55 55 01 00 resyncs.
    if (_rxIdx == 1 && b != 0x01)      _rxIdx = 0;
    else if (_rxIdx == 2 && b != 0x00) _rxIdx = 0;
    if (_rxIdx == 0 && b != 0x55) return;

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

// =============================================================================
//  poll()  —  Read UART bytes and assemble + decode complete packets.
//             Must be called every loop() iteration.
// =============================================================================
void GDOBus::poll() {
    while (GDO_SERIAL.available()) {
        processRxByte((uint8_t)GDO_SERIAL.read());
    }

    // Execute any TX deferred from handleDecoded() — keeps the TX out of the
    // packet-assembly path.  sendDoorCommand() itself waits for bus idle.
    if (_pendingLearnTX) {
        _pendingLearnTX = false;
        sendDoorCommand();
    }
}

// =============================================================================
//  processPacket()  —  Decode a complete 19-byte packet and hand it to
//                      handleDecoded().
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
    handleDecoded(rolling, device_id, command, payload);
}

// =============================================================================
//  handleDecoded()  —  Act on an already-decoded packet: log it, track door
//                      state / obstruction, and run learn-mode detection.
//                      Called from processPacket() and from verify() (which
//                      decodes packets itself for its diagnostic counters).
// =============================================================================
void GDOBus::handleDecoded(uint32_t rolling, uint64_t device_id,
                           uint16_t command, uint32_t payload) {
    // Ignore the half-duplex echo of our own transmissions
    if (device_id == _deviceId) {
#if GDO_DEBUG_LEVEL >= 2
        ts(); Serial.printf("[GDO] RX  (own echo)  cmd=0x%03X\n", command);
#endif
        return;
    }

#if GDO_DEBUG_LEVEL >= 1
    ts(); Serial.printf("[GDO] RX  cmd=0x%03X  dev=0x%010llX  roll=0x%07X  payload=0x%05X\n",
                        command, device_id, rolling, payload);
#endif

    // ── STATUS packets (0x081) — sent by the opener in response to GET_STATUS
    //    or on its own when its state changes.  NOT broadcast periodically.
    //
    //  Payload field mapping (verified against ratgdo secplus2.cpp
    //  handle_command(), and against a live capture: payload 0x16062 with the
    //  door fully open = nibble 1 (OPEN), light on, unlocked, clear):
    //    nibble = (payload >> 16) & 0xF :
    //        door state  0=unknown 1=open 2=closed 3=stopped 4=opening 5=closing
    //    byte1  = (payload >> 8) & 0xFF :
    //        bit 6 : obstruction — INVERTED: 1 = clear, 0 = obstructed
    //    byte2  =  payload & 0xFF :
    //        bit 0 : lock   1 = locked
    //        bit 1 : light  1 = on
    //        bit 5 : learn  1 = learn mode active (Learn LED lit)
    if (command == GDO_CMD_STATUS) {
        uint8_t nibble = (payload >> 16) & 0x0F;
        uint8_t byte1  = (payload >> 8)  & 0xFF;
        uint8_t byte2  =  payload        & 0xFF;

        // Timestamp every status packet (even when the state is unchanged) so
        // callers can judge the freshness of getDoorState()/getObstruction().
        _lastStateTime = millis();
        _obstruction   = ((byte1 >> 6) & 1) == 0;   // raw 0 = obstructed
        _lightOn       = (byte2 >> 1) & 1;
        _locked        =  byte2       & 1;

        GDODoorState newState = GDODoorState::UNKNOWN;
        switch (nibble) {
            case 1:  newState = GDODoorState::OPEN;    break;
            case 2:  newState = GDODoorState::CLOSED;  break;
            case 3:  newState = GDODoorState::STOPPED; break;
            case 4:  newState = GDODoorState::OPENING; break;
            case 5:  newState = GDODoorState::CLOSING; break;
            default: break;   // 0 = unknown — leave cached state unchanged
        }

        bool nowInLearn = (byte2 >> 5) & 1;

#if GDO_DEBUG_LEVEL >= 1
        {
            static const char *doorNames[] = {"?","OPEN","CLOSED","STOPPED","OPENING","CLOSING"};
            ts(); Serial.printf("[GDO]     door=%-7s  light=%-3s  lock=%-8s  obstruction=%-3s  learn=%s\n",
                               nibble <= 5 ? doorNames[nibble] : "?",
                               _lightOn ? "ON"      : "off",
                               _locked  ? "LOCKED"  : "unlocked",
                               _obstruction ? "YES" : "no",
                               nowInLearn ? "ACTIVE" : "off");
        }
#endif

        // ── Learn-mode detection (STATUS byte2 bit 5 = Learn LED lit) ─────────
        if (nowInLearn && !_inLearnMode) {
            // Just entered learn mode
            _inLearnMode = true;
            ts(); Serial.println(F("[GDO] *** Learn mode ACTIVE — type 't' to enrol this device ***"));
            if (_learnEnrollArmed) {
                _learnEnrollArmed = false;
                clearLearnArmNVS();         // arm consumed — never leave a stale flag for next boot
                _pendingLearnTX   = true;   // executed in poll() after byte loop
                ts(); Serial.println(F("[GDO]     (auto-enrol armed — TX queued)"));
            }
        } else if (!nowInLearn && _inLearnMode) {
            // Learn window closed
            _inLearnMode = false;
            if (_learnEnrollArmed) {
                // Window closed with the arm still set — expire it (RAM + NVS)
                _learnEnrollArmed = false;
                clearLearnArmNVS();
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
//  When a STATUS packet reports learn mode active (byte2 bit5 — Learn LED
//  lit), handleDecoded() will queue a sendDoorCommand() call via
//  _pendingLearnTX.  The TX fires in poll() once the serial byte loop
//  finishes; the TX path waits for bus idle before writing.
//
//  NOTE: enrolment is NOT required for wireline door control — the opener
//  accepts wired commands from any device id.  Kept as a diagnostic tool.
//
//  Safe to call before or during the learn window — if already in learn mode,
//  the TX is queued immediately.  Fires exactly once per arm() call.
// =============================================================================
void GDOBus::armLearnEnroll() {
    if (_inLearnMode) {
        // Already in learn mode — queue the TX immediately.  The arm is
        // consumed on the spot, so nothing is persisted to NVS.
        _pendingLearnTX = true;
        ts(); Serial.println(F("[GDO] Learn mode already active — TX queued"));
        return;
    }

    _learnEnrollArmed = true;

    // Persist the arm to NVS so it survives a reboot, enabling the enrolment TX
    // to fire during verify() early in boot — before the wall unit can respond
    // to the learn window.  The flag is cleared on next boot by loadIdentity(),
    // or by clearLearnArmNVS() if the arm fires (or expires) without a reboot.
    {
        Preferences prefs;
        prefs.begin(NVS_NS, false);
        prefs.putBool(NVS_LEARN_ARM, true);
        prefs.end();
    }

    // Warn if the door is currently in motion — the opener cannot enter Learn mode
    // while moving.  The arm stays set so the TX fires if Learn is pressed later.
    if (_doorState == GDODoorState::OPENING || _doorState == GDODoorState::CLOSING) {
        ts(); Serial.println(F("[GDO] *** WARNING: Door is in motion — wait for it to stop before pressing Learn ***"));
    }
    ts(); Serial.println(F("[GDO] Learn-enrol armed (persists across reboot) — press Learn or reboot with Learn active"));
}

// =============================================================================
//  waitBusIdle()  —  Wait for RX_GAP_MS of bus silence before transmitting
//                    (half-duplex).  Incoming bytes are drained through
//                    processRxByte() so _lastRxTime stays current and we never
//                    transmit on top of a packet that is still arriving.
//                    (poll() does not run during this wait, so the UART must
//                    be serviced here.)
// =============================================================================
void GDOBus::waitBusIdle() {
    unsigned long waitStart = millis();
    bool timedOut = false;
    while ((millis() - _lastRxTime) < RX_GAP_MS) {
        if (millis() - waitStart >= TX_WAIT_MS) { timedOut = true; break; }
        while (GDO_SERIAL.available()) {
            processRxByte((uint8_t)GDO_SERIAL.read());
        }
        delay(1);
    }
    if (timedOut) {
        ts(); Serial.println(F("[GDO] TX WARN: bus busy timeout — transmitting anyway"));
    }
}

// =============================================================================
//  sendPreamble()  —  Security+ 2.0 frame preamble.
//
//  Before every packet the bus must be held LOW for ~1.3 ms then released for
//  ~130 µs (verified against ratgdo's RMT implementation).  The opener and the
//  wall unit both do this — it shows up in our RX as one 0x00 byte before each
//  packet.  WITHOUT the preamble the opener ignores our packets entirely.
//
//  Implementation: the TX pin is temporarily detached from UART2 via the GPIO
//  matrix and driven as a plain GPIO.  GPIO HIGH turns on the TX MOSFET which
//  pulls the bus LOW (raw GPIO levels are NOT affected by the UART invert
//  flag, so no inversion is applied here).
// =============================================================================
void GDOBus::sendPreamble() {
    gpio_set_level((gpio_num_t)_txPin, 1);   // pre-set level: bus LOW once routed
    esp_rom_gpio_connect_out_signal(_txPin, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_delay_us(PREAMBLE_LOW_US);
    gpio_set_level((gpio_num_t)_txPin, 0);   // release — bus floats HIGH
    esp_rom_delay_us(PREAMBLE_HIGH_US);
    // Hand the pin back to UART2 (its inverted idle drives the pin LOW = bus HIGH)
    esp_rom_gpio_connect_out_signal(_txPin, U2TXD_OUT_IDX, false, false);
    esp_rom_delay_us(5);
}

// =============================================================================
//  txPacket()  —  Encode and transmit one wireline packet.
//                 Waits for bus idle, sends the preamble, then the 19 bytes.
//                 Increments + persists the rolling code if requested.
// =============================================================================
bool GDOBus::txPacket(uint16_t command, uint32_t payload, bool incrementRolling) {
    uint8_t pkt[GDO_PACKET_LEN];

    if (encode_wireline_command(_rolling, _deviceId, command, payload, pkt) < 0) {
        ts(); Serial.println(F("[GDO] TX ERROR: encode_wireline_command failed!"));
        return false;
    }

    waitBusIdle();
    sendPreamble();

    GDO_SERIAL.write(pkt, GDO_PACKET_LEN);
    GDO_SERIAL.flush();
    _statTxCmds++;

#if GDO_DEBUG_LEVEL >= 1
    ts(); Serial.printf("[GDO] TX  cmd=0x%03X  payload=0x%05X  roll=0x%07X\n",
                        command, payload, _rolling);
#endif
#if GDO_DEBUG_LEVEL >= 2
    ts(); Serial.print(F("[GDO] TX raw: "));
    printHex(pkt, GDO_PACKET_LEN);
#endif

    if (incrementRolling) {
        _rolling = (_rolling + 1) & 0x0FFFFFFFu;
        // Save sparsely to limit flash wear; the boot bump covers the rest.
        if (++s_unsavedRolling >= ROLLING_SAVE_EVERY) {
            s_unsavedRolling = 0;
            saveRolling();
        }
    }
    return true;
}

// =============================================================================
//  sendDoorAction()  —  Transmit an explicit door action (OPEN/CLOSE/TOGGLE/STOP).
//
//  Per ratgdo (secplus2.cpp door_command): a door action is a button-PRESS
//  packet (byte1=1, byte2=1) followed ~150 ms later by a RELEASE packet
//  (byte1=0, byte2=1), with the action in the nibble.  Both packets carry the
//  SAME rolling code; the counter increments once, after the release.
//
//  Payload layout (as produced by encode_wireline_command):
//    bits [19:16] = nibble (action)   bits [15:8] = byte1   bits [7:0] = byte2
//
//  Wall-unit capture cross-check: a wall button press shows payload=0x20001 =
//  TOGGLE(2) + byte1=0 + byte2=1 — i.e., the RELEASE half of this sequence.
// =============================================================================
void GDOBus::sendDoorAction(GDODoorAction action) {
    uint32_t a       = (uint32_t)action & 0x0F;
    uint32_t press   = (a << 16) | (1u << 8) | 1u;
    uint32_t release = (a << 16) | 1u;

    if (!txPacket(GDO_CMD_DOOR_ACTION, press, /*incrementRolling=*/false)) return;

    // Hold the press/release spacing while continuing to drain RX
    unsigned long start = millis();
    while (millis() - start < DOOR_RELEASE_DELAY_MS) {
        while (GDO_SERIAL.available()) {
            processRxByte((uint8_t)GDO_SERIAL.read());
        }
        delay(1);
    }

    txPacket(GDO_CMD_DOOR_ACTION, release, /*incrementRolling=*/true);
}

// =============================================================================
//  requestStatus()  —  Ask the opener to report its status.
//                      It replies with a 0x081 STATUS packet (handled in
//                      handleDecoded), which refreshes the cached door state,
//                      obstruction flag, and lastStateTimestamp().
// =============================================================================
void GDOBus::requestStatus() {
    txPacket(GDO_CMD_GET_STATUS, 0, /*incrementRolling=*/true);
}

// =============================================================================
//  sendLightAction() / sendLockAction()
//
//  Unlike door actions, LIGHT and LOCK are single packets with the action in
//  the nibble — no press/release pair (verified against ratgdo secplus2.cpp:
//  light_action()/lock_action() call send_command() directly).
//    LIGHT (0x281): nibble 0=off 1=on 2=toggle
//    LOCK  (0x18C): nibble 0=unlock 1=lock 2=toggle  (remote lockout)
// =============================================================================
void GDOBus::sendLightAction(bool on) {
    txPacket(GDO_CMD_LIGHT, (uint32_t)(on ? 1 : 0) << 16, /*incrementRolling=*/true);
}

void GDOBus::sendLockAction(bool lock) {
    txPacket(GDO_CMD_LOCK, (uint32_t)(lock ? 1 : 0) << 16, /*incrementRolling=*/true);
}
