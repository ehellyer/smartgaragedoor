// =============================================================================
//  GDOBus.cpp  —  Security+ 2.0 Wireline Bus Driver  (implementation)
// =============================================================================

#include "GDOBus.h"
#include "DuaLogger.h"
#include "secplus.h"        // argilo/secplus  — GPL-3.0
#include <Preferences.h>    // ESP32 NVS (non-volatile storage)
#include <esp_random.h>     // Hardware RNG
#include <driver/gpio.h>    // gpio_set_level (TX preamble)
#include <esp_rom_gpio.h>   // esp_rom_gpio_connect_out_signal (GPIO-matrix switch)
#include <esp_rom_sys.h>    // esp_rom_delay_us
#include <soc/gpio_sig_map.h> // U2TXD_OUT_IDX / SIG_GPIO_OUT_IDX

// ── Rolling-code persistence ───────────────────────────────────────────────────
// The opener validates rolling codes per device and rejects replays (codes
// lower than the last accepted value).  To survive reboots and crashes we
// persist the counter to NVS, saving every ROLLING_SAVE_EVERY increments to
// limit flash wear.  On each boot the counter is bumped by ROLLING_BOOT_BUMP
// (> ROLLING_SAVE_EVERY) so it is always ahead of what the opener last saw,
// even when there were unsaved increments at the time of the last power-off.
static constexpr uint32_t ROLLING_SAVE_EVERY = 16;
static constexpr uint32_t ROLLING_BOOT_BUMP  = 64;
static uint32_t s_unsavedRolling = 0;

static const char *NVS_NS        = "gdo";
static const char *NVS_ROLLING   = "rolling";
static const char *NVS_DEVICE_ID = "devId";

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
uint32_t         GDOBus::_statBytesRx    = 0;
uint32_t         GDOBus::_statPacketsRx  = 0;
uint32_t         GDOBus::_statDecodeErr  = 0;
uint32_t         GDOBus::_statTxCmds    = 0;

// =============================================================================
//  begin()  —  Initialise UART and load or generate the device identity.
// =============================================================================
void GDOBus::begin(uint8_t tx_pin, uint8_t rx_pin) {
    _txPin = tx_pin;
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

    // Flush any stale bytes that arrived during power-on.
    delay(100);
    while (GDO_SERIAL.available()) GDO_SERIAL.read();

    // Reset stats.
    _statBytesRx   = 0;
    _statPacketsRx = 0;
    _statDecodeErr = 0;
    _statTxCmds    = 0;

    Log.println(F("[GDO] ── Bus Initialised ──────────────────────────────────"));
    Log.printf( "[GDO]   UART         : Serial2  %d baud  8N1  inverted\n", GDO_BAUD);
    Log.printf( "[GDO]   TX pin       : GPIO%d  (AO3400A gate)\n",  tx_pin);
    Log.printf( "[GDO]   RX pin       : GPIO%d  (2N7000 drain)\n",  rx_pin);
    Log.printf( "[GDO]   Device ID    : 0x%010llX\n", _deviceId);
    Log.printf( "[GDO]   Rolling code : 0x%07X\n",    _rolling);
    Log.println(F("[GDO] ─────────────────────────────────────────────────────\n"));
}

// =============================================================================
//  verify()  —  Blocking bus health check for use in setup().
//               Listens for timeoutMs ms, then prints a diagnostic report.
//               Returns true if at least one packet decoded successfully.
// =============================================================================
bool GDOBus::verify(uint32_t timeoutMs) {
    Log.println(F("[GDO] ── Bus Verification ────────────────────────────────"));
    Log.printf( "[GDO]   Listening for %lu ms ...\n", timeoutMs);
    Log.println(F("[GDO]   (Press the wall button to send a packet immediately)"));

    uint32_t rawBytes    = 0;
    uint32_t syncFound   = 0;
    uint32_t fullPackets = 0;
    uint32_t decodeOK    = 0;
    uint32_t decodeErr   = 0;

    uint8_t  buf[GDO_PACKET_LEN];
    size_t   idx         = 0;
    unsigned long lastRx = 0;

    unsigned long start = millis();

    while (millis() - start < timeoutMs) {
        while (GDO_SERIAL.available()) {
            uint8_t b         = (uint8_t)GDO_SERIAL.read();
            unsigned long now = millis();
            rawBytes++;

            if (idx > 0 && (now - lastRx) > RX_GAP_MS) idx = 0;
            lastRx = now;

            if (idx == 1 && b != 0x01)      idx = 0;
            else if (idx == 2 && b != 0x00) idx = 0;
            if (idx == 0 && b != 0x55) continue;

            buf[idx++] = b;
            if (idx == 3) syncFound++;

            if (idx == GDO_PACKET_LEN) {
                fullPackets++;
                idx = 0;

                uint32_t rolling; uint64_t device_id; uint16_t command; uint32_t payload;
                if (decode_wireline_command(buf, &rolling, &device_id, &command, &payload) < 0) {
                    decodeErr++;
                    _statDecodeErr++;
                    Log.print(F("[GDO]   Decode error — raw bytes: "));
                    printHex(buf, GDO_PACKET_LEN);
                } else {
                    decodeOK++;
                    _statPacketsRx++;
                    handleDecoded(rolling, device_id, command, payload);
                }
            }
        }
        delay(1);
    }

    Log.println(F("[GDO] ── Verification Report ────────────────────────────"));
    Log.printf( "[GDO]   Raw bytes received  : %lu\n", rawBytes);
    Log.printf( "[GDO]   Sync headers found  : %lu\n", syncFound);
    Log.printf( "[GDO]   Complete packets    : %lu\n", fullPackets);
    Log.printf( "[GDO]   Decoded OK          : %lu\n", decodeOK);
    Log.printf( "[GDO]   Decode errors       : %lu\n", decodeErr);

    if (rawBytes == 0) {
        Log.println(F("[GDO]   >> No bytes received — check wiring:"));
        Log.println(F("[GDO]      - RED/WHITE terminals connected?"));
        Log.println(F("[GDO]      - Q1 (2N7000) gate/drain/source orientation?"));
        Log.println(F("[GDO]      - R2 pull-up to 3.3V on drain?"));
        Log.println(F("[GDO]      - Correct RX pin in main.cpp?"));
    } else if (syncFound == 0) {
        Log.println(F("[GDO]   >> Bytes arriving but no valid sync (0x55 0x01 0x00)"));
        Log.println(F("[GDO]      - Wrong baud rate? Try GDO_BAUD 4800 in GDOBus.h"));
        Log.println(F("[GDO]      - UART invert=true present in begin()?"));
    } else if (decodeOK == 0 && fullPackets > 0) {
        Log.println(F("[GDO]   >> Packets assembled but decode failing"));
        Log.println(F("[GDO]      - Protocol version mismatch?"));
        Log.println(F("[GDO]      - Check raw bytes above against secplus.h"));
    } else if (decodeOK == 0) {
        Log.println(F("[GDO]   >> Some bytes received but no complete packet yet"));
        Log.println(F("[GDO]      - Opener may be idle — press wall button and re-verify"));
    }

    bool passed = (decodeOK > 0);
    Log.printf("[GDO]   Result : %s\n", passed ? "PASS" : "FAIL");
    Log.println(F("[GDO] ─────────────────────────────────────────────────────\n"));

    _statBytesRx += rawBytes;
    return passed;
}

// =============================================================================
//  printStats()  —  One-line running stats summary.
// =============================================================================
void GDOBus::printStats() {
    Log.printf("[GDO] Stats  rx=%lu pkts  %lu bytes  %lu decodeErr  tx=%lu cmds\n",
                  _statPacketsRx, _statBytesRx, _statDecodeErr, _statTxCmds);
}

// =============================================================================
//  loadIdentity()  —  Restore device_id and rolling counter from NVS.
//                     On first boot a random identity is generated and stored.
//                     On every subsequent boot the rolling counter is bumped
//                     forward so it is always ahead of what the opener last saw.
// =============================================================================
void GDOBus::loadIdentity() {
    Preferences prefs;
    prefs.begin(NVS_NS, false);

    _deviceId = prefs.getULong64(NVS_DEVICE_ID, 0);
    _rolling  = prefs.getULong(NVS_ROLLING, 0);

    if (_deviceId != 0) {
        // Bump past any increments that were unsaved at last power-off.
        _rolling = (_rolling + ROLLING_BOOT_BUMP) & 0x0FFFFFFFu;
        prefs.putULong(NVS_ROLLING, _rolling);
    } else {
        // First boot — generate a random 40-bit wired-device identity.
        uint32_t rand32;
        esp_fill_random(&rand32, sizeof(rand32));
        _deviceId = 0xF000000000ULL | (uint64_t)rand32;
        _rolling  = 0x1000 + (rand32 & 0x0FFF);
        prefs.putULong64(NVS_DEVICE_ID, _deviceId);
        prefs.putULong(NVS_ROLLING, _rolling);
        Log.println(F("[GDO] First boot — new identity generated and stored in NVS"));
    }

    prefs.end();
}

// =============================================================================
//  saveRolling()  —  Persist the rolling counter to NVS.
//                    Called every ROLLING_SAVE_EVERY increments to limit
//                    flash wear while keeping the saved value close to current.
// =============================================================================
void GDOBus::saveRolling() {
    Preferences prefs;
    prefs.begin(NVS_NS, false);
    prefs.putULong(NVS_ROLLING, _rolling);
    prefs.end();
}

// =============================================================================
//  printHex()  —  Dump a byte buffer as hex (for diagnostics).
// =============================================================================
void GDOBus::printHex(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) Log.printf("%02X ", buf[i]);
    Log.println();
}

// =============================================================================
//  processRxByte()  —  Feed one received byte through the packet assembler.
//                      Shared by poll() and the bus-idle wait in sendDoorAction()
//                      so bytes arriving during TX waits are not lost.
// =============================================================================
void GDOBus::processRxByte(uint8_t b) {
    unsigned long now = millis();
    _statBytesRx++;

    if (_rxIdx > 0 && (now - _lastRxTime) > RX_GAP_MS) {
#if GDO_DEBUG_LEVEL >= 2
        Log.printf("[GDO] RX gap reset (had %u bytes)\n", (unsigned)_rxIdx);
#endif
        _rxIdx = 0;
    }
    _lastRxTime = now;

    if (_rxIdx == 1 && b != 0x01)      _rxIdx = 0;
    else if (_rxIdx == 2 && b != 0x00) _rxIdx = 0;
    if (_rxIdx == 0 && b != 0x55) return;

    _rxBuf[_rxIdx++] = b;

    if (_rxIdx == GDO_PACKET_LEN) {
#if GDO_DEBUG_LEVEL >= 2
        Log.print(F("[GDO] RX raw: "));
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
}

// =============================================================================
//  processPacket()  —  Decode a complete 19-byte packet.
// =============================================================================
void GDOBus::processPacket(const uint8_t *pkt) {
    uint32_t rolling;
    uint64_t device_id;
    uint16_t command;
    uint32_t payload;

    if (decode_wireline_command(pkt, &rolling, &device_id, &command, &payload) < 0) {
        _statDecodeErr++;
        Log.print(F("[GDO] RX decode error — raw: "));
        printHex(pkt, GDO_PACKET_LEN);
        return;
    }

    _statPacketsRx++;
    handleDecoded(rolling, device_id, command, payload);
}

// =============================================================================
//  handleDecoded()  —  Act on a decoded packet: log it and update cached state.
// =============================================================================
void GDOBus::handleDecoded(uint32_t rolling, uint64_t device_id,
                           uint16_t command, uint32_t payload) {
    // Ignore the half-duplex echo of our own transmissions.
    if (device_id == _deviceId) {
#if GDO_DEBUG_LEVEL >= 2
        Log.printf("[GDO] RX  (own echo)  cmd=0x%03X\n", command);
#endif
        return;
    }

#if GDO_DEBUG_LEVEL >= 1
    Log.printf("[GDO] RX  cmd=0x%03X  dev=0x%010llX  roll=0x%07X  payload=0x%05X\n",
                        command, device_id, rolling, payload);
#endif

    // ── STATUS packets (0x081) ────────────────────────────────────────────────
    // Sent by the opener in response to GET_STATUS or when its state changes.
    // NOT broadcast periodically.
    //
    //  Payload field mapping (verified against ratgdo secplus2.cpp):
    //    nibble = (payload >> 16) & 0xF  →  door state
    //        0=unknown 1=open 2=closed 3=stopped 4=opening 5=closing
    //    byte1  = (payload >> 8) & 0xFF
    //        bit 6 : obstruction — INVERTED: 1 = clear, 0 = obstructed
    //    byte2  =  payload & 0xFF
    //        bit 0 : lock   1 = locked
    //        bit 1 : light  1 = on
    if (command == GDO_CMD_STATUS) {
        uint8_t nibble = (payload >> 16) & 0x0F;
        uint8_t byte1  = (payload >> 8)  & 0xFF;
        uint8_t byte2  =  payload        & 0xFF;

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
            default: break;
        }

#if GDO_DEBUG_LEVEL >= 1
        {
            static const char *doorNames[] = {"?","OPEN","CLOSED","STOPPED","OPENING","CLOSING"};
            Log.printf("[GDO]     door=%-7s  light=%-3s  lock=%-8s  obstruction=%s\n",
                               nibble <= 5 ? doorNames[nibble] : "?",
                               _lightOn     ? "ON"     : "off",
                               _locked      ? "LOCKED" : "unlocked",
                               _obstruction ? "YES"    : "no");
        }
#endif

        if (newState != GDODoorState::UNKNOWN && newState != _doorState) {
            _doorState = newState;
            if (_stateCallback) _stateCallback(newState);
        }
    }
}

// =============================================================================
//  waitBusIdle()  —  Wait for RX_GAP_MS of bus silence before transmitting.
//                    Drains incoming bytes through processRxByte() so no
//                    packets are lost while we wait.
// =============================================================================
void GDOBus::waitBusIdle() {
    unsigned long waitStart = millis();
    bool timedOut = false;
    while ((millis() - _lastRxTime) < RX_GAP_MS) {
        if (millis() - waitStart >= TX_WAIT_MS) { timedOut = true; break; }
        while (GDO_SERIAL.available()) processRxByte((uint8_t)GDO_SERIAL.read());
        delay(1);
    }
    if (timedOut) {
        Log.println(F("[GDO] TX WARN: bus busy timeout — transmitting anyway"));
    }
}

// =============================================================================
//  sendPreamble()  —  Security+ 2.0 frame preamble (~1.3 ms bus LOW).
//
//  Before every packet the bus must be held LOW for ~1.3 ms then released for
//  ~130 µs.  Without this preamble the opener ignores transmitted packets.
//
//  The TX pin is temporarily detached from UART2 and driven as a plain GPIO.
//  GPIO HIGH turns on the TX MOSFET which pulls the bus LOW (raw GPIO levels
//  are NOT affected by the UART invert flag).
// =============================================================================
void GDOBus::sendPreamble() {
    gpio_set_level((gpio_num_t)_txPin, 1);
    esp_rom_gpio_connect_out_signal(_txPin, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_delay_us(PREAMBLE_LOW_US);
    gpio_set_level((gpio_num_t)_txPin, 0);
    esp_rom_delay_us(PREAMBLE_HIGH_US);
    esp_rom_gpio_connect_out_signal(_txPin, U2TXD_OUT_IDX, false, false);
    esp_rom_delay_us(5);
}

// =============================================================================
//  txPacket()  —  Encode and transmit one wireline packet.
// =============================================================================
bool GDOBus::txPacket(uint16_t command, uint32_t payload, bool incrementRolling) {
    uint8_t pkt[GDO_PACKET_LEN];

    if (encode_wireline_command(_rolling, _deviceId, command, payload, pkt) < 0) {
        Log.println(F("[GDO] TX ERROR: encode_wireline_command failed!"));
        return false;
    }

    waitBusIdle();
    sendPreamble();

    GDO_SERIAL.write(pkt, GDO_PACKET_LEN);
    GDO_SERIAL.flush();
    _statTxCmds++;

#if GDO_DEBUG_LEVEL >= 1
    Log.printf("[GDO] TX  cmd=0x%03X  payload=0x%05X  roll=0x%07X\n",
                        command, payload, _rolling);
#endif
#if GDO_DEBUG_LEVEL >= 2
    Log.print(F("[GDO] TX raw: "));
    printHex(pkt, GDO_PACKET_LEN);
#endif

    if (incrementRolling) {
        _rolling = (_rolling + 1) & 0x0FFFFFFFu;
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
//  A door action is a PRESS packet (byte1=1, byte2=1) followed ~150 ms later
//  by a RELEASE packet (byte1=0, byte2=1), both carrying the same rolling code.
//  The counter increments once after the release.
// =============================================================================
void GDOBus::sendDoorAction(GDODoorAction action) {
    uint32_t a       = (uint32_t)action & 0x0F;
    uint32_t press   = (a << 16) | (1u << 8) | 1u;
    uint32_t release = (a << 16) | 1u;

    if (!txPacket(GDO_CMD_DOOR_ACTION, press, /*incrementRolling=*/false)) return;

    unsigned long start = millis();
    while (millis() - start < DOOR_RELEASE_DELAY_MS) {
        while (GDO_SERIAL.available()) processRxByte((uint8_t)GDO_SERIAL.read());
        delay(1);
    }

    txPacket(GDO_CMD_DOOR_ACTION, release, /*incrementRolling=*/true);
}

// =============================================================================
//  requestStatus()  —  Ask the opener to report its current status.
//                      The opener replies with a 0x081 STATUS packet handled
//                      in handleDecoded(), refreshing all cached state.
// =============================================================================
void GDOBus::requestStatus() {
    txPacket(GDO_CMD_GET_STATUS, 0, /*incrementRolling=*/true);
}

// =============================================================================
//  sendLightAction() / sendLockAction()
//
//  LIGHT and LOCK are single packets (no press/release pair).
//    LIGHT (0x281): nibble 0=off  1=on  2=toggle
//    LOCK  (0x18C): nibble 0=unlock  1=lock  (remote lockout)
// =============================================================================
void GDOBus::sendLightAction(bool on) {
    txPacket(GDO_CMD_LIGHT, (uint32_t)(on ? 1 : 0) << 16, /*incrementRolling=*/true);
}

void GDOBus::sendLockAction(bool lock) {
    txPacket(GDO_CMD_LOCK, (uint32_t)(lock ? 1 : 0) << 16, /*incrementRolling=*/true);
}
