// =============================================================================
//  GDOBus.cpp  —  Security+ 2.0 Wireline Bus Driver  (implementation)
// =============================================================================

#include "GDOBus.h"
#include "secplus.h"      // argilo/secplus  — GPL-3.0
#include <Preferences.h>  // ESP32 NVS (non-volatile storage)
#include <esp_random.h>   // Hardware RNG

// ─────────────────────────────────────────────────────────────────────────────
//  Debug flag  —  set to 1 to print every received packet to Serial.
//  Useful during first installation to identify your opener's command codes.
// ─────────────────────────────────────────────────────────────────────────────
#define GDO_DEBUG_RX   1   // 0 = silent,  1 = verbose RX logging

// ── Static member definitions ─────────────────────────────────────────────────
GDODoorState     GDOBus::_doorState      = GDODoorState::UNKNOWN;
GDOStateCallback GDOBus::_stateCallback  = nullptr;
uint32_t         GDOBus::_rolling        = 0;
uint64_t         GDOBus::_deviceId       = 0;
uint8_t          GDOBus::_rxBuf[GDO_PACKET_LEN] = {};
size_t           GDOBus::_rxIdx          = 0;
unsigned long    GDOBus::_lastRxTime     = 0;

// NVS namespace and key names
static const char *NVS_NS         = "gdo";
static const char *NVS_ROLLING    = "rolling";
static const char *NVS_DEVICE_ID  = "devId";

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
    //    • RX MOSFET (2N7000): Bus HIGH → MOSFET on → GPIO LOW; Bus LOW → GPIO HIGH.
    //      Without inversion UART sees inverted logic.
    //
    //  With invert=true the UART handles both inversions transparently.
    GDO_SERIAL.begin(GDO_BAUD, SERIAL_8N1, rx_pin, tx_pin, /*invert=*/true);

    // Flush any stale bytes picked up during power-on
    delay(100);
    while (GDO_SERIAL.available()) GDO_SERIAL.read();

    Serial.printf("[GDO] Bus ready  |  device_id=0x%010llX  rolling=0x%07X\n",
                  _deviceId, _rolling);
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

    if (_deviceId == 0) {
        // ── First boot — generate a random 40-bit wired-device identity ──────
        //
        // Format used by ratgdo community for wired-panel device IDs:
        //   Top byte = 0xF0  (identifies device as a wired controller)
        //   Lower 32 bits = random
        //
        // NOTE: The Security+ 2.0 wireline bus is an open, trusted bus — any
        //       device physically wired to the RED/WHITE terminals can send
        //       commands.  Rolling codes prevent replay attacks but there is
        //       no pairing ceremony required for wired accessories.
        uint32_t rand32;
        esp_fill_random(&rand32, sizeof(rand32));
        _deviceId = 0xF000000000ULL | (uint64_t)rand32;

        // Start rolling code well away from 0 to avoid the initial dead zone
        _rolling = 0x1000 + (rand32 & 0x0FFF);

        prefs.putULong64(NVS_DEVICE_ID, _deviceId);
        prefs.putULong(NVS_ROLLING, _rolling);

        Serial.printf("[GDO] First boot — generated device_id=0x%010llX  rolling=0x%07X\n",
                      _deviceId, _rolling);
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
//  poll()  —  Read UART bytes and assemble + decode complete packets.
//             Must be called every loop() iteration.
// =============================================================================
void GDOBus::poll() {
    while (GDO_SERIAL.available()) {
        uint8_t b        = (uint8_t)GDO_SERIAL.read();
        unsigned long now = millis();

        // If there has been a long gap since the last byte, the previous
        // (partial) packet is corrupt — reset the buffer.
        if (_rxIdx > 0 && (now - _lastRxTime) > RX_GAP_MS) {
            if (_rxIdx > 0) {
                Serial.printf("[GDO] RX gap reset (had %u bytes)\n", (unsigned)_rxIdx);
            }
            _rxIdx = 0;
        }
        _lastRxTime = now;

        // Validate sync header bytes before buffering
        if (_rxIdx == 0 && b != 0x55) continue;   // Wait for first sync byte
        if (_rxIdx == 1 && b != 0x01) { _rxIdx = 0; continue; }
        if (_rxIdx == 2 && b != 0x00) { _rxIdx = 0; continue; }

        _rxBuf[_rxIdx++] = b;

        if (_rxIdx == GDO_PACKET_LEN) {
            processPacket(_rxBuf);
            _rxIdx = 0;
        }
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
        Serial.println("[GDO] RX: decode error (bad parity/format)");
        return;
    }

#if GDO_DEBUG_RX
    Serial.printf("[GDO] RX  cmd=0x%03X  dev=0x%010llX  roll=0x%07X  payload=0x%05X\n",
                  command, device_id, rolling, payload);
#endif

    // ── Interpret status broadcasts from the door controller ─────────────────
    //
    //  The opener periodically sends status packets that contain the current
    //  door state, light state, and lock state in the payload field.
    //
    //  Payload bit mapping (Security+ 2.0 wireline — ratgdo community research):
    //    bits [2:0] = door state   0=open  1=closed  2=opening  3=closing  4=stopped
    //    bit  [6]   = light state  0=off   1=on
    //    bit  [9]   = lock state   0=unlocked  1=locked
    //    bit  [12]  = obstruction  0=clear  1=obstructed
    //
    //  IMPORTANT: These bit positions were determined by reverse engineering and
    //  may vary between firmware revisions.  If your opener reports incorrect
    //  states, cross-reference with the latest ESPHome ratgdo source:
    //    https://github.com/ratgdo/esphome-ratgdo/blob/main/components/ratgdo/ratgdo.cpp
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

        if (newState != GDODoorState::UNKNOWN && newState != _doorState) {
            _doorState = newState;
            static const char *names[] = {"OPEN","CLOSED","OPENING","CLOSING","STOPPED"};
            Serial.printf("[GDO] Door state → %s\n", names[(int)newState]);
            if (_stateCallback) _stateCallback(newState);
        }

#if GDO_DEBUG_RX
        bool lightOn  = (payload >> 6) & 1;
        bool locked   = (payload >> 9) & 1;
        bool obst     = (payload >> 12) & 1;
        Serial.printf("[GDO]     light=%s  lock=%s  obstruction=%s\n",
                      lightOn ? "ON" : "off",
                      locked  ? "LOCKED" : "unlocked",
                      obst    ? "YES" : "no");
#endif
    }
    // Add handling for other command codes here if needed (e.g. motion sensor,
    // battery status from door sensor, etc.)
}

// =============================================================================
//  sendDoorCommand()  —  Transmit a door-toggle command on the serial bus.
// =============================================================================
void GDOBus::sendDoorCommand() {
    uint8_t pkt[GDO_PACKET_LEN];

    // Encode the door-action command
    if (encode_wireline_command(_rolling, _deviceId, GDO_CMD_DOOR_ACTION, 0, pkt) < 0) {
        Serial.println("[GDO] TX: encode_wireline_command failed!");
        return;
    }

    // ── Wait for bus idle ─────────────────────────────────────────────────────
    //  Since this is a half-duplex bus we must not transmit while another
    //  device is sending.  _lastRxTime is updated every time we receive a byte;
    //  waiting until the bus has been quiet for at least RX_GAP_MS ensures
    //  we're not mid-packet.
    unsigned long deadline = millis() + TX_WAIT_MS;
    while ((millis() - _lastRxTime) < RX_GAP_MS && millis() < deadline) {
        delay(1);
    }
    if (millis() >= deadline) {
        Serial.println("[GDO] TX: bus busy timeout — transmitting anyway");
    }

    // ── Transmit ──────────────────────────────────────────────────────────────
    GDO_SERIAL.write(pkt, GDO_PACKET_LEN);
    GDO_SERIAL.flush();  // Block until all bytes sent

    Serial.printf("[GDO] TX  cmd=0x%03X  dev=0x%010llX  roll=0x%07X\n",
                  GDO_CMD_DOOR_ACTION, _deviceId, _rolling);

    // Increment rolling code and persist.  The Security+ 2.0 wireline receiver
    // accepts codes within a forward window of ~3000 increments, so a reboot
    // with a saved value is always accepted.
    _rolling = (_rolling + 1) & 0x0FFFFFFFu;
    saveRolling();
}
