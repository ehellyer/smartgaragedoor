#pragma once

#include <Arduino.h>
#include <TelnetStream.h>

// Fans out every write() to two Print streams simultaneously.
// Default: Serial (UART0) + TelnetStream (TCP:23).
// Pass custom Print& references to the constructor to override either stream.
//
// Lifecycle — two valid patterns:
//   A) Call logger.begin(port) after WiFi is up. Safe to call from multiple
//      DuaLogger instances; TelnetStream is only started once.
//   B) Call TelnetStream.begin(port) directly in setup(), then use the logger
//      without ever calling logger.begin(). Both patterns work correctly.
//
// If a custom secondary is provided, begin() is a no-op — initialise it yourself.
class DuaLogger : public Print {
public:
    DuaLogger(Print& primary = Serial, Print& secondary = TelnetStream)
        : _primary(primary),
          _secondary(secondary),
          _manageTelnet(&secondary == static_cast<Print*>(&TelnetStream)) {}

    void begin(uint16_t port = 23) {
        if (_manageTelnet) {
            static bool started = false;
            if (!started) {
                TelnetStream.begin(port);
                started = true;
            }
        }
    }

    size_t write(uint8_t b) override {
        _primary.write(b);
        _secondary.write(b);
        return 1;
    }

    size_t write(const uint8_t* buf, size_t size) override {
        _primary.write(buf, size);
        _secondary.write(buf, size);
        return size;
    }

private:
    Print&     _primary;
    Print&     _secondary;
    const bool _manageTelnet;
};

// Global dual-output logger instance — defined in main.cpp.
extern DuaLogger Log;
