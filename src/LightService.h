// =============================================================================
//  LightService.h  —  HomeSpan LightBulb service for the opener's work light
//
//  Sends LIGHT (0x281) actions on the Security+ 2.0 wireline bus and mirrors
//  the light state reported in STATUS packets (byte2 bit 1).
//
//  The opener does not broadcast status on its own, so after commanding the
//  light we schedule one requestStatus() to confirm the result; wall-console
//  changes are picked up by that opener's change broadcasts and by the
//  GarageDoorService 60 s background poll.
// =============================================================================

#pragma once
#include "HomeSpan.h"
#include "GDOBus.h"

struct LightService : Service::LightBulb {

    SpanCharacteristic *power;
    unsigned long confirmAt;   // millis() to send a confirming GET_STATUS; 0 = none

    static const unsigned long CONFIRM_DELAY_MS = 750;

    LightService() : Service::LightBulb()
    {
        new Characteristic::Name("Opener Light");
        power     = new Characteristic::On(false);
        confirmAt = 0;
    }

    // HomeKit toggled the light
    boolean update() override {
        bool on = power->getNewVal() > 0;
        LOG1("[HAP] Opener light → %s\n", on ? "ON" : "off");
        GDOBus::sendLightAction(on);
        confirmAt = millis() + CONFIRM_DELAY_MS;
        return true;
    }

    void loop() override {
        // Confirm a recent command by asking the opener for its status
        if (confirmAt != 0 && millis() >= confirmAt) {
            confirmAt = 0;
            GDOBus::requestStatus();
        }

        // Mirror the opener-reported light state (valid after first STATUS)
        if (GDOBus::lastStateTimestamp() != 0) {
            int actual = GDOBus::getLightOn() ? 1 : 0;
            if (power->getVal() != actual) {
                LOG1("[Bus] Opener light is %s\n", actual ? "ON" : "off");
                power->setVal(actual);
            }
        }
    }
};
