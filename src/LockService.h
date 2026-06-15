// =============================================================================
//  LockService.h  —  HomeSpan LockMechanism service for the remote lockout
//
//  The Security+ 2.0 "lock" (LOCK 0x18C) is the wall-console lockout: when
//  engaged, the opener ignores wireless remotes.  The WIRED bus keeps working,
//  so HomeKit (this device) and the wall console still control the door.
//
//  State is mirrored from STATUS packets (byte2 bit 0).  HAP values:
//    LockCurrentState: 0=unsecured 1=secured 2=jammed 3=unknown
//    LockTargetState:  0=unsecured 1=secured
// =============================================================================

#pragma once
#include "HomeSpan.h"
#include "GDOBus.h"
#include "DuaLogger.h"

struct LockService : Service::LockMechanism {

    SpanCharacteristic *lockCurrent;
    SpanCharacteristic *lockTarget;
    unsigned long confirmAt;   // millis() to send a confirming GET_STATUS; 0 = none

    static const unsigned long CONFIRM_DELAY_MS = 750;

    LockService() : Service::LockMechanism()
    {
        new Characteristic::Name("Remote Lock");
        lockCurrent = new Characteristic::LockCurrentState(3);   // 3 = unknown until first STATUS
        lockTarget  = new Characteristic::LockTargetState(0);
        confirmAt   = 0;
    }

    // HomeKit changed the lock target
    boolean update() override {
        bool lock = lockTarget->getNewVal() == 1;
        Log.printf("[HAP] Remote lockout → %s\n", lock ? "LOCKED" : "unlocked");
        GDOBus::sendLockAction(lock);
        confirmAt = millis() + CONFIRM_DELAY_MS;
        return true;
    }

    void loop() override {
        // Confirm a recent command by asking the opener for its status
        if (confirmAt != 0 && millis() >= confirmAt) {
            confirmAt = 0;
            GDOBus::requestStatus();
        }

        // Mirror the opener-reported lock state (valid after first STATUS)
        if (GDOBus::lastStateTimestamp() != 0) {
            int actual = GDOBus::getLocked() ? 1 : 0;
            if (lockCurrent->getVal() != actual) {
                Log.printf("[Bus] Remote lockout is %s\n", actual ? "LOCKED" : "unlocked");
                lockCurrent->setVal(actual);
            }
            // Keep the target aligned when the state was changed externally
            // (wall console) and no HomeKit command is pending confirmation.
            if (confirmAt == 0 && lockTarget->getVal() != actual) {
                lockTarget->setVal(actual);
            }
        }
    }
};
