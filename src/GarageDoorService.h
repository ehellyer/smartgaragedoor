// =============================================================================
//  GarageDoor.h  —  HomeSpan GarageDoorOpener Service  (v1.1 — dual reed)
//
//  Two magnetic reed switches provide definitive hardware end-stop signals:
//
//    REED_CLOSED (GPIO25) — mounted at the CLOSED position on the door frame
//    REED_OPEN   (GPIO26) — mounted at the FULLY OPEN position on the door frame
//
//  Both switches are normally-closed (NC).  The contact shorts to GND when
//  the magnet is absent and opens when the magnet is present.  External pull-
//  ups (reinforced by INPUT_PULLUP) hold the pin HIGH while the switch is open.
//    HIGH = magnet present = switch open   = door IS at that end-stop
//    LOW  = magnet absent  = switch closed = door is NOT at that end-stop
//
//  State truth table:
//  ┌──────────────┬────────────┬──────────────────────────────────────────────┐
//  │ REED_CLOSED  │ REED_OPEN  │ CurrentDoorState                             │
//  ├──────────────┼────────────┼──────────────────────────────────────────────┤
//  │ HIGH         │ LOW        │ CLOSED (1) — definitive hardware signal       │
//  │ LOW          │ HIGH       │ OPEN   (0) — definitive hardware signal       │
//  │ LOW          │ LOW        │ In between → OPENING(2) / CLOSING(3) /       │
//  │              │            │              STOPPED(4) — inferred from       │
//  │              │            │              last command + bus status        │
//  │ HIGH         │ HIGH       │ Error — both triggered simultaneously         │
//  │              │            │ (mechanically impossible; ignored)            │
//  └──────────────┴────────────┴──────────────────────────────────────────────┘
//
//  State priority (highest first):
//    1. Reed switches  — hardware truth for CLOSED and OPEN end-stops
//    2. GDOBus status packets — OPENING / CLOSING direction from the opener
//    3. Last command + target — inferred direction when bus data not available
//    4. Travel timeout — fall back to STOPPED after TRAVEL_TIMEOUT_MS
//
//  HAP CurrentDoorState values:
//    0 = Open   1 = Closed   2 = Opening   3 = Closing   4 = Stopped
// =============================================================================

#pragma once
#include "HomeSpan.h"
#include "GDOBus.h"

struct GarageDoorService : Service::GarageDoorOpener {

    // ── HomeKit Characteristics ────────────────────────────────────────────────
    SpanCharacteristic *currentState;
    SpanCharacteristic *targetState;
    SpanCharacteristic *obstruction;

    // ── Reed switch pins and debounced state ───────────────────────────────────
    uint8_t      reedClosedPin;
    uint8_t      reedOpenPin;

    // Debounced logical states: true = magnet present = door IS at that position
    bool         reedClosedActive;
    bool         reedOpenActive;

    // Timestamps used by the debounce filter
    unsigned long closedChangeTime;
    unsigned long openChangeTime;

    static const unsigned long DEBOUNCE_MS = 50;   // Switch bounce filter
    static const unsigned long TRAVEL_TIMEOUT_MS = 18000; // 18 s max door travel

    // ── Travel state ───────────────────────────────────────────────────────────
    unsigned long cmdSentTime;   // millis() when last door command was sent; 0 = none

    // ── Constructor ────────────────────────────────────────────────────────────
    GarageDoorService(uint8_t closedPin, uint8_t openPin) : Service::GarageDoorOpener()
    {
        reedClosedPin = closedPin;
        reedOpenPin   = openPin;

        pinMode(reedClosedPin, INPUT_PULLUP);
        pinMode(reedOpenPin,   INPUT_PULLUP);

        // Read initial hardware state (HIGH = magnet present = door at end-stop)
        reedClosedActive = (digitalRead(reedClosedPin) == HIGH);
        reedOpenActive   = (digitalRead(reedOpenPin)   == HIGH);
        closedChangeTime = 0;
        openChangeTime   = 0;
        cmdSentTime      = 0;

        // Determine starting HAP state from reed switches
        int initState = resolveHardwareState();

        currentState = new Characteristic::CurrentDoorState(initState);
        targetState  = new Characteristic::TargetDoorState(initState == 1 ? 1 : 0);
        obstruction  = new Characteristic::ObstructionDetected(false);

        LOG1("GarageDoorService ready  closedReed=%s  openReed=%s  initState=%d\n",
             reedClosedActive ? "ACTIVE" : "inactive",
             reedOpenActive   ? "ACTIVE" : "inactive",
             initState);
    }

    // ==========================================================================
    //  update()  —  Called by HomeSpan when HomeKit changes TargetDoorState
    // ==========================================================================
    boolean update() override {
        int newTarget = targetState->getNewVal();
        int curState  = currentState->getVal();

        LOG1("[HAP] TargetDoorState → %d  (current=%d)\n", newTarget, curState);

        if (newTarget == 0) {
            // HomeKit wants OPEN
            if (curState != 0 && curState != 2) {
                LOG1("[HAP] Sending OPEN command\n");
                GDOBus::sendDoorCommand();
                currentState->setVal(2);    // Show "Opening" immediately in Home app
                cmdSentTime = millis();
            }
        } else {
            // HomeKit wants CLOSED
            if (curState != 1 && curState != 3) {
                LOG1("[HAP] Sending CLOSE command\n");
                GDOBus::sendDoorCommand();
                currentState->setVal(3);    // Show "Closing" immediately in Home app
                cmdSentTime = millis();
            }
        }

        return true;
    }

    // ==========================================================================
    //  loop()  —  Called by homeSpan.poll() every main-loop iteration
    // ==========================================================================
    void loop() override {

        // ── 1. Debounce both reed switches ────────────────────────────────────
        // HIGH = magnet present = door IS at that end-stop (NC switch + pull-up)
        bool rawClosed = (digitalRead(reedClosedPin) == HIGH);
        bool rawOpen   = (digitalRead(reedOpenPin)   == HIGH);

        bool closedChanged = debounce(rawClosed, reedClosedActive, closedChangeTime);
        bool openChanged   = debounce(rawOpen,   reedOpenActive,   openChangeTime);

        // Accept debounced state changes
        if (closedChanged) {
            reedClosedActive = rawClosed;
            LOG1("[Reed] CLOSED switch → %s\n", reedClosedActive ? "ACTIVE" : "inactive");
            onClosedReedChange();
        }
        if (openChanged) {
            reedOpenActive = rawOpen;
            LOG1("[Reed] OPEN switch → %s\n", reedOpenActive ? "ACTIVE" : "inactive");
            onOpenReedChange();
        }

        // ── 2. Bus status update (only used when neither end-stop is active) ──
        //
        //  When both reed switches are LOW (door is in between), we trust the
        //  opener's status broadcast for OPENING vs CLOSING direction.
        //  When either end-stop is active, the reed switch has authority.
        if (!reedClosedActive && !reedOpenActive) {
            GDODoorState busState = GDOBus::getDoorState();

            if (busState == GDODoorState::OPENING || busState == GDODoorState::CLOSING) {
                int hapVal = (int)busState;
                if (hapVal != currentState->getVal()) {
                    LOG1("[Bus] In-transit direction → %s\n",
                         busState == GDODoorState::OPENING ? "OPENING" : "CLOSING");
                    currentState->setVal(hapVal);
                }
            }
        }

        // ── 3. Travel timeout ─────────────────────────────────────────────────
        //
        //  If the door is still showing Opening or Closing after TRAVEL_TIMEOUT_MS
        //  without either end-stop activating, something went wrong — mark Stopped.
        if (cmdSentTime != 0 && millis() - cmdSentTime > TRAVEL_TIMEOUT_MS) {
            int cur = currentState->getVal();
            if (cur == 2 || cur == 3) {
                LOG1("[Timeout] No end-stop after %lu ms — marking Stopped\n",
                     TRAVEL_TIMEOUT_MS);
                currentState->setVal(4);   // Stopped
            }
            cmdSentTime = 0;
        }
    }

private:

    // ==========================================================================
    // NC switches + pull-ups: HIGH = magnet present = door at that end-stop
    // | SW1 (GPIO25) | SW2 (GPIO26) | Door state                                    |
    // | CLD end-stop | OPN end stop |                                               |
    // |--------------|--------------|-----------------------------------------------|
    // | HIGH         | LOW          | **Closed**                                    |
    // | LOW          | HIGH         | **Open**                                      |
    // | LOW          | LOW          | **In between** — Opening, Closing, or Stopped |
    // | HIGH         | HIGH         | Error (impossible mechanically)               |
    //  resolveHardwareState()
    //  Translate the two reed switch readings into a HAP door state integer.
    //  Called only at construction time for the initial state.
    // ==========================================================================
    int resolveHardwareState() {
        if (!reedClosedActive && reedOpenActive) return 0;   // Open
        if (reedClosedActive && !reedOpenActive) return 1;   // Closed
        if (!reedClosedActive && !reedOpenActive) return 4;  // In-between → Stopped
        // Both active simultaneously is an error; default to Stopped
        Serial.println("[Reed] WARNING: Both reed switches active — check wiring!   Defaulting to Stopped state.");
        return 4;
    }

    // ==========================================================================
    //  debounce()
    //  Returns true if the raw reading has been stable for DEBOUNCE_MS and
    //  differs from the last accepted state.
    //  Updates changeTime in-place.
    // ==========================================================================
    // Compare rawNow against the last *accepted* (debounced) state.
    // Returns true once rawNow has held a value different from `accepted`
    // continuously for DEBOUNCE_MS, signalling a confirmed state change.
    // (Previous version compared against the last *raw* reading, which reset
    //  the timer every other iteration and prevented the window from filling.)
    bool debounce(bool rawNow, bool accepted, unsigned long &changeTime) {
        if (rawNow == accepted) {
            changeTime = 0;   // Steady at current accepted value — nothing pending
            return false;
        }
        // rawNow differs from accepted — start or keep the stability timer
        if (changeTime == 0) changeTime = millis();
        if (millis() - changeTime >= DEBOUNCE_MS) {
            changeTime = 0;
            return true;      // Held long enough — accept the change
        }
        return false;         // Still within bounce window
    }

    // ==========================================================================
    //  onClosedReedChange()  —  Called when the CLOSED reed switch changes state
    // ==========================================================================
    void onClosedReedChange() {
        if (reedClosedActive) {
            // ── Door just arrived at the CLOSED end-stop ──────────────────────
            // This is definitive — override any bus or inferred state.
            currentState->setVal(1);    // Closed
            targetState->setVal(1);
            cmdSentTime = 0;
        } else {
            // ── Door just left the CLOSED end-stop ────────────────────────────
            // It's moving away from closed; if the target is Open, show Opening.
            // If we have no target context, leave the current state as-is until
            // the bus or travel timeout resolve it.
            if (targetState->getVal() == 0 || currentState->getVal() == 1) {
                currentState->setVal(2);    // Opening
            }
        }
    }

    // ==========================================================================
    //  onOpenReedChange()  —  Called when the OPEN reed switch changes state
    // ==========================================================================
    void onOpenReedChange() {
        if (reedOpenActive) {
            // ── Door just arrived at the FULLY OPEN end-stop ──────────────────
            // Definitive — override any bus or inferred state.
            currentState->setVal(0);    // Open
            targetState->setVal(0);
            cmdSentTime = 0;
        } else {
            // ── Door just left the FULLY OPEN end-stop ────────────────────────
            // It's moving away from open; if the target is Closed, show Closing.
            if (targetState->getVal() == 1 || currentState->getVal() == 0) {
                currentState->setVal(3);    // Closing
            }
        }
    }
};
