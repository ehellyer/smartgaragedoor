// =============================================================================
//  GarageDoorService.h  —  HomeSpan GarageDoorOpener Service  (v2.0)
//
//  Door state is driven entirely by the Security+ 2.0 wireline bus:
//
//    • The opener broadcasts STATUS (0x081) when its state changes, and
//      replies to GET_STATUS (0x080) requests.  It does NOT broadcast
//      periodically on its own, so this service polls it.
//
//    • When a HomeKit goal is active (door commanded open or closed), the
//      bus is polled every ACTIVE_POLL_MS to detect arrival at the end-stop.
//
//    • A background poll every BG_POLL_MS keeps door, light, and lock state
//      in sync with changes made at the wall console or by remotes.
//
//    • TRAVEL_TIMEOUT_MS guards against openers that do not broadcast arrival
//      (e.g. slow battery-backup travel).  The door is marked Stopped if no
//      end-stop is confirmed within the window, then an immediate poll is
//      requested so the bus can correct the state quickly.
//
//    • The GDOBus state-change callback only fires when the decoded state
//      DIFFERS from its cached value.  A direct cache check in loop() catches
//      the case where the HAP state becomes stuck (e.g. after an external
//      command) while the bus cache already holds the correct end-stop state.
//
//  HAP CurrentDoorState:  0 = Open   1 = Closed   2 = Opening
//                         3 = Closing              4 = Stopped
// =============================================================================

#pragma once
#include "HomeSpan.h"
#include "GDOBus.h"
#include "DuaLogger.h"

struct GarageDoorService : Service::GarageDoorOpener {

    // Single-instance pointer used by the GDOBus callback trampoline.
    inline static GarageDoorService *instance = nullptr;

    // ── HomeKit Characteristics ────────────────────────────────────────────────
    SpanCharacteristic *currentState;
    SpanCharacteristic *targetState;
    SpanCharacteristic *obstruction;

    // ── Goal seeker ───────────────────────────────────────────────────────────
    int8_t        goalState;      // -1 = idle  0 = seeking Open  1 = seeking Closed
    unsigned long travelStart;    // millis() when travel began; 0 = not travelling
    unsigned long lastActivePoll; // millis() of last status poll while goal is active

    static const unsigned long TRAVEL_TIMEOUT_MS = 20000; // covers battery-backup travel speed
    static const unsigned long ACTIVE_POLL_MS    =  2000; // status poll rate while goal is active
    static const unsigned long BG_POLL_MS        = 60000; // idle sync poll interval

    // ── Background poll ────────────────────────────────────────────────────────
    unsigned long lastBgPoll;

    // ── Hardware obstruction sensor ────────────────────────────────────────────
    //
    //  The opener's safety-beam wire has three states:
    //    clear      → line HIGH with a brief LOW pulse every ~7 ms
    //    obstructed → line steady HIGH (pulses stop)
    //    asleep     → line steady LOW
    //
    //  A falling-edge ISR counts the LOW pulses.  Every OBST_CHECK_MS the
    //  window is evaluated:
    //    ≥ OBST_MIN_PULSES in the window → beam is pulsing = clear
    //    steady HIGH for > OBST_HIGH_MS  → beam broken = obstructed
    //    steady LOW                      → sensor asleep (no change reported)
    //
    //  Obstruction is only reported once pulses have been seen at least once
    //  since boot (obstSensorSeen), so an unwired pin cannot raise false alarms.
    uint8_t       obstPin;
    bool          hwObstructed;
    bool          obstSensorSeen;
    unsigned long lastObstCheck;
    unsigned long lastObstLow;

    static volatile uint32_t obstLowCount; // pulse counter (ISR); defined in GarageDoorService.cpp

    static const unsigned long OBST_CHECK_MS   = 50;
    static const uint32_t      OBST_MIN_PULSES = 3;
    static const unsigned long OBST_HIGH_MS    = 70;

    // IRAM_ATTR definition lives in GarageDoorService.cpp to avoid an Xtensa
    // linker literal-pool error ("dangerous relocation: l32r").
    static void obstISR();

    // ── Constructor ────────────────────────────────────────────────────────────
    // obstructionPin: GPIO wired to the opener's safety-beam wire via a
    //   resistor divider.  Pass 0xFF if no hardware obstruction sensor is fitted.
    GarageDoorService(uint8_t obstructionPin = 0xFF)
        : Service::GarageDoorOpener()
    {
        obstPin        = obstructionPin;
        goalState      = -1;
        travelStart    = 0;
        lastActivePoll = 0;
        lastBgPoll     = millis();

        hwObstructed   = false;
        obstSensorSeen = false;
        lastObstCheck  = millis();
        lastObstLow    = millis();

        if (obstPin != 0xFF) {
            pinMode(obstPin, INPUT);
            attachInterrupt(digitalPinToInterrupt(obstPin), obstISR, FALLING);
        }

        // Start in Stopped/unknown until the first bus STATUS response arrives.
        currentState = new Characteristic::CurrentDoorState(4);
        targetState  = new Characteristic::TargetDoorState(1);
        obstruction  = new Characteristic::ObstructionDetected(false);

        instance = this;
        GDOBus::onStateChange(busStateChanged);

        Log.printf("[Door] Service ready — waiting for first bus STATUS\n");
    }

    // ==========================================================================
    //  update()  —  Called by HomeSpan when HomeKit changes TargetDoorState.
    //               0 = Open, 1 = Closed.
    // ==========================================================================
    boolean update() override {
        int target = targetState->getNewVal();
        Log.printf("[Door] HomeKit → %s\n", target == 0 ? "OPEN" : "CLOSE");
        setGoal(target);
        return true;
    }

    // ==========================================================================
    //  loop()  —  Called by homeSpan.poll() every main-loop iteration.
    // ==========================================================================
    void loop() override {

        // ── 1. Obstruction sensor (pulse-window evaluation) ───────────────────
        evaluateObstruction();

        // ── 2. Active-goal management ─────────────────────────────────────────
        if (goalState >= 0) {

            // Poll the bus regularly while waiting for the opener to confirm.
            if (millis() - lastActivePoll >= ACTIVE_POLL_MS) {
                lastActivePoll = millis();
                GDOBus::requestStatus();
            }

            // Travel timeout: mark Stopped if no end-stop arrives in time,
            // then immediately poll so the bus can correct the state quickly.
            if (travelStart != 0 && millis() - travelStart > TRAVEL_TIMEOUT_MS) {
                Log.printf("[Door] Travel timeout (%lu ms) — marking Stopped\n",
                           TRAVEL_TIMEOUT_MS);
                currentState->setVal(4);
                goalState   = -1;
                travelStart = 0;
                GDOBus::requestStatus();
            }
        }

        // ── 3. Background sync poll (idle) ────────────────────────────────────
        if (goalState < 0 && millis() - lastBgPoll >= BG_POLL_MS) {
            lastBgPoll = millis();
            GDOBus::requestStatus();
        }

        // ── 4. Direct cache reconciliation ────────────────────────────────────
        // The state-change callback only fires when GDOBus decodes a state
        // DIFFERENT from its cached value.  If the bus already cached CLOSED
        // or OPEN from a prior poll, a subsequent poll returning the same state
        // does not re-fire the callback — the HAP state stays stuck in a
        // transitional value indefinitely.  Reading the cached state directly
        // here corrects that without waiting for a state change.
        int cur = currentState->getVal();
        if (goalState < 0 && (cur == 2 || cur == 3 || cur == 4)) {
            GDODoorState bus = GDOBus::getDoorState();
            if (bus == GDODoorState::CLOSED || bus == GDODoorState::OPEN) {
                int hapVal = (int)bus;
                if (cur != hapVal) {
                    Log.printf("[Door] Reconcile: HAP stuck at %d; bus cache = %s\n",
                               cur,
                               bus == GDODoorState::CLOSED ? "CLOSED" : "OPEN");
                    currentState->setVal(hapVal);
                    if (targetState->getVal() != hapVal) targetState->setVal(hapVal);
                    travelStart = 0;
                }
            }
        }
    }

    // Combined obstruction verdict: hardware beam sensor OR opener status flag.
    bool obstructionDetected() {
        return hwObstructed || GDOBus::getObstruction();
    }

private:

    // ==========================================================================
    //  busStateChanged()  —  Static trampoline registered with GDOBus.
    //                        Runs inside GDOBus::poll().
    // ==========================================================================
    static void busStateChanged(GDODoorState s) {
        if (instance) instance->onBusState(s);
    }

    // ==========================================================================
    //  onBusState()  —  Handle a door-state event from the wireline bus.
    //                   Only sets HAP values; called from GDOBus::poll().
    // ==========================================================================
    void onBusState(GDODoorState s) {
        switch (s) {

            case GDODoorState::OPEN:
            case GDODoorState::CLOSED: {
                // Opener confirmed an end-stop — authoritative.
                int hapVal = (int)s;
                Log.printf("[Door] Bus → %s\n",
                           s == GDODoorState::OPEN ? "OPEN" : "CLOSED");
                currentState->setVal(hapVal);
                if (targetState->getVal() != hapVal) targetState->setVal(hapVal);
                goalState   = -1;
                travelStart = 0;
                break;
            }

            case GDODoorState::OPENING:
            case GDODoorState::CLOSING: {
                int hapVal = (int)s;
                Log.printf("[Door] Bus → %s\n",
                           s == GDODoorState::OPENING ? "OPENING" : "CLOSING");
                if (currentState->getVal() != hapVal) currentState->setVal(hapVal);
                if (goalState < 0) {
                    // External actor (wall button or remote).
                    // Sync HomeKit target and start the travel timer so the
                    // timeout guards this trip too.
                    int tgt = (s == GDODoorState::OPENING) ? 0 : 1;
                    if (targetState->getVal() != tgt) targetState->setVal(tgt);
                    if (travelStart == 0) travelStart = millis();
                }
                break;
            }

            case GDODoorState::STOPPED:
                Log.printf("[Door] Bus → STOPPED\n");
                if (currentState->getVal() != 4) currentState->setVal(4);
                travelStart = 0;
                break;

            default:
                break;
        }
    }

    // ==========================================================================
    //  setGoal()  —  Record a HomeKit goal and issue the bus command.
    // ==========================================================================
    void setGoal(int target) {
        // Never close while an obstruction is active.
        if (target == 1 && obstructionDetected()) {
            Log.printf("[Door] Obstruction — CLOSE suppressed\n");
            return;
        }

        // Already at the target per the bus cache — no command needed.
        GDODoorState bus = GDOBus::getDoorState();
        if ((target == 0 && bus == GDODoorState::OPEN) ||
            (target == 1 && bus == GDODoorState::CLOSED)) {
            goalState = -1;
            currentState->setVal(target);
            if (targetState->getVal() != target) targetState->setVal(target);
            return;
        }

        goalState      = (int8_t)target;
        travelStart    = millis();
        lastActivePoll = millis();

        // Optimistic HAP state so the Home app shows motion immediately.
        currentState->setVal(target == 0 ? 2 : 3);

        GDOBus::sendDoorAction(target == 0 ? GDODoorAction::OPEN
                                           : GDODoorAction::CLOSE);
    }

    // ==========================================================================
    //  evaluateObstruction()  —  Pulse-window evaluation of the safety-beam wire.
    // ==========================================================================
    void evaluateObstruction() {
        if (obstPin == 0xFF) return;
        if (millis() - lastObstCheck < OBST_CHECK_MS) return;

        lastObstCheck   = millis();
        uint32_t pulses = obstLowCount;
        obstLowCount    = 0;

        if (pulses >= OBST_MIN_PULSES) {
            hwObstructed   = false;
            obstSensorSeen = true;
        } else if (digitalRead(obstPin) == LOW) {
            lastObstLow = millis();
        } else if (obstSensorSeen && millis() - lastObstLow > OBST_HIGH_MS) {
            hwObstructed = true;
        }

        bool obst = obstructionDetected();
        if (obstruction->getVal() != (obst ? 1 : 0)) {
            Log.printf("[Obst] → %s  (hw=%d  bus=%d)\n",
                       obst ? "DETECTED" : "clear", hwObstructed, GDOBus::getObstruction());
            obstruction->setVal(obst);
            // Re-sync door state after the obstruction clears — the door may
            // have changed position while the beam was blocked.
            if (!obst) GDOBus::requestStatus();
        }
    }
};
