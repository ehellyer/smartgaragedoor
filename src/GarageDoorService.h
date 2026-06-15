// =============================================================================
//  GarageDoorService.h  —  HomeSpan GarageDoorOpener Service
//  (v1.3.0 — dual reed + explicit-action sequencer + hardware obstruction)
//
//  Two magnetic reed switches provide definitive hardware end-stop signals:
//
//    REED_CLOSED (GPIO25) — mounted at the CLOSED position on the door frame
//    REED_OPEN   (GPIO26) — mounted at the FULLY OPEN position on the overhead track
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
//  │ HIGH         │ LOW        │ CLOSED (1) — definitive hardware signal      │
//  │ LOW          │ HIGH       │ OPEN   (0) — definitive hardware signal      │
//  │ LOW          │ LOW        │ In between → OPENING(2) / CLOSING(3) /       │
//  │              │            │              STOPPED(4) — inferred from      │
//  │              │            │              last command + bus status       │
//  │ HIGH         │ HIGH       │ Error — both triggered simultaneously        │
//  │              │            │ (mechanically impossible)                    │
//  └──────────────┴────────────┴──────────────────────────────────────────────┘
//
//  State priority (highest first):
//    1. Reed switches   — hardware truth for the CLOSED and OPEN end-stops
//    2. Bus status      — OPENING / CLOSING / STOPPED events pushed by GDOBus
//                         via the onStateChange callback (event-driven, no
//                         polling, no staleness)
//    3. Optimistic command state — set immediately when HomeKit commands the
//                         door, then verified/corrected by the sequencer
//    4. Travel timeout  — any travel (HomeKit, wall button, or remote) that
//                         never reaches an end-stop is marked STOPPED after
//                         TRAVEL_TIMEOUT_MS
//
//  Command sequencer
//  ─────────────────
//  Security+ 2.0 wireline supports EXPLICIT door actions (OPEN / CLOSE — see
//  GDODoorAction), so no toggle guesswork is needed: the opener resolves the
//  direction itself, and repeating an action is harmless (idempotent).
//
//  update() records a goal (Open or Closed); seekGoal(), called from loop(),
//  sends the matching explicit action, then verifies progress using reed and
//  bus evidence.  Because the opener does NOT broadcast status on its own
//  (only on request or state change), seekGoal() polls GDOBus::requestStatus()
//  every STATUS_POLL_MS while a goal is active.  If no evidence of correct
//  motion or arrival appears within ACTION_RETRY_MS, the action is re-sent,
//  up to MAX_ACTIONS times before the goal is abandoned (the travel timeout
//  then settles the displayed state).
//
//  HAP CurrentDoorState values:
//    0 = Open   1 = Closed   2 = Opening   3 = Closing   4 = Stopped
// =============================================================================

#pragma once
#include "HomeSpan.h"
#include "GDOBus.h"
#include "DuaLogger.h"

struct GarageDoorService : Service::GarageDoorOpener {

    // Single-instance pointer used by the GDOBus callback trampoline
    inline static GarageDoorService *instance = nullptr;

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

    static const unsigned long DEBOUNCE_MS       = 50;    // Switch bounce filter
    static const unsigned long TRAVEL_TIMEOUT_MS = 18000; // 18 s max door travel

    // ── Command sequencer (goal seeker) ─────────────────────────────────────────
    int8_t        goalState;       // -1 = idle, 0 = seeking Open, 1 = seeking Closed
    uint8_t       actionCount;     // explicit actions sent for the current goal
    unsigned long lastActionTime;  // millis() of the last action; 0 = none yet
    unsigned long lastStatusPoll;  // millis() of the last GET_STATUS query
    unsigned long travelStartTime; // millis() when current travel began; 0 = no travel
    unsigned long lastBackgroundPoll; // millis() of the last idle status refresh

    static const unsigned long ACTION_RETRY_MS   = 4000;  // wait for evidence before re-sending the action
    static const unsigned long STATUS_POLL_MS    = 2000;  // GET_STATUS interval while a goal is active
    static const unsigned long STATUS_FRESH_MS   = 3000;  // status younger than this counts as evidence
    static const unsigned long STATUS_REFRESH_MS = 60000; // idle status refresh (light/lock/door sync)
    static const uint8_t       MAX_ACTIONS       = 4;     // give up on a goal after this many actions

    // ── Hardware obstruction sensor (safety-beam wire on a GPIO divider) ────────
    //
    //  The opener's obstruction-sensor wire has three states (ratgdo research):
    //    clear      → line HIGH with a brief LOW pulse every ~7 ms
    //    obstructed → line steady HIGH (pulses stop)
    //    asleep     → line steady LOW
    //  A falling-edge ISR counts the pulses; every OBST_CHECK_MS we evaluate:
    //  pulses present = clear; no pulses while HIGH for > OBST_HIGH_MS =
    //  OBSTRUCTED; steady LOW = asleep (no change).  To avoid false alarms
    //  when the sensor isn't wired, OBSTRUCTED is only reported after pulses
    //  have been seen at least once since boot (obstSensorSeen).
    uint8_t       obstPin;            // 0xFF = no hardware sensor
    bool          hwObstructed;       // current hardware verdict
    bool          obstSensorSeen;     // pulses observed since boot → sensor present
    unsigned long lastObstCheck;      // last evaluation window
    unsigned long lastObstLow;        // last time the line was seen LOW (pulse or asleep)

    static volatile uint32_t obstLowCount;   // pulse counter (ISR); defined in GarageDoorService.cpp

    static const unsigned long OBST_CHECK_MS  = 50;  // evaluation window
    static const uint32_t      OBST_MIN_PULSES = 3;  // pulses per window when clear (~7 per 50 ms)
    static const unsigned long OBST_HIGH_MS   = 70;  // steady-HIGH time that means obstructed

    // IRAM_ATTR definition lives in GarageDoorService.cpp — an IRAM function
    // defined inline in a header makes the Xtensa linker place its literal
    // pool after the code ("dangerous relocation: l32r" at link time).
    static void obstISR();

    // ── Constructor ────────────────────────────────────────────────────────────
    // obstructionPin: GPIO wired to the safety-sensor wire via a resistor
    // divider (pass 0xFF if no hardware sensor is connected).
    GarageDoorService(uint8_t closedPin, uint8_t openPin, uint8_t obstructionPin = 0xFF)
        : Service::GarageDoorOpener()
    {
        reedClosedPin = closedPin;
        reedOpenPin   = openPin;
        obstPin       = obstructionPin;

        pinMode(reedClosedPin, INPUT_PULLUP);
        pinMode(reedOpenPin,   INPUT_PULLUP);

        // Read initial hardware state (HIGH = magnet present = door at end-stop)
        reedClosedActive = (digitalRead(reedClosedPin) == HIGH);
        reedOpenActive   = (digitalRead(reedOpenPin)   == HIGH);
        closedChangeTime = 0;
        openChangeTime   = 0;
        goalState        = -1;
        actionCount      = 0;
        lastActionTime   = 0;
        lastStatusPoll   = 0;
        travelStartTime  = 0;
        lastBackgroundPoll = millis();   // setup() already queried status at boot

        // Hardware obstruction sensor input (divider-driven; external levels
        // define the pin, so plain INPUT — GPIO 34-39 have no pull-ups anyway)
        hwObstructed   = false;
        obstSensorSeen = false;
        lastObstCheck  = millis();
        lastObstLow    = millis();
        if (obstPin != 0xFF) {
            pinMode(obstPin, INPUT);
            attachInterrupt(digitalPinToInterrupt(obstPin), obstISR, FALLING);
        }

        // Determine starting HAP state from reed switches
        int initState = resolveHardwareState();

        currentState = new Characteristic::CurrentDoorState(initState);
        targetState  = new Characteristic::TargetDoorState(initState == 1 ? 1 : 0);
        obstruction  = new Characteristic::ObstructionDetected(false);

        // Receive door-state events decoded from the wireline bus.
        // The callback runs inside GDOBus::poll() — it only sets HAP values.
        instance = this;
        GDOBus::onStateChange(busStateChanged);

        LOG1("GarageDoorService ready  closedReed=%s  openReed=%s  initState=%d\n",
             reedClosedActive ? "ACTIVE" : "inactive",
             reedOpenActive   ? "ACTIVE" : "inactive",
             initState);
    }

    // ==========================================================================
    //  update()  —  Called by HomeSpan when HomeKit changes TargetDoorState.
    //               Records the goal; seekGoal() does the actual pressing.
    // ==========================================================================
    boolean update() override {
        int newTarget = targetState->getNewVal();   // 0 = Open, 1 = Closed
        LOG1("[HAP] TargetDoorState → %d  (current=%d)\n",
             newTarget, currentState->getVal());
        setGoal(newTarget);
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

        // ── 2. Command sequencer ──────────────────────────────────────────────
        seekGoal();

        // ── 3. Travel timeout ─────────────────────────────────────────────────
        //
        //  Applies to ALL travel — HomeKit commands, wall button, and remotes
        //  (travelStartTime is armed by setGoal(), sendGoalAction(), the reed
        //  departure handlers, and bus motion events).  If neither end-stop
        //  activates within TRAVEL_TIMEOUT_MS, mark the door Stopped.
        if (travelStartTime != 0 && millis() - travelStartTime > TRAVEL_TIMEOUT_MS) {
            int cur = currentState->getVal();
            if (cur == 2 || cur == 3) {
                LOG1("[Timeout] No end-stop after %lu ms — marking Stopped\n",
                     TRAVEL_TIMEOUT_MS);
                currentState->setVal(4);   // Stopped
            }
            travelStartTime = 0;
            goalState       = -1;          // abandon any active goal
        }

        // ── 4. Hardware obstruction sensor (pulse-window evaluation) ──────────
        if (obstPin != 0xFF && millis() - lastObstCheck >= OBST_CHECK_MS) {
            lastObstCheck   = millis();
            uint32_t pulses = obstLowCount;
            obstLowCount    = 0;

            if (pulses >= OBST_MIN_PULSES) {
                hwObstructed   = false;            // beam pulsing = clear
                obstSensorSeen = true;
            } else if (digitalRead(obstPin) == LOW) {
                lastObstLow = millis();            // steady LOW = sensor asleep
            } else if (obstSensorSeen && millis() - lastObstLow > OBST_HIGH_MS) {
                hwObstructed = true;               // steady HIGH = beam broken
            }
        }

        // ── 5. Obstruction characteristic (hardware sensor OR opener status) ──
        bool obst = obstructionDetected();
        if (obstruction->getVal() != (obst ? 1 : 0)) {
            LOG1("[Obst] Obstruction → %s  (hw=%d bus=%d)\n",
                 obst ? "DETECTED" : "clear", hwObstructed, GDOBus::getObstruction());
            obstruction->setVal(obst);
        }

        // ── 6. Background status refresh ──────────────────────────────────────
        //  The opener never broadcasts unsolicited periodic status, so poll
        //  occasionally while idle to keep door/light/lock state in sync with
        //  changes made at the wall console or by remotes.
        if (goalState < 0 && millis() - lastBackgroundPoll >= STATUS_REFRESH_MS) {
            lastBackgroundPoll = millis();
            GDOBus::requestStatus();
        }
    }

    // Combined obstruction verdict: hardware beam sensor OR opener status flag.
    bool obstructionDetected() {
        return hwObstructed || GDOBus::getObstruction();
    }

private:

    // ==========================================================================
    //  busStateChanged()  —  Static trampoline registered with GDOBus.
    // ==========================================================================
    static void busStateChanged(GDODoorState s) {
        if (instance) instance->onBusState(s);
    }

    // ==========================================================================
    //  onBusState()  —  Door-state event decoded from the wireline bus.
    //                   Runs inside GDOBus::poll(); only sets HAP values.
    // ==========================================================================
    void onBusState(GDODoorState s) {
        // End-stop reeds are hardware truth — bus events only refine the
        // in-between zone.
        if (reedClosedActive || reedOpenActive) return;

        if (s == GDODoorState::OPENING || s == GDODoorState::CLOSING) {
            int hapVal = (int)s;
            if (currentState->getVal() != hapVal) {
                LOG1("[Bus] In-transit direction → %s\n",
                     s == GDODoorState::OPENING ? "OPENING" : "CLOSING");
                currentState->setVal(hapVal);
            }
            // Motion with no active HomeKit goal = wall button or remote.
            // Keep TargetDoorState in sync so the Home app UI reads correctly.
            if (goalState < 0) {
                int tgt = (s == GDODoorState::OPENING) ? 0 : 1;
                if (targetState->getVal() != tgt) targetState->setVal(tgt);
            }
            if (travelStartTime == 0) travelStartTime = millis();
        }
        else if (s == GDODoorState::STOPPED && goalState < 0) {
            // Opener reports a mid-travel stop and no goal is active to retry.
            // (With a goal active, seekGoal() handles STOPPED by re-pressing.)
            if (currentState->getVal() != 4) {
                LOG1("[Bus] Door stopped mid-travel\n");
                currentState->setVal(4);
            }
            travelStartTime = 0;
        }
        // OPEN/CLOSED broadcasts are not applied here — seekGoal() consumes
        // them as arrival evidence, and the reed switches own the final
        // OPEN/CLOSED truth.
    }

    // ==========================================================================
    //  setGoal()  —  Record a new HomeKit goal (0 = Open, 1 = Closed) and set
    //                the optimistic HAP state.  seekGoal() sends the actions.
    // ==========================================================================
    void setGoal(int target) {
        bool atTarget = (target == 0) ? reedOpenActive : reedClosedActive;
        if (atTarget) {
            goalState = -1;            // already there — nothing to do
            return;
        }

        goalState      = (int8_t)target;
        actionCount    = 0;
        lastActionTime = 0;            // 0 → seekGoal() sends the action immediately
        lastStatusPoll = millis();

        // Optimistic UI: show the intended direction right away.  The actual
        // outcome is verified by seekGoal() using reed and bus evidence.
        currentState->setVal(target == 0 ? 2 : 3);
        travelStartTime = millis();
    }

    // ==========================================================================
    //  seekGoal()  —  Drive the door toward the active goal using explicit
    //                 OPEN/CLOSE actions (idempotent — safe to repeat).
    //
    //  Evidence model (per loop iteration):
    //    • reed at goal end-stop         → goal achieved, stop
    //    • bus = desired motion (fresh)  → moving correctly, wait
    //    • bus = arrival state  (fresh)  → opener says done, accept
    //    • otherwise                     → poll status every STATUS_POLL_MS;
    //                                      re-send action after ACTION_RETRY_MS,
    //                                      give up after MAX_ACTIONS attempts
    // ==========================================================================
    void seekGoal() {
        if (goalState < 0) return;

        // Arrived?  (The reed handlers also clear the goal — belt and braces.)
        if ((goalState == 0 && reedOpenActive) || (goalState == 1 && reedClosedActive)) {
            goalState = -1;
            return;
        }

        // Safety: never keep driving a CLOSE goal while an obstruction is
        // reported (hardware beam sensor or opener status) — the opener will
        // refuse or auto-reverse anyway, and repeated commands could drive
        // the door into the obstacle again.
        if (goalState == 1 && obstructionDetected()) {
            LOG1("[Seq] Obstruction reported — abandoning CLOSE goal\n");
            goalState = -1;
            return;
        }

        // First action fires immediately after setGoal()
        if (lastActionTime == 0) {
            sendGoalAction();
            return;
        }

        GDODoorState desired = (goalState == 0) ? GDODoorState::OPENING : GDODoorState::CLOSING;
        GDODoorState arrived = (goalState == 0) ? GDODoorState::OPEN    : GDODoorState::CLOSED;

        GDODoorState bus   = GDOBus::getDoorState();
        bool         fresh = (millis() - GDOBus::lastStateTimestamp()) < STATUS_FRESH_MS;

        if (fresh && bus == desired) return;    // moving the right way — let it run

        // Opener reports arrival but the goal-side reed hasn't confirmed
        // (sensor gap or misalignment).  Don't fight the opener — accept and
        // stop sequencing.  A reed event remains authoritative if it fires.
        bool wrongEndReed = (goalState == 0) ? reedClosedActive : reedOpenActive;
        if (fresh && bus == arrived && !wrongEndReed) {
            LOG1("[Seq] Opener reports %s (reed unconfirmed) — accepting\n",
                 goalState == 0 ? "OPEN" : "CLOSED");
            currentState->setVal((int)arrived);
            travelStartTime = 0;
            goalState       = -1;
            return;
        }

        // No fresh evidence — the opener does not broadcast status on its own,
        // so ask for it while the goal is active.
        if (millis() - lastStatusPoll >= STATUS_POLL_MS) {
            lastStatusPoll = millis();
            GDOBus::requestStatus();
            return;
        }

        if (millis() - lastActionTime < ACTION_RETRY_MS) return;

        if (actionCount >= MAX_ACTIONS) {
            LOG1("[Seq] Goal %s abandoned after %u actions\n",
                 goalState == 0 ? "OPEN" : "CLOSED", actionCount);
            goalState = -1;     // travel timeout will mark Stopped if needed
            return;
        }

        sendGoalAction();
    }

    // ==========================================================================
    //  sendGoalAction()  —  Send the explicit door action for the active goal
    //                       and restart the timers.
    // ==========================================================================
    void sendGoalAction() {
        actionCount++;
        LOG1("[Seq] Door action %s (%u of %u)\n",
             goalState == 0 ? "OPEN" : "CLOSE", actionCount, MAX_ACTIONS);
        GDOBus::sendDoorAction(goalState == 0 ? GDODoorAction::OPEN
                                              : GDODoorAction::CLOSE);
        lastActionTime  = millis();
        travelStartTime = millis();   // restart the stall timer with each action
    }

    // ==========================================================================
    //  resolveHardwareState()  —  Translate the two reed switch readings into a
    //  HAP door state integer.  Called only at construction time.
    // ==========================================================================
    int resolveHardwareState() {
        if (!reedClosedActive && reedOpenActive) return 0;   // Open
        if (reedClosedActive && !reedOpenActive) return 1;   // Closed
        if (!reedClosedActive && !reedOpenActive) return 4;  // In-between → Stopped
        // Both active simultaneously is an error; default to Stopped
        Log.println("[Reed] WARNING: Both reed switches active — check wiring!   Defaulting to Stopped state.");
        return 4;
    }

    // ==========================================================================
    //  debounce()  —  Returns true once rawNow has held a value different from
    //  `accepted` continuously for DEBOUNCE_MS, signalling a confirmed change.
    //  Updates changeTime in-place.
    // ==========================================================================
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
            // Definitive hardware truth — override any bus or inferred state.
            currentState->setVal(1);    // Closed
            travelStartTime = 0;
            if (goalState == 0) {
                // We wanted OPEN but the door ended CLOSED (obstruction
                // auto-reverse or toggle overshoot) — leave the goal active;
                // seekGoal() will press again.  Don't touch targetState.
            } else {
                goalState = -1;         // goal CLOSED achieved, or no goal active
                if (targetState->getVal() != 1) targetState->setVal(1);
            }
        } else {
            // ── Door just left the CLOSED end-stop ────────────────────────────
            // Leaving the closed position means the door is opening, whoever
            // commanded it (HomeKit, wall button, or remote).
            currentState->setVal(2);    // Opening
            if (goalState < 0 && targetState->getVal() != 0) {
                // External actor — keep the HomeKit target in sync.
                targetState->setVal(0);
            }
            if (travelStartTime == 0) travelStartTime = millis();
        }
    }

    // ==========================================================================
    //  onOpenReedChange()  —  Called when the OPEN reed switch changes state
    // ==========================================================================
    void onOpenReedChange() {
        if (reedOpenActive) {
            // ── Door just arrived at the FULLY OPEN end-stop ──────────────────
            // Definitive hardware truth — override any bus or inferred state.
            currentState->setVal(0);    // Open
            travelStartTime = 0;
            if (goalState == 1) {
                // We wanted CLOSED but the door ended OPEN (obstruction
                // auto-reverse) — leave the goal active; seekGoal() will
                // press again.  Don't touch targetState.
            } else {
                goalState = -1;         // goal OPEN achieved, or no goal active
                if (targetState->getVal() != 0) targetState->setVal(0);
            }
        } else {
            // ── Door just left the FULLY OPEN end-stop ────────────────────────
            // Leaving the open position means the door is closing, whoever
            // commanded it (HomeKit, wall button, or remote).
            currentState->setVal(3);    // Closing
            if (goalState < 0 && targetState->getVal() != 1) {
                // External actor — keep the HomeKit target in sync.
                targetState->setVal(1);
            }
            if (travelStartTime == 0) travelStartTime = millis();
        }
    }
};
