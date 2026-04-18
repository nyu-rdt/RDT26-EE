// test/test_depo_door/test_main.cpp
//
// Hardware deposition door monitor — NOT a unit test.
// Repeatedly commands open/close in a non-blocking loop and streams
// state/current/timing telemetry in Teleplot format (>name:value).
//
// H-bridge wiring: IN1=pin40, IN2=pin41, ENA=pin14
// Use DD_open_ms / DD_close_ms output to tune DEPOSITION_DOOR_OPEN_TRAVEL_MS /
// DEPOSITION_DOOR_CLOSE_TRAVEL_MS in config.h.
// Set DEPOSITION_DOOR_ENABLE_CURRENT_STOP=1 in config.h once threshold is known.
//
// Build + upload:  pio run -e teensy41_depo_door_test -t upload
// Monitor:         Teleplot extension or Serial Monitor @ 115200 baud

#include <Arduino.h>
#include "config.h"
#include "depo_door_driver.h"

#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif

enum DoorTestPhase {
    PHASE_COMMAND_OPEN = 0,
    PHASE_WAIT_OPEN_COMPLETE,
    PHASE_HOLD_OPEN,
    PHASE_COMMAND_CLOSE,
    PHASE_WAIT_CLOSE_COMPLETE,
    PHASE_HOLD_CLOSE
};

static DoorTestPhase phase = PHASE_COMMAND_OPEN;
static unsigned long phaseStartMs = 0;
static unsigned long motionStartMs = 0;
static unsigned long openDurationMs = 0;
static unsigned long closeDurationMs = 0;
static unsigned long lastPrintMs = 0;
static DepoDoorState lastState = DEPO_DOOR_STATE_CLOSED;

static constexpr unsigned long kPrintPeriodMs = 100;
static constexpr unsigned long kHoldOpenMs = 1000;
static constexpr unsigned long kHoldCloseMs = 1000;

static const char* doorStateToString(DepoDoorState state) {
    switch (state) {
        case DEPO_DOOR_STATE_CLOSED:  return "CLOSED";
        case DEPO_DOOR_STATE_OPENED:  return "OPENED";
        case DEPO_DOOR_STATE_OPENING: return "OPENING";
        case DEPO_DOOR_STATE_CLOSING: return "CLOSING";
        default: return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);

    DEPO_DOOR_Init();
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);

#if CURRENT_SENSE_ENABLED
    CURRENT_SENSORS_Init();
    Serial.println("# current sensors: ok");
#else
    Serial.println("# current sensors: DISABLED in config.h");
#endif

    phaseStartMs = millis();
    motionStartMs = millis();

    Serial.println("# depo door test started");
    Serial.println("# cycling OPEN and CLOSE automatically");
    Serial.println("# Teleplot streams: DD_state, DD_busy, DD_current, DD_open_ms, DD_close_ms, DD_phase");
}

void loop() {
    const unsigned long now = millis();

#if CURRENT_SENSE_ENABLED
    CURRENT_SENSORS_Update();
#if (DEPOSITION_DOOR_CURRENT_SENSOR_INDEX >= 0) && (DEPOSITION_DOOR_CURRENT_SENSOR_INDEX < NUM_CURRENT_SENSORS)
    DEPO_DOOR_SetMeasuredCurrent(CURRENT_SENSORS_GetBuffer()[DEPOSITION_DOOR_CURRENT_SENSOR_INDEX]);
#endif
#endif

    DEPO_DOOR_Update();
    const DepoDoorState state = DEPO_DOOR_GetState();

    if (state != lastState) {
        Serial.print("# state -> ");
        Serial.println(doorStateToString(state));
        lastState = state;
    }

    switch (phase) {
        case PHASE_COMMAND_OPEN:
            DEPO_DOOR_Open();
            motionStartMs = now;
            phase = PHASE_WAIT_OPEN_COMPLETE;
            phaseStartMs = now;
            break;

        case PHASE_WAIT_OPEN_COMPLETE:
            if (!DEPO_DOOR_IsBusy()) {
                openDurationMs = now - motionStartMs;
                phase = PHASE_HOLD_OPEN;
                phaseStartMs = now;
                Serial.print("# open complete in ms: ");
                Serial.println(openDurationMs);
            }
            break;

        case PHASE_HOLD_OPEN:
            if (now - phaseStartMs >= kHoldOpenMs) {
                phase = PHASE_COMMAND_CLOSE;
                phaseStartMs = now;
            }
            break;

        case PHASE_COMMAND_CLOSE:
            DEPO_DOOR_Close();
            motionStartMs = now;
            phase = PHASE_WAIT_CLOSE_COMPLETE;
            phaseStartMs = now;
            break;

        case PHASE_WAIT_CLOSE_COMPLETE:
            if (!DEPO_DOOR_IsBusy()) {
                closeDurationMs = now - motionStartMs;
                phase = PHASE_HOLD_CLOSE;
                phaseStartMs = now;
                Serial.print("# close complete in ms: ");
                Serial.println(closeDurationMs);
            }
            break;

        case PHASE_HOLD_CLOSE:
            if (now - phaseStartMs >= kHoldCloseMs) {
                phase = PHASE_COMMAND_OPEN;
                phaseStartMs = now;
            }
            break;
    }

    if (now - lastPrintMs >= kPrintPeriodMs) {
        lastPrintMs = now;

        Serial.print(">DD_state:");
        Serial.println(static_cast<int>(state));
        Serial.print(">DD_busy:");
        Serial.println(DEPO_DOOR_IsBusy() ? 1 : 0);
        Serial.print(">DD_phase:");
        Serial.println(static_cast<int>(phase));

#if CURRENT_SENSE_ENABLED && (DEPOSITION_DOOR_CURRENT_SENSOR_INDEX >= 0) && (DEPOSITION_DOOR_CURRENT_SENSOR_INDEX < NUM_CURRENT_SENSORS)
        Serial.print(">DD_current:");
        Serial.println(CURRENT_SENSORS_GetBuffer()[DEPOSITION_DOOR_CURRENT_SENSOR_INDEX], 2);
#else
        Serial.println(">DD_current:-1");
#endif

        Serial.print(">DD_open_ms:");
        Serial.println(openDurationMs);
        Serial.print(">DD_close_ms:");
        Serial.println(closeDurationMs);
    }
}
