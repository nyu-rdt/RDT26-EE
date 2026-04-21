#include <Arduino.h>
#include "config.h"
#include "depo_door_driver.h"

static DepoDoorState doorState = DEPO_DOOR_STATE_CLOSED;
static int activeDirection = 0;
static unsigned long motionStartMs = 0;
static float doorCurrentAmps = 0.0f;

static void setMotorDirection(int direction) {
    if (direction > 0) {
        digitalWrite(DEPOSITION_DOOR_IN1_PIN, HIGH);
        digitalWrite(DEPOSITION_DOOR_IN2_PIN, LOW);
    } else if (direction < 0) {
        digitalWrite(DEPOSITION_DOOR_IN1_PIN, LOW);
        digitalWrite(DEPOSITION_DOOR_IN2_PIN, HIGH);
    } else {
        digitalWrite(DEPOSITION_DOOR_IN1_PIN, LOW);
        digitalWrite(DEPOSITION_DOOR_IN2_PIN, LOW);
    }
}

void DEPO_DOOR_Init() {
    pinMode(DEPOSITION_DOOR_IN1_PIN, OUTPUT);
    pinMode(DEPOSITION_DOOR_IN2_PIN, OUTPUT);
    pinMode(DEPOSITION_DOOR_ENA_PIN, OUTPUT);

    setMotorDirection(0);

#if ANALOG_DOOR_CONTROL
    analogWrite(DEPOSITION_DOOR_ENA_PIN, DEPOSITION_DOOR_ENA_DUTY);
#else
    digitalWrite(DEPOSITION_DOOR_ENA_PIN, HIGH);
#endif
    activeDirection = 0;
    doorState = DEPO_DOOR_STATE_CLOSED;
}

void DEPO_DOOR_SetDirection(int direction) {
    int newDirection = (direction > 0) ? 1 : (direction < 0 ? -1 : 0);
    if (newDirection == activeDirection) {
        return;
    }

    setMotorDirection(newDirection);
    activeDirection = newDirection;
    doorCurrentAmps = 0.0f; // clear stale current so it doesn't trip detection on the new move
    if (activeDirection > 0) {
        doorState = DEPO_DOOR_STATE_OPENING;
        motionStartMs = millis();
    } else if (activeDirection < 0) {
        doorState = DEPO_DOOR_STATE_CLOSING;
        motionStartMs = millis();
    }
}

void DEPO_DOOR_Open() {
    if (activeDirection == 1) return;
    if (doorState == DEPO_DOOR_STATE_OPENED) return;
    DEPO_DOOR_SetDirection(1);
}

void DEPO_DOOR_Close() {
    if (activeDirection == -1) return;
    if (doorState == DEPO_DOOR_STATE_CLOSED) return;
    DEPO_DOOR_SetDirection(-1);
}

void DEPO_DOOR_EmergencyStop() {
    setMotorDirection(0);
    activeDirection = 0;
    // State intentionally preserved: OPENING/CLOSING tells SW the last known direction.
#if SERIAL_DEBUG
    Serial.println("DEPO DOOR: emergency stop — position unknown");
#endif
}

void DEPO_DOOR_SetMeasuredCurrent(float currentAmps) {
    doorCurrentAmps = currentAmps;
}

void DEPO_DOOR_Update() {
    if (activeDirection == 0) {
        return;
    }

    unsigned long now = millis();
    unsigned long elapsed = now - motionStartMs;
    unsigned long travelLimit = (activeDirection > 0) ? DEPOSITION_DOOR_OPEN_TRAVEL_MS : DEPOSITION_DOOR_CLOSE_TRAVEL_MS;

#if DEPOSITION_DOOR_ENABLE_CURRENT_STOP
    // CURRENT_DETECT_MIN_MS is the startup blind spot. Real worst-case detection latency is
    // CURRENT_DETECT_MIN_MS + current sensor cycle time (~80 ms for 8 channels at 10 ms each).
    if (elapsed >= DEPOSITION_DOOR_CURRENT_DETECT_MIN_MS && doorCurrentAmps >= DEPOSITION_DOOR_CURRENT_THRESHOLD_A) {
        doorState = (activeDirection > 0) ? DEPO_DOOR_STATE_OPENED : DEPO_DOOR_STATE_CLOSED;
        activeDirection = 0;
        setMotorDirection(0);
#if SERIAL_DEBUG
        Serial.println(doorState == DEPO_DOOR_STATE_OPENED ? "DEPO DOOR: current stop — opened" : "DEPO DOOR: current stop — closed");
#endif
        return;
    }
#endif

    if (elapsed >= travelLimit) {
        doorState = (activeDirection > 0) ? DEPO_DOOR_STATE_OPENED : DEPO_DOOR_STATE_CLOSED;
        activeDirection = 0;
        setMotorDirection(0);
#if SERIAL_DEBUG
        Serial.println(doorState == DEPO_DOOR_STATE_OPENED ? "DEPO DOOR: timeout — opened" : "DEPO DOOR: timeout — closed");
#endif
    }
}

DepoDoorState DEPO_DOOR_GetState() {
    return doorState;
}

bool DEPO_DOOR_IsBusy() {
    return (activeDirection != 0);
}
