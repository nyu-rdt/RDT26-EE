#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "depo_door_driver.h"

static Servo depoDoorActuator;

volatile static DepoDoorState doorState = DEPO_DOOR_STATE_CLOSED;
static int activeDirection = 0;
static unsigned long motionStartMs = 0;
static volatile float doorCurrentAmps = 0.0f;
static bool isArmed = false;
static unsigned long armReadyMs = 0;

static int getDoorPulseWidthUs(int direction) {
    return (direction > 0) ? DEPOSITION_DOOR_PULSE_OPEN_US:
            (direction < 0) ? DEPOSITION_DOOR_PULSE_CLOSE_US:
             DEPOSITION_DOOR_PULSE_STOP_US;
}

void DEPO_DOOR_Init() {
    depoDoorActuator.attach(DEPOSITION_DOOR_ACTUATOR_PIN, 500, 2500);
    depoDoorActuator.writeMicroseconds(getDoorPulseWidthUs(0));
    activeDirection = 0;
    doorState = DEPO_DOOR_STATE_CLOSED;
    isArmed = false;
    armReadyMs = millis() + DEPOSITION_DOOR_ARM_DELAY_MS;
}

void DEPO_DOOR_SetDirection(int direction) {
    int newDirection = (direction > 0) ? 1 : (direction < 0 ? -1 : 0);
    // Avoid resetting motionStartMs if already moving in the same direction
    if (newDirection == activeDirection) {
        return;
    }

    depoDoorActuator.writeMicroseconds(getDoorPulseWidthUs(direction));
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

static bool checkArmed() {
    if (isArmed) return true;
    if (millis() >= armReadyMs) {
        isArmed = true;
        return true;
    }
#if SERIAL_DEBUG
    Serial.println("DEPO DOOR: command rejected — actuator not yet armed");
#endif
    return false;
}

void DEPO_DOOR_Open() {
    if (!checkArmed()) return;
    if (activeDirection == 1) return;
    if (doorState == DEPO_DOOR_STATE_OPENED) return;
    DEPO_DOOR_SetDirection(1);
}

void DEPO_DOOR_Close() {
    if (!checkArmed()) return;
    if (activeDirection == -1) return;
    if (doorState == DEPO_DOOR_STATE_CLOSED) return;
    DEPO_DOOR_SetDirection(-1);
}

void DEPO_DOOR_EmergencyStop() {
    depoDoorActuator.writeMicroseconds(getDoorPulseWidthUs(0));
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
        depoDoorActuator.writeMicroseconds(getDoorPulseWidthUs(0));
#if SERIAL_DEBUG
        Serial.println(doorState == DEPO_DOOR_STATE_OPENED ? "DEPO DOOR: current stop — opened" : "DEPO DOOR: current stop — closed");
#endif
        return;
    }
#endif

    if (elapsed >= travelLimit) {
        doorState = (activeDirection > 0) ? DEPO_DOOR_STATE_OPENED : DEPO_DOOR_STATE_CLOSED;
        activeDirection = 0;
        depoDoorActuator.writeMicroseconds(getDoorPulseWidthUs(0));
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
