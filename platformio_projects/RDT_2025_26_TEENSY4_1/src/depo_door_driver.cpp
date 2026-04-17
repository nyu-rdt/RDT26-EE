#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "depo_door_driver.h"

static Servo depoDoorActuator;

static DepoDoorState doorState = DEPO_DOOR_STATE_STOPPED;
static int activeDirection = 0;
static unsigned long motionStartMs = 0;
static float doorCurrentAmps = 0.0f;

static int getDoorPulseWidthUs(int direction) {
    return (direction > 0) ? DEPOSITION_DOOR_PULSE_OPEN_US:
            (direction < 0) ? DEPOSITION_DOOR_PULSE_CLOSE_US:
             DEPOSITION_DOOR_PULSE_STOP_US;
}

void DEPO_DOOR_Init() {
    depoDoorActuator.attach(DEPOSITION_DOOR_ACTUATOR_PIN, 500, 2500);
    DEPO_DOOR_Stop();
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
    } else {
        doorState = DEPO_DOOR_STATE_STOPPED;
    }
}

void DEPO_DOOR_Open() {
    if (activeDirection != 1) {
        DEPO_DOOR_SetDirection(1);
    }
}

void DEPO_DOOR_Close() {
    if (activeDirection != -1) {
        DEPO_DOOR_SetDirection(-1);
    }
}

void DEPO_DOOR_Stop() {
    int pulseWidthUs = getDoorPulseWidthUs(0);
    depoDoorActuator.writeMicroseconds(pulseWidthUs);
    activeDirection = 0;
    doorState = DEPO_DOOR_STATE_STOPPED;
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
    // If the actuator current spikes after startup, assume we've hit travel limit.
    if (elapsed >= DEPOSITION_DOOR_CURRENT_DETECT_MIN_MS && doorCurrentAmps >= DEPOSITION_DOOR_CURRENT_THRESHOLD_A) {
        if (activeDirection > 0) {
            doorState = DEPO_DOOR_STATE_OPENED;
        } else {
            doorState = DEPO_DOOR_STATE_CLOSED;
        }
        activeDirection = 0;
        depoDoorActuator.writeMicroseconds(getDoorPulseWidthUs(0));
        return;
    }
#endif

    if (elapsed >= travelLimit) {
        if (activeDirection > 0) {
            doorState = DEPO_DOOR_STATE_TIMEOUT_OPEN;
        } else {
            doorState = DEPO_DOOR_STATE_TIMEOUT_CLOSE;
        }
        activeDirection = 0;
        depoDoorActuator.writeMicroseconds(getDoorPulseWidthUs(0));
    }
}

DepoDoorState DEPO_DOOR_GetState() {
    return doorState;
}

bool DEPO_DOOR_IsBusy() {
    return (activeDirection != 0);

}
