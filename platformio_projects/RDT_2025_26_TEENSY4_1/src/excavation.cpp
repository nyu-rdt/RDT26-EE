#include <Arduino.h>
#include "config.h"
#include "excavation.h"
#include "can_driver.h"
#include "stepper_driver.h"
#if STRING_POT_ENABLED
#include "string_pot.h"
#endif

// Belt spin direction (CAN motor): +1 forward, -1 reverse, 0 stopped
static int beltDirection = 0;
// Vertical direction (stepper — raises/lowers the belt assembly): +1 up, -1 down, 0 stopped
static int vertDirection = 0;

#if RAMP_UP
static float targetSpeed = 0.0f;
static float currentSpeed = 0.0f;
static unsigned long lastRampMs = 0;

static float slew(float cur, float tgt, float maxDelta) {
    float d = tgt - cur;
    return (d > maxDelta) ? cur + maxDelta : (d < -maxDelta) ? cur - maxDelta : tgt;
}
#endif

#if STRING_POT_ENABLED
static unsigned long lastPotReadMs = 0;
#endif

static void applyBeltSpeed(float speed) {
#if RAMP_UP
    targetSpeed = speed;
#else
    CAN_SendExcavation(speed);
#endif
}

void EXCAV_Init() {
    beltDirection = 0;
    vertDirection = 0;
    STEPPER_Init();
#if STRING_POT_ENABLED
    STRINGPOT_Init();
    STRINGPOT_SetMoving(false);
#endif
#if RAMP_UP
    targetSpeed = 0.0f;
    currentSpeed = 0.0f;
    lastRampMs = millis();
#else
    CAN_SendExcavation(0.0f);
#endif
#if SERIAL_DEBUG
    Serial.println("[excav] init");
#endif
}

void EXCAV_Update() {
#if STRING_POT_ENABLED
    if (millis() - lastPotReadMs >= 50) {
        lastPotReadMs = millis();
        STRINGPOT_ReadDistance();
        STRINGPOT_UpdateState();
    }
#endif

    STEPPER_Update(EXCAVATION_STEP_PERIOD);

#if RAMP_UP
    if (millis() - lastRampMs >= TX_PERIOD_MS) {
        lastRampMs = millis();
        currentSpeed = slew(currentSpeed, targetSpeed, MAX_EXCAV_DELTA_PER_TICK);
        CAN_SendExcavation(currentSpeed);
    }
#endif

    // Stop vertical movement if the assembly has reached the travel limit it's moving toward.
    // Stepper-only — belt spin is independent and may still be running (e.g. digging in position).
#if STRING_POT_ENABLED
    if (vertDirection != 0) {
        int state = STRINGPOT_GetState();
        if ((vertDirection > 0 && state == STRING_HIGHEST) ||
            (vertDirection < 0 && state == STRING_LOWEST)) {
            vertDirection = 0;
            STEPPER_SetDirection(0);
            STRINGPOT_SetMoving(false);
#if SERIAL_DEBUG
            Serial.println("[excav] travel limit reached — stepper stopped, belt unchanged");
#endif
        }
    }
#endif
}

void EXCAV_SetBeltDirection(int direction) {
    beltDirection = (direction > 0) ? 1 : (direction < 0) ? -1 : 0;
    applyBeltSpeed(beltDirection * EXCAVATION_DUTY_CYCLE);
#if SERIAL_DEBUG
    Serial.print("[excav] belt: ");
    Serial.println(beltDirection > 0 ? "fwd" : beltDirection < 0 ? "rev" : "stop");
#endif
}

void EXCAV_SetVertDirection(int direction) {
    int newDir = (direction > 0) ? 1 : (direction < 0) ? -1 : 0;

#if STRING_POT_ENABLED
    // Force a fresh read — don't guard on state up to 50ms stale
    STRINGPOT_ReadDistance();
    STRINGPOT_UpdateState();
    int state = STRINGPOT_GetState();
    if ((newDir > 0 && state == STRING_HIGHEST) ||
        (newDir < 0 && state == STRING_LOWEST)) {
        // Already at limit — ensure stepper is stopped rather than silently ignoring
        vertDirection = 0;
        STEPPER_SetDirection(0);
        STRINGPOT_SetMoving(false);
#if SERIAL_DEBUG
        Serial.print("[excav] vert blocked at limit, pos=");
        Serial.println(STRINGPOT_GetCachedDistance(), 2);
#endif
        return;
    }
#endif

    vertDirection = newDir;
#if STRING_POT_ENABLED
    STRINGPOT_SetMoving(vertDirection != 0);
#endif
    STEPPER_SetDirection(vertDirection);
#if SERIAL_DEBUG
    Serial.print("[excav] vert: ");
    Serial.println(vertDirection > 0 ? "up" : vertDirection < 0 ? "down" : "stop");
#endif
}

float EXCAV_GetConveyorDistance() {
#if STRING_POT_ENABLED
    return STRINGPOT_GetCachedDistance();
#else
    return 0.0f;
#endif
}

void EXCAV_Stop() {
    beltDirection = 0;
    vertDirection = 0;
#if RAMP_UP
    // Reset ramp state so we don't slew back up after the stop
    targetSpeed = 0.0f;
    currentSpeed = 0.0f;
#endif
    CAN_SendExcavation(0.0f); // hard stop — bypass ramp
    STEPPER_SetDirection(0);
#if STRING_POT_ENABLED
    STRINGPOT_SetMoving(false);
#endif
#if SERIAL_DEBUG
    Serial.println("[excav] hard stop");
#endif
}
