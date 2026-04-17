#include <Arduino.h>
#include "config.h"
#include "excavation.h"
#include "can_driver.h"
#include "stepper_driver.h"
#if STRING_POT_ENABLED
#include "string_pot.h"
#endif

// Belt direction: +1 up, -1 down, 0 stopped
static int beltDirection = 0;

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

    // Stop belt if we've reached the travel limit we're moving toward
#if STRING_POT_ENABLED
    if (beltDirection != 0) {
        int state = STRINGPOT_GetState();
        if ((beltDirection > 0 && state == STRING_HIGHEST) ||
            (beltDirection < 0 && state == STRING_LOWEST)) {
            EXCAV_Stop();
        }
    }
#endif
}

void EXCAV_SetBeltDirection(int direction) {
    int newDir = (direction > 0) ? 1 : (direction < 0) ? -1 : 0;

#if STRING_POT_ENABLED
    // Force a fresh read so we don't guard on state that's up to 50ms old
    STRINGPOT_ReadDistance();
    STRINGPOT_UpdateState();
    int state = STRINGPOT_GetState();
    if ((newDir > 0 && state == STRING_HIGHEST) ||
        (newDir < 0 && state == STRING_LOWEST)) {
        return;
    }
#endif

    beltDirection = newDir;
#if STRING_POT_ENABLED
    STRINGPOT_SetMoving(beltDirection != 0);
#endif
    applyBeltSpeed(beltDirection * EXCAVATION_DUTY_CYCLE);
}

void EXCAV_SetVertDirection(int direction) {
    STEPPER_SetDirection(direction);
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
}
