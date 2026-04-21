#include <Arduino.h>
#include "config.h"
#include "locomotion.h"
#include "can_driver.h"
#include "math_utils.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif

#if RAMP_UP
static float currentLeft = 0.0f, currentRight = 0.0f;
static float targetLeft = 0.0f, targetRight = 0.0f;
static unsigned long lastTxMs = 0;
#endif

void LOCO_Init() {
#if RAMP_UP
    currentLeft = currentRight = targetLeft = targetRight = 0.0f;
    lastTxMs = millis();
#else
    CAN_SendLocomotion(0.0f, 0.0f);
#endif
}

void LOCO_Update() {
#if RAMP_UP
    if (millis() - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = millis();
        currentLeft  = slew(currentLeft,  targetLeft,  MAX_SPEED_DELTA_PER_TICK);
        currentRight = slew(currentRight, targetRight, MAX_SPEED_DELTA_PER_TICK);
        CAN_SendLocomotion(currentLeft, currentRight);
    }
#endif
}

void LOCO_SetSpeeds(float left, float right) {
#if RAMP_UP
    targetLeft  = left;
    targetRight = right;
#else
    CAN_SendLocomotion(left, right);
#endif
}

void LOCO_Stop() {
#if RAMP_UP
    targetLeft = targetRight = 0.0f;
#else
    CAN_SendLocomotion(0.0f, 0.0f);
#endif
}

void LOCO_EmergencyStop() {
#if RAMP_UP
    targetLeft = targetRight = 0.0f;
    currentLeft = currentRight = 0.0f;
#endif
    CAN_SendLocomotion(0.0f, 0.0f);
#if SERIAL_DEBUG
    Serial.println("[loco] e-stop");
#endif
}

bool LOCO_IsStalled() {
#if CURRENT_SENSE_ENABLED && RAMP_UP
    if (CURRENT_LOCO_STALL_A == 0.0f) return false;
    if (targetLeft == 0.0f && targetRight == 0.0f) return false;
    const float* c = CURRENT_SENSORS_GetBuffer();
    return c[0] > CURRENT_LOCO_STALL_A || c[1] > CURRENT_LOCO_STALL_A;
#else
    return false;
#endif
}
