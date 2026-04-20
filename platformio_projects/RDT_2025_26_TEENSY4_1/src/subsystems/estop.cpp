#include <Arduino.h>
#include "config.h"
#include "estop.h"

static constexpr uint8_t MAX_STOP_CALLBACKS = 8;
static StopFn stopCallbacks[MAX_STOP_CALLBACKS];
static uint8_t stopCallbackCount = 0;

void ESTOP_RegisterCallback(StopFn fn) {
    if (fn != nullptr && stopCallbackCount < MAX_STOP_CALLBACKS) {
        stopCallbacks[stopCallbackCount++] = fn;
        return;
    }

#if SERIAL_DEBUG
    if (fn == nullptr) {
        Serial.println("[estop] register callback failed: null callback");
    } else {
        Serial.println("[estop] register callback failed: callback list full");
    }
#endif
}

void ESTOP_StopAllMotors() {
    for (uint8_t i = 0; i < stopCallbackCount; i++) {
        stopCallbacks[i]();
    }
}

void ESTOP_Trigger() {
    ESTOP_StopAllMotors();
#if SERIAL_DEBUG
    Serial.println("[estop] triggered");
#endif
}
