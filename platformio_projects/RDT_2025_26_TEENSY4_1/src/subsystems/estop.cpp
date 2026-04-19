#include <Arduino.h>
#include "config.h"
#include "estop.h"
#include "excavation.h"
#include "deposition.h"

static StopFn stopLocomotion = nullptr;

void ESTOP_RegisterCallbacks(StopFn stopLocomotionFn) {
    stopLocomotion = stopLocomotionFn;
}

void ESTOP_StopAllMotors() {
    if (stopLocomotion != nullptr) stopLocomotion();
    EXCAV_Stop();
    DEPO_EmergencyStop();
}

void ESTOP_Trigger() {
    ESTOP_StopAllMotors();
#if SERIAL_DEBUG
    Serial.println("[estop] triggered");
#endif
}
