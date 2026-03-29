#include <Arduino.h>
#include "config.h"
#include "system.h"
#include "stepper_driver.h"
#include "depo_door_driver.h"
#include "vib_motor_driver.h"

static SystemStopLocomotionFn stopLocomotion = nullptr;
static SystemStopExcavationFn stopExcavation = nullptr;

void SYSTEM_Init() {
    pinMode(E_STOP_PIN, OUTPUT);
    digitalWrite(E_STOP_PIN, HIGH);
}

void SYSTEM_Update() {
    // Keep relay energized continuously for fail-safe behavior.
    digitalWrite(E_STOP_PIN, HIGH);
}

void SYSTEM_RegisterStopCallbacks(SystemStopLocomotionFn stopLocomotionFn,
                                  SystemStopExcavationFn stopExcavationFn) {
    stopLocomotion = stopLocomotionFn;
    stopExcavation = stopExcavationFn;
}

void SYSTEM_StopAllMotors() {
    if (stopLocomotion != nullptr) {
        stopLocomotion();
    }
    if (stopExcavation != nullptr) {
        stopExcavation();
    }

    STEPPER_SetDirection(0);
    DEPO_DOOR_SetDirection(0);
    VIB_drive(0);
}
