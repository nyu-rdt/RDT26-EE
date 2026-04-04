#include <Arduino.h>
#include "config.h"
#include "system.h"
#include "stepper_driver.h"
#include "depo_door_driver.h"
#include "vib_motor_driver.h"

static SystemStopLocomotionFn stopLocomotion = nullptr;
static SystemStopExcavationFn stopExcavation = nullptr;
volatile bool relay_state = false;
volatile bool relay_3s_low = false;
volatile bool relay_6s_low = false;

void SYSTEM_Init() {
    pinMode(RELAY_DRIVER_PIN, OUTPUT);
    pinMode(RELAY_READ_PIN, INPUT_PULLDOWN);
    pinMode(RELAY_3S_LOW_PIN, INPUT_PULLDOWN);
    pinMode(RELAY_6S_LOW_PIN, INPUT_PULLDOWN);
    digitalWrite(RELAY_DRIVER_PIN, HIGH);
}

void SYSTEM_Update() {
    digitalWrite(RELAY_DRIVER_PIN, HIGH);
    relay_state = digitalRead(RELAY_READ_PIN);
    relay_3s_low = digitalRead(RELAY_3S_LOW_PIN);
    relay_6s_low = digitalRead(RELAY_6S_LOW_PIN);
}

void SYSTEM_RegisterStopCallbacks(SystemStopLocomotionFn stopLocomotionFn,
                                  SystemStopExcavationFn stopExcavationFn) {
    stopLocomotion = stopLocomotionFn;
    stopExcavation = stopExcavationFn;
}

// Returns relay pin states packed into one byte: bit0=relay, bit1=3s_low, bit2=6s_low
uint8_t SYSTEM_GetRelayStatus() {
    return (uint8_t)(((uint8_t)relay_6s_low << 2) | ((uint8_t)relay_3s_low << 1) | (uint8_t)relay_state);
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
