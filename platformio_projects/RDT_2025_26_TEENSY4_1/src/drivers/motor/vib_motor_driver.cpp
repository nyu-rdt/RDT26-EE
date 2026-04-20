#include <Arduino.h>
#include "config.h"
#include "vib_motor_driver.h"

static bool vibActive = false;

static void VIB_Apply(bool enable) {
#if ANALOG_VIB_CONTROL
    int pwmValue = enable ? (int)(VIB_MOTOR_DUTY_CYCLE * 255.0f) : 0;
    analogWrite(VIB_MOTOR_PIN, pwmValue);
#else
    digitalWrite(VIB_MOTOR_PIN, enable ? HIGH : LOW);
#endif
}

void VIB_Init() {
    pinMode(VIB_MOTOR_PIN, OUTPUT);
    VIB_drive(0);
}

void VIB_drive(int direction) {
    vibActive = direction > 0;
    VIB_Apply(vibActive);
}

void VIB_EmergencyStop() {
    vibActive = false;
    VIB_Apply(false);
}

bool VIB_IsActive() {
    return vibActive;
}
