#include <Arduino.h>
#include "config.h"
#include "vib_motor_driver.h"

static bool vibEnabledFromDirection(int direction) {
    return direction > 0;
}

void VIB_Init() {
    pinMode(VIB_MOTOR_PIN, OUTPUT);
    VIB_drive(0);
}

void VIB_drive(int direction) {
    bool enable = vibEnabledFromDirection(direction);
#if ANALOG_VIB_CONTROL
    // Active-HIGH stop wiring: lower duty drives the motor harder.
    int pwmValue = enable
                       ? (int)((VIB_MOTOR_DUTY_CYCLE) * 255.0f)
                       : 0;
    analogWrite(VIB_MOTOR_PIN, pwmValue);
#else
    digitalWrite(VIB_MOTOR_PIN, enable ? HIGH : LOW);
#endif
}
