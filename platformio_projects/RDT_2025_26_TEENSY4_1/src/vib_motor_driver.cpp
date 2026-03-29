#include <Arduino.h>
#include "config.h"
#include "vib_motor_driver.h"

void VIB_Init() {
    pinMode(VIB_MOTOR_PIN, OUTPUT);
    digitalWrite(VIB_MOTOR_PIN, HIGH); // Active HIGH to stop it
}

void VIB_drive(int direction) {
    #if ANALOG_VIB_CONTROL
        if (direction != 0) {
            analogWrite(VIB_MOTOR_PIN, static_cast<int>(VIB_MOTOR_DUTY_CYCLE * 255));
        } else {
            analogWrite(VIB_MOTOR_PIN, 0);
        }
    #else
        digitalWrite(VIB_MOTOR_PIN, (direction != 0) ? LOW : HIGH);
    #endif
}
