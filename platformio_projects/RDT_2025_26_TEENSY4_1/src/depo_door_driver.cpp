#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "depo_door_driver.h"

static Servo depoDoorActuator;

static int getDoorPulseWidthUs(int direction) {
    return (direction > 0) ? DEPOSITION_DOOR_PULSE_OPEN_US:
            (direction < 0) ? DEPOSITION_DOOR_PULSE_CLOSE_US:
             DEPOSITION_DOOR_PULSE_STOP_US;
}

void DEPO_DOOR_Init() {
    depoDoorActuator.attach(DEPOSITION_DOOR_ACTUATOR_PIN, 500, 2500);
    DEPO_DOOR_SetDirection(0);
    delay(DEPOSITION_DOOR_ARM_DELAY_MS);
}

void DEPO_DOOR_SetDirection(int direction) {
    int pulseWidthUs = getDoorPulseWidthUs(direction);
    depoDoorActuator.writeMicroseconds(pulseWidthUs);

#if SERIAL_DEBUG
    Serial.print("Depo door dir: ");
    Serial.print(direction);
    Serial.print(" pulse: ");
    Serial.println(pulseWidthUs);
#endif
}
