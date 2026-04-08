#include <Arduino.h>
#include "config.h"
#include "current_sensors.h"

void CURRENT_SENSORS_Init() {
    pinMode(CURRENT_INPUT_PIN, INPUT);
    pinMode(CURRENT_SELECT_PIN_0, OUTPUT);
    pinMode(CURRENT_SELECT_PIN_1, OUTPUT);
    pinMode(CURRENT_SELECT_PIN_2, OUTPUT);
}

void CURRENT_SENSORS_Update( float *currents ) {
    for (int currently_read = 0; currently_read < NUM_CURRENT_SENSORS; currently_read++) {
        // Set the select pins to choose which current sensor to read
        digitalWrite(CURRENT_SELECT_PIN_0, currently_read & 0x01);
        digitalWrite(CURRENT_SELECT_PIN_1, (currently_read >> 1) & 0x01);
        digitalWrite(CURRENT_SELECT_PIN_2, (currently_read >> 2) & 0x01);
        delay(10);

        int rawValue = analogRead(CURRENT_INPUT_PIN);
        currents[currently_read] = rawValue * CURRENT_SCALING;
    }
}