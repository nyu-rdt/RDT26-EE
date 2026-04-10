#include <Arduino.h>
#include "config.h"
#include "current_sensors.h"

// State machine for non-blocking current sensor reads
static int current_read_index = 0;
static unsigned long last_channel_switch_ms = 0;

void CURRENT_SENSORS_Init() {
    analogReadResolution(10); 
    pinMode(CURRENT_INPUT_PIN, INPUT);
    pinMode(CURRENT_SELECT_PIN_0, OUTPUT);
    pinMode(CURRENT_SELECT_PIN_1, OUTPUT);
    pinMode(CURRENT_SELECT_PIN_2, OUTPUT);
    
    // Initialize multiplexer to first channel
    digitalWrite(CURRENT_SELECT_PIN_0, 0);
    digitalWrite(CURRENT_SELECT_PIN_1, 0);
    digitalWrite(CURRENT_SELECT_PIN_2, 0);
    last_channel_switch_ms = millis();
}

void CURRENT_SENSORS_Update( float *currents ) {
    unsigned long now = millis();
    
    // Check if enough time has passed since last channel switch for settling
    if (now - last_channel_switch_ms >= CHANNEL_SETTLE_MS) {
        // Read the current channel
        int rawValue = analogRead(CURRENT_INPUT_PIN);
        currents[current_read_index] = rawValue * CURRENT_SCALING;
        
        // Move to next channel
        if(++current_read_index == NUM_CURRENT_SENSORS) {
            current_read_index = 0;
        }
        
        
        // Set the select pins for the next channel
        digitalWrite(CURRENT_SELECT_PIN_0, current_read_index & 0x01);
        digitalWrite(CURRENT_SELECT_PIN_1, (current_read_index >> 1) & 0x01);
        digitalWrite(CURRENT_SELECT_PIN_2, (current_read_index >> 2) & 0x01);
        last_channel_switch_ms = now;
    }
}