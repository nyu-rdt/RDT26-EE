#include <Arduino.h>
#include "config.h"
#include "current_sensors.h"

static float buffer[NUM_CURRENT_SENSORS] = {0};
static int current_read_index = 0;
static unsigned long last_channel_switch_ms = 0;

void CURRENT_SENSORS_Init() {
    analogReadResolution(10);
    analogReadAveraging(16);
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

void CURRENT_SENSORS_Update() {
    unsigned long now = millis();
    if (now - last_channel_switch_ms >= CHANNEL_SETTLE_MS) {
        int32_t acc = 0;
        for (int i = 0; i < CURRENT_ADC_OVERSAMPLE; i++) acc += analogRead(CURRENT_INPUT_PIN);
        float raw = (static_cast<float>(acc) / CURRENT_ADC_OVERSAMPLE) * CURRENT_SCALING;
        buffer[current_read_index] = CURRENT_EWA_ALPHA * raw + (1.0f - CURRENT_EWA_ALPHA) * buffer[current_read_index];
        if (++current_read_index == NUM_CURRENT_SENSORS) current_read_index = 0;
        digitalWrite(CURRENT_SELECT_PIN_0, current_read_index & 0x01);
        digitalWrite(CURRENT_SELECT_PIN_1, (current_read_index >> 1) & 0x01);
        digitalWrite(CURRENT_SELECT_PIN_2, (current_read_index >> 2) & 0x01);
        last_channel_switch_ms = now;
    }
}

const float* CURRENT_SENSORS_GetBuffer() {
    return buffer;
}

