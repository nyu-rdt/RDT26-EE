#include <Arduino.h>
#include "config.h"
#include "ee_box.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif

static volatile bool relay_state   = false;
static volatile bool relay_3s_low  = false;
static volatile bool relay_6s_low  = false;

void EE_BOX_Init() {
    pinMode(RELAY_DRIVER_PIN, OUTPUT);
    pinMode(RELAY_READ_PIN, INPUT_PULLDOWN);
    pinMode(RELAY_3S_LOW_PIN, INPUT_PULLDOWN);
    pinMode(RELAY_6S_LOW_PIN, INPUT_PULLDOWN);
    EE_BOX_EnableRelay();
}

void EE_BOX_Update() {
    relay_state  = digitalRead(RELAY_READ_PIN);
    relay_3s_low = digitalRead(RELAY_3S_LOW_PIN);
    relay_6s_low = digitalRead(RELAY_6S_LOW_PIN);
}

void EE_BOX_EnableRelay()  { digitalWrite(RELAY_DRIVER_PIN, HIGH); }
void EE_BOX_DisableRelay() { digitalWrite(RELAY_DRIVER_PIN, LOW); }

uint8_t EE_BOX_GetRelayStatus() {
    return (uint8_t)(((uint8_t)relay_6s_low << 2) | ((uint8_t)relay_3s_low << 1) | (uint8_t)relay_state);
}

bool EE_BOX_IsRelayEngaged() {
    return relay_state;
}

bool EE_BOX_IsOvercurrent() {
#if CURRENT_SENSE_ENABLED
    if (CURRENT_OVERCURRENT_ANY_A == 0.0f) return false;
    const float* c = CURRENT_SENSORS_GetBuffer();
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
        if (c[i] > CURRENT_OVERCURRENT_ANY_A) return true;
    }
#endif
    return false;
}
