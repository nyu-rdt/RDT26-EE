#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"
#include "rtos_config.h"

// Current mux: pins match superloop pins.h
#define CS_INPUT   26
#define CS_SEL0    27
#define CS_SEL1    28
#define CS_SEL2    29
#define STRING_POT_PIN_R 39
#define ENC1_A_R   21
#define ENC1_B_R   20
#define ENC2_A_R   19
#define ENC2_B_R   18

static float readCurrentChannel(uint8_t ch) {
    digitalWrite(CS_SEL0, (ch >> 0) & 1);
    digitalWrite(CS_SEL1, (ch >> 1) & 1);
    digitalWrite(CS_SEL2, (ch >> 2) & 1);
    vTaskDelay(pdMS_TO_TICKS(10));   // settle time
    float raw = (float)analogRead(CS_INPUT);
    return raw * (33.0f / 1023.0f);  // CURRENT_SCALING
}

void TaskSensor(void*) {
    // Setup mux pins
    pinMode(CS_INPUT,  INPUT);
    pinMode(CS_SEL0,   OUTPUT);
    pinMode(CS_SEL1,   OUTPUT);
    pinMode(CS_SEL2,   OUTPUT);
    pinMode(STRING_POT_PIN_R, INPUT);

    TickType_t lastWake = xTaskGetTickCount();
    uint8_t    muxCh   = 0;

    for (;;) {
        // Step one mux channel per tick (10ms per channel × 8 = 80ms full scan)
        gSensorSnapshot.motor_currents[muxCh] = readCurrentChannel(muxCh);
        muxCh = (muxCh + 1) % 8;

        // String pot (arm position)
        float raw = (float)analogRead(STRING_POT_PIN_R);
        gSensorSnapshot.string_pot_cm = (raw * 37.125f / 1023.0f * 3.3f) + 3.511f;

        // Relay state for telemetry flags
        gSensorSnapshot.relay_engaged = (digitalRead(3) == HIGH);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}
