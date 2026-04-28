#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"
#include "rtos_config.h"

float ENCODER_GetAngle(uint8_t enc);

static void selectMuxChannel(uint8_t ch) {
    digitalWrite(PIN_CURRENT_SEL0, (ch >> 0) & 1);
    digitalWrite(PIN_CURRENT_SEL1, (ch >> 1) & 1);
    digitalWrite(PIN_CURRENT_SEL2, (ch >> 2) & 1);
}

void TaskSensor(void*) {
#if RTOS_SERIAL_DEBUG
    Serial.println("[sensor] task started");
#endif
    pinMode(PIN_CURRENT_INPUT,  INPUT);
    pinMode(PIN_CURRENT_SEL0,   OUTPUT);
    pinMode(PIN_CURRENT_SEL1,   OUTPUT);
    pinMode(PIN_CURRENT_SEL2,   OUTPUT);
    pinMode(PIN_STRING_POT,     INPUT);

    // Pre-select channel 0 so the first vTaskDelayUntil provides full settle time
    selectMuxChannel(0);

    TickType_t lastWake = xTaskGetTickCount();
    uint8_t    muxCh   = 0;

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));

        // Read channel selected on the previous tick (settle time = task period)
#if SENSOR_CURRENT_ENABLED
        float raw = (float)analogRead(PIN_CURRENT_INPUT);
        gSensorSnapshot.motor_currents[muxCh] = raw * (33.0f / 1023.0f);
#endif

        // Advance mux to next channel; it will be read next tick after settling
        muxCh = (muxCh + 1) % 8;
        selectMuxChannel(muxCh);

#if SENSOR_STRING_POT_ENABLED
        float rawPot = (float)analogRead(PIN_STRING_POT);
        gSensorSnapshot.string_pot_cm = (rawPot * 37.125f / 1023.0f * 3.3f) + 3.511f;
#endif

        gSensorSnapshot.relay_engaged = (digitalRead(PIN_RELAY_READ) == HIGH);

#if SENSOR_ENCODERS_ENABLED
        gSensorSnapshot.encoder_left_deg  = ENCODER_GetAngle(1);
        gSensorSnapshot.encoder_right_deg = ENCODER_GetAngle(2);
#endif
    }
}
