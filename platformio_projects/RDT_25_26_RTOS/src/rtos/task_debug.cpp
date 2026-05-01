#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

void TaskDebug(void*) {
    // LED on pin 13: blinks every PERIOD_DEBUG_MS regardless of Serial.
    // If you see the LED blinking but no serial output, Serial is broken
    // post-scheduler (flush hang or USB issue). If LED is dark, task never ran.
    pinMode(13, OUTPUT);

#if RTOS_SERIAL_DEBUG
    Serial.println("[debug] task started");
#endif

    TickType_t lastWake = xTaskGetTickCount();
    bool ledState = false;

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_DEBUG_MS));

        ledState = !ledState;
        digitalWriteFast(13, ledState);

#if RTOS_DEBUG_INSTRUMENTATION
        EventBits_t safety = xEventGroupGetBits(egSafetyBits);

        Serial.printf(">safety_bits:%lu\n", (unsigned long)safety);
        Serial.printf(">string_pot:%.2f\n", gSensorSnapshot.string_pot_cm);
        Serial.printf(">relay:%d\n",        gSensorSnapshot.relay_engaged ? 1 : 0);

        for (int i = 0; i < 8; i++) {
            Serial.printf(">current_%d:%.2f\n", i, gSensorSnapshot.motor_currents[i]);
        }

        Serial.printf(">enc_left:%.1f\n",  gSensorSnapshot.encoder_left_deg);
        Serial.printf(">enc_right:%.1f\n", gSensorSnapshot.encoder_right_deg);

        for (int i = 0; i < 4; i++) {
            Serial.printf(">load_cell_%d:%.3f\n", i, gSensorSnapshot.load_cells_kg[i]);
        }

        Serial.printf(">i2c_overflow:%lu\n",(unsigned long)diag_i2c_rx_overflow);
        Serial.printf(">cmd_rejected:%lu\n",(unsigned long)diag_cmd_rejected_killswitch);
        Serial.printf(">cmd_invalid:%lu\n", (unsigned long)diag_cmd_invalid);
        Serial.printf(">timeouts:%lu\n",    (unsigned long)diag_timeout_events);
        Serial.printf(">heap_free:%lu\n",   (unsigned long)xPortGetFreeHeapSize());
#endif

#if RTOS_SERIAL_DEBUG && TICK_IN_DEBUG
        Serial.println("[debug] tick");
        // No flush — let the USB interrupt drain the buffer naturally.
#endif
    }
}
