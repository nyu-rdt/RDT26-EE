#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

void TaskDebug(void*) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_DEBUG_MS));

        EventBits_t safety = xEventGroupGetBits(egSafetyBits);

        Serial.printf(">safety_bits:%lu\n", (unsigned long)safety);
        Serial.printf(">string_pot:%.2f\n", gSensorSnapshot.string_pot_cm);
        Serial.printf(">relay:%d\n",        gSensorSnapshot.relay_engaged ? 1 : 0);
        Serial.printf(">i2c_overflow:%lu\n",(unsigned long)diag_i2c_rx_overflow);
        Serial.printf(">cmd_rejected:%lu\n",(unsigned long)diag_cmd_rejected_killswitch);
        Serial.printf(">cmd_invalid:%lu\n", (unsigned long)diag_cmd_invalid);
        Serial.printf(">timeouts:%lu\n",    (unsigned long)diag_timeout_events);

#if RTOS_DEBUG_INSTRUMENTATION
        Serial.printf(">heap_free:%lu\n",   (unsigned long)xPortGetFreeHeapSize());
#endif
    }
}
