#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"
#include "rtos_config.h"

void TaskDebug(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_DEBUG_MS));
    }
}
