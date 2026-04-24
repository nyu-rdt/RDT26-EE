#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

void TaskSafety(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_SAFETY_MS));
    }
}
