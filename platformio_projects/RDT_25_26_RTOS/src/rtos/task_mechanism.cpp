#include <Arduino.h>
#include <climits>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

void TaskMechanism(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    bool       stopped  = true;

    for (;;) {
        uint32_t notif = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &notif, 0) == pdTRUE) {
            stopped = (notif == 1U);
        }
        EventBits_t safety = xEventGroupGetBits(egSafetyBits);
        if (safety & SAFETY_ANY_STOP) stopped = true;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MECHANISM_MS));
    }
}
