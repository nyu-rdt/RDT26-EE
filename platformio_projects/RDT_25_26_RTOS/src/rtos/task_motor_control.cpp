#include <Arduino.h>
#include <climits>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

void TaskMotorControl(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    bool       stopped  = true;   // start stopped until safety clears

    for (;;) {
        // Check hard-stop notification (non-blocking poll)
        uint32_t notif = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &notif, 0) == pdTRUE) {
            stopped = (notif == 1U);
        }

        // Also enforce via event bits (persistent state)
        EventBits_t safety = xEventGroupGetBits(egSafetyBits);
        if (safety & SAFETY_ANY_STOP) stopped = true;

        // will add actual CAN output here.
        // For now just wait.
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MOTOR_CTRL_MS));
    }
}
