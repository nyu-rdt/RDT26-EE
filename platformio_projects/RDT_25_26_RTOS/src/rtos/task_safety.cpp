#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

void TaskSafety(void*) {
    EventBits_t prevBits = 0;
    TickType_t  lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_SAFETY_MS));

        EventBits_t bits = 0;

        // HW e-stop: relay read pin is 3
        bool relayEngaged = (digitalRead(3) == HIGH);
        if (!relayEngaged) bits |= SAFETY_HW_ESTOP;

        // Comms timeout: read DesiredState.last_cmd_ms under mutex
        uint32_t lastCmd = 0;
        if (xSemaphoreTake(mDesiredState, 0) == pdTRUE) {
            lastCmd = gDesiredState.last_cmd_ms;
            xSemaphoreGive(mDesiredState);
        }
        if ((millis() - lastCmd) > 500U) {   // COMMAND_TIMEOUT_MS
            bits |= SAFETY_COMMS_TIMEOUT;
        }

        // Write the bits — SafetyTask is the sole writer
        xEventGroupClearBits(egSafetyBits, SAFETY_ANY_STOP);
        xEventGroupSetBits(egSafetyBits, bits);

        // Notify actuator tasks on any new stop assertion
        bool newStop = (bits & SAFETY_ANY_STOP) && !(prevBits & SAFETY_ANY_STOP);
        if (newStop) {
            diag_safety_transitions++;
            xTaskNotify(hMotorCtrlTask, 1U, eSetValueWithOverwrite);
            xTaskNotify(hMechanismTask, 1U, eSetValueWithOverwrite);
        } else if (!bits && prevBits) {
            // All stop bits cleared — notify release
            xTaskNotify(hMotorCtrlTask, 0U, eSetValueWithOverwrite);
            xTaskNotify(hMechanismTask, 0U, eSetValueWithOverwrite);
        }
        prevBits = bits;
    }
}
