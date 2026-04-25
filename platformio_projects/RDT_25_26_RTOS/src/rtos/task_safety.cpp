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

        bool relayEngaged = (digitalRead(PIN_RELAY_READ) == HIGH);
        if (!relayEngaged) bits |= SAFETY_HW_ESTOP;

        // Read DesiredState fields under mutex. Timeout 1ms (not 0) to avoid
        // a false comms-timeout on boot when lastCmd is still 0.
        uint32_t lastCmd         = millis();   // default: no timeout if mutex miss
        bool     swEstopPending  = false;
        if (xSemaphoreTake(mDesiredState, pdMS_TO_TICKS(1)) == pdTRUE) {
            lastCmd        = gDesiredState.last_cmd_ms;
            swEstopPending = gDesiredState.sw_estop_requested;
            if (swEstopPending) gDesiredState.sw_estop_requested = false;  // consume
            xSemaphoreGive(mDesiredState);
        }
        if ((millis() - lastCmd) > 500U) {   // COMMAND_TIMEOUT_MS
            bits |= SAFETY_COMMS_TIMEOUT;
            if (!(prevBits & SAFETY_COMMS_TIMEOUT)) diag_timeout_events++;  // count transitions only
        }
        if (swEstopPending) bits |= SAFETY_SW_ESTOP;

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
