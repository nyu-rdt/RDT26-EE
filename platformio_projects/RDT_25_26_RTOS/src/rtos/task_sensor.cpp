#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"
#include "rtos_config.h"

void TaskSensor(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}
