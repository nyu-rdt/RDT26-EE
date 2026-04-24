#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "shared_state.h"
#include "rtos_config.h"

void TaskCmdDecode(void*) {
    uint8_t raw;
    for (;;) {
        xQueueReceive(qI2cRxBytes, &raw, portMAX_DELAY);
    }
}
