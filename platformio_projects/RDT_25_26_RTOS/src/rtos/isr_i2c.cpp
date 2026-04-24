#include <Arduino.h>
#include <Wire.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "shared_state.h"

// Called by Wire2 on I2C receive. Must be ISR-safe — only *FromISR RTOS calls.
// diag_i2c_rx_overflow: single writer (this ISR only). Tasks that read it for
// display must snapshot inside taskENTER_CRITICAL() / taskEXIT_CRITICAL() to
// avoid a torn 32-bit read on Cortex-M7.
void ISR_I2C_OnReceive(int) {
    BaseType_t woken = pdFALSE;
    while (Wire2.available()) {
        uint8_t b = Wire2.read();
        if (xQueueSendFromISR(qI2cRxBytes, &b, &woken) != pdTRUE) {
            diag_i2c_rx_overflow++;
        }
    }
    portYIELD_FROM_ISR(woken);
}

// Called by Wire2 on I2C request. No RTOS calls — must be minimal and fast.
// diag_i2c_short_packets: single writer (this ISR only). Same read-site rule
// as diag_i2c_rx_overflow — snapshot under taskENTER_CRITICAL() in task context.
void ISR_I2C_OnRequest() {
    uint8_t idx = gTelemetryReady;
    // Cast away volatile: safe because the ready slot is not being written
    // by TelemetryTask while this ISR runs (double-buffer invariant).
    int written = Wire2.write((const uint8_t*)gTelemetryBuf[idx], sizeof(gTelemetryBuf[idx]));
    if (written != (int)sizeof(gTelemetryBuf[idx])) {
        diag_i2c_short_packets++;
    }
}
