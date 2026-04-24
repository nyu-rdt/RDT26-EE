#include <Arduino.h>
#include <Wire.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "shared_state.h"

// Called by Wire2 on I2C receive. Must be ISR-safe — only *FromISR RTOS calls.
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
void ISR_I2C_OnRequest() {
    uint8_t idx = gTelemetryReady;
    int written = Wire2.write(gTelemetryBuf[idx], sizeof(gTelemetryBuf[idx]));
    if (written != (int)sizeof(gTelemetryBuf[idx])) {
        diag_i2c_short_packets++;
    }
}
