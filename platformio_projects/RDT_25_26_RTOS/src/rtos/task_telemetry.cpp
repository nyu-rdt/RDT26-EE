#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

static_assert(sizeof(gTelemetryBuf[0]) == 18, "packet size mismatch");

void TaskTelemetry(void*) {
#if RTOS_SERIAL_DEBUG
    Serial.println("[telemetry] task started");
#endif
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_TELEMETRY_MS));

        uint8_t  build = 1 - gTelemetryReady;   // write to the non-active buffer
        uint8_t* pkt   = (uint8_t*)gTelemetryBuf[build];  // safe: only task writes this buffer

        // [0-7] current channels
        for (int i = 0; i < 8; i++) {
            float scaled = gSensorSnapshot.motor_currents[i] * 12.75f;
            pkt[i] = (uint8_t)constrain(scaled, 0.0f, 255.0f);
        }

        // [8-9] encoders
#if SENSOR_ENCODERS_ENABLED
        pkt[8] = (uint8_t)constrain(gSensorSnapshot.encoder_left_deg  * (255.0f / 360.0f), 0.0f, 255.0f);
        pkt[9] = (uint8_t)constrain(gSensorSnapshot.encoder_right_deg * (255.0f / 360.0f), 0.0f, 255.0f);
#else
        pkt[8] = 0xFF;
        pkt[9] = 0xFF;
#endif

        // [10-13] load cells — 0.1 kg/LSB, clamped 0-255
#if SENSOR_LOAD_CELLS_ENABLED
        for (int i = 0; i < 4; i++) {
            float scaled = gSensorSnapshot.load_cells_kg[i] * 10.0f;
            pkt[10 + i] = (uint8_t)constrain(scaled, 0.0f, 255.0f);
        }
#else
        pkt[10] = pkt[11] = pkt[12] = pkt[13] = 0xFF;
#endif

        // [14] string pot
        pkt[14] = (uint8_t)constrain(gSensorSnapshot.string_pot_cm, 0.0f, 255.0f);

        // [15] door state
        pkt[15] = gSensorSnapshot.depo_door_state;

        // [16] flags
        EventBits_t safety = xEventGroupGetBits(egSafetyBits);
        uint8_t flags = 0;
        if (!gSensorSnapshot.relay_engaged)         flags |= 0x01;   // FLAG_ESTOP
        if (safety & SAFETY_OVERCURRENT)            flags |= 0x02;   // FLAG_OVERCURRENT
        pkt[16] = flags;

        // [17] sentinel
        pkt[17] = 0xFF;

        // M7 store buffer: ensure all 18 bytes are visible to the ISR before
        // the index flip is. Without this barrier the Cortex-M7 can retire the
        // gTelemetryReady store before earlier pkt[] stores drain to memory.
        __asm__ volatile("dmb" ::: "memory");
        gTelemetryReady = build;   // atomic uint8 swap — ISR sees new buffer next request
    }
}
