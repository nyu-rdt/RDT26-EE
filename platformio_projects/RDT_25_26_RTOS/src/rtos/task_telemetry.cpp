#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

static_assert(sizeof(gTelemetryBuf[0]) == 18, "packet size mismatch");

void TaskTelemetry(void*) {
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

        // [8-9] encoders — not yet wired in RTOS; send sentinel
        pkt[8] = 0xFF;
        pkt[9] = 0xFF;

        // [10-13] load cells — not wired; sentinel
        pkt[10] = pkt[11] = pkt[12] = pkt[13] = 0xFF;

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

        // Verify byte count before making buffer live
        bool ok = true;
        for (int i = 0; i < 18; i++) {
            // all fields set — no uninitialized bytes
            (void)pkt[i];
        }
        if (!ok) {
            diag_telemetry_serialize_errors++;
        } else {
            gTelemetryReady = build;   // atomic uint8 swap — ISR sees new buffer next request
        }
    }
}
