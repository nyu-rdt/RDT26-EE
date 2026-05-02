#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "shared_state.h"
#include "rtos_config.h"
#if SENSOR_LOAD_CELLS_ENABLED
#include "HX711.h"
static HX711   lcScales[LC_NUM_CELLS];
static uint8_t lcCell = 0;
#endif

float ENCODER_GetAngle(uint8_t enc);

static void selectMuxChannel(uint8_t ch) {
    digitalWrite(PIN_CURRENT_SEL0, (ch >> 0) & 1);
    digitalWrite(PIN_CURRENT_SEL1, (ch >> 1) & 1);
    digitalWrite(PIN_CURRENT_SEL2, (ch >> 2) & 1);
}

void TaskSensor(void*) {
#if RTOS_SERIAL_DEBUG
    Serial.println("[sensor] task started");
#endif
    analogReadResolution(ADC_RESOLUTION_BITS);
    pinMode(PIN_CURRENT_INPUT,  INPUT);
    pinMode(PIN_CURRENT_SEL0,   OUTPUT);
    pinMode(PIN_CURRENT_SEL1,   OUTPUT);
    pinMode(PIN_CURRENT_SEL2,   OUTPUT);
    pinMode(PIN_STRING_POT,     INPUT);
#if SENSOR_LOAD_CELLS_ENABLED
    {
        static const uint8_t dout[LC_NUM_CELLS] = { LC_DOUT1, LC_DOUT2, LC_DOUT3, LC_DOUT4 };
        static const uint8_t clk [LC_NUM_CELLS] = { LC_CLK1,  LC_CLK2,  LC_CLK3,  LC_CLK4  };
        static const long    off [LC_NUM_CELLS] = { LC_OFFSET1, LC_OFFSET2, LC_OFFSET3, LC_OFFSET4 };
        static const float   cal [LC_NUM_CELLS] = { LC_CAL1, LC_CAL2, LC_CAL3, LC_CAL4 };
        for (uint8_t i = 0; i < LC_NUM_CELLS; i++) {
            lcScales[i].begin(dout[i], clk[i]);
            lcScales[i].tare();
            lcScales[i].set_scale(cal[i]);
        }
    }
#endif

    // Pre-select channel 0 so the first vTaskDelayUntil provides full settle time
    selectMuxChannel(0);

    TickType_t lastWake = xTaskGetTickCount();
    uint8_t    muxCh   = 0;

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));

        // Read channel selected on the previous tick (settle time = task period)
#if SENSOR_CURRENT_ENABLED
        float raw = (float)analogRead(PIN_CURRENT_INPUT);
        gSensorSnapshot.motor_currents[muxCh] = raw * (33.0f / (float)ADC_MAX_COUNT);
#endif

        // Advance mux to next channel; it will be read next tick after settling
        muxCh = (muxCh + 1) % 8;
        selectMuxChannel(muxCh);

#if SENSOR_STRING_POT_ENABLED
        float rawPot  = (float)analogRead(PIN_STRING_POT);
        float measured = (rawPot * 37.125f / (float)ADC_MAX_COUNT * 3.3f) + 3.511f;
        // EMA — seed on first sample so the latch in TaskMechanism doesn't
        // trip on a zero-initialized snapshot before the first real read.
        static float strpotEma = -1.0f;
        if (strpotEma < 0.0f) strpotEma = measured;
        else strpotEma = STRPOT_EMA_ALPHA * measured + (1.0f - STRPOT_EMA_ALPHA) * strpotEma;
        gSensorSnapshot.string_pot_cm = strpotEma;
#endif

#if SENSOR_LOAD_CELLS_ENABLED
        if (lcScales[lcCell].is_ready())
            gSensorSnapshot.load_cells_kg[lcCell] = lcScales[lcCell].get_units(1);
        lcCell = (lcCell + 1) % LC_NUM_CELLS;
#endif

        gSensorSnapshot.relay_engaged = (digitalRead(PIN_RELAY_READ) == HIGH);

#if SENSOR_ENCODERS_ENABLED
        gSensorSnapshot.encoder_left_deg  = ENCODER_GetAngle(1);
        gSensorSnapshot.encoder_right_deg = ENCODER_GetAngle(2);
#endif
    }
}
