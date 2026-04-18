#include <Arduino.h>
#include "config.h"

#if PLOT_DATA
#include "debug.h"
#include "excavation.h"
#include "deposition.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif
#if ROTARY_ENCODERS_ENABLED
#include "rotary_encoders.h"
#endif

static unsigned long lastPlotMs = 0;

void DEBUG_Update() {
    if (millis() - lastPlotMs < PLOT_PERIOD_MS) return;
    lastPlotMs = millis();

#if CURRENT_SENSE_ENABLED
    const float* currents = CURRENT_SENSORS_GetBuffer();
    Serial.print(">I0:"); Serial.println(currents[0], 2);
    Serial.print(">I1:"); Serial.println(currents[1], 2);
    Serial.print(">I2:"); Serial.println(currents[2], 2);
    Serial.print(">I3:"); Serial.println(currents[3], 2);
    Serial.print(">I4:"); Serial.println(currents[4], 2);
    Serial.print(">I5:"); Serial.println(currents[5], 2);
    Serial.print(">I6:"); Serial.println(currents[6], 2);
    Serial.print(">I7:"); Serial.println(currents[7], 2);
#endif
#if ROTARY_ENCODERS_ENABLED
    Serial.print(">Enc1:"); Serial.println(ROTARY_ENCODER_getEncoderAngle(1), 1);
    Serial.print(">Enc2:"); Serial.println(ROTARY_ENCODER_getEncoderAngle(2), 1);
#endif
#if STRING_POT_ENABLED
    Serial.print(">StrPot:"); Serial.println(EXCAV_GetConveyorDistance(), 2);
#endif
#if GATE_POS_ENABLED
    Serial.print(">DD_state:"); Serial.println(static_cast<int>(DEPO_GetDoorState()));
#endif
    Serial.println();
}

#endif // PLOT_DATA
