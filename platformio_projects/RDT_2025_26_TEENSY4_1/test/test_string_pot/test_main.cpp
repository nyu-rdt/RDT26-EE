// tests/test_string_pot/test_main.cpp
//
// Hardware calibration monitor for the string potentiometer.
// Purpose: verify low-end/high-end calibration and check measurement stability.
//
// Build + upload:  pio run -e teensy41_string_pot_test -t upload
// Monitor:         Serial Monitor or Teleplot @ 115200 baud

#include <Arduino.h>
#include "config.h"
#include "string_pot.h"

static constexpr unsigned long kPrintPeriodMs = 100;
static unsigned long lastPrintMs = 0;

// Running observed extrema for this session.
static float observedMin = 1.0e9f;
static float observedMax = -1.0e9f;

// Driver clamp/calibration endpoints from string_pot.cpp.
static constexpr float expectedMin = 0.0f;
static constexpr float expectedMax = (27.0f * 3.3f) - 0.719f; // 88.381

void setup()
{
    Serial.begin(115200);
    STRINGPOT_Init();

    Serial.println("# string pot test started");
    Serial.println("# move mechanism to LOW stop, then HIGH stop");
    Serial.print("# expected range: ");
    Serial.print(expectedMin, 3);
    Serial.print(" to ");
    Serial.println(expectedMax, 3);
    Serial.println("# Teleplot streams: SP_dist, SP_min, SP_max, SP_raw, SP_volt");
}

void loop()
{
    if (millis() - lastPrintMs < kPrintPeriodMs) {
        return;
    }
    lastPrintMs = millis();

    const int raw = analogRead(STRING_POT_PIN);
    const float voltage = raw * (3.3f / 1023.0f);
    const float distance = STRINGPOT_ReadDistance();

    if (distance < observedMin) {
        observedMin = distance;
    }
    if (distance > observedMax) {
        observedMax = distance;
    }

    // Teleplot-compatible output
    Serial.print(">SP_dist:"); Serial.println(distance, 3);
    Serial.print(">SP_min:"); Serial.println(observedMin, 3);
    Serial.print(">SP_max:"); Serial.println(observedMax, 3);
    Serial.print(">SP_raw:"); Serial.println(raw);
    Serial.print(">SP_volt:"); Serial.println(voltage, 3);

    // Readable status line for terminal monitor
    Serial.print("dist=");
    Serial.print(distance, 3);
    Serial.print("  observed[min,max]=[");
    Serial.print(observedMin, 3);
    Serial.print(", ");
    Serial.print(observedMax, 3);
    Serial.print("]  raw=");
    Serial.print(raw);
    Serial.print("  V=");
    Serial.println(voltage, 3);
}
