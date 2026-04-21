// test/test_load_cell.cpp
//
// Single HX711 load-cell hardware monitor.
// Based on the RDT_2024_PF sensors.cpp HX711 setup.
//
// Build + upload:  pio run -e teensy41_load_cell_test -t upload
// Monitor:         Serial Monitor or Teleplot @ 115200 baud

#include <Arduino.h>
#include "HX711.h"

// Select which load cell channel to test (1..4)
static constexpr uint8_t kLoadCellIndex = 3;

// Pin map and calibration factors from RDT_2024_PF/include/config.h.
static constexpr uint8_t HX711_DOUT_PINS[4] = {24, 26, 34, 20};
static constexpr uint8_t HX711_CLK_PINS[4] = {25, 27, 33, 21};
static constexpr float HX711_CAL_FACTORS[4] = {-102.0f, 105.0f, -102.0f, 111.0f};

// Keep sample count aligned with sensors.cpp for comparable behavior.
static constexpr uint8_t kSamplesPerRead = 2;
static constexpr unsigned long kPrintPeriodMs = 100;

static HX711 scale;
static unsigned long lastPrintMs = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Teensy USB serial warm-up (non-blocking timeout).
    }

    if (kLoadCellIndex < 1 || kLoadCellIndex > 4) {
        Serial.println("# ERROR: kLoadCellIndex must be 1..4");
        return;
    }

    const uint8_t idx = static_cast<uint8_t>(kLoadCellIndex - 1);
    scale.begin(HX711_DOUT_PINS[idx], HX711_CLK_PINS[idx]);
    scale.set_scale(HX711_CAL_FACTORS[idx]);

    Serial.print("# load cell ");
    Serial.print(kLoadCellIndex);
    Serial.println(" init: tare in progress...");
    scale.tare();

    Serial.println("# load cell tare complete");
    Serial.println("# Teleplot stream: LC_weight");
}

void loop()
{
    if (kLoadCellIndex < 1 || kLoadCellIndex > 4) {
        delay(500);
        return;
    }

    if (millis() - lastPrintMs < kPrintPeriodMs) {
        return;
    }
    lastPrintMs = millis();

    const float weight = scale.get_units(kSamplesPerRead);

    // Teleplot-compatible line.
    Serial.print(">LC_weight:");
    Serial.println(weight, 3);

    // Human-readable terminal line.
    Serial.print("load_cell_");
    Serial.print(kLoadCellIndex);
    Serial.print(" weight=");
    Serial.println(weight, 3);
}
