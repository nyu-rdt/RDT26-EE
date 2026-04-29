// test/test_load_cells/test_load_cell.cpp
//
// Four-channel HX711 load-cell hardware monitor.
// Based on the RDT_2024_PF sensors.cpp HX711 setup.
//
// Build + upload:  pio run -e teensy41_load_cell_test -t upload
// Monitor:         Serial Monitor or Teleplot @ 115200 baud

#include <Arduino.h>
#include "HX711.h"

// Self-contained test calibration values.
static constexpr uint8_t HX711_DOUT_PINS[4] = {31, 33, 36, 37};
static constexpr uint8_t HX711_CLK_PINS[4] = {32, 34, 35, 38};
// static constexpr long HX711_OFFSETS[4] = {155330L, 60071L, 191491L, 193401L}; //offset with depo bin off
static constexpr float HX711_CAL_FACTORS[4] = {102.0f, 105.0f, 102.0f, 105.0f}; //inspired by last year
// static constexpr float HX711_CAL_FACTORS[4] = {72.0f, 80.4f, 75.8f, 72.6f}; //arduino calibrated
static constexpr uint8_t kLoadCellCount = 4;

// Keep sample count aligned with sensors.cpp for comparable behavior.
static constexpr uint8_t kSamplesPerRead = 2;
static constexpr unsigned long kPrintPeriodMs = 100;

static HX711 scales[kLoadCellCount];
static unsigned long lastPrintMs = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Teensy USB serial warm-up (non-blocking timeout).
    }

    Serial.println("# init: all load cells tare in progress...");
    for (uint8_t i = 0; i < kLoadCellCount; ++i) {
        scales[i].begin(HX711_DOUT_PINS[i], HX711_CLK_PINS[i]);
        scales[i].set_scale(HX711_CAL_FACTORS[i]);
        scales[i].tare(20);

        Serial.print("# load cell ");
        Serial.print(i + 1);
        Serial.println(" tare complete");
    }

    Serial.println("# Teleplot streams: LC1_weight, LC2_weight, LC3_weight, LC4_weight, LC_total_weight, LC_avg_weight");
}

void loop()
{
    if (millis() - lastPrintMs < kPrintPeriodMs) {
        return;
    }
    lastPrintMs = millis();

    float weights[kLoadCellCount];
    for (uint8_t i = 0; i < kLoadCellCount; ++i) {
        const long rawAverage = scales[i].read_average(kSamplesPerRead);
        const long offset = scales[i].get_offset();
        const float scaleFactor = scales[i].get_scale();
        const long netCounts = rawAverage - offset;
        const float weight = static_cast<float>(netCounts) / scaleFactor;
        weights[i] = weight;
        // const float filteredWeight = (weight < 0.0f) ? 0.0f : weight;
        // weights[i] = filteredWeight;

        // Teleplot-compatible lines (one time-series per load cell).
        Serial.print(">LC");
        Serial.print(i + 1);
        Serial.print("_weight:");
        Serial.println(weight, 3);
        // Serial.println(filteredWeight, 3);
    }

    const float totalWeight = weights[0] + weights[1] + weights[2] + weights[3];
    const float averageWeight = totalWeight / static_cast<float>(kLoadCellCount);

    // Teleplot-compatible total and average line.
    Serial.print(">LC_total_weight:");
    Serial.println(totalWeight, 3);
    Serial.print(">LC_avg_weight:");
    Serial.println(averageWeight, 3);

    // Human-readable combined diagnostic line.
    Serial.print("weights kg-ish: LC1=");
    Serial.print(weights[0], 3);
    Serial.print(" LC2=");
    Serial.print(weights[1], 3);
    Serial.print(" LC3=");
    Serial.print(weights[2], 3);
    Serial.print(" LC4=");
    Serial.println(weights[3], 3);
    Serial.print(" LCs total=");
    Serial.println(totalWeight, 3);
    Serial.print(" LCs average=");
    Serial.println(averageWeight, 3);
}
