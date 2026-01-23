/**
 * @file sensors.cpp
 * @brief Sensor implementation (HX711 load cells, string potentiometer)
 */

#include "sensors.h"
#include "config.h"
#include "HX711.h"

// HX711 load cell instances
static HX711 scale1, scale2, scale3, scale4;

// Cached sensor values
static float cachedLength = 0.0f;
static float cachedWeight = 0.0f;

void Sensors_Init(void) {
    // Initialize HX711 load cells
    scale1.begin(HX711_DOUT1, HX711_CLK1);
    scale2.begin(HX711_DOUT2, HX711_CLK2);
    scale3.begin(HX711_DOUT3, HX711_CLK3);
    scale4.begin(HX711_DOUT4, HX711_CLK4);
    
    // Set calibration factors
    scale1.set_scale(HX711_CAL_FACTOR_1);
    scale2.set_scale(HX711_CAL_FACTOR_2);
    scale3.set_scale(HX711_CAL_FACTOR_3);
    scale4.set_scale(HX711_CAL_FACTOR_4);
    
    // Tare all scales
    scale1.tare();
    scale2.tare();
    scale3.tare();
    scale4.tare();
    
    Serial.println("Sensors Initialized");
}

float Sensors_GetStringLength(void) {
    int rawValue = analogRead(STRING_POT_PIN);
    float voltage = rawValue * (5.0f / 1023.0f);
    cachedLength = STRING_POT_SCALE * voltage - STRING_POT_OFFSET;
    return cachedLength;
}

float Sensors_GetWeight(void) {
    // Read from all 4 load cells (10 samples each for stability)
    float w1 = scale1.get_units(10);
    float w2 = scale2.get_units(10);
    float w3 = scale3.get_units(10);
    float w4 = scale4.get_units(10);
    
    cachedWeight = (w1 + w2 + w3 + w4) / 4.0f;
    return cachedWeight;
}

void Sensors_Update(void) {
    Sensors_GetStringLength();
    // Note: Weight reading is slow (10 samples each), 
    // only call when needed
}

float Sensors_GetCachedLength(void) {
    return cachedLength;
}

float Sensors_GetCachedWeight(void) {
    return cachedWeight;
}
