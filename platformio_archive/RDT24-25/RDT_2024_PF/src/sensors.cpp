/**
 * @file sensors.cpp
 * @brief Sensor implementation (HX711 load cells, string potentiometer, encoders)
 */

#include "sensors.h"
#include "config.h"
#include "HX711.h"

// HX711 load cell instances
static HX711 scale1, scale2, scale3, scale4;

// Cached sensor values
static float cachedLength = 0.0f;
static float cachedWeight = 0.0f;

// Encoder counts - volatile because they're modified in interrupts
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount3 = 0;
volatile long encoderCount4 = 0;

// Currently active encoder (1-4) for sending to Pi
static uint8_t activeEncoder = 2;

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
    
    // Configure encoder pins as inputs with pull-ups
    pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
    pinMode(ENC3_A, INPUT_PULLUP); pinMode(ENC3_B, INPUT_PULLUP);
    pinMode(ENC4_A, INPUT_PULLUP); pinMode(ENC4_B, INPUT_PULLUP);
    
    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(ENC1_A), Sensors_ISR_Enc1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), Sensors_ISR_Enc1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), Sensors_ISR_Enc2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), Sensors_ISR_Enc2B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC3_A), Sensors_ISR_Enc3A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC3_B), Sensors_ISR_Enc3B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC4_A), Sensors_ISR_Enc4A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC4_B), Sensors_ISR_Enc4B, CHANGE);
    
    Serial.println("Sensors Initialized (including encoders)");
}

float Sensors_GetStringLength(void) {
    float rawValue = analogRead(STRING_POT_PIN);
    float voltage = rawValue * (5.0f / 1023.0f);
    cachedLength = STRING_POT_SCALE * voltage - STRING_POT_OFFSET;
    return cachedLength;
}

float Sensors_GetWeight(void) {
    // Read from scales 3 and 4 only (2 samples for speed)
    cachedWeight = scale3.get_units(2) + scale4.get_units(2);
    return cachedWeight;
}

void Sensors_Update(void) {
    Sensors_GetStringLength();
    // Note: Weight reading is slow, only call when needed
}

float Sensors_GetCachedLength(void) {
    return cachedLength;
}

float Sensors_GetCachedWeight(void) {
    return cachedWeight;
}

float Sensors_CountsToAngle(long counts) {
    float angle = fmod(counts * DEGREES_PER_COUNT, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle;
}

void Sensors_SetActiveEncoder(uint8_t encoderNum) {
    if (encoderNum >= 1 && encoderNum <= 4) {
        activeEncoder = encoderNum;
    }
}

uint8_t Sensors_GetActiveEncoder(void) {
    return activeEncoder;
}

float Sensors_GetEncoderAngle(uint8_t encoderNum) {
    long count = Sensors_GetEncoderCount(encoderNum);
    return Sensors_CountsToAngle(count);
}

long Sensors_GetEncoderCount(uint8_t encoderNum) {
    noInterrupts();
    long count;
    switch (encoderNum) {
        case 1: count = encoderCount1; break;
        case 2: count = encoderCount2; break;
        case 3: count = encoderCount3; break;
        case 4: count = encoderCount4; break;
        default: count = 0; break;
    }
    interrupts();
    return count;
}

// ============== Encoder ISR Implementations ==============
// Encoders 1, 2, and 4 have inverted direction

void Sensors_ISR_Enc1A(void) { 
    encoderCount1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : +1; 
}
void Sensors_ISR_Enc1B(void) { 
    encoderCount1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? -1 : +1; 
}

void Sensors_ISR_Enc2A(void) { 
    encoderCount2 += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : +1; 
}
void Sensors_ISR_Enc2B(void) { 
    encoderCount2 += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? -1 : +1; 
}

void Sensors_ISR_Enc3A(void) { 
    encoderCount3 += (digitalRead(ENC3_A) == digitalRead(ENC3_B)) ? +1 : -1; 
}
void Sensors_ISR_Enc3B(void) { 
    encoderCount3 += (digitalRead(ENC3_A) != digitalRead(ENC3_B)) ? +1 : -1; 
}

void Sensors_ISR_Enc4A(void) { 
    encoderCount4 += (digitalRead(ENC4_A) == digitalRead(ENC4_B)) ? -1 : +1; 
}
void Sensors_ISR_Enc4B(void) { 
    encoderCount4 += (digitalRead(ENC4_A) != digitalRead(ENC4_B)) ? -1 : +1; 
}
