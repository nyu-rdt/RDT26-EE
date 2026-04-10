// idk how many encoders we have so im just gonna make 2 bc child.cpp says left and right encoder in requestData packet
#include <Arduino.h>
#include "config.h"
#include "rotary_encoders.h"

// define all encoder A and B pins 
// random encoder pins 

#define ENC1_A 21
#define ENC1_B 20
#define ENC2_A 23
#define ENC2_B 22

volatile long count1 = 0;
volatile long count2 = 0; 
#define angle1 = 0.0;
#define angle2 = 0.0;   

volatile uint8_t activeEncoder = 2; 

void ROTARY_ENCODER_Init() {
    // set encoder pins as inputs with pull-up resistors
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    // attach interrupts for encoder A and B pins
    attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), isr1B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), isr2B, CHANGE);                
}

void isr1A() { count1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : +1; } // Swapped +/- signs
void isr1B() { count1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? -1 : +1; } // Swapped +/- signs

void isr2A() { count2 += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : +1; } // Swapped +/- signs
void isr2B() { count2 += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? -1 : +1; } // Swapped +/- signs


void setActiveEncoder(uint8_t encoderNum) {
  if (encoderNum >= 1 && encoderNum <= 2) {
    activeEncoder = encoderNum;
  }
}

uint8_t getActiveEncoder(void) {
    return activeEncoder;
}

float getEncoderAngle(uint8_t encoderNum) {
    long count = getEncoderCount(encoderNum);
    return countsToAngle(count);
}

long getEncoderCount(uint8_t encoderNum) {
    noInterrupts();
    long count;
    switch (encoderNum) {
        case 1: count = count1; break;
        case 2: count = count2; break;
        default: count = 0; break;
    }
    interrupts();
    return count;
}

float countsToAngle(long counts) {
    float angle = fmod(counts * DEGREES_PER_COUNT, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle;
}
