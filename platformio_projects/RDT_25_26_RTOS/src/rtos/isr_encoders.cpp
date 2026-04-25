#include <Arduino.h>
#include <cmath>
#include "rtos_config.h"

static volatile long s_count1 = 0;
static volatile long s_count2 = 0;

static void isr1A() { s_count1 += (digitalRead(PIN_ENC1_A) == digitalRead(PIN_ENC1_B)) ? -1 : +1; }
static void isr1B() { s_count1 += (digitalRead(PIN_ENC1_A) != digitalRead(PIN_ENC1_B)) ? -1 : +1; }
static void isr2A() { s_count2 += (digitalRead(PIN_ENC2_A) == digitalRead(PIN_ENC2_B)) ? -1 : +1; }
static void isr2B() { s_count2 += (digitalRead(PIN_ENC2_A) != digitalRead(PIN_ENC2_B)) ? -1 : +1; }

void ENCODER_Init() {
    pinMode(PIN_ENC1_A, INPUT_PULLUP);
    pinMode(PIN_ENC1_B, INPUT_PULLUP);
    pinMode(PIN_ENC2_A, INPUT_PULLUP);
    pinMode(PIN_ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), isr1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC1_B), isr1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), isr2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC2_B), isr2B, CHANGE);
}

// Atomically read a count by disabling interrupts — safe from task context.
static long getCount(uint8_t enc) {
    uint32_t primask;
    __asm volatile ("mrs %0, primask\n cpsid i" : "=r"(primask) :: "memory");
    long c = (enc == 1) ? s_count1 : s_count2;
    __asm volatile ("msr primask, %0" :: "r"(primask) : "memory");
    return c;
}

float ENCODER_GetAngle(uint8_t enc) {
    long count = getCount(enc);
    float angle = fmodf(count * (360.0f / ENCODER_COUNTS_PER_REV), 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}
