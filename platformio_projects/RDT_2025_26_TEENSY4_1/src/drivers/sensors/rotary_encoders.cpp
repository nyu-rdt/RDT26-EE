#include <Arduino.h>
#include <cmath>
#include "config.h"
#include "rotary_encoders.h"

volatile long count1 = 0;
volatile long count2 = 0;

static void isr1A() { count1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : +1; }
static void isr1B() { count1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? -1 : +1; }
static void isr2A() { count2 += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : +1; }
static void isr2B() { count2 += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? -1 : +1; }

// Save PRIMASK and disable interrupts, returning the saved state.
// Restoring with restore_irq() re-enables only if they were on before —
// safe to call from ISR context or a future RTOS task.
static inline uint32_t save_and_disable_irq(void) {
    uint32_t primask;
    __asm volatile ("mrs %0, primask\n cpsid i" : "=r"(primask) :: "memory");
    return primask;
}

static inline void restore_irq(uint32_t primask) {
    __asm volatile ("msr primask, %0" :: "r"(primask) : "memory");
}

void ROTARY_ENCODER_Init() {
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), isr1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), isr2B, CHANGE);
}


long ROTARY_ENCODER_getCount(uint8_t encoderNum) {
    uint32_t primask = save_and_disable_irq();
    long count;
    switch (encoderNum) {
        case 1:  count = count1; break;
        case 2:  count = count2; break;
        default: count = 0;      break;
    }
    restore_irq(primask);
    return count;
}

float ROTARY_ENCODER_getEncoderAngle(uint8_t encoderNum) {
    long count = ROTARY_ENCODER_getCount(encoderNum);
    float angle = fmod(count * DEGREES_PER_COUNT, 360.0f);
    // angl= (count * DEGREES_PER_COUNT) % 360.0f;
    if (angle < 0) angle += 360.0f;
    return angle;
}
