#pragma once

void ROTARY_ENCODER_Init();
void isr1A();
void isr1B();
void isr2A();
void isr2B();
float ROTARY_ENCODER_getEncoderAngle(uint8_t encoderNum); 
long ROTARY_ENCODER_getCount(uint8_t encoderNum);
float ROTARY_ENCODER_countsToAngle(long counts);
    


