#pragma once

#include <stdint.h>

void ROTARY_ENCODER_Init();
float ROTARY_ENCODER_getEncoderAngle(uint8_t encoderNum);
long  ROTARY_ENCODER_getCount(uint8_t encoderNum);
