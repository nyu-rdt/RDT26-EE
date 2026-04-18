#pragma once
#include <stdint.h>

void EE_BOX_Init();
void EE_BOX_Update();
void EE_BOX_EnableRelay();
void EE_BOX_DisableRelay();

// Returns relay pin states packed into one byte: bit0=relay, bit1=3s_low, bit2=6s_low
uint8_t EE_BOX_GetRelayStatus();
