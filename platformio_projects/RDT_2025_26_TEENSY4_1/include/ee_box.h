#pragma once
#include <stdint.h>

void EE_BOX_Init();
void EE_BOX_Update();
void EE_BOX_EnableRelay();
void EE_BOX_DisableRelay();

// Returns relay pin states packed into one byte: bit0=relay, bit1=3s_low, bit2=6s_low
uint8_t EE_BOX_GetRelayStatus();

// True when relay is energized (kill switch NOT pressed). False = mechanical e-stop active.
bool EE_BOX_IsRelayEngaged();

// True when any current channel exceeds CURRENT_OVERCURRENT_ANY_A.
// Always false until that threshold is set in config.h.
bool EE_BOX_IsOvercurrent();
