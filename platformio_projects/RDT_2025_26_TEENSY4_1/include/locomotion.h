#pragma once

void LOCO_Init();
void LOCO_Update();

// Set target wheel speeds: +1.0 = full forward, -1.0 = full reverse, 0 = stop.
void LOCO_SetSpeeds(float left, float right);

// Graceful stop — ramps both wheels to zero.
void LOCO_Stop();

// Hard stop — bypasses ramp, sends zero immediately. Called by e-stop and timeout.
void LOCO_EmergencyStop();

// True when motors are commanded but current exceeds CURRENT_LOCO_STALL_A.
// Always false until that threshold is set in config.h.
bool LOCO_IsStalled();
