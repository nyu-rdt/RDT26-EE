#pragma once
#include <stdint.h>

// Registers the Wire2 onRequest callback for outbound I2C packet assembly.
// Call after Wire2.begin().
void COMMS_Init();

// Called from the main loop each iteration — computes the telemetry flag byte
// from current subsystem state and stores it safely for the ISR to read.
// Must NOT be called from ISR context.
void COMMS_UpdateFlags();
