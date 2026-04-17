#pragma once

void CURRENT_SENSORS_Init();
void CURRENT_SENSORS_Update();

// Returns pointer to internal buffer — valid between Update() calls.
const float* CURRENT_SENSORS_GetBuffer();