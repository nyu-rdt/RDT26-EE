#pragma once

// Excavation subsystem — coordinates belt motor and string pot position feedback.
// Enforces travel limits: belt is stopped automatically when the conveyor reaches
// STRING_POT_LOWEST_THRESHOLD or STRING_POT_HIGHEST_THRESHOLD (set in config.h).

void EXCAV_Init();
void EXCAV_Update();

// Command the belt in a direction (+1 up, -1 down, 0 stop).
// Ignored if the conveyor is already at the limit in that direction.
void EXCAV_SetBeltDirection(int direction);

// Command vertical (stepper) movement (+1 up, -1 down, 0 stop).
void EXCAV_SetVertDirection(int direction);

// Hard stop — belt, stepper, and moving flag. Called by e-stop and timeout.
void EXCAV_Stop();

// Returns the last cached conveyor position (0 to STRING_POT_MAX_DISTANCE).
// ISR-safe — does not trigger an ADC read.
float EXCAV_GetConveyorDistance();
