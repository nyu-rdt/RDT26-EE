#pragma once

// Excavation subsystem — coordinates belt motor and string pot position feedback.
// Enforces travel limits: belt is stopped automatically when the conveyor reaches
// STRING_POT_LOWEST_THRESHOLD or STRING_POT_HIGHEST_THRESHOLD (set in config.h).

void EXCAV_Init();
void EXCAV_Update();

// Command belt spin (CAN motor): +1 forward, -1 reverse, 0 stop.
void EXCAV_SetBeltDirection(int direction);

// Command vertical movement (stepper — raises/lowers the belt assembly): +1 up, -1 down, 0 stop.
// Enforces string pot travel limits; stops and clears moving flag if already at the limit.
void EXCAV_SetVertDirection(int direction);

// Returns true while the belt motor is commanded to spin (i.e. actively digging).
bool EXCAV_GetBeltActive();

// Hard stop — belt, stepper, and moving flag. Bypasses ramp.
void EXCAV_EmergencyStop();

// Returns the last cached conveyor position (0 to STRING_POT_MAX_DISTANCE).
// ISR-safe — does not trigger an ADC read.
float EXCAV_GetConveyorDistance();

// True when stepper is descending and current exceeds CURRENT_STEPPER_OBSTRUCT_A.
// Always false until that threshold is set in config.h.
bool EXCAV_IsArmObstructed();

// True when belt is active and current stays below CURRENT_EXCAV_EMPTY_CUT_A
// for longer than EXCAV_EMPTY_CUT_PERSIST_MS — nothing to excavate, reposition rover.
// Always false until CURRENT_EXCAV_EMPTY_CUT_A is set in config.h.
bool EXCAV_IsEmptyCut();

// True when belt is active, current exceeds CURRENT_EXCAV_STALL_A, and mass
// gain over EXCAV_STALL_WINDOW_MS is below EXCAV_STALL_MASS_DELTA_KG.
// Always false when LOAD_CELLS_ENABLED=0 or CURRENT_EXCAV_STALL_A is unset.
bool EXCAV_IsBeltStalled();
