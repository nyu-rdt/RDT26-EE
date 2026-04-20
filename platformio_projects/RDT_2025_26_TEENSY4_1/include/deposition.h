#pragma once
#include "depo_door_driver.h"

void DEPO_Init();
void DEPO_Update();

void DEPO_OpenDoor();
void DEPO_CloseDoor();
void DEPO_SetVib(int direction);

void DEPO_EmergencyStop();

// Returns the last cached door state — ISR-safe (no hardware access).
DepoDoorState DEPO_GetDoorState();

// True when total bin mass is below LOAD_CELL_EMPTY_THRESHOLD_KG.
// Always false when LOAD_CELLS_ENABLED=0.
bool DEPO_IsBinEmpty();
