#pragma once
#include <stdint.h>

void  LOAD_CELL_Init();
void  LOAD_CELL_Update();               // call from main loop — reads all 4 cells
float LOAD_CELL_GetMass(uint8_t index); // single cell in kg, index 0-3
float LOAD_CELL_GetTotalMass();         // sum of all 4 cells in kg
