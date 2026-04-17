#pragma once

void  STRINGPOT_Init();
float STRINGPOT_ReadDistance();       // triggers ADC read — call from main loop only
float STRINGPOT_GetCachedDistance();  // ISR-safe, returns last computed value
int   STRINGPOT_GetLastRaw();
int   STRINGPOT_GetState();
void  STRINGPOT_SetMoving(bool isMoving);
void  STRINGPOT_UpdateState();
