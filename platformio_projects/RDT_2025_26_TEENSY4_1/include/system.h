#pragma once

typedef void (*SystemStopLocomotionFn)();
typedef void (*SystemStopExcavationFn)();

void SYSTEM_Init();
void SYSTEM_Update();
void SYSTEM_RegisterStopCallbacks(SystemStopLocomotionFn stopLocomotionFn,
                                  SystemStopExcavationFn stopExcavationFn);
void SYSTEM_StopAllMotors();
uint8_t SYSTEM_GetRelayStatus();
