#pragma once

typedef void (*StopFn)();

void ESTOP_RegisterCallbacks(StopFn stopLocomotion);
void ESTOP_StopAllMotors();
void ESTOP_Trigger();
