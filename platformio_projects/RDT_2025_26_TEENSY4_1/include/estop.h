#pragma once

typedef void (*StopFn)();

// Register a stop callback. Call once per subsystem during init.
// Every registered function is called by ESTOP_StopAllMotors — don't forget to register new subsystems.
void ESTOP_RegisterCallback(StopFn fn);
void ESTOP_StopAllMotors();
void ESTOP_Trigger();
