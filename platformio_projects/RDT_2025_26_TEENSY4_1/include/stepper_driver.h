#pragma once

void STEPPER_Init();
void setMicrostep(int mode);
void STEPPER_SetDirection(int direction);
void STEPPER_Update(unsigned long periodMicroseconds);
