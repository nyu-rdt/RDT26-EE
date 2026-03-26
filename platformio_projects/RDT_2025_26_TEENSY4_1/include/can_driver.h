#pragma once

void CAN_Init();
void CAN_SendMotorSpeed(uint32_t canId, float speed);
void CAN_SendLocomotion(float left, float right);
void CAN_SendExcavation(float speed);
