/**
 * CAN Bus Driver Header
 * Rover Control System - STM32F446RE
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* CAN message structure (compatible with original protocol) */
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
    bool     extended;
} CAN_Msg_t;

/* Initialize CAN peripheral at 500kbps */
void CAN_Driver_Init(void);

/* Send a CAN message (blocking, with timeout) */
int CAN_Driver_Send(CAN_Msg_t *msg);

/* Craft a motor command message (same format as original) */
CAN_Msg_t CAN_CraftMotorMessage(int msgType, float value, uint8_t motorId);

/* Send motor command helper */
int CAN_SendMotorCommand(uint8_t motorId, float speedPercent);

/* Get CAN handle for HAL operations */
CAN_HandleTypeDef* CAN_GetHandle(void);

#endif /* CAN_DRIVER_H */
