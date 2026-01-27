/**
 * @file can_driver.h
 * @brief CAN bus communication driver for motor controllers
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "config.h"

/**
 * @brief Initialize CAN bus interface
 */
void CAN_Init(void);

/**
 * @brief Send speed command to a motor
 * @param canId Motor's CAN ID
 * @param speedPercent Speed as fraction (-1.0 to 1.0)
 * @return 1 on success, 0 on failure
 */
int CAN_SendMotorSpeed(uint32_t canId, float speedPercent);

/**
 * @brief Craft a CAN message with speed data
 * @param msgType Message type (usually 0 for speed)
 * @param value Speed value (-1.0 to 1.0)
 * @param id Target motor ID
 * @return Constructed CAN message
 */
CAN_message_t CAN_CraftMessage(int msgType, float value, uint8_t id);

#endif // CAN_DRIVER_H
