/**
 * @file i2c_commands.h
 * @brief I2C slave command interface
 */

#ifndef I2C_COMMANDS_H
#define I2C_COMMANDS_H

#include <Arduino.h>

/**
 * @brief Initialize I2C slave interface
 */
void I2C_Init(void);

/**
 * @brief Process received command
 * @param command Command byte from master
 */
void I2C_ProcessCommand(int command);

/**
 * @brief Emergency stop all systems
 */
void EmergencyStop(void);

/**
 * @brief Get last received command (for debugging)
 */
int I2C_GetLastCommand(void);

/**
 * @brief Check and handle E-Stop relay state
 * @return true if E-Stop is engaged (motors should be stopped)
 */
bool I2C_CheckEStop(void);

/**
 * @brief Get E-Stop engaged state
 */
bool I2C_IsEStopEngaged(void);

/**
 * @brief Set E-Stop engaged state
 */
void I2C_SetEStopEngaged(bool engaged);

/**
 * @brief Handle position command interruption
 * @return true if a position command was interrupted and should be processed
 */
bool I2C_HandleInterruptedCommand(void);

/**
 * @brief Refresh active motor commands (call periodically for CAN keepalive)
 */
void I2C_RefreshMotorCommands(void);

/**
 * @brief Update sensor data and prepare data packet
 */
void I2C_UpdateData(void);

#endif // I2C_COMMANDS_H
