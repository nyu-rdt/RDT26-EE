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

#endif // I2C_COMMANDS_H
