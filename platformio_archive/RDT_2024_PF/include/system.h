/**
 * @file system.h
 * @brief System-level functions (E-stop, emergency stop, main loop helpers)
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>

/**
 * @brief Initialize system-level hardware (relay pin, etc.)
 */
void System_Init(void);

/**
 * @brief Check hardware E-Stop relay status
 * @return true if E-Stop is engaged (motors should be stopped)
 */
bool System_CheckEStop(void);

/**
 * @brief Get E-Stop engaged state
 */
bool System_IsEStopEngaged(void);

/**
 * @brief Set E-Stop engaged state (used internally)
 */
void System_SetEStopEngaged(bool engaged);

/**
 * @brief Emergency stop all systems
 */
void System_EmergencyStop(void);

/**
 * @brief Periodic system update (weight reading, etc.)
 * Call from main loop.
 */
void System_Update(void);

/**
 * @brief Get cached weight value
 */
float System_GetCachedWeight(void);

#endif // SYSTEM_H
