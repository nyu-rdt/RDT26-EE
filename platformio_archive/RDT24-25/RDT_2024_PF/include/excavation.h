/**
 * @file excavation.h
 * @brief Excavation and deposition system control
 */

#ifndef EXCAVATION_H
#define EXCAVATION_H

#include <Arduino.h>
#include "config.h"

/**
 * @brief Initialize excavation PWM outputs
 */
void Excavation_Init(void);

/**
 * @brief Stop excavation arm movement
 */
void Excavation_Stop(void);

/**
 * @brief Move excavation arm up
 */
void Excavation_Up(void);

/**
 * @brief Move excavation arm down
 */
void Excavation_Down(void);

/**
 * @brief Stop conveyor belt (via CAN)
 */
void Excavation_BeltStop(void);

/**
 * @brief Run belt outward (eject material, via CAN)
 */
void Excavation_BeltOutward(void);

/**
 * @brief Run belt inward (collect material, via CAN)
 */
void Excavation_BeltInward(void);

/**
 * @brief Zero/stop excavation command
 */
void Excavation_Zero(void);

/**
 * @brief Set current position state
 * @param position New position state
 */
void Excavation_SetPosition(ExcavationPosition_t position);

/**
 * @brief Move to locomotion position (arm up)
 * @return true if position reached, false on timeout or interrupt
 */
bool Excavation_MoveToLocomotionPosition(void);

/**
 * @brief Move to excavation position (arm down)
 * @return true if position reached, false on timeout or interrupt
 */
bool Excavation_MoveToExcavationPosition(void);

/**
 * @brief Get current excavation position state
 * @return Current position enum
 */
ExcavationPosition_t Excavation_GetPosition(void);

/**
 * @brief Get active belt speed for command refresh
 * @return Active belt speed (-1.0 to 1.0)
 */
float Excavation_GetActiveBeltSpeed(void);

/**
 * @brief Get active deposition speed for command refresh
 * @return Active deposition speed (-1.0 to 1.0)
 */
float Excavation_GetActiveDepositionSpeed(void);

/**
 * @brief Refresh active motor commands (call periodically for CAN keepalive)
 */
void Excavation_RefreshCommands(void);

/**
 * @brief Rotate deposition to collection position
 */
void Deposition_RotateCollection(void);

/**
 * @brief Rotate deposition to dumping position
 */
void Deposition_RotateDumping(void);

/**
 * @brief Stop deposition rotation
 */
void Deposition_Stop(void);

#endif // EXCAVATION_H
