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
 * @brief Stop conveyor belt
 */
void Excavation_BeltStop(void);

/**
 * @brief Run belt outward (eject material)
 */
void Excavation_BeltOutward(void);

/**
 * @brief Run belt inward (collect material)
 */
void Excavation_BeltInward(void);

/**
 * @brief Zero the excavation position using current string pot reading
 */
void Excavation_Zero(void);

/**
 * @brief Move to locomotion position (arm down)
 * @return true if position reached, false on timeout
 */
bool Excavation_MoveToLocomotionPosition(void);

/**
 * @brief Move to excavation position (arm up)
 * @return true if position reached, false on timeout
 */
bool Excavation_MoveToExcavationPosition(void);

/**
 * @brief Get current excavation position state
 * @return Current position enum
 */
ExcavationPosition_t Excavation_GetPosition(void);

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
