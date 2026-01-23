/**
 * @file locomotion.h
 * @brief Locomotion control for 4-wheel drive system
 */

#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Arduino.h>

/**
 * @brief Stop all drive motors
 */
void Locomotion_Stop(void);

/**
 * @brief Drive forward
 * @param speedFactor Speed multiplier (0.0 to 1.0)
 */
void Locomotion_Forward(float speedFactor);

/**
 * @brief Drive backward
 * @param speedFactor Speed multiplier (0.0 to 1.0)
 */
void Locomotion_Backward(float speedFactor);

/**
 * @brief Turn left (tank turn)
 * @param speedFactor Speed multiplier (0.0 to 1.0)
 */
void Locomotion_TurnLeft(float speedFactor);

/**
 * @brief Turn right (tank turn)
 * @param speedFactor Speed multiplier (0.0 to 1.0)
 */
void Locomotion_TurnRight(float speedFactor);

#endif // LOCOMOTION_H
