/**
 * @file sensors.h
 * @brief Sensor interfaces (HX711 load cells, string potentiometer)
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

/**
 * @brief Initialize all sensors
 */
void Sensors_Init(void);

/**
 * @brief Read string potentiometer length
 * @return Length in inches
 */
float Sensors_GetStringLength(void);

/**
 * @brief Read weight from load cells (average of 4)
 * @return Weight in configured units
 */
float Sensors_GetWeight(void);

/**
 * @brief Update all sensor readings (call periodically)
 */
void Sensors_Update(void);

/**
 * @brief Get cached string length (no new reading)
 */
float Sensors_GetCachedLength(void);

/**
 * @brief Get cached weight (no new reading)
 */
float Sensors_GetCachedWeight(void);

#endif // SENSORS_H
