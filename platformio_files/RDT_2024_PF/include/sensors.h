/**
 * @file sensors.h
 * @brief Sensor interfaces (HX711 load cells, string potentiometer, encoders)
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

/**
 * @brief Initialize all sensors (load cells, string pot, encoders)
 */
void Sensors_Init(void);

/**
 * @brief Read string potentiometer length
 * @return Length in inches
 */
float Sensors_GetStringLength(void);

/**
 * @brief Read weight from load cells (sum of 2 active scales)
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

/**
 * @brief Convert encoder counts to angle (0-360 degrees)
 * @param counts Raw encoder count
 * @return Angle in degrees
 */
float Sensors_CountsToAngle(long counts);

/**
 * @brief Set which encoder is active for data reporting
 * @param encoderNum Encoder number (1-4)
 */
void Sensors_SetActiveEncoder(uint8_t encoderNum);

/**
 * @brief Get currently active encoder number
 * @return Active encoder (1-4)
 */
uint8_t Sensors_GetActiveEncoder(void);

/**
 * @brief Get angle of specific encoder
 * @param encoderNum Encoder number (1-4)
 * @return Angle in degrees
 */
float Sensors_GetEncoderAngle(uint8_t encoderNum);

/**
 * @brief Get raw count of specific encoder
 * @param encoderNum Encoder number (1-4)
 * @return Raw encoder count
 */
long Sensors_GetEncoderCount(uint8_t encoderNum);

// Encoder ISR declarations (called by interrupt system)
void Sensors_ISR_Enc1A(void);
void Sensors_ISR_Enc1B(void);
void Sensors_ISR_Enc2A(void);
void Sensors_ISR_Enc2B(void);
void Sensors_ISR_Enc3A(void);
void Sensors_ISR_Enc3B(void);
void Sensors_ISR_Enc4A(void);
void Sensors_ISR_Enc4B(void);

#endif // SENSORS_H
