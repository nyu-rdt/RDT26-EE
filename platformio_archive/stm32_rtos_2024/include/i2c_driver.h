/**
 * I2C Slave Driver Header
 * Rover Control System - STM32F446RE + FreeRTOS
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Callback function type for data requests */
typedef void (*I2C_RequestCallback)(uint8_t *data, uint8_t *length);

/* Initialize I2C1 as slave with address 0x24 */
void I2C_Driver_Init(void);

/* Register callback for data requests (master read) */
void I2C_RegisterRequestCallback(I2C_RequestCallback callback);

/* Get I2C handle */
I2C_HandleTypeDef* I2C_GetHandle(void);

/* Must be called from I2C1 event IRQ handler */
void I2C_EventIRQHandler(void);

/* Must be called from I2C1 error IRQ handler */
void I2C_ErrorIRQHandler(void);

#endif /* I2C_DRIVER_H */
