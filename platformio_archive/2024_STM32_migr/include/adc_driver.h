/**
 * ADC Driver Header
 * Rover Control System - STM32F446RE
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Initialize ADC1 for string potentiometer reading */
void ADC_Driver_Init(void);

/* Read string potentiometer raw ADC value (12-bit) */
uint16_t ADC_ReadStringPot(void);

/* Get string length in physical units */
float ADC_GetStringLength(void);

/* Get ADC handle */
ADC_HandleTypeDef* ADC_GetHandle(void);

#endif /* ADC_DRIVER_H */
