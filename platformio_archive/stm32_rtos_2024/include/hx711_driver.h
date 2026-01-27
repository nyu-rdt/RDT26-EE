/**
 * HX711 Load Cell Driver Header
 * Rover Control System - STM32F446RE
 * 
 * GPIO bit-bang driver for HX711 ADC
 */

#ifndef HX711_DRIVER_H
#define HX711_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* HX711 gain modes */
typedef enum {
    HX711_GAIN_128 = 1,  /* Channel A, gain 128 (25 pulses) */
    HX711_GAIN_32  = 2,  /* Channel B, gain 32 (26 pulses) */
    HX711_GAIN_64  = 3   /* Channel A, gain 64 (27 pulses) */
} HX711_Gain_t;

/* HX711 instance structure */
typedef struct {
    GPIO_TypeDef *doutPort;
    uint16_t      doutPin;
    GPIO_TypeDef *clkPort;
    uint16_t      clkPin;
    HX711_Gain_t  gain;
    float         scale;
    int32_t       offset;
} HX711_t;

/* Initialize HX711 instance */
void HX711_Init(HX711_t *hx, 
                GPIO_TypeDef *doutPort, uint16_t doutPin,
                GPIO_TypeDef *clkPort, uint16_t clkPin);

/* Check if HX711 is ready (DOUT low) */
bool HX711_IsReady(HX711_t *hx);

/* Wait for HX711 to be ready with timeout */
bool HX711_WaitReady(HX711_t *hx, uint32_t timeoutMs);

/* Read raw 24-bit value from HX711 */
int32_t HX711_ReadRaw(HX711_t *hx);

/* Tare (zero) the scale */
void HX711_Tare(HX711_t *hx, uint8_t samples);

/* Set the scale factor */
void HX711_SetScale(HX711_t *hx, float scale);

/* Get weight in calibrated units */
float HX711_GetUnits(HX711_t *hx, uint8_t samples);

/* Power down the HX711 */
void HX711_PowerDown(HX711_t *hx);

/* Power up the HX711 */
void HX711_PowerUp(HX711_t *hx);

#endif /* HX711_DRIVER_H */
