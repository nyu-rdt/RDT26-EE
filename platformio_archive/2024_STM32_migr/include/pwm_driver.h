/**
 * PWM Servo Driver Header
 * Rover Control System - STM32F446RE
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Initialize TIM2 for servo PWM output (50Hz, 1000-2000us pulses) */
void PWM_Driver_Init(void);

/* Set excavation belt servo pulse width (microseconds) */
void PWM_SetBeltPulse(uint16_t pulseUs);

/* Set excavation system servo pulse width (microseconds) */
void PWM_SetExcavationPulse(uint16_t pulseUs);

/* Convenience functions */
void PWM_BeltStop(void);
void PWM_BeltOutward(void);
void PWM_BeltInward(void);
void PWM_ExcavationStop(void);
void PWM_ExcavationUp(void);
void PWM_ExcavationDown(void);

/* Get timer handle */
TIM_HandleTypeDef* PWM_GetTimerHandle(void);

#endif /* PWM_DRIVER_H */
