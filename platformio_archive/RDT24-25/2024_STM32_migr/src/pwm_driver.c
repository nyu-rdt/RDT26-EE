/**
 * PWM Servo Driver Implementation
 * Rover Control System - STM32F446RE
 * 
 * TIM2 configured for 50Hz PWM (servo control)
 * Channel 1: Excavation Belt
 * Channel 2: Excavation System
 */

#include "pwm_driver.h"
#include "pins.h"
#include "commands.h"

static TIM_HandleTypeDef htim2;

void PWM_Driver_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure GPIO pins for PWM output */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = EXCAVATION_BELT_PWM_PIN | EXCAVATION_SYSTEM_PWM_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = PWM_TIM_AF;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure TIM2 for 50Hz PWM (20ms period) */
    /* APB1 timer clock = 90MHz (assuming 180MHz sysclk, APB1 prescaler /2, timer x2) */
    /* For 50Hz: period = 20ms = 20000us */
    /* Prescaler = 90 -> timer clock = 1MHz (1us resolution) */
    /* ARR = 20000 - 1 = 19999 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 89;  /* 90MHz / 90 = 1MHz */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999;  /* 20000 counts = 20ms */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim2);
    
    /* Configure PWM channels */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = SERVO_NEUTRAL_US;  /* 1500us neutral */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, EXCAVATION_BELT_TIM_CHANNEL);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, EXCAVATION_SYSTEM_TIM_CHANNEL);
    
    /* Start PWM on both channels */
    HAL_TIM_PWM_Start(&htim2, EXCAVATION_BELT_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, EXCAVATION_SYSTEM_TIM_CHANNEL);
}

void PWM_SetBeltPulse(uint16_t pulseUs)
{
    __HAL_TIM_SET_COMPARE(&htim2, EXCAVATION_BELT_TIM_CHANNEL, pulseUs);
}

void PWM_SetExcavationPulse(uint16_t pulseUs)
{
    __HAL_TIM_SET_COMPARE(&htim2, EXCAVATION_SYSTEM_TIM_CHANNEL, pulseUs);
}

void PWM_BeltStop(void)
{
    PWM_SetBeltPulse(SERVO_NEUTRAL_US);
}

void PWM_BeltOutward(void)
{
    PWM_SetBeltPulse(SERVO_BELT_OUTWARD_US);
}

void PWM_BeltInward(void)
{
    PWM_SetBeltPulse(SERVO_BELT_INWARD_US);
}

void PWM_ExcavationStop(void)
{
    PWM_SetExcavationPulse(SERVO_NEUTRAL_US);
}

void PWM_ExcavationUp(void)
{
    PWM_SetExcavationPulse(SERVO_EXCAVATION_UP_US);
}

void PWM_ExcavationDown(void)
{
    PWM_SetExcavationPulse(SERVO_EXCAVATION_DOWN_US);
}

TIM_HandleTypeDef* PWM_GetTimerHandle(void)
{
    return &htim2;
}
