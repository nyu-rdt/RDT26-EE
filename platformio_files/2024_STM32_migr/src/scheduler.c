/**
 * Time-Triggered Scheduler Implementation
 * Rover Control System - STM32F446RE
 * 
 * Uses TIM6 basic timer for precise 1ms tick
 * No floating point, minimal ISR overhead
 */

#include "scheduler.h"

/* Global scheduler state */
volatile Scheduler_State_t scheduler = {0};

/* TIM6 handle */
static TIM_HandleTypeDef htim6;

/* Internal counters for each task period */
static volatile uint32_t counter_command = 0;
static volatile uint32_t counter_sensor = 0;
static volatile uint32_t counter_control = 0;
static volatile uint32_t counter_debug = 0;

void Scheduler_Init(void)
{
    /* Enable TIM6 clock */
    __HAL_RCC_TIM6_CLK_ENABLE();
    
    /* Configure TIM6 for 1ms tick */
    /* APB1 timer clock = 90MHz (180MHz sysclk, APB1 prescaler /2, timer x2) */
    /* For 1kHz: prescaler = 9000-1 = 8999 -> 90MHz/9000 = 10kHz */
    /*           period = 10-1 = 9 -> 10kHz/10 = 1kHz (1ms) */
    /* Alternative: prescaler = 90-1 = 89 -> 1MHz, period = 1000-1 = 999 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 89;          /* 90MHz / 90 = 1MHz */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 999;            /* 1MHz / 1000 = 1kHz (1ms) */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    HAL_TIM_Base_Init(&htim6);
    
    /* Enable update interrupt */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);  /* Highest priority */
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    
    /* Start timer with interrupt */
    HAL_TIM_Base_Start_IT(&htim6);
}

uint32_t Scheduler_GetTick(void)
{
    return scheduler.tick_count;
}

void Scheduler_DelayMs(uint32_t ms)
{
    uint32_t start = scheduler.tick_count;
    while ((scheduler.tick_count - start) < ms) {
        __NOP();  /* Busy wait - only for init */
    }
}

bool Scheduler_ShouldRunCommand(void)
{
    if (scheduler.run_command) {
        scheduler.run_command = false;
        return true;
    }
    return false;
}

bool Scheduler_ShouldRunSensor(void)
{
    if (scheduler.run_sensor) {
        scheduler.run_sensor = false;
        return true;
    }
    return false;
}

bool Scheduler_ShouldRunControl(void)
{
    if (scheduler.run_control) {
        scheduler.run_control = false;
        return true;
    }
    return false;
}

bool Scheduler_ShouldRunDebug(void)
{
    if (scheduler.run_debug) {
        scheduler.run_debug = false;
        return true;
    }
    return false;
}

/**
 * Timer tick ISR - runs every 1ms
 * Keep this FAST - no function calls if possible
 */
void Scheduler_TickHandler(void)
{
    /* Clear interrupt flag */
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
    
    /* Increment global tick */
    scheduler.tick_count++;
    
    /* Check each task period */
    counter_command++;
    if (counter_command >= PERIOD_COMMAND_MS) {
        counter_command = 0;
        scheduler.run_command = true;
    }
    
    counter_sensor++;
    if (counter_sensor >= PERIOD_SENSOR_MS) {
        counter_sensor = 0;
        scheduler.run_sensor = true;
    }
    
    counter_control++;
    if (counter_control >= PERIOD_CONTROL_MS) {
        counter_control = 0;
        scheduler.run_control = true;
    }
    
    counter_debug++;
    if (counter_debug >= PERIOD_DEBUG_MS) {
        counter_debug = 0;
        scheduler.run_debug = true;
    }
}

/**
 * HAL requires this callback - we handle in Scheduler_TickHandler directly
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Not used - we handle in direct IRQ for minimum latency */
    (void)htim;
}

/* Provide HAL_GetTick using our scheduler */
uint32_t HAL_GetTick(void)
{
    return scheduler.tick_count;
}

/* HAL_IncTick is not used since we override HAL_GetTick */
