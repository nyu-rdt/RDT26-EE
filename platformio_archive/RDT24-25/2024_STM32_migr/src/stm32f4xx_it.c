/**
 * STM32F4xx Interrupt Handlers
 * Rover Control System - STM32F446RE
 * 
 * Bare-metal implementation - all interrupt handlers
 */

#include "stm32f4xx_hal.h"
#include "scheduler.h"
#include "i2c_driver.h"

/* ============== Cortex-M4 Processor Exception Handlers ============== */

void NMI_Handler(void)
{
    while (1) {
        __NOP();
    }
}

void HardFault_Handler(void)
{
    while (1) {
        __NOP();
    }
}

void MemManage_Handler(void)
{
    while (1) {
        __NOP();
    }
}

void BusFault_Handler(void)
{
    while (1) {
        __NOP();
    }
}

void UsageFault_Handler(void)
{
    while (1) {
        __NOP();
    }
}

void SVC_Handler(void)
{
    /* Not used in bare-metal */
}

void DebugMon_Handler(void)
{
    /* Nothing needed */
}

void PendSV_Handler(void)
{
    /* Not used in bare-metal */
}

void SysTick_Handler(void)
{
    /* We use TIM6 for timing, but HAL may use SysTick */
    /* HAL_IncTick is not needed since we override HAL_GetTick in scheduler.c */
}

/* ============== TIM6 Scheduler Tick Handler ============== */
void TIM6_DAC_IRQHandler(void)
{
    Scheduler_TickHandler();
}

/* ============== I2C1 Handlers ============== */
void I2C1_EV_IRQHandler(void)
{
    I2C_EventIRQHandler();
}

void I2C1_ER_IRQHandler(void)
{
    I2C_ErrorIRQHandler();
}
