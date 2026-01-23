/**
 * STM32F4xx Interrupt Handlers
 * Rover Control System - STM32F446RE + FreeRTOS
 * 
 * Note: SVC_Handler, PendSV_Handler, and SysTick_Handler are 
 * provided by FreeRTOS via the mappings in FreeRTOSConfig.h
 */

#include "stm32f4xx_hal.h"
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

void DebugMon_Handler(void)
{
    /* Nothing needed */
}

/* Note: SVC_Handler, PendSV_Handler, SysTick_Handler are provided by FreeRTOS */
/* Mapped via FreeRTOSConfig.h:
 *   #define vPortSVCHandler     SVC_Handler
 *   #define xPortPendSVHandler  PendSV_Handler
 *   #define xPortSysTickHandler SysTick_Handler
 */

/* ============== I2C1 Handlers ============== */
void I2C1_EV_IRQHandler(void)
{
    I2C_EventIRQHandler();
}

void I2C1_ER_IRQHandler(void)
{
    I2C_ErrorIRQHandler();
}
