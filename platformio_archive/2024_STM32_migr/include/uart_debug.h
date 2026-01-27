/**
 * UART Debug Driver Header
 * Rover Control System - STM32F446RE
 */

#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Initialize USART2 for debug output (ST-Link VCP) */
void UART_Debug_Init(void);

/* Print string to debug UART */
void UART_Print(const char *str);

/* Print string with newline */
void UART_Println(const char *str);

/* Print integer */
void UART_PrintInt(int32_t value);

/* Get UART handle */
UART_HandleTypeDef* UART_GetHandle(void);

#endif /* UART_DEBUG_H */
