/**
 * UART Debug Driver Implementation
 * Rover Control System - STM32F446RE
 * 
 * USART2 connected to ST-Link VCP for debug output
 */

#include "uart_debug.h"
#include "pins.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef huart2;

void UART_Debug_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure GPIO pins */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DEBUG_UART_TX_PIN | DEBUG_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = DEBUG_UART_AF;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure USART2 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HAL_UART_Init(&huart2);
}

void UART_Print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

void UART_Println(const char *str)
{
    UART_Print(str);
    UART_Print("\r\n");
}

void UART_PrintInt(int32_t value)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)value);
    UART_Print(buf);
}

UART_HandleTypeDef* UART_GetHandle(void)
{
    return &huart2;
}
