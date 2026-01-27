/**
 * ADC Driver Implementation
 * Rover Control System - STM32F446RE
 * 
 * ADC1 Channel 4 (PA4) for string potentiometer
 * 12-bit resolution, 3.3V reference
 */

#include "adc_driver.h"
#include "pins.h"
#include "commands.h"

static ADC_HandleTypeDef hadc1;

void ADC_Driver_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure GPIO pin as analog input */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = STRING_POT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(STRING_POT_PORT, &GPIO_InitStruct);
    
    /* Configure ADC1 */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    
    HAL_ADC_Init(&hadc1);
    
    /* Configure channel */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = STRING_POT_CHANNEL;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

uint16_t ADC_ReadStringPot(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

float ADC_GetStringLength(void)
{
    uint16_t adcValue = ADC_ReadStringPot();
    
    /* Convert ADC to voltage (12-bit, 3.3V ref) */
    float voltage = (float)adcValue * (ADC_VREF / ADC_MAX_VALUE);
    
    /* Apply same linear conversion as original */
    /* Note: Original used 5V sensor, may need recalibration for 3.3V */
    /* If sensor outputs 0-5V, you need a voltage divider or level shifter */
    /* For now, assuming sensor output is scaled to 0-3.3V range */
    float length = VOLTAGE_LENGTH_CONVERT * voltage - VOLTAGE_LENGTH_Y_INTERCEPT;
    
    return length;
}

ADC_HandleTypeDef* ADC_GetHandle(void)
{
    return &hadc1;
}
