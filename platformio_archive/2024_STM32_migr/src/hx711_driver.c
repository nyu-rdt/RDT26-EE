/**
 * HX711 Load Cell Driver Implementation
 * Rover Control System - STM32F446RE
 * 
 * GPIO bit-bang driver for HX711 ADC
 * Based on HX711 protocol: 24-bit ADC with serial clock
 */

#include "hx711_driver.h"

/* Microsecond delay using DWT cycle counter */
static inline void delay_us(uint32_t us)
{
    /* Simple busy wait - for more precise timing, use DWT or timer */
    volatile uint32_t count = us * 45;  /* Approximate for 180MHz */
    while (count--) {
        __NOP();
    }
}

static void gpio_set_high(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static void gpio_set_low(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static uint8_t gpio_read(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET ? 1 : 0;
}

void HX711_Init(HX711_t *hx, 
                GPIO_TypeDef *doutPort, uint16_t doutPin,
                GPIO_TypeDef *clkPort, uint16_t clkPin)
{
    hx->doutPort = doutPort;
    hx->doutPin = doutPin;
    hx->clkPort = clkPort;
    hx->clkPin = clkPin;
    hx->gain = HX711_GAIN_128;
    hx->scale = 1.0f;
    hx->offset = 0;
    
    /* Enable GPIO clocks */
    if (doutPort == GPIOA || clkPort == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    if (doutPort == GPIOB || clkPort == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    if (doutPort == GPIOC || clkPort == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure DOUT as input with pull-up */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = doutPin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(doutPort, &GPIO_InitStruct);
    
    /* Configure CLK as output, initially low */
    GPIO_InitStruct.Pin = clkPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(clkPort, &GPIO_InitStruct);
    
    gpio_set_low(clkPort, clkPin);
}

bool HX711_IsReady(HX711_t *hx)
{
    return gpio_read(hx->doutPort, hx->doutPin) == 0;
}

bool HX711_WaitReady(HX711_t *hx, uint32_t timeoutMs)
{
    uint32_t startTick = HAL_GetTick();
    while (!HX711_IsReady(hx)) {
        if ((HAL_GetTick() - startTick) > timeoutMs) {
            return false;
        }
    }
    return true;
}

int32_t HX711_ReadRaw(HX711_t *hx)
{
    /* Wait for DOUT to go low (data ready) */
    if (!HX711_WaitReady(hx, 100)) {
        return 0;  /* Timeout */
    }
    
    int32_t value = 0;
    
    /* Read 24 bits */
    for (int i = 0; i < 24; i++) {
        gpio_set_high(hx->clkPort, hx->clkPin);
        delay_us(1);
        
        value = (value << 1) | gpio_read(hx->doutPort, hx->doutPin);
        
        gpio_set_low(hx->clkPort, hx->clkPin);
        delay_us(1);
    }
    
    /* Send additional pulses to set gain for next reading */
    for (int i = 0; i < (int)hx->gain; i++) {
        gpio_set_high(hx->clkPort, hx->clkPin);
        delay_us(1);
        gpio_set_low(hx->clkPort, hx->clkPin);
        delay_us(1);
    }
    
    /* Convert 24-bit two's complement to 32-bit signed */
    if (value & 0x800000) {
        value |= 0xFF000000;  /* Sign extend */
    }
    
    return value;
}

void HX711_Tare(HX711_t *hx, uint8_t samples)
{
    int64_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += HX711_ReadRaw(hx);
    }
    hx->offset = (int32_t)(sum / samples);
}

void HX711_SetScale(HX711_t *hx, float scale)
{
    hx->scale = scale;
}

float HX711_GetUnits(HX711_t *hx, uint8_t samples)
{
    int64_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += HX711_ReadRaw(hx);
    }
    int32_t avg = (int32_t)(sum / samples);
    return (float)(avg - hx->offset) / hx->scale;
}

void HX711_PowerDown(HX711_t *hx)
{
    gpio_set_low(hx->clkPort, hx->clkPin);
    gpio_set_high(hx->clkPort, hx->clkPin);
    delay_us(100);  /* Keep high for >60us to enter power down */
}

void HX711_PowerUp(HX711_t *hx)
{
    gpio_set_low(hx->clkPort, hx->clkPin);
}
