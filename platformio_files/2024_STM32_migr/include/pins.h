/**
 * Pin Definitions for STM32F446RE Nucleo
 * Rover Control System
 * 
 * Pin mapping from Teensy 4.1 to STM32F446RE
 */

#ifndef PINS_H
#define PINS_H

#include "stm32f4xx_hal.h"

/* ============== CAN Bus Pins ============== */
/* CAN1: PA11 (RX), PA12 (TX) - on Nucleo CN10 */
#define CAN_TX_PORT         GPIOA
#define CAN_TX_PIN          GPIO_PIN_12
#define CAN_RX_PORT         GPIOA
#define CAN_RX_PIN          GPIO_PIN_11
#define CAN_AF              GPIO_AF9_CAN1

/* ============== I2C Slave Pins ============== */
/* I2C1: PB8 (SCL), PB9 (SDA) - on Nucleo CN5 D15/D14 */
#define I2C_SCL_PORT        GPIOB
#define I2C_SCL_PIN         GPIO_PIN_8
#define I2C_SDA_PORT        GPIOB
#define I2C_SDA_PIN         GPIO_PIN_9
#define I2C_AF              GPIO_AF4_I2C1
#define I2C_SLAVE_ADDRESS   0x24

/* ============== PWM Servo Pins (TIM2) ============== */
/* Excavation Belt PWM: PA0 (TIM2_CH1) - on Nucleo CN7 A0 */
#define EXCAVATION_BELT_PWM_PORT    GPIOA
#define EXCAVATION_BELT_PWM_PIN     GPIO_PIN_0
#define EXCAVATION_BELT_TIM_CHANNEL TIM_CHANNEL_1

/* Excavation System PWM: PA1 (TIM2_CH2) - on Nucleo CN7 A1 */
#define EXCAVATION_SYSTEM_PWM_PORT    GPIOA
#define EXCAVATION_SYSTEM_PWM_PIN     GPIO_PIN_1
#define EXCAVATION_SYSTEM_TIM_CHANNEL TIM_CHANNEL_2

#define PWM_TIM_AF          GPIO_AF1_TIM2

/* ============== ADC String Pot Pin ============== */
/* String Pot: PA4 (ADC1_IN4) - on Nucleo CN8 A2 */
#define STRING_POT_PORT     GPIOA
#define STRING_POT_PIN      GPIO_PIN_4
#define STRING_POT_CHANNEL  ADC_CHANNEL_4

/* ============== HX711 Load Cell Pins (GPIO bit-bang) ============== */
/* Scale 1: PC0 (DOUT), PC1 (CLK) */
#define HX711_1_DOUT_PORT   GPIOC
#define HX711_1_DOUT_PIN    GPIO_PIN_0
#define HX711_1_CLK_PORT    GPIOC
#define HX711_1_CLK_PIN     GPIO_PIN_1

/* Scale 2: PC2 (DOUT), PC3 (CLK) */
#define HX711_2_DOUT_PORT   GPIOC
#define HX711_2_DOUT_PIN    GPIO_PIN_2
#define HX711_2_CLK_PORT    GPIOC
#define HX711_2_CLK_PIN     GPIO_PIN_3

/* Scale 3: PB0 (DOUT), PB1 (CLK) */
#define HX711_3_DOUT_PORT   GPIOB
#define HX711_3_DOUT_PIN    GPIO_PIN_0
#define HX711_3_CLK_PORT    GPIOB
#define HX711_3_CLK_PIN     GPIO_PIN_1

/* Scale 4: PB2 (DOUT), PB4 (CLK) */
#define HX711_4_DOUT_PORT   GPIOB
#define HX711_4_DOUT_PIN    GPIO_PIN_2
#define HX711_4_CLK_PORT    GPIOB
#define HX711_4_CLK_PIN     GPIO_PIN_4

/* ============== Debug UART Pins ============== */
/* USART2: PA2 (TX), PA3 (RX) - connected to ST-Link VCP */
#define DEBUG_UART_TX_PORT  GPIOA
#define DEBUG_UART_TX_PIN   GPIO_PIN_2
#define DEBUG_UART_RX_PORT  GPIOA
#define DEBUG_UART_RX_PIN   GPIO_PIN_3
#define DEBUG_UART_AF       GPIO_AF7_USART2

#endif /* PINS_H */
