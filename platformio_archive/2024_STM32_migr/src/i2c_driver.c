    /**
 * I2C Slave Driver Implementation
 * Rover Control System - STM32F446RE
 * 
 * Bare-metal implementation without RTOS
 * Uses simple ring buffer for command queueing
 * 
 * I2C1 configured as slave at address 0x24
 * Receives single-byte commands from master
 * Responds with 3-byte data packet on request
 */

#include "i2c_driver.h"
#include "pins.h"

/* ============== Ring Buffer for Commands ============== */
#define CMD_BUFFER_SIZE     16
#define CMD_BUFFER_MASK     (CMD_BUFFER_SIZE - 1)

typedef struct {
    volatile uint8_t buffer[CMD_BUFFER_SIZE];
    volatile uint8_t head;  /* Write index (ISR) */
    volatile uint8_t tail;  /* Read index (main) */
} CmdRingBuffer_t;

static CmdRingBuffer_t cmdBuffer = {0};

/* ============== I2C Handle and Callbacks ============== */
static I2C_HandleTypeDef hi2c1;
static I2C_RequestCallback requestCallback = NULL;

static uint8_t rxBuffer[16];
static uint8_t txBuffer[8];
static uint8_t txLength = 0;

/* ============== Ring Buffer Operations ============== */
static inline bool RingBuffer_IsEmpty(void)
{
    return (cmdBuffer.head == cmdBuffer.tail);
}

static inline bool RingBuffer_IsFull(void)
{
    return (((cmdBuffer.head + 1) & CMD_BUFFER_MASK) == cmdBuffer.tail);
}

static inline void RingBuffer_Put(uint8_t data)
{
    if (!RingBuffer_IsFull()) {
        cmdBuffer.buffer[cmdBuffer.head] = data;
        cmdBuffer.head = (cmdBuffer.head + 1) & CMD_BUFFER_MASK;
    }
}

static inline bool RingBuffer_Get(uint8_t *data)
{
    if (RingBuffer_IsEmpty()) {
        return false;
    }
    *data = cmdBuffer.buffer[cmdBuffer.tail];
    cmdBuffer.tail = (cmdBuffer.tail + 1) & CMD_BUFFER_MASK;
    return true;
}

/* ============== Public API ============== */
void I2C_Driver_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure GPIO pins for I2C */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = I2C_AF;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Configure I2C1 as slave */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;  /* 100kHz standard mode */
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = I2C_SLAVE_ADDRESS << 1;  /* 7-bit address shifted */
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    HAL_I2C_Init(&hi2c1);
    
    /* Initialize ring buffer */
    cmdBuffer.head = 0;
    cmdBuffer.tail = 0;
    
    /* Enable I2C interrupts */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    
    /* Enable address match interrupt and start listening */
    HAL_I2C_EnableListen_IT(&hi2c1);
}

void I2C_RegisterRequestCallback(I2C_RequestCallback callback)
{
    requestCallback = callback;
}

bool I2C_GetCommand(uint8_t *cmd)
{
    return RingBuffer_Get(cmd);
}

I2C_HandleTypeDef* I2C_GetHandle(void)
{
    return &hi2c1;
}

void I2C_EventIRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C_ErrorIRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

/* ============== HAL Callbacks for I2C Slave Mode ============== */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    (void)AddrMatchCode;
    
    if (hi2c->Instance == I2C1) {
        if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
            /* Master is sending data to us */
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, 1, I2C_FIRST_AND_LAST_FRAME);
        } else {
            /* Master is requesting data from us */
            if (requestCallback != NULL) {
                requestCallback(txBuffer, &txLength);
            }
            if (txLength > 0) {
                HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, txLength, I2C_FIRST_AND_LAST_FRAME);
            }
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        /* Command received - put in ring buffer */
        RingBuffer_Put(rxBuffer[0]);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* Transmission complete - nothing special needed */
    (void)hi2c;
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* Re-enable listening after transfer complete */
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    /* Re-enable listening on error */
    HAL_I2C_EnableListen_IT(hi2c);
}

/* ============== IRQ Handlers ============== */
/* Note: These are also in stm32f4xx_it.c - ensure only one definition */
