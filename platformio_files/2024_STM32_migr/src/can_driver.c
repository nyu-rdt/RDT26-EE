/**
 * CAN Bus Driver Implementation
 * Rover Control System - STM32F446RE
 * 
 * Uses STM32 HAL CAN for bxCAN peripheral
 * 500kbps, extended IDs supported
 */

#include "can_driver.h"
#include "pins.h"
#include "commands.h"

static CAN_HandleTypeDef hcan1;
static CAN_TxHeaderTypeDef txHeader;
static uint32_t txMailbox;

void CAN_Driver_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure CAN GPIO pins */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = CAN_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = CAN_AF;
    HAL_GPIO_Init(CAN_TX_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = CAN_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAN_RX_PORT, &GPIO_InitStruct);
    
    /* Configure CAN peripheral */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 9;  /* 180MHz / 9 = 20MHz time quantum */
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
    /* Baud = 20MHz / (1 + 13 + 6) = 20MHz / 20 = 1MHz ... wait */
    /* For 500kbps: 20MHz / 40 = 500kbps, so prescaler should give us that */
    /* Actually: APB1 = 45MHz typically on Nucleo */
    /* Let's use prescaler=5, BS1=6TQ, BS2=2TQ -> 45/(5*(1+6+2)) = 45/45 = 1MHz... */
    /* For 500kbps with 45MHz APB1: prescaler=5, seg1=6, seg2=2 -> 45/(5*9)=1MHz NO */
    /* prescaler=9, seg1=6, seg2=2 -> 45/(9*9)=555kHz close */
    /* prescaler=9, seg1=7, seg2=2 -> 45/(9*10)=500kHz YES */
    hcan1.Init.Prescaler = 9;
    hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    HAL_CAN_Init(&hcan1);
    
    /* Configure filter to accept all messages (for potential RX) */
    CAN_FilterTypeDef filterConfig;
    filterConfig.FilterBank = 0;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterIdHigh = 0x0000;
    filterConfig.FilterIdLow = 0x0000;
    filterConfig.FilterMaskIdHigh = 0x0000;
    filterConfig.FilterMaskIdLow = 0x0000;
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation = ENABLE;
    filterConfig.SlaveStartFilterBank = 14;
    
    HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
    
    /* Start CAN */
    HAL_CAN_Start(&hcan1);
}

int CAN_Driver_Send(CAN_Msg_t *msg)
{
    if (msg->extended) {
        txHeader.IDE = CAN_ID_EXT;
        txHeader.ExtId = msg->id;
    } else {
        txHeader.IDE = CAN_ID_STD;
        txHeader.StdId = msg->id;
    }
    
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = msg->len;
    txHeader.TransmitGlobalTime = DISABLE;
    
    /* Wait for free mailbox with timeout */
    uint32_t startTick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if ((HAL_GetTick() - startTick) > 10) {
            return -1;  /* Timeout */
        }
    }
    
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, msg->data, &txMailbox) != HAL_OK) {
        return -1;
    }
    
    return 0;
}

CAN_Msg_t CAN_CraftMotorMessage(int msgType, float value, uint8_t motorId)
{
    CAN_Msg_t msg;
    
    msg.extended = true;
    msg.id = ((uint32_t)msgType << 8) | motorId;
    msg.len = 4;
    
    /* Convert float to fixed-point (same as original: val * 100000) */
    int32_t fixedVal = (int32_t)(value * 100000.0f);
    
    msg.data[0] = (fixedVal >> 24) & 0xFF;
    msg.data[1] = (fixedVal >> 16) & 0xFF;
    msg.data[2] = (fixedVal >> 8) & 0xFF;
    msg.data[3] = fixedVal & 0xFF;
    
    return msg;
}

int CAN_SendMotorCommand(uint8_t motorId, float speedPercent)
{
    CAN_Msg_t msg = CAN_CraftMotorMessage(0, speedPercent, motorId);
    return CAN_Driver_Send(&msg);
}

CAN_HandleTypeDef* CAN_GetHandle(void)
{
    return &hcan1;
}
