/**
 * @file can_driver.cpp
 * @brief CAN bus communication driver implementation
 */

#include "can_driver.h"

// CAN bus instance (Teensy FlexCAN)
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void CAN_Init(void) {
    can1.begin();
    can1.setBaudRate(CAN_BAUD_RATE);
    Serial.println("CAN Bus Initialized");
}

CAN_message_t CAN_CraftMessage(int msgType, float value, uint8_t id) {
    CAN_message_t msg;
    msg.flags.extended = 1;  // Extended frame
    msg.id = (msgType << 8) + id;
    msg.len = 4;
    
    // Convert float to fixed-point (5 decimal places)
    int32_t fixedValue = (int32_t)(value * 100000.0f);
    
    // Big-endian encoding
    msg.buf[0] = (fixedValue >> 24) & 0xFF;
    msg.buf[1] = (fixedValue >> 16) & 0xFF;
    msg.buf[2] = (fixedValue >> 8) & 0xFF;
    msg.buf[3] = fixedValue & 0xFF;
    
    return msg;
}

int CAN_SendMotorSpeed(uint32_t canId, float speedPercent) {
    CAN_message_t msg = CAN_CraftMessage(0, speedPercent, (uint8_t)canId);
    return can1.write(msg);
}
