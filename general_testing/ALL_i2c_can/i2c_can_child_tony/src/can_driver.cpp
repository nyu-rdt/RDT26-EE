#include <FlexCAN_T4.h>
#include "can_driver.h"
#include "config.h"

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void CAN_Init() {
    can1.begin();
    can1.setBaudRate(CAN_BAUD_RATE);
}

void CAN_SendMotorSpeed(uint32_t canId, float speed) {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id = canId;
    msg.len = 4;
    
    int32_t value = (int32_t)(speed * 100000.0f);
    msg.buf[0] = (value >> 24) & 0xFF;
    msg.buf[1] = (value >> 16) & 0xFF;
    msg.buf[2] = (value >> 8) & 0xFF;
    msg.buf[3] = value & 0xFF;
    
    can1.write(msg);
}

void CAN_SendLocomotion(float left, float right) {
    CAN_SendMotorSpeed(CAN_ID_LEFT_MOTOR, left);
    CAN_SendMotorSpeed(CAN_ID_RIGHT_MOTOR, right);
}
