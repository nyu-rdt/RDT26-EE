#include <Wire.h>
#include <FlexCAN_T4.h>
#include "child.h"
#include "config.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

#define TEST_MOTOR_CAN_ID 0x4c
#define CAN_MESSAGE_INTERVAL 50

static uint8_t pendingAction = 0;
static uint8_t pendingSpeed  = 0;
static bool    awaitingSpeed = false;  // true after action byte received
static float   currentSpeed  = 0.0;
static unsigned long lastSendTime = 0;

CAN_message_t craftMessage(float val, uint8_t id) {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id = id;
    msg.len = 4;
    int32_t value = (int32_t)(val * 100000);
    msg.buf[0] = (value >> 24) & 0xFF;
    msg.buf[1] = (value >> 16) & 0xFF;
    msg.buf[2] = (value >> 8)  & 0xFF;
    msg.buf[3] = value & 0xFF;
    return msg;
}

void processCommand(uint8_t action, uint8_t speed) {
    float spd = speed / 100.0;  // convert 0-100 to 0.0-1.0

    switch (action) {
        case CMD_FORWARD:
            currentSpeed = spd;
            Serial.print("FORWARD at ");
            break;
        case CMD_BACKWARD:
            currentSpeed = -spd;
            Serial.print("BACKWARD at ");
            break;
        case CMD_STOP:
            currentSpeed = 0.0;
            Serial.print("STOP at ");
            break;
        case CMD_TURN_LEFT:
            currentSpeed = spd * 0.5;  // half speed for turn
            Serial.print("TURN LEFT at ");
            break;
        default:
            Serial.println("Unknown command");
            return;
    }

    Serial.println(currentSpeed);
}

void receiveEvent(int numBytes) {
    while (Wire.available()) {
        uint8_t value = Wire.read();

        if (!awaitingSpeed) {
            // first byte is the action
            pendingAction = value;
            awaitingSpeed = true;
            Serial.print("Action received: ");
            Serial.println(value);
        } else {
            // second byte is the speed
            pendingSpeed  = value;
            awaitingSpeed = false;
            Serial.print("Speed received: ");
            Serial.println(value);
            processCommand(pendingAction, pendingSpeed);
        }
    }
}

void i2c_child_init() {
    Wire.begin(I2C_CHILD_ADDRESS);
    Wire.onReceive(receiveEvent);

    can1.begin();
    can1.setBaudRate(500000);

    Serial.println("Child ready");
}

void i2c_child_update() {
    unsigned long now = millis();
    if (now - lastSendTime >= CAN_MESSAGE_INTERVAL) {
        lastSendTime = now;
        CAN_message_t msg = craftMessage(currentSpeed, TEST_MOTOR_CAN_ID);
        int result = can1.write(msg);
        Serial.print("CAN TX [");
        Serial.print(result == 1 ? "OK" : "FAIL");
        Serial.print("] speed=");
        Serial.println(currentSpeed);
    }
}