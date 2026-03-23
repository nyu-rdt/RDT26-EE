#include <Arduino.h>
#include "parent.h"
#include "config.h"

void sendCommand(uint8_t action, uint8_t speed) {
    i2c_parent_sendByte(action);
    delay(10);
    i2c_parent_sendByte(speed);
}

void setup() {
    Serial.begin(9600);
    i2c_parent_init();
    delay(1000);
}

void loop() {
    Serial.println("Sending: FORWARD 50%");
    sendCommand(CMD_FORWARD, SPEED_50);
    delay(3000);

    Serial.println("Sending: TURN LEFT 25%");
    sendCommand(CMD_TURN_LEFT, SPEED_25);
    delay(3000);

    Serial.println("Sending: BACKWARD 75%");
    sendCommand(CMD_BACKWARD, SPEED_75);
    delay(3000);

    Serial.println("Sending: STOP");
    sendCommand(CMD_STOP, 0);
    delay(3000);
}