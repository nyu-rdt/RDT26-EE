#include <Arduino.h>
#include <Wire.h>
#include "config.h"

void sendCommand(uint8_t cmd)
{
    Wire.beginTransmission(I2C_SLAVE_ADDRESS);
    Wire.write(cmd);
    Wire.endTransmission();
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(); // master
}

void loop()
{
    sendCommand(CMD_FORWARD);
    delay(3000);

    sendCommand(CMD_STOP);
    delay(1000);

    sendCommand(CMD_BACKWARD);
    delay(3000);

    sendCommand(CMD_STOP);
    delay(1000);
}