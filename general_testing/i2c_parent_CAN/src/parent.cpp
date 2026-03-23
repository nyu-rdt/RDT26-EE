#include <Wire.h>
#include "parent.h"
#include "config.h"

void i2c_parent_init() {
    Wire.begin();
}

void i2c_parent_sendCommand(uint8_t group, uint8_t param) {
    i2c_parent_sendByte(BUILD_I2C_CMD(group, param));
}

void i2c_parent_sendByte(uint8_t data) {
    Wire.beginTransmission(I2C_CHILD_ADDRESS);
    Wire.write(data);
    Wire.endTransmission();
}