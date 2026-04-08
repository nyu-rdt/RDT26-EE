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

// Sends GRP_DATA command then reads `len` bytes from child into buf.
// Returns number of bytes actually received.
uint8_t i2c_parent_requestData(uint8_t* buf, uint8_t len) {
    i2c_parent_sendCommand(GRP_DATA, 0);
    uint8_t count = Wire.requestFrom(I2C_CHILD_ADDRESS, (uint8_t)len);
    for (uint8_t i = 0; i < count; i++) {
        buf[i] = Wire.read();
    }
    return count;
}