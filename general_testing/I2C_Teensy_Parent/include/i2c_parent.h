#ifndef I2C_PARENT_H
#define I2C_PARENT_H

#include <Arduino.h>

void i2c_parent_init();
void i2c_parent_sendByte(uint8_t data);

#endif
