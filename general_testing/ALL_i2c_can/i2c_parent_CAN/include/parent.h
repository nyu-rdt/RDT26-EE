#pragma once
#include <Arduino.h>

void i2c_parent_init();
void i2c_parent_sendCommand(uint8_t group, uint8_t param);
void i2c_parent_sendByte(uint8_t data);
uint8_t i2c_parent_requestData(uint8_t* buf, uint8_t len);