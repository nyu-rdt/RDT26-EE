#include <Wire.h>
#include "i2c_child.h"
#include "config.h"

void receiveEvent(int numBytes)
{
    while (Wire.available())
    {
        uint8_t value = Wire.read();
        Serial.print("Received: ");
        Serial.println(value);
    }
}

void i2c_child_init()
{
    Wire.begin(I2C_CHILD_ADDRESS);   // start as child
    Wire.onReceive(receiveEvent);
}
