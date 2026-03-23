#include <Arduino.h>
#include "i2c_child.h"

void setup()
{
    Serial.begin(9600);
    i2c_child_init();
}

void loop()
{
    // child waits for data
}
