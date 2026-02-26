#include <Arduino.h>
#include "i2c_parent.h"

void setup()
{
    Serial.begin(9600);
    i2c_parent_init();
}

void loop()
{
    static uint8_t counter = 0;

    i2c_parent_sendByte(counter);

    Serial.print("Sent: ");
    Serial.println(counter);

    counter++;
    delay(1000);
}
