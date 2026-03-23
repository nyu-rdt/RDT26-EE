#include <Arduino.h>
#include "child.h"

void setup() {
    Serial.begin(9600);
    i2c_child_init();
}

void loop() {
    i2c_child_update();  // keeps sending CAN at 50ms intervals
}