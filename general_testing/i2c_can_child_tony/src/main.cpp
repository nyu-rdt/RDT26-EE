#include <Arduino.h>
#include "child.h"

void setup() {
    Serial.begin(115200);
    child_init();
}

void loop() {
    child_update();
}