#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"

void APP_Init();

void setup() {
    APP_Init();
    vTaskStartScheduler();
    Serial.println("ERROR: scheduler exited — halt");
    while (true) {}
}

void loop() {}
