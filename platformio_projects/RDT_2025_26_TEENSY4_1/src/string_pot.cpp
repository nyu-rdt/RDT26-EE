#include <Arduino.h>
#include "config.h"
#include "string_pot.h"

static float potScale = 27.0f;
static float potOffset = 0.719f;

static float cachedDistance = 0.0f;

static float minDistance = 0.0f;
static float maxDistance = 1023.0f;  

void STRINGPOT_Init()
{
    analogReadResolution(10);
    pinMode(STRING_POT_PIN, INPUT);
}

float STRINGPOT_ReadDistance()
{
    int raw = analogRead(STRING_POT_PIN); 

    // convert to voltage
    float voltage = raw * (3.3f / maxDistance); 

    // convert to distance using calibration
    cachedDistance = potScale * voltage - potOffset; // at this point the largest number using the set variables is 88.381

    return cachedDistance;
}



