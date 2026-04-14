#include <Arduino.h>
#include "config.h"
#include "string_pot.h"

static bool moving = false;
static int STRINGPOT_state = STRING_MIDDLE;

static constexpr float potScale = 27.0f;
static constexpr float potOffset = 0.719f;

static float cachedDistance = 0.0f;

static float minDistance = 0.0f;
static constexpr float maxDistance = (potScale * 3.3f) - potOffset; //88.381
static float maxRawReading = 1023.0f;

void STRINGPOT_Init()
{
    analogReadResolution(10);
    pinMode(STRING_POT_PIN, INPUT);
}

float STRINGPOT_ReadDistance()
{
    int raw = analogRead(STRING_POT_PIN); 

    // convert to voltage
    float voltage = raw * (3.3f / maxRawReading); 

    // convert to distance using calibration
    cachedDistance = potScale * voltage - potOffset;
    cachedDistance = constrain(cachedDistance, minDistance, maxDistance);

    return cachedDistance;
}

void STRINGPOT_SetMoving(bool isMoving) {
    moving = isMoving;
}

void STRINGPOT_UpdateState(){
    // test to see if these thresholds make sense
    STRINGPOT_state = moving ? STRING_MOVING :
                      (cachedDistance > 1013.0f) ? STRING_HIGHEST :
                      (cachedDistance < 10.0f) ? STRING_LOWEST :
                      STRING_MIDDLE;                       
}


