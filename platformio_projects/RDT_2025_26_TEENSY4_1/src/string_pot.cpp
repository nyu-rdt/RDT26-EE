#include <Arduino.h>
#include "config.h"
#include "string_pot.h"

static bool moving = false;
static int STRINGPOT_state = STRING_MIDDLE;

static float cachedDistance = 0.0f;
static int lastRaw = 0;

void STRINGPOT_Init()
{
    analogReadResolution(10);
    pinMode(STRING_POT_PIN, INPUT);
}

float STRINGPOT_ReadDistance()
{
    lastRaw = analogRead(STRING_POT_PIN);

    // convert to voltage, then to distance using calibration from config.h
    float voltage = lastRaw * (3.3f / STRING_POT_MAX_RAW);
    cachedDistance = STRING_POT_SCALE * voltage - STRING_POT_OFFSET;

    return cachedDistance;
}

int STRINGPOT_GetLastRaw()
{
    return lastRaw;
}

void STRINGPOT_SetMoving(bool isMoving) {
    moving = isMoving;
}

void STRINGPOT_UpdateState(){
    STRINGPOT_state = moving                                        ? STRING_MOVING  :
                      (cachedDistance > STRING_POT_HIGHEST_THRESHOLD) ? STRING_HIGHEST :
                      (cachedDistance < STRING_POT_LOWEST_THRESHOLD)  ? STRING_LOWEST  :
                      STRING_MIDDLE;
}
