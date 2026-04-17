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

// Returns the last computed distance without triggering a new ADC read.
// Use this from ISR context (e.g. requestEvent) — safe to call anytime.
float STRINGPOT_GetCachedDistance()
{
    return cachedDistance;
}

int STRINGPOT_GetState() {
    return STRINGPOT_state;
}

void STRINGPOT_SetMoving(bool isMoving) {
    moving = isMoving;
}

void STRINGPOT_UpdateState(){
    // Check thresholds first — a limit takes priority over the moving flag
    // so the belt can't mask an overtravel condition by still being in motion
    if (cachedDistance > STRING_POT_HIGHEST_THRESHOLD)
        STRINGPOT_state = STRING_HIGHEST;
    else if (cachedDistance < STRING_POT_LOWEST_THRESHOLD)
        STRINGPOT_state = STRING_LOWEST;
    else
        STRINGPOT_state = moving ? STRING_MOVING : STRING_MIDDLE;
}
