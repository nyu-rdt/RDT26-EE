/**
 * @file system.cpp
 * @brief System-level functions implementation
 */

#include "system.h"
#include "config.h"
#include "locomotion.h"
#include "excavation.h"
#include "sensors.h"

// E-Stop state
static bool eStopEngaged = true;  // Start engaged until relay confirms power
static bool currentRelayStatus = false;

// Cached weight for non-blocking reads
static float cachedWeight = 0.0f;
static unsigned long nextWeightReadTime = 0;

void System_Init(void) {
    // Initialize E-Stop relay pin
    pinMode(RELAY_PIN, INPUT_PULLUP);
    currentRelayStatus = digitalRead(RELAY_PIN);
    
    Serial.println("System initialized (E-Stop relay configured)");
}

bool System_CheckEStop(void) {
    currentRelayStatus = digitalRead(RELAY_PIN);
    
    // E-stop activated (LOW when relay is OFF/E-stop engaged)
    if (currentRelayStatus == LOW) {
        if (!eStopEngaged) {
            System_EmergencyStop();
            eStopEngaged = true;
            Serial.println("E-STOP ACTIVATED - Motors stopped");
        }
        return true;
    }
    
    // Power restored
    if (eStopEngaged) {
        eStopEngaged = false;
        Serial.println("Power restored - Ready to accept commands");
    }
    
    return false;
}

bool System_IsEStopEngaged(void) {
    return eStopEngaged;
}

void System_SetEStopEngaged(bool engaged) {
    eStopEngaged = engaged;
}

void System_EmergencyStop(void) {
    // Order of operations: 1. Locomotion 2. Excavation 3. Belt 4. Deposition
    Locomotion_Stop();
    Excavation_Stop();
    Excavation_BeltStop();
    Deposition_Stop();
    Serial.println("Emergency Stop Activated");
}

void System_Update(void) {
    unsigned long currentMillis = millis();
    
    // Non-blocking weight reading (every 500ms)
    if (currentMillis >= nextWeightReadTime) {
        cachedWeight = Sensors_GetWeight();
        nextWeightReadTime = currentMillis + 500;
    }
}

float System_GetCachedWeight(void) {
    return cachedWeight;
}
