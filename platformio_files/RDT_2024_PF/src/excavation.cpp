/**
 * @file excavation.cpp
 * @brief Excavation and deposition system implementation
 */

#include "excavation.h"
#include "can_driver.h"
#include "sensors.h"
#include "system.h"
#include "i2c_commands.h"
#include <Servo.h>
#include <Wire.h>

// PWM servo object for arm only (belt uses CAN now)
static Servo excavationArmServo;

// Position state
static ExcavationPosition_t currentPosition = POSITION_UNKNOWN;
static float locomotionThreshold = DEFAULT_LOCOMOTION_THRESHOLD;
static float excavationThreshold = DEFAULT_EXCAVATION_THRESHOLD;

// Active motor speeds for command refresh
static float activeBeltSpeed = 0.0f;
static float activeDepositionSpeed = 0.0f;

void Excavation_Init(void) {
    excavationArmServo.attach(EXCAVATION_SYSTEM_PWM_PIN);
    
    // Start in neutral
    excavationArmServo.writeMicroseconds(PWM_NEUTRAL);
    
    Serial.println("Excavation System Initialized");
}

void Excavation_Stop(void) {
    excavationArmServo.writeMicroseconds(PWM_NEUTRAL);
    Serial.println("Excavation Arm Stopped");
}

void Excavation_Up(void) {
    excavationArmServo.writeMicroseconds(PWM_EXCAVATION_UP);
    Serial.println("Excavation Arm Moving Up");
}

void Excavation_Down(void) {
    excavationArmServo.writeMicroseconds(PWM_EXCAVATION_DOWN);
    Serial.println("Excavation Arm Moving Down");
}

void Excavation_BeltStop(void) {
    CAN_SendMotorSpeed(CAN_ID_EXCAVATION_BELT, 0.0f);
    activeBeltSpeed = 0.0f;
    Serial.println("Excavation belt stopped");
}

void Excavation_BeltOutward(void) {
    float speed = -EXCAVATION_DUTY_CYCLE;
    CAN_SendMotorSpeed(CAN_ID_EXCAVATION_BELT, speed);
    activeBeltSpeed = speed;
    Serial.println("Excavation belt moving outward");
}

void Excavation_BeltInward(void) {
    float speed = EXCAVATION_DUTY_CYCLE;
    CAN_SendMotorSpeed(CAN_ID_EXCAVATION_BELT, speed);
    activeBeltSpeed = speed;
    Serial.println("Excavation belt moving inward");
}

void Excavation_Zero(void) {
    // Note: Currently used as excavation stop command
    Excavation_Stop();
    Serial.println("Excavation Stopped (Zero command)");
}

void Excavation_SetPosition(ExcavationPosition_t position) {
    currentPosition = position;
}

bool Excavation_MoveToLocomotionPosition(void) {
    if (currentPosition == POSITION_LOCOMOTION) {
        return true;  // Already there
    }
    
    float length = Sensors_GetStringLength();
    
    if (length >= locomotionThreshold) {
        Serial.println("Already at or beyond locomotion position threshold");
        Excavation_Stop();
        currentPosition = POSITION_LOCOMOTION;
        return true;
    }
    
    Excavation_Up();
    unsigned long lastCheckTime = 0;
    unsigned long lastRefreshTime = 0;
    
    while (length < locomotionThreshold) {
        unsigned long currentTime = millis();
        
        // Check for E-Stop activation
        if (digitalRead(RELAY_PIN) == LOW) {
            System_SetEStopEngaged(true);
            System_EmergencyStop();
            Serial.println("E-STOP ACTIVATED during locomotion positioning - Operation aborted");
            currentPosition = POSITION_UNKNOWN;
            return false;
        }
        
        // Check for incoming commands (non-blocking)
        if (Wire.available()) {
            char c = Wire.read();
            if (int(c) == CMD_EXCAVATION_ZERO) {
                Excavation_Stop();
                currentPosition = POSITION_UNKNOWN;
                return false;
            }
            else if (int(c) == CMD_EXCAVATION_POSITION) {
                Excavation_Stop();
                currentPosition = POSITION_UNKNOWN;
                Serial.println("Locomotion positioning interrupted by excavation position command");
                return false;
            }
            I2C_ProcessCommand(int(c));
        }
        
        // Check position at intervals (every 150ms)
        if (currentTime - lastCheckTime >= 150) {
            length = Sensors_GetStringLength();
            lastCheckTime = currentTime;
        }
        
        // Refresh motor commands periodically
        if (currentTime - lastRefreshTime >= COMMAND_REFRESH_INTERVAL) {
            if (!System_IsEStopEngaged()) {
                Excavation_RefreshCommands();
            }
            lastRefreshTime = currentTime;
        }
        
        yield();
    }
    
    Excavation_Stop();
    currentPosition = POSITION_LOCOMOTION;
    Serial.println("Successfully reached locomotion position");
    return true;
}

bool Excavation_MoveToExcavationPosition(void) {
    if (currentPosition == POSITION_EXCAVATION) {
        return true;  // Already there
    }
    
    float length = Sensors_GetStringLength();
    
    if (length <= excavationThreshold) {
        Serial.println("Already at or beyond excavation position threshold");
        Excavation_Stop();
        currentPosition = POSITION_EXCAVATION;
        return true;
    }
    
    Excavation_Down();
    unsigned long lastCheckTime = 0;
    unsigned long lastRefreshTime = 0;
    
    while (length > excavationThreshold) {
        unsigned long currentTime = millis();
        
        // Check for E-Stop activation
        if (digitalRead(RELAY_PIN) == LOW) {
            System_SetEStopEngaged(true);
            System_EmergencyStop();
            Serial.println("E-STOP ACTIVATED during excavation positioning - Operation aborted");
            currentPosition = POSITION_UNKNOWN;
            return false;
        }
        
        // Check for incoming commands (non-blocking)
        if (Wire.available()) {
            char c = Wire.read();
            if (int(c) == CMD_EXCAVATION_ZERO) {
                Excavation_Stop();
                currentPosition = POSITION_UNKNOWN;
                return false;
            }
            else if (int(c) == CMD_EXCAVATION_LOCOMOTION_POS) {
                Excavation_Stop();
                currentPosition = POSITION_UNKNOWN;
                Serial.println("Excavation positioning interrupted by locomotion position command");
                return false;
            }
            I2C_ProcessCommand(int(c));
        }
        
        // Check position at intervals (every 150ms)
        if (currentTime - lastCheckTime >= 150) {
            length = Sensors_GetStringLength();
            lastCheckTime = currentTime;
        }
        
        // Refresh motor commands periodically
        if (currentTime - lastRefreshTime >= COMMAND_REFRESH_INTERVAL) {
            if (!System_IsEStopEngaged()) {
                Excavation_RefreshCommands();
            }
            lastRefreshTime = currentTime;
        }
        
        yield();
    }
    
    Excavation_Stop();
    currentPosition = POSITION_EXCAVATION;
    Serial.println("Successfully reached excavation position");
    return true;
}

ExcavationPosition_t Excavation_GetPosition(void) {
    return currentPosition;
}

float Excavation_GetActiveBeltSpeed(void) {
    return activeBeltSpeed;
}

float Excavation_GetActiveDepositionSpeed(void) {
    return activeDepositionSpeed;
}

void Excavation_RefreshCommands(void) {
    if (activeBeltSpeed != 0.0f) {
        CAN_SendMotorSpeed(CAN_ID_EXCAVATION_BELT, activeBeltSpeed);
    }
    if (activeDepositionSpeed != 0.0f) {
        CAN_SendMotorSpeed(CAN_ID_DEPOSITION, activeDepositionSpeed);
    }
}

void Deposition_RotateCollection(void) {
    float speed = DEPOSITION_DUTY_CYCLE;
    CAN_SendMotorSpeed(CAN_ID_DEPOSITION, speed);
    activeDepositionSpeed = speed;
    Serial.println("Rotating to Collection Position");
}

void Deposition_RotateDumping(void) {
    float speed = -DEPOSITION_DUTY_CYCLE;
    CAN_SendMotorSpeed(CAN_ID_DEPOSITION, speed);
    activeDepositionSpeed = speed;
    Serial.println("Rotating to Dumping Position");
}

void Deposition_Stop(void) {
    CAN_SendMotorSpeed(CAN_ID_DEPOSITION, 0.0f);
    activeDepositionSpeed = 0.0f;
    Serial.println("Deposition Rotation Stopped");
}
