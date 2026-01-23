/**
 * @file excavation.cpp
 * @brief Excavation and deposition system implementation
 */

#include "excavation.h"
#include "can_driver.h"
#include "sensors.h"
#include <Servo.h>

// PWM servo objects
static Servo excavationBeltServo;
static Servo excavationArmServo;

// Position state
static ExcavationPosition_t currentPosition = POSITION_UNKNOWN;
static float locomotionThreshold = DEFAULT_LOCOMOTION_THRESHOLD;
static float excavationThreshold = DEFAULT_EXCAVATION_THRESHOLD;

void Excavation_Init(void) {
    excavationBeltServo.attach(EXCAVATION_BELT_PWM_PIN);
    excavationArmServo.attach(EXCAVATION_SYSTEM_PWM_PIN);
    
    // Start in neutral
    excavationBeltServo.writeMicroseconds(PWM_NEUTRAL);
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
    excavationBeltServo.writeMicroseconds(PWM_NEUTRAL);
    Serial.println("Belt Stopped");
}

void Excavation_BeltOutward(void) {
    excavationBeltServo.writeMicroseconds(PWM_BELT_REVERSE);
    Serial.println("Belt Moving Outward");
}

void Excavation_BeltInward(void) {
    excavationBeltServo.writeMicroseconds(PWM_BELT_FORWARD);
    Serial.println("Belt Moving Inward");
}

void Excavation_Zero(void) {
    locomotionThreshold = Sensors_GetStringLength();
    currentPosition = POSITION_LOCOMOTION;
    Serial.print("Excavation Zeroed at ");
    Serial.print(locomotionThreshold);
    Serial.println(" inches");
}

bool Excavation_MoveToLocomotionPosition(void) {
    if (currentPosition == POSITION_LOCOMOTION) {
        return true;  // Already there
    }
    
    Excavation_Down();
    unsigned long startTime = millis();
    float length;
    
    while (true) {
        length = Sensors_GetStringLength();
        
        if (length <= locomotionThreshold) {
            Excavation_Stop();
            currentPosition = POSITION_LOCOMOTION;
            Serial.println("Reached Locomotion Position");
            return true;
        }
        
        if (millis() - startTime > POSITION_MOVE_TIMEOUT_MS) {
            Excavation_Stop();
            Serial.println("ERROR: Locomotion position timeout");
            return false;
        }
        
        delay(50);  // Check every 50ms
    }
}

bool Excavation_MoveToExcavationPosition(void) {
    if (currentPosition == POSITION_EXCAVATION) {
        return true;  // Already there
    }
    
    Excavation_Up();
    unsigned long startTime = millis();
    float length;
    
    while (true) {
        length = Sensors_GetStringLength();
        
        if (length >= excavationThreshold) {
            Excavation_Stop();
            currentPosition = POSITION_EXCAVATION;
            Serial.println("Reached Excavation Position");
            return true;
        }
        
        if (millis() - startTime > POSITION_MOVE_TIMEOUT_MS) {
            Excavation_Stop();
            Serial.println("ERROR: Excavation position timeout");
            return false;
        }
        
        delay(50);  // Check every 50ms
    }
}

ExcavationPosition_t Excavation_GetPosition(void) {
    return currentPosition;
}

void Deposition_RotateCollection(void) {
    CAN_SendMotorSpeed(CAN_ID_DEPOSITION, DEPOSITION_DUTY_CYCLE);
    Serial.println("Rotating to Collection Position");
}

void Deposition_RotateDumping(void) {
    CAN_SendMotorSpeed(CAN_ID_DEPOSITION, -DEPOSITION_DUTY_CYCLE);
    Serial.println("Rotating to Dumping Position");
}

void Deposition_Stop(void) {
    CAN_SendMotorSpeed(CAN_ID_DEPOSITION, 0.0f);
    Serial.println("Deposition Rotation Stopped");
}
