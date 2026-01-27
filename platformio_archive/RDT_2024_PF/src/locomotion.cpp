/**
 * @file locomotion.cpp
 * @brief Locomotion control implementation
 * 
 * Motor direction mapping matches 2024_teensy_original implementation.
 * Uses consistent order: Front Left, Front Right, Rear Left, Rear Right
 */

#include "locomotion.h"
#include "can_driver.h"
#include "config.h"

void Locomotion_Stop(void) {
    // Consistent order: Front Left, Front Right, Rear Left, Rear Right
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, 0.0f);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, 0.0f);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, 0.0f);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, 0.0f);
    Serial.println("Locomotion Stopped");
}

void Locomotion_Forward(float speedFactor) {
    float speed = speedFactor * LOCOMOTION_DUTY_CYCLE;
    
    // Motor directions for forward motion
    // Consistent order: Front Left, Front Right, Rear Left, Rear Right
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, speed);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, -speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, speed);
    
    Serial.println("Moving Forward");
}

void Locomotion_Backward(float speedFactor) {
    float speed = speedFactor * LOCOMOTION_DUTY_CYCLE;
    
    // Reverse of forward
    // Consistent order: Front Left, Front Right, Rear Left, Rear Right
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, -speed);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, -speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, -speed);
    
    Serial.println("Moving Backward");
}

void Locomotion_TurnLeft(float speedFactor) {
    float speed = speedFactor * LOCOMOTION_DUTY_CYCLE;
    
    // Tank turn left
    // Consistent order: Front Left, Front Right, Rear Left, Rear Right
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, speed);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, -speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, -speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, -speed);
    
    Serial.println("Turning Right");  // Note: matches original code's print
}

void Locomotion_TurnRight(float speedFactor) {
    float speed = speedFactor * LOCOMOTION_DUTY_CYCLE;
    
    // Tank turn right
    // Consistent order: Front Left, Front Right, Rear Left, Rear Right
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, -speed);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, speed);
    
    Serial.println("Turning Left");  // Note: matches original code's print
}
