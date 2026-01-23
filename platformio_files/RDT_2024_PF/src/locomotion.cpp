/**
 * @file locomotion.cpp
 * @brief Locomotion control implementation
 */

#include "locomotion.h"
#include "can_driver.h"
#include "config.h"

void Locomotion_Stop(void) {
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, 0.0f);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, 0.0f);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, 0.0f);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, 0.0f);
    Serial.println("Locomotion Stopped");
}

void Locomotion_Forward(float speedFactor) {
    float speed = speedFactor * LOCOMOTION_DUTY_CYCLE;
    
    // Motor directions for forward motion
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, -speed);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, -speed);
    
    Serial.print("Moving Forward at ");
    Serial.print(speedFactor * 100);
    Serial.println("%");
}

void Locomotion_Backward(float speedFactor) {
    float speed = speedFactor * LOCOMOTION_DUTY_CYCLE;
    
    // Reverse of forward
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, speed);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, -speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, -speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, speed);
    
    Serial.print("Moving Backward at ");
    Serial.print(speedFactor * 100);
    Serial.println("%");
}

void Locomotion_TurnLeft(float speedFactor) {
    float speed = speedFactor * LOCOMOTION_DUTY_CYCLE;
    
    // Tank turn: left side backward, right side forward
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, -speed);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, -speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, speed);
    
    Serial.print("Turning Left at ");
    Serial.print(speedFactor * 100);
    Serial.println("%");
}

void Locomotion_TurnRight(float speedFactor) {
    float speed = speedFactor * LOCOMOTION_DUTY_CYCLE;
    
    // Tank turn: right side backward, left side forward
    CAN_SendMotorSpeed(CAN_ID_FRONT_LEFT, speed);
    CAN_SendMotorSpeed(CAN_ID_FRONT_RIGHT, speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_LEFT, -speed);
    CAN_SendMotorSpeed(CAN_ID_REAR_RIGHT, -speed);
    
    Serial.print("Turning Right at ");
    Serial.print(speedFactor * 100);
    Serial.println("%");
}
