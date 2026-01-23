/**
 * @file i2c_commands.cpp
 * @brief I2C slave command processing implementation
 */

#include "i2c_commands.h"
#include "config.h"
#include "locomotion.h"
#include "excavation.h"
#include "sensors.h"
#include <Wire.h>

// State variables
static int lastCommand = 0;
static bool isAutonomousMode = false;
static uint8_t dataPacket[3] = {POSITION_UNKNOWN, 0, 0};

// Forward declarations for I2C callbacks
static void onReceiveCallback(int numBytes);
static void onRequestCallback(void);

void I2C_Init(void) {
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceiveCallback);
    Wire.onRequest(onRequestCallback);
    Serial.println("I2C Slave Initialized");
}

static void onReceiveCallback(int numBytes) {
    while (Wire.available()) {
        int cmd = Wire.read();
        Serial.print("I2C Received: ");
        Serial.println(cmd);
        I2C_ProcessCommand(cmd);
        lastCommand = (cmd == CMD_EMERGENCY_STOP) ? 1000 : cmd;
    }
}

static void onRequestCallback(void) {
    // Update data packet before sending
    float length = Sensors_GetCachedLength();
    float weight = Sensors_GetCachedWeight();
    
    dataPacket[0] = (uint8_t)Excavation_GetPosition();
    dataPacket[1] = (uint8_t)((int)length >> 8);
    dataPacket[2] = (uint8_t)((int)weight & 0xFF);
    
    Wire.write(dataPacket, 3);
}

void I2C_ProcessCommand(int command) {
    switch (command) {
        // Emergency & Stop
        case CMD_EMERGENCY_STOP:
            EmergencyStop();
            break;
        case CMD_LOCOMOTION_STOP:
            Locomotion_Stop();
            break;
            
        // Forward Movement
        case CMD_FORWARD_25:
            Locomotion_Forward(0.25f);
            break;
        case CMD_FORWARD_50:
            Locomotion_Forward(0.50f);
            break;
        case CMD_FORWARD_75:
            Locomotion_Forward(0.75f);
            break;
        case CMD_FORWARD_100:
            Locomotion_Forward(1.00f);
            break;
            
        // Backward Movement
        case CMD_BACKWARD_25:
            Locomotion_Backward(0.25f);
            break;
        case CMD_BACKWARD_50:
            Locomotion_Backward(0.50f);
            break;
        case CMD_BACKWARD_75:
            Locomotion_Backward(0.75f);
            break;
        case CMD_BACKWARD_100:
            Locomotion_Backward(1.00f);
            break;
            
        // Turn Left
        case CMD_LEFT_25:
            Locomotion_TurnLeft(0.25f);
            break;
        case CMD_LEFT_50:
            Locomotion_TurnLeft(0.50f);
            break;
        case CMD_LEFT_75:
            Locomotion_TurnLeft(0.75f);
            break;
        case CMD_LEFT_100:
            Locomotion_TurnLeft(1.00f);
            break;
            
        // Turn Right
        case CMD_RIGHT_25:
            Locomotion_TurnRight(0.25f);
            break;
        case CMD_RIGHT_50:
            Locomotion_TurnRight(0.50f);
            break;
        case CMD_RIGHT_75:
            Locomotion_TurnRight(0.75f);
            break;
        case CMD_RIGHT_100:
            Locomotion_TurnRight(1.00f);
            break;
            
        // Excavation System
        case CMD_EXCAVATION_ZERO:
            Excavation_Zero();
            break;
        case CMD_EXCAVATION_LOCOMOTION_POS:
            Excavation_MoveToLocomotionPosition();
            break;
        case CMD_EXCAVATION_POSITION:
            Excavation_MoveToExcavationPosition();
            break;
        case CMD_BELT_STOP:
            Excavation_BeltStop();
            break;
        case CMD_BELT_OUTWARD:
            Excavation_BeltOutward();
            break;
        case CMD_BELT_INWARD:
            Excavation_BeltInward();
            break;
            
        // Deposition System
        case CMD_DEPOSITION_ROTATE_COLLECTION:
            Deposition_RotateCollection();
            break;
        case CMD_DEPOSITION_ROTATE_DUMPING:
            Deposition_RotateDumping();
            break;
        case CMD_DEPOSITION_ROTATE_STOP:
            Deposition_Stop();
            break;
            
        // Data & Mode
        case CMD_REQUEST_DATA:
            // Data is sent via onRequest callback
            Sensors_Update();
            break;
        case CMD_SWITCH_AUTONOMOUS:
            isAutonomousMode = true;
            Serial.println("Switched to Autonomous Mode");
            break;
            
        default:
            Serial.print("Unknown command: ");
            Serial.println(command);
            break;
    }
}

void EmergencyStop(void) {
    Locomotion_Stop();
    Excavation_Stop();
    Excavation_BeltStop();
    Deposition_Stop();
    Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
}

int I2C_GetLastCommand(void) {
    return lastCommand;
}
