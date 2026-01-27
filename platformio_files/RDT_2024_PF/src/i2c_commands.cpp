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

// E-Stop and relay state
static bool eStopEngaged = true;
static bool currentRelayStatus = false;

// Command interruption state
static byte pendingCommand = 0;
static bool commandPending = false;

// Cached weight for non-blocking reads
static float lastWeight = 0.0f;
static unsigned long nextWeightReadTime = 0;

// Command refresh timing
static unsigned long lastCommandRefreshTime = 0;

// Forward declaration for I2C callback
static void onRequestCallback(void);

void I2C_Init(void) {
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onRequest(onRequestCallback);
    Wire.setClock(400000);
    
    // Initialize E-Stop relay pin
    pinMode(RELAY_PIN, INPUT_PULLUP);
    currentRelayStatus = digitalRead(RELAY_PIN);
    
    Serial.println("I2C Slave Initialized");
}

static void onRequestCallback(void) {
    I2C_UpdateData();
    Wire.write(dataPacket, sizeof(dataPacket));
    
    Serial.print("Sent to Pi - Weight: ");
    Serial.print(dataPacket[0]);
    Serial.print(", Position: ");
    Serial.print(dataPacket[1]);
    Serial.print(", Encoder (");
    Serial.print(Sensors_GetActiveEncoder());
    Serial.print("): ");
    Serial.println(dataPacket[2]);
}

void I2C_UpdateData(void) {
    float length = Sensors_GetStringLength();
    float weight = lastWeight / 20.0f;  // Use cached weight
    
    // Get active encoder angle
    uint8_t activeEnc = Sensors_GetActiveEncoder();
    float activeAngle = Sensors_GetEncoderAngle(activeEnc);
    
    // Convert to integers for the data packet
    uint8_t weight_int = (uint8_t)constrain(weight, 0, 255);
    uint8_t length_int = (uint8_t)constrain(length, 0, 255);
    uint8_t angle_int = (uint8_t)(activeAngle * 255.0f / 360.0f);
    
    // Update the data packet
    // Data order: weight, string_length, encoder angle
    dataPacket[0] = weight_int;
    dataPacket[1] = length_int;
    dataPacket[2] = angle_int;
}

bool I2C_CheckEStop(void) {
    currentRelayStatus = digitalRead(RELAY_PIN);
    
    // E-stop activated (LOW when relay is OFF/E-stop engaged)
    if (currentRelayStatus == LOW) {
        if (!eStopEngaged) {
            EmergencyStop();
            eStopEngaged = true;
            commandPending = false;
            Serial.println("E-STOP ACTIVATED - Motors stopped");
        }
        
        // Still allow data requests during E-stop
        if (Wire.available()) {
            char c = Wire.read();
            if (int(c) == CMD_REQUEST_DATA) {
                Sensors_Update();
            }
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

bool I2C_IsEStopEngaged(void) {
    return eStopEngaged;
}

void I2C_SetEStopEngaged(bool engaged) {
    eStopEngaged = engaged;
}

bool I2C_HandleInterruptedCommand(void) {
    if (commandPending) {
        I2C_ProcessCommand(int(pendingCommand));
        commandPending = false;
        return true;
    }
    return false;
}

void I2C_RefreshMotorCommands(void) {
    unsigned long currentMillis = millis();
    
    // Non-blocking weight reading
    if (currentMillis >= nextWeightReadTime && !Wire.available()) {
        lastWeight = Sensors_GetWeight();
        nextWeightReadTime = currentMillis + 500;
    }
    
    // Periodic command refresh for CAN motors
    if (currentMillis - lastCommandRefreshTime >= COMMAND_REFRESH_INTERVAL) {
        if (!eStopEngaged) {
            Excavation_RefreshCommands();
        }
        lastCommandRefreshTime = currentMillis;
    }
}

void I2C_ProcessCommand(int command) {
    // Any command except data request clears E-stop state
    if (command != CMD_REQUEST_DATA) {
        eStopEngaged = false;
    }
    
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
        case CMD_ACME_UP:
            Excavation_Up();
            break;
        case CMD_ACME_DOWN:
            Excavation_Down();
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
            Sensors_Update();
            break;
        case CMD_SWITCH_AUTONOMOUS:
            isAutonomousMode = true;
            Serial.println("Switched to Autonomous Mode");
            break;
            
        default:
            break;
    }
}

void EmergencyStop(void) {
    // Order of operations: 1. Locomotion 2. Excavation 3. Belt 4. Deposition
    Locomotion_Stop();
    Excavation_Stop();
    Excavation_BeltStop();
    Deposition_Stop();
    Serial.println("Emergency Stop Activated");
}

int I2C_GetLastCommand(void) {
    return lastCommand;
}
