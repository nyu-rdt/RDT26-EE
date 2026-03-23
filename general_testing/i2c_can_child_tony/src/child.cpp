#include <Wire.h>
#include "child.h"
#include "config.h"
#include "can_driver.h"

// Forward declarations
static void receiveEvent(int numBytes);
static bool processCommand(uint8_t cmd);
static void registerHandlers();
static void grp_Control(uint8_t param);
static void grp_LocoStop(uint8_t param);
static void grp_Forward(uint8_t param);
static void grp_Backward(uint8_t param);
static void grp_TurnLeft(uint8_t param);
static void grp_TurnRight(uint8_t param);
static void grp_Excavation(uint8_t param);
static void grp_Deposition(uint8_t param);
static void grp_Data(uint8_t param);

//isr stuff
volatile uint8_t latestCommand = 0x10;
volatile bool newCommand = false;
static unsigned long lastCommandTime = 0;

// speed table
static const float speedTable[4] = SPEED_TABLE;

//groups
static GroupHandler groups[16] = {nullptr};

void child_init() {
    Wire.begin(I2C_CHILD_ADDRESS);
    Wire.onReceive(receiveEvent);
    
    CAN_Init();
    registerHandlers();
    
    Serial.println("Ready");
}

static void receiveEvent(int numBytes) {
    if (Wire.available()) {
        latestCommand = Wire.read();
        newCommand = true;
    }
}

bool child_update() {
    if (newCommand) {
        newCommand = false;
        lastCommandTime = millis();
        return processCommand(latestCommand);
    }
    
    if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
        processCommand(0x10);
        lastCommandTime = millis();
        return true;
    }
    return false;
}

static bool processCommand(uint8_t cmd) {
    uint8_t group = CMD_GROUP(cmd);
    uint8_t param = CMD_PARAM(cmd);
    
    if (groups[group] != nullptr) {
        groups[group](param);
        return true;
    }
    return false;
}

static void registerHandlers() {
    groups[GRP_CONTROL]    = grp_Control;
    groups[GRP_LOCO_STOP]  = grp_LocoStop;
    groups[GRP_FORWARD]    = grp_Forward;
    groups[GRP_BACKWARD]   = grp_Backward;
    groups[GRP_LEFT]       = grp_TurnLeft;
    groups[GRP_RIGHT]      = grp_TurnRight;
    groups[GRP_EXCAVATION] = grp_Excavation;
    groups[GRP_DEPOSITION] = grp_Deposition;
    groups[GRP_DATA]       = grp_Data;
}


// Group Handlers

static void grp_Control(uint8_t param) {
    if (param == 0x01) {
        CAN_SendLocomotion(0.0f, 0.0f);
    }
}

static void grp_LocoStop(uint8_t param) {
    CAN_SendLocomotion(0.0f, 0.0f);
}

static void grp_Forward(uint8_t param) {
    float spd = GET_SPEED(param);
    CAN_SendLocomotion(spd, spd);
}

static void grp_Backward(uint8_t param) {
    float spd = GET_SPEED(param);
    CAN_SendLocomotion(-spd, -spd);
}

static void grp_TurnLeft(uint8_t param) {
    float spd = GET_SPEED(param);
    CAN_SendLocomotion(-spd, spd);
}

static void grp_TurnRight(uint8_t param) {
    float spd = GET_SPEED(param);
    CAN_SendLocomotion(spd, -spd);
}

static void grp_Excavation(uint8_t param) {
}

static void grp_Deposition(uint8_t param) {
}

static void grp_Data(uint8_t param) {
}
