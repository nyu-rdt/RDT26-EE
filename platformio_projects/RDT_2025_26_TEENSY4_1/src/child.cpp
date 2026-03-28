#include <Wire.h>
#include "child.h"
#include "config.h"
#include "can_driver.h"
#include "stepper_driver.h"

// Forward declarations
static void receiveEvent(int numBytes);
static bool processCommand(uint8_t cmd);
static void registerHandlers();
static void sendLocomotion(float left, float right);
static void grp_Control(uint8_t param);
static void grp_LocoStop(uint8_t param);
static void grp_Forward(uint8_t param);
static void grp_Backward(uint8_t param);
static void grp_TurnLeft(uint8_t param);
static void grp_TurnRight(uint8_t param);
static void grp_ExcavationBelt(uint8_t param);
static void grp_ExcavationVert(uint8_t param);
static void grp_Deposition(uint8_t param);
static void grp_Data(uint8_t param);

volatile uint8_t latestCommand = 0x10;
volatile bool newCommand = false;
static unsigned long lastCommandTime = 0;
static GroupHandler groups[16] = {nullptr};
volatile float current_Excav_Speed = 0.0f; // For ramping excavation speed if needed

#if RAMP_UP
static float currentLeft = 0.0f, currentRight = 0.0f;
static float targetLeft = 0.0f, targetRight = 0.0f;
static float targetExcav = 0.0f, currentExcav = 0.0f;
static unsigned long lastTxMs = 0;

static float slew(float cur, float tgt, float maxDelta) {
    float d = tgt - cur;
    return (d > maxDelta) ? cur + maxDelta : (d < -maxDelta) ? cur - maxDelta : tgt;
}
#endif

void child_init() {
    Wire.begin(I2C_CHILD_ADDRESS);
    Wire.onReceive(receiveEvent);
    CAN_Init();
    STEPPER_Init();
    registerHandlers();

#if RAMP_UP
    currentLeft = currentRight = targetLeft = targetRight = 0.0f;
    currentExcav = targetExcav = 0.0f;
    lastTxMs = millis();
    Serial.println("Ready (ramping ON)");
#else
    CAN_SendLocomotion(0.0f, 0.0f);
    CAN_SendExcavation(0.0f);
    Serial.println("Ready");
#endif
}

static void receiveEvent(int numBytes) {
    if (Wire.available()) {
        latestCommand = Wire.read();
        newCommand = true;
    }
}


static void sendLocomotion(float left, float right) {
#if RAMP_UP
    targetLeft = left;
    targetRight = right;
#else
    CAN_SendLocomotion(left, right);
#endif
}

static void sendExcavation(float speed) {
#if RAMP_UP
    targetExcav = speed;
#else
    CAN_SendExcavation(speed);
#endif
}

bool child_update() {
    if (newCommand) {
        newCommand = false;
        lastCommandTime = millis();
#if SERIAL_DEBUG
        Serial.print("cmd: 0x");
        Serial.println(latestCommand, HEX);
#endif
        return processCommand(latestCommand);
    }

#if USE_TIMEOUT
    if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
#if SERIAL_DEBUG
        Serial.print("Command timeout: 0x");
        Serial.println(16, HEX);
#endif
        sendLocomotion(0.0f, 0.0f);
        sendExcavation(0.0f);
        lastCommandTime = millis();
        return true;
    }
#endif

#if RAMP_UP
    if (millis() - lastTxMs >= TX_PERIOD_MS) {
        lastTxMs = millis();
        currentLeft = slew(currentLeft, targetLeft, MAX_SPEED_DELTA_PER_TICK);
        currentRight = slew(currentRight, targetRight, MAX_SPEED_DELTA_PER_TICK);
        currentExcav = slew(currentExcav, targetExcav, MAX_EXCAV_DELTA_PER_TICK);
        CAN_SendLocomotion(currentLeft, currentRight);
        CAN_SendExcavation(currentExcav);
    }
#endif
    STEPPER_Update(EXCAVATION_STEP_PERIOD); // manages the stepper motor

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
    groups[GRP_EXCAVATION_BELT] = grp_ExcavationBelt;
    groups[GRP_EXCAVATION_VERT] = grp_ExcavationVert;
    groups[GRP_DEPOSITION] = grp_Deposition;
    groups[GRP_DATA]       = grp_Data;
}


// Group Handlers
static void grp_Control(uint8_t param) {
    if (param == 0x01) sendLocomotion(0.0f, 0.0f);
}

static void grp_LocoStop(uint8_t param) {
    sendLocomotion(0.0f, 0.0f);
}

static void grp_Forward(uint8_t param) {
    float spd = GET_SPEED(param);
    sendLocomotion(-spd, spd);
}

static void grp_Backward(uint8_t param) {
    float spd = GET_SPEED(param);
    sendLocomotion(spd, -spd);
}

static void grp_TurnLeft(uint8_t param) {
    float spd = GET_SPEED(param);
    sendLocomotion(spd, spd);
}

static void grp_TurnRight(uint8_t param) {
    float spd = GET_SPEED(param);
    sendLocomotion(-spd, -spd);
}

static void grp_ExcavationBelt(uint8_t param) {
    float spd = GET_DIRECTION(param) * EXCAVATION_DUTY_CYCLE;
    sendExcavation(spd);
}

static void grp_ExcavationVert(uint8_t param) {
    STEPPER_SetDirection(GET_DIRECTION(param)); 
}

static void grp_Deposition(uint8_t param) {}
static void grp_Data(uint8_t param) {}
