#include <Arduino.h>
#include <Wire.h>
#include "main.h"
#include "config.h"
#include "ee_box.h"
#include "estop.h"
#include "can_driver.h"
#include "locomotion.h"
#include "excavation.h"
#include "deposition.h"
#include "comms.h"
#include "debug.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif
#if ROTARY_ENCODERS_ENABLED
#include "rotary_encoders.h"
#endif
#if LOAD_CELLS_ENABLED
#include "load_cell.h"
#endif


static volatile uint8_t latestCommand = 0x10;
static volatile bool newCommand = false;
static unsigned long lastCommandTime;
static bool timedOut = false;
static bool killSwitchActive = false;
static GroupHandler groups[16] = {nullptr};

void setup()  { ROVER_init(); }
void loop()   { ROVER_update(); }

void ROVER_init() {
    Serial.begin(115200);
    EE_BOX_Init();
    CAN_Init();
    LOCO_Init();
    EXCAV_Init();
    DEPO_Init();
#if CURRENT_SENSE_ENABLED
    CURRENT_SENSORS_Init();
#endif
#if ROTARY_ENCODERS_ENABLED
    ROTARY_ENCODER_Init();
#endif
#if LOAD_CELLS_ENABLED
    LOAD_CELL_Init();
#endif
    ESTOP_RegisterCallback(LOCO_EmergencyStop);
    ESTOP_RegisterCallback(EXCAV_EmergencyStop);
    ESTOP_RegisterCallback(DEPO_EmergencyStop);
    Wire2.begin(I2C_CHILD_ADDRESS);
    Wire2.onReceive(receiveEvent);
    COMMS_Init();
    registerHandlers();
#if SERIAL_DEBUG && !PLOT_DATA
    Serial.println("Ready");
#endif
}

void ROVER_update() {
    EE_BOX_Update();

    if (!killSwitchActive && !EE_BOX_IsRelayEngaged()) {
        killSwitchActive = true;
#if SERIAL_DEBUG && !PLOT_DATA
        Serial.println("[estop] kill switch engaged");
#endif
        ESTOP_Trigger();
    } else if (killSwitchActive && EE_BOX_IsRelayEngaged()) {
        killSwitchActive = false;
#if SERIAL_DEBUG && !PLOT_DATA
        Serial.println("[estop] kill switch released");
#endif
    }

    if (newCommand) {
        newCommand = false;
            lastCommandTime = millis();
            if (timedOut) {
                timedOut = false;
#if SERIAL_DEBUG && !PLOT_DATA
                Serial.println("[comms] restored");
#endif
            }
#if SERIAL_DEBUG && !PLOT_DATA
            Serial.print("cmd: 0x");
            Serial.println(latestCommand, HEX);
#endif
        if (!killSwitchActive) {
            processCommand(latestCommand);
        }
    }

#if USE_TIMEOUT
    if (!timedOut && millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
        timedOut = true;
#if SERIAL_DEBUG && !PLOT_DATA
        Serial.println("[timeout] comms lost");
#endif
        ESTOP_Trigger();
    }
#endif

    LOCO_Update();
    EXCAV_Update();
#if CURRENT_SENSE_ENABLED
    CURRENT_SENSORS_Update();
#endif
#if LOAD_CELLS_ENABLED
    LOAD_CELL_Update();
#endif
    DEPO_Update();
    COMMS_UpdateFlags();
#if PLOT_DATA
    DEBUG_Update();
#endif

}

static void receiveEvent(int numBytes) {
    if (Wire2.available()) {
        latestCommand = Wire2.read();
        newCommand = true;
    }
    // Drain extra bytes. If Jetson sends >1 byte, leftovers sit in the buffer and
    // would be read as the command on the next receiveEvent, corrupting that command.
    while (Wire2.available()) Wire2.read();
}

static void processCommand(uint8_t cmd) {
    uint8_t group = CMD_GROUP(cmd);
    uint8_t param = CMD_PARAM(cmd);
    if (groups[group] != nullptr) groups[group](param);
}

static void registerHandlers() {
    groups[GRP_CONTROL]   = grp_Control;
    groups[GRP_LOCO_STOP] = grp_LocoStop;
    groups[GRP_FORWARD]   = grp_Forward;
    groups[GRP_BACKWARD]  = grp_Backward;
    groups[GRP_LEFT]      = grp_TurnLeft;
    groups[GRP_RIGHT]     = grp_TurnRight;
#if USE_OLD_HEX_MAPPING
    groups[GRP_EXCAVATION] = grp_Excavation;
    groups[GRP_DEPOSITION] = grp_Deposition;
#else
    groups[GRP_EXCAVATION_BELT] = grp_ExcavationBelt;
    groups[GRP_EXCAVATION_VERT] = grp_ExcavationVert;
    groups[GRP_DEPOSITION_DOOR] = grp_DepositionDoor;
    groups[GRP_DEPOSITION_VIB]  = grp_DepositionVib;
#endif
    groups[GRP_DATA] = grp_Data;
}

// Group Handlers
static void grp_Control(uint8_t param) {
    if (param == 0x01) {
#if SERIAL_DEBUG && !PLOT_DATA
        Serial.println("[estop] software sent estop cmd");
#endif
        ESTOP_Trigger();
    }
}

static void grp_LocoStop(uint8_t param)  { LOCO_Stop(); }
static void grp_Forward(uint8_t param)   { float s = getSpeed(param, EXCAV_GetBeltActive() ? LOCOMOTION_DUTY_CYCLE_EXCAV : LOCOMOTION_DUTY_CYCLE); LOCO_SetSpeeds(-s,  s); }
static void grp_Backward(uint8_t param)  { float s = getSpeed(param, EXCAV_GetBeltActive() ? LOCOMOTION_DUTY_CYCLE_EXCAV : LOCOMOTION_DUTY_CYCLE); LOCO_SetSpeeds( s, -s); }
static void grp_TurnLeft(uint8_t param)  { float s = getSpeed(param, EXCAV_GetBeltActive() ? LOCOMOTION_DUTY_CYCLE_EXCAV : LOCOMOTION_DUTY_CYCLE); LOCO_SetSpeeds( s,  s); }
static void grp_TurnRight(uint8_t param) { float s = getSpeed(param, EXCAV_GetBeltActive() ? LOCOMOTION_DUTY_CYCLE_EXCAV : LOCOMOTION_DUTY_CYCLE); LOCO_SetSpeeds(-s, -s); }

#if USE_OLD_HEX_MAPPING
static void grp_Excavation(uint8_t param) {
    if (param < 3) grp_ExcavationVert(param);
    else           grp_ExcavationBelt(param - 3);
}

static void grp_Deposition(uint8_t param) {
    if (param < 2) grp_DepositionDoor(param);
    else           grp_DepositionVib(param - 2);
}
#endif

static void grp_ExcavationBelt(uint8_t param) { EXCAV_SetBeltDirection(GET_DIRECTION(param)); }
static void grp_ExcavationVert(uint8_t param) { EXCAV_SetVertDirection(GET_DIRECTION(param)); }

static void grp_DepositionDoor(uint8_t param) {
    if      (param == 0) DEPO_OpenDoor();
    else if (param == 1) DEPO_CloseDoor();
#if SERIAL_DEBUG && !PLOT_DATA
    else { Serial.print("DEPO DOOR: unknown param "); Serial.println(param); }
#endif
}

static void grp_DepositionVib(uint8_t param) { DEPO_SetVib(GET_DIRECTION(param)); }
static void grp_Data(uint8_t param) {}
