#include <Arduino.h>
#include <Wire.h>
#include "child.h"
#include "config.h"
#include "can_driver.h"
#include "stepper_driver.h"
#include "depo_door_driver.h"
#include "vib_motor_driver.h"
#include "system.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif
#if ROTARY_ENCODERS_ENABLED
#include "rotary_encoders.h"
#endif

// Forward declarations
static void receiveEvent(int numBytes);
static void requestEvent();
static bool processCommand(uint8_t cmd);
static void registerHandlers();
static void sendLocomotion(float left, float right);
static void stopLocomotion();
static void stopExcavation();
static void grp_Control(uint8_t param);
static void grp_LocoStop(uint8_t param);
static void grp_Forward(uint8_t param);
static void grp_Backward(uint8_t param);
static void grp_TurnLeft(uint8_t param);
static void grp_TurnRight(uint8_t param);
#if USE_OLD_HEX_MAPPING
static void grp_Excavation(uint8_t param);
static void grp_Deposition(uint8_t param);
#endif
static void grp_ExcavationBelt(uint8_t param);
static void grp_ExcavationVert(uint8_t param);

static void grp_DepositionDoor(uint8_t param);
static void grp_DepositionVib(uint8_t param);

static void grp_Data(uint8_t param);

volatile uint8_t latestCommand = 0x10;
volatile bool newCommand = false;
static unsigned long lastCommandTime = 0;
static GroupHandler groups[16] = {nullptr};

#if CURRENT_SENSE_ENABLED
static float currents[NUM_CURRENT_SENSORS] = {0};
static unsigned long lastCurrentMs = 0;
#endif

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
    Wire2.begin(I2C_CHILD_ADDRESS);
    Wire2.onReceive(receiveEvent);
    Wire2.onRequest(requestEvent);

    SYSTEM_Init();
    CAN_Init();
    STEPPER_Init();
    DEPO_DOOR_Init();
    VIB_Init();
#if CURRENT_SENSE_ENABLED
    CURRENT_SENSORS_Init();
    lastCurrentMs = millis();
#endif
#if ROTARY_ENCODERS_ENABLED
    ROTARY_ENCODER_Init();
#endif
    SYSTEM_RegisterStopCallbacks(stopLocomotion, stopExcavation);
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
    if (Wire2.available()) {
        latestCommand = Wire2.read();
        newCommand = true;
    }
}

// Fires when master calls requestFrom() — always sends DATA_PACKET_SIZE bytes.
//
// Packet layout (18 bytes):
//   [0-7]  motor_currents[8]  uint8, 0-255 = 0-20A
//   [8]    left_encoder       uint8
//   [9]    right_encoder      uint8
//   [10-13] load_cells[4]     uint8 each
//   [14]   string_pot         uint8 (conveyor position)
//   [15]   gate_pos           uint8
//   [16]   flags              uint8 (bit field, bit0 = relay status)
//   [17]   fixes_attempted    uint8
//
// With STUB_MISSING_SENSORS true, any field without a driver sends 0xFF so SW
// always receives exactly DATA_PACKET_SIZE bytes and can detect unimplemented
// sensors by their sentinel value.
static void requestEvent() {
#if STUB_MISSING_SENSORS
    uint8_t pkt[DATA_PACKET_SIZE];

    // Bytes 0-7: motor currents (0-255 = 0-20A, scale = 255/20 = 12.75)
#if CURRENT_SENSE_ENABLED
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
        pkt[i] = (uint8_t)(currents[i] * 12.75f);
    }
#else
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
        pkt[i] = 0xFF;
    }
#endif

#if ROTARY_ENCODERS_ENABLED
    // Pack angle as 0-255 = 0-360° (same encoding used by parent keyboard.cpp display)
    pkt[8] = (uint8_t)(ROTARY_ENCODER_getEncoderAngle(1) * 255.0f / 360.0f);
    pkt[9] = (uint8_t)(ROTARY_ENCODER_getEncoderAngle(2) * 255.0f / 360.0f);
#else
    pkt[8]  = 0xFF; // left_encoder  — disabled
    pkt[9]  = 0xFF; // right_encoder — disabled
#endif

    pkt[10] = 0xFF; // load_cell[0]  — no driver yet
    pkt[11] = 0xFF; // load_cell[1]  — no driver yet
    pkt[12] = 0xFF; // load_cell[2]  — no driver yet
    pkt[13] = 0xFF; // load_cell[3]  — no driver yet

    pkt[14] = 0xFF; // string_pot (conveyor_pos) — no driver yet
    pkt[15] = 0xFF; // gate_pos                  — no driver yet

    // Byte 16: flags — bit 0 = relay status (rest reserved/0 for now)
    pkt[16] = SYSTEM_GetRelayStatus() & 0x01;

    pkt[17] = 0xFF; // fixes_attempted — no driver yet

    Wire2.write(pkt, DATA_PACKET_SIZE);
#else
    // Legacy compact format (pre-SW-packet): relay + motor speeds [+ currents]
    Wire2.write(SYSTEM_GetRelayStatus());
#if RAMP_UP
    Wire2.write((uint8_t)((int8_t)(currentLeft  * 100.0f)));
    Wire2.write((uint8_t)((int8_t)(currentRight * 100.0f)));
#else
    Wire2.write((uint8_t)0);
    Wire2.write((uint8_t)0);
#endif
#if CURRENT_SENSE_ENABLED
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
        Wire2.write((uint8_t)(currents[i] * 10));
    }
#endif
#endif // STUB_MISSING_SENSORS
}

bool child_update() {
    SYSTEM_Update();

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
        Serial.println("Command timeout");
#endif
        SYSTEM_StopAllMotors();
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

#if CURRENT_SENSE_ENABLED
    if (millis() - lastCurrentMs >= CURRENT_PERIOD_MS) {
        lastCurrentMs = millis();
        CURRENT_SENSORS_Update(currents);
    }
#endif

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
    #if USE_OLD_HEX_MAPPING
    groups[GRP_EXCAVATION] = grp_Excavation;
    groups[GRP_DEPOSITION] = grp_Deposition;
    #else
    groups[GRP_EXCAVATION_BELT] = grp_ExcavationBelt;
    groups[GRP_EXCAVATION_VERT] = grp_ExcavationVert;
    groups[GRP_DEPOSITION_DOOR] = grp_DepositionDoor;
    groups[GRP_DEPOSITION_VIB] = grp_DepositionVib; 
    #endif
    groups[GRP_DATA]       = grp_Data;
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

static void stopLocomotion() {
    sendLocomotion(0.0f, 0.0f);
}

static void stopExcavation() {
    sendExcavation(0.0f);
}


// Group Handlers
static void grp_Control(uint8_t param) {
    if (param == 0x01) {
        SYSTEM_StopAllMotors();
    }
}

static void grp_LocoStop(uint8_t param) {
    stopLocomotion();
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

#if USE_OLD_HEX_MAPPING
static void grp_Excavation(uint8_t param) {
    if (param<3){
        grp_ExcavationVert(param);
    }
    else {
        grp_ExcavationBelt(param-3);
    }
}

static void grp_Deposition(uint8_t param) {
    if (param<3){
        grp_DepositionDoor(param);
    }
    else {
        grp_DepositionVib(param-3);
    }
}
#endif

static void grp_ExcavationBelt(uint8_t param) {
    float spd = GET_DIRECTION(param) * EXCAVATION_DUTY_CYCLE;
    sendExcavation(spd);
}

static void grp_ExcavationVert(uint8_t param) {
    STEPPER_SetDirection(GET_DIRECTION(param)); 
}

static void grp_DepositionDoor(uint8_t param) {
    DEPO_DOOR_SetDirection(GET_DIRECTION(param));
}

static void grp_DepositionVib(uint8_t param) {
    VIB_drive(GET_DIRECTION(param));
}


static void grp_Data(uint8_t param) {
    // response is sent by requestEvent() when master calls Wire.requestFrom()
}
