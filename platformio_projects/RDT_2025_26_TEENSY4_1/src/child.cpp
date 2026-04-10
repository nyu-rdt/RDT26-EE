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

#if PLOT_DATA
static unsigned long lastPlotMs = 0;
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
#if SERIAL_DEBUG && !PLOT_DATA
    Serial.println("Ready (ramping ON)");
#endif
#else
    CAN_SendLocomotion(0.0f, 0.0f);
    CAN_SendExcavation(0.0f);
#if SERIAL_DEBUG && !PLOT_DATA
    Serial.println("Ready");
#endif
#endif
}

static void receiveEvent(int numBytes) {
    if (Wire2.available()) {
        latestCommand = Wire2.read();
        newCommand = true;
    }
}

// Fires when master calls requestFrom() — always sends exactly DATA_PACKET_SIZE bytes.
// Disabled sensors send 0xFF as a sentinel so SW can detect them.
//
// Packet layout (18 bytes):
//   [0-7]   motor_currents[8]  uint8, 0-255 = 0-20A  (CURRENT_SENSE_ENABLED)
//   [8]     left_encoder       uint8, 0-255 = 0-360°  (ROTARY_ENCODERS_ENABLED)
//   [9]     right_encoder      uint8, 0-255 = 0-360°  (ROTARY_ENCODERS_ENABLED)
//   [10-13] load_cells[4]      uint8 each             (LOAD_CELLS_ENABLED)
//   [14]    string_pot         uint8 (conveyor pos)   (STRING_POT_ENABLED)
//   [15]    gate_pos           uint8                  (GATE_POS_ENABLED)
//   [16]    flags              uint8 (bit0=relay, bit1=3s_low, bit2=6s_low)
//   [17]    fixes_attempted    uint8                  (no driver yet)
static void requestEvent() {
    uint8_t pkt[DATA_PACKET_SIZE];

    // Bytes 0-7: motor currents (0-255 = 0-20A, scale = 255/20 = 12.75)
#if CURRENT_SENSE_ENABLED
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
        pkt[i] = (uint8_t)(currents[i] * 12.75f);
    }
#else
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) { pkt[i] = 0xFF; }
#endif

    // Bytes 8-9: encoder angles, 0-255 = 0-360°
#if ROTARY_ENCODERS_ENABLED
    pkt[8] = (uint8_t)(ROTARY_ENCODER_getEncoderAngle(1) * 255.0f / 360.0f);
    pkt[9] = (uint8_t)(ROTARY_ENCODER_getEncoderAngle(2) * 255.0f / 360.0f);
#else
    pkt[8] = 0xFF;
    pkt[9] = 0xFF;
#endif

    // Bytes 10-13: load cells
#if LOAD_CELLS_ENABLED
    // TODO: fill from load cell driver
    
#else
    pkt[10] = pkt[11] = pkt[12] = pkt[13] = 0xFF;
#endif

    // Byte 14: string pot (conveyor position)
#if STRING_POT_ENABLED
    // TODO: fill from string pot driver
    
#else
    pkt[14] = 0xFF;
#endif

    // Byte 15: gate position
#if GATE_POS_ENABLED
    // TODO: fill from gate position driver
    
#else
    pkt[15] = 0xFF;
#endif

    // Byte 16: flags — bit0=relay, bit1=3s_low, bit2=6s_low
    pkt[16] = SYSTEM_GetRelayStatus() & 0x07;
    //TODO: add more flags here

    pkt[17] = 0xFF; // fixes_attempted — no driver yet

    Wire2.write(pkt, DATA_PACKET_SIZE);
}

bool child_update() {
    SYSTEM_Update();

    if (newCommand) {
        newCommand = false;
        lastCommandTime = millis();
#if SERIAL_DEBUG && !PLOT_DATA
        Serial.print("cmd: 0x");
        Serial.println(latestCommand, HEX);
#endif
        return processCommand(latestCommand);
    }

#if USE_TIMEOUT
    if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
#if SERIAL_DEBUG && !PLOT_DATA
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

#if PLOT_DATA
    if (millis() - lastPlotMs >= PLOT_PERIOD_MS) {
        lastPlotMs = millis();
#if CURRENT_SENSE_ENABLED
        Serial.print("I0:"); Serial.print(currents[0], 2); Serial.print("\t");
        Serial.print("I1:"); Serial.print(currents[1], 2); Serial.print("\t");
        Serial.print("I2:"); Serial.print(currents[2], 2); Serial.print("\t");
        Serial.print("I3:"); Serial.print(currents[3], 2); Serial.print("\t");
        Serial.print("I4:"); Serial.print(currents[4], 2); Serial.print("\t");
        Serial.print("I5:"); Serial.print(currents[5], 2); Serial.print("\t");
        Serial.print("I6:"); Serial.print(currents[6], 2); Serial.print("\t");
        Serial.print("I7:"); Serial.print(currents[7], 2); Serial.print("\t");
#endif
#if ROTARY_ENCODERS_ENABLED
        Serial.print("Enc1:"); Serial.print(ROTARY_ENCODER_getEncoderAngle(1), 1); Serial.print("\t");
        Serial.print("Enc2:"); Serial.print(ROTARY_ENCODER_getEncoderAngle(2), 1); Serial.print("\t");
#endif
#if STRING_POT_ENABLED
        // TODO: replace with string pot driver read when available
        Serial.print("StrPot:"); Serial.print(analogRead(A0)); Serial.print("\t");
#endif
        Serial.println();
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
