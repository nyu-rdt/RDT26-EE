#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "comms.h"
#include "ee_box.h"
#include "locomotion.h"
#include "excavation.h"
#include "deposition.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif
#if ROTARY_ENCODERS_ENABLED
#include "rotary_encoders.h"
#endif
#if LOAD_CELLS_ENABLED
#include "load_cell.h"
#endif

static void requestEvent();

// Written by COMMS_UpdateFlags() (main loop), read by requestEvent() (ISR).
// volatile ensures the ISR always sees the latest byte written by the main loop.
// uint8_t is a single-byte type so reads/writes are inherently atomic on Cortex-M.
static volatile uint8_t s_flags = 0;

void COMMS_Init() {
    Wire2.onRequest(requestEvent);
}

void COMMS_UpdateFlags() {
    uint8_t flags = 0;
    if (!EE_BOX_IsRelayEngaged()) flags |= FLAG_ESTOP;
#if FLAGS_ENABLED
    if (EE_BOX_IsOvercurrent())      flags |= FLAG_OVERCURRENT;
    // FLAG_MACRO_ACTIVE: set by macro subsystem when implemented
    if (LOCO_IsStalled())            flags |= FLAG_LOCO_STALL;
    if (EXCAV_IsArmObstructed())     flags |= FLAG_EXCAV_ARM_OBSTRUCTED;
    if (EXCAV_IsEmptyCut())          flags |= FLAG_EXCAV_EMPTY_CUT;
    if (EXCAV_IsBeltStalled())       flags |= FLAG_EXCAV_STALL;
    if (DEPO_IsBinEmpty())           flags |= FLAG_DEPO_BIN_EMPTY;
#endif
    s_flags = flags;
}

// Fires when master calls requestFrom() — always sends exactly DATA_PACKET_SIZE bytes.
// Disabled sensors send 0xFF as a sentinel so SW can detect them.
//
// Packet layout (18 bytes):
//   [0-7]   motor_currents[8]  uint8, 0-255 = 0-20A  (CURRENT_SENSE_ENABLED)
//   [8]     left_encoder       uint8, 0-255 = 0-360°  (ROTARY_ENCODERS_ENABLED)
//              NOTE: encoders are 8192 counts/rev — mapping to 1 byte loses precision.
//              If SW needs odometry accuracy, expand to 2 bytes each (requires packet resize).
//   [9]     right_encoder      uint8, 0-255 = 0-360°  (ROTARY_ENCODERS_ENABLED)
//   [10-13] load_cells[4]      uint8 each, kg*10      (LOAD_CELLS_ENABLED)
//   [14]    string_pot         uint8, direct cm value (STRING_POT_ENABLED)
//   [15]    depo_door_state    uint8, DepoDoorState enum value (GATE_POS_ENABLED)
//   [16]    flags              uint8, all active-high (FLAGS_ENABLED controls bits 1-7)
//             bit0 FLAG_ESTOP                — relay not engaged, hardware e-stop active
//             bit1 FLAG_OVERCURRENT          — any current channel over threshold
//             bit2 FLAG_MACRO_ACTIVE         — MCU running autonomous fix, hold commands (unimplemented)
//             bit3 FLAG_LOCO_STALL           — locomotion stalled, needs SW reset
//             bit4 FLAG_EXCAV_ARM_OBSTRUCTED — stepper current spike while descending
//             bit5 FLAG_EXCAV_EMPTY_CUT      — belt on, low current, reposition rover
//             bit6 FLAG_EXCAV_STALL          — belt on, high current, no mass change; latched until belt stops
//             bit7 FLAG_DEPO_BIN_EMPTY       — bin mass below empty threshold
//   [17]    fixes_attempted    uint8, 0xFF = unimplemented sentinel
static void requestEvent() {
    uint8_t pkt[DATA_PACKET_SIZE];

#if CURRENT_SENSE_ENABLED
    const float* currents = CURRENT_SENSORS_GetBuffer();
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
        float scaledCurrent = currents[i] * 12.75f;
        pkt[i] = (uint8_t)constrain(scaledCurrent, 0.0f, 255.0f);
    }
#else
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) { pkt[i] = 0xFF; }
#endif

#if ROTARY_ENCODERS_ENABLED
    pkt[8] = (uint8_t)(ROTARY_ENCODER_getEncoderAngle(1) * 255.0f / 360.0f);
    pkt[9] = (uint8_t)(ROTARY_ENCODER_getEncoderAngle(2) * 255.0f / 360.0f);
#else
    pkt[8] = 0xFF;
    pkt[9] = 0xFF;
#endif

#if LOAD_CELLS_ENABLED
    for (int i = 0; i < NUM_LOAD_CELLS; i++) {
        float kg = LOAD_CELL_GetMass(i);
        pkt[10 + i] = (uint8_t)constrain(kg * 10.0f, 0.0f, 255.0f); // 0-25.5 kg range, 0.1 kg resolution
    }
#else
    pkt[10] = pkt[11] = pkt[12] = pkt[13] = 0xFF;
#endif

#if STRING_POT_ENABLED
    pkt[14] = (uint8_t)constrain(EXCAV_GetConveyorDistance(), 0.0f, 255.0f);
#else
    pkt[14] = 0xFF;
#endif

#if GATE_POS_ENABLED
    pkt[15] = (uint8_t)DEPO_GetDoorState();
#else
    pkt[15] = 0xFF;
#endif

    pkt[16] = s_flags;
    pkt[17] = 0xFF; // fixes_attempted — no driver yet

    Wire2.write(pkt, DATA_PACKET_SIZE);
}
