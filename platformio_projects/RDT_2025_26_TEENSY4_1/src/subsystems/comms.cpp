#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "comms.h"
#include "ee_box.h"
#include "excavation.h"
#include "deposition.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif
#if ROTARY_ENCODERS_ENABLED
#include "rotary_encoders.h"
#endif

static void requestEvent();

void COMMS_Init() {
    Wire2.onRequest(requestEvent);
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
//   [15]    depo_door_state    uint8, DepoDoorState enum value (GATE_POS_ENABLED)
//   [16]    flags              uint8 (bit0=relay, bit1=3s_low, bit2=6s_low)
//   [17]    fixes_attempted    uint8                  (no driver yet)
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
    // TODO: fill from load cell driver
#else
    pkt[10] = pkt[11] = pkt[12] = pkt[13] = 0xFF;
#endif

#if STRING_POT_ENABLED
    pkt[14] = (uint8_t)(constrain(EXCAV_GetConveyorDistance() * (255.0f / STRING_POT_MAX_DISTANCE), 0, 255));
#else
    pkt[14] = 0xFF;
#endif

#if GATE_POS_ENABLED
    pkt[15] = (uint8_t)DEPO_GetDoorState();
#else
    pkt[15] = 0xFF;
#endif

    pkt[16] = EE_BOX_GetRelayStatus() & 0x07;
    pkt[17] = 0xFF; // fixes_attempted — no driver yet

    Wire2.write(pkt, DATA_PACKET_SIZE);
}
