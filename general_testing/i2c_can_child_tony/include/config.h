#ifndef CONFIG_H
#define CONFIG_H

// I2C
#define I2C_CHILD_ADDRESS 0x08

// CAN
#define CAN_BAUD_RATE 500000
#define CAN_ID_LEFT_MOTOR  0x78
#define CAN_ID_RIGHT_MOTOR 0x16

// Duty cycle limit
#define LOCOMOTION_DUTY_CYCLE 0.33f

// Command timeout (ms)
#define COMMAND_TIMEOUT_MS 200

// Command Groups (high nibble)
#define GRP_CONTROL     0x0
#define GRP_LOCO_STOP   0x1
#define GRP_FORWARD     0x2
#define GRP_BACKWARD    0x3
#define GRP_LEFT        0x4
#define GRP_RIGHT       0x5
#define GRP_EXCAVATION  0x6
#define GRP_DEPOSITION  0x7
#define GRP_DATA        0x8

// Command parsing macros
#define CMD_GROUP(cmd)  (((cmd) >> 4) & 0x0F)
#define CMD_PARAM(cmd)  ((cmd) & 0x0F)

// Speed from param index
#define SPEED_TABLE { 0.25f, 0.50f, 0.75f, 1.00f }

static inline float getSpeed(uint8_t idx)
{
    static constexpr float speedTable[] = SPEED_TABLE;
    if (idx > 3U) {
        idx = 3U;
    }
    return speedTable[idx] * LOCOMOTION_DUTY_CYCLE;
}

#define GET_SPEED(idx) (getSpeed(static_cast<uint8_t>(idx)))

// Handler function type
typedef void (*GroupHandler)(uint8_t param);

#endif
