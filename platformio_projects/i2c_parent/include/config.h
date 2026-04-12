#ifndef CONFIG_H
#define CONFIG_H

// Control mode
#define USE_WASD 1

// default I2C bus (Wire)
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

// Child Address
#define I2C_CHILD_ADDRESS 0x08

// Command groups (upper nibble)
#define GRP_CONTROL     0x0
#define GRP_LOCO_STOP   0x1
#define GRP_FORWARD     0x2
#define GRP_BACKWARD    0x3
#define GRP_LEFT        0x4
#define GRP_RIGHT       0x5
#define GRP_EXCAVATION       0x6
#define GRP_DEPOSITION       0x7
#define GRP_DATA             0xA

// Group parameter (lower nibble) for speed presets used by locomotion groups.
#define SPEED_PARAM_25   0x0
#define SPEED_PARAM_50   0x1
#define SPEED_PARAM_75   0x2
#define SPEED_PARAM_100  0x3

#define FORWARD 1
#define REVERSE 2
#define STOP 0


// Build single-byte command expected by i2c_can_child_tony.
#define BUILD_I2C_CMD(group, param) ((((group) & 0x0F) << 4) | ((param) & 0x0F))

// Must match child firmware config.h — keep in sync with RDT_2025_26_TEENSY4_1/include/config.h
#define CURRENT_SENSE_ENABLED   1
#define ROTARY_ENCODERS_ENABLED 1
#define LOAD_CELLS_ENABLED      0
#define STRING_POT_ENABLED      0
#define GATE_POS_ENABLED        0
#define NUM_CURRENT_SENSORS     8

// Child always sends the full 18-byte SW telemetry packet
#define RESPONSE_BYTES 18

#endif
