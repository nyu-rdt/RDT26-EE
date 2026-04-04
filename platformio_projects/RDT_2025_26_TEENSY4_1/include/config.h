#ifndef CONFIG_H
#define CONFIG_H

// setting flags:
#define RAMP_UP true
#define SERIAL_DEBUG true
#define USE_TIMEOUT true
#define ANALOG_VIB_CONTROL false
#define USE_OLD_HEX_MAPPING false

#define I2C_CHILD_ADDRESS 0x08
#define RELAY_DRIVER_PIN 2
#define RELAY_READ_PIN 3
#define RELAY_3S_LOW_PIN 4
#define RELAY_6S_LOW_PIN 5


#define CAN_BAUD_RATE 500000
#define COMMAND_TIMEOUT_MS 500

#define CAN_ID_LEFT_MOTOR  0x4c
#define CAN_ID_RIGHT_MOTOR 0x78
#define CAN_ID_EXCAVATION_MOTOR 0x48 // CHANGE TO REAL VALUE!!!!!!!!!!!!

#define LOCOMOTION_DUTY_CYCLE 0.33f
#define EXCAVATION_DUTY_CYCLE 0.4f
#define EXCAVATION_STEP_PERIOD 800 // microseconds, time between each step change in excavation speed

// Ramping parameters
#define TX_PERIOD_MS 20
#define MAX_SPEED_DELTA_PER_TICK 0.01f
#define MAX_EXCAV_DELTA_PER_TICK 0.01f

// Deposition door actuator (SPARKmini-style PWM)
#define DEPOSITION_DOOR_ACTUATOR_PIN 9

#define DEPOSITION_DOOR_ARM_DELAY_MS 2000
#define DEPOSITION_DOOR_PULSE_STOP_US 1500
#define DEPOSITION_DOOR_PULSE_OPEN_US 2500
#define DEPOSITION_DOOR_PULSE_CLOSE_US 500

#define VIB_MOTOR_PIN 30

#if ANALOG_VIB_CONTROL
    #define VIB_MOTOR_DUTY_CYCLE 0.6f // not super critical since we just want it on/off, but can be tuned for stronger/weaker vibration
#endif

// Command Groups
#define GRP_CONTROL            0x0
#define GRP_LOCO_STOP          0x1
#define GRP_FORWARD            0x2
#define GRP_BACKWARD           0x3
#define GRP_LEFT               0x4
#define GRP_RIGHT              0x5
#if USE_OLD_HEX_MAPPING
#define GRP_EXCAVATION          0x6
#define GRP_DEPOSITION         0x7
#else
#define GRP_EXCAVATION_BELT    0x6
#define GRP_EXCAVATION_VERT    0x7
#define GRP_DEPOSITION_DOOR    0x8
#define GRP_DEPOSITION_VIB     0x9
#endif
#define GRP_DATA               0xA

#define CMD_GROUP(c) (((c)>>4)&0xF)   
#define CMD_PARAM(c) ((c)&0xF)
//#define GET_SPEED(i) (({static const float t[]=SPEED_TABLE;t[(i)>3?3:(i)]*LOCOMOTION_DUTY_CYCLE;}))

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
#define GET_DIRECTION(param) (((param) == 0x0) ? (0) : (((param) == 0x1) ? (1) : ((param) == 0x2) ? (-1) : (0)))
// Handler function type
typedef void (*GroupHandler)(uint8_t param);

#endif
