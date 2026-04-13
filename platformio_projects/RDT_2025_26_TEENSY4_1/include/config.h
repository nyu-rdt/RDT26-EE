#ifndef CONFIG_H
#define CONFIG_H

// setting flags:
#define RAMP_UP 1
#define SERIAL_DEBUG 1
// When PLOT_DATA=1, outputs sensor data in Arduino Serial Plotter format.
// SERIAL_DEBUG and PLOT_DATA are mutually exclusive — enabling both wastes bandwidth
// and corrupts the plotter stream with debug text. Set only one to 1 at a time.
#define PLOT_DATA 0
#define PLOT_PERIOD_MS 50
#define USE_TIMEOUT 1
#define ANALOG_VIB_CONTROL 0
#define USE_OLD_HEX_MAPPING 1
// Size of the SW telemetry packet (bytes). Must match SW expectation.
// requestEvent() always sends exactly this many bytes; unimplemented sensors
// send 0xFF as a sentinel so SW can detect them.
#define DATA_PACKET_SIZE 18

// Set to 1 to enable every sensor at once.
// Set to 0 and flip individual flags below to selectively enable.
#define ALL_SENSORS_ENABLED 0

#if ALL_SENSORS_ENABLED
    // Raw current readings — NOT for production; enable only for sensor bring-up.
    #define CURRENT_SENSE_ENABLED   1
    #define ROTARY_ENCODERS_ENABLED 1
    #define LOAD_CELLS_ENABLED      1
    #define STRING_POT_ENABLED      1
    #define GATE_POS_ENABLED        1
#else
    #define CURRENT_SENSE_ENABLED   1
    #define ROTARY_ENCODERS_ENABLED 1
    #define LOAD_CELLS_ENABLED      0
    #define STRING_POT_ENABLED      0
    #define GATE_POS_ENABLED        0
#endif

#define ENC1_A 21
#define ENC1_B 20
#define ENC2_A 19
#define ENC2_B 18

#define ENCODER_COUNTS_PER_REV 8192.0f
#define DEGREES_PER_COUNT (360.0f / ENCODER_COUNTS_PER_REV)

#define I2C_CHILD_ADDRESS 0x08
#define RELAY_DRIVER_PIN 2
#define RELAY_READ_PIN 3
#define RELAY_3S_LOW_PIN 4
#define RELAY_6S_LOW_PIN 5

#define NUM_CURRENT_SENSORS 8
#define CURRENT_INPUT_PIN 26
#define CURRENT_SELECT_PIN_0 27
#define CURRENT_SELECT_PIN_1 28
#define CURRENT_SELECT_PIN_2 29
// Sensor: 0-20A maps to 0-2V. ADC: 0-3.3V -> 0-1023. Formula: A = raw * (3.3/1023) * (20/2)
#define CURRENT_SCALING (33.0f / 1023.0f)
#define CURRENT_PERIOD_MS 200
#define CHANNEL_SETTLE_MS 10

#define CAN_BAUD_RATE 500000
#define COMMAND_TIMEOUT_MS 500

#define CAN_ID_LEFT_MOTOR  0x4c
#define CAN_ID_RIGHT_MOTOR 0x78
#define CAN_ID_EXCAVATION_MOTOR 0x48 // CHANGE TO REAL VALUE!!!!!!!!!!!!

#define LOCOMOTION_DUTY_CYCLE 0.33f
#define EXCAVATION_DUTY_CYCLE 0.4f
// stepper_test established ~900 µs (STEP_DELAY=450 µs half-period) as the no-load speed limit.
// 800 µs (1250 Hz) exceeded that limit, causing startup stalls and noise under load.
// 1200 µs (833 Hz) gives ~25% margin and matches well-within-torque-band operation.
// Tune down toward 900 µs only after verifying reliable start under full mechanical load.
#define EXCAVATION_STEP_PERIOD 1000 // microseconds, time between each step change in excavation speed

#define STEPPER_DIR_PIN 6
#define STEPPER_STEP_PIN 7
#define STEPPER_ENABLE_PIN 8

#define STRING_POT_PIN 39

// Microstepping pins on DRV8825:
#define STEPPER_M0_PIN 10
#define STEPPER_M1_PIN 11
#define STEPPER_M2_PIN 12

#define STEPS_PER_REVOLUTION 200  // full-step count
#define MICROSTEPPING_FACTOR 1  // set this to 1,2,4,8,16,32 depending on desired microstepping mode

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
#define GRP_EXCAVATION         0x6
#define GRP_DEPOSITION         0x7
#define GRP_DATA               0x8

#else
#define GRP_EXCAVATION_BELT    0x6
#define GRP_EXCAVATION_VERT    0x7
#define GRP_DEPOSITION_DOOR    0x8
#define GRP_DEPOSITION_VIB     0x9
#define GRP_DATA               0xA

#endif

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
