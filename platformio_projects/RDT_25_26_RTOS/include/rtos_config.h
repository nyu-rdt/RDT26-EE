#pragma once
#include <stdint.h>

// Stack sizes (words)
#define STACK_SAFETY        640
#define STACK_CMD_DECODE    640
#define STACK_MOTOR_CTRL    640
#define STACK_MECHANISM     640
#define STACK_SENSOR        512
#define STACK_TELEMETRY     640
#define STACK_DEBUG         640

// Task priorities (higher number = higher priority)
#define PRI_SAFETY      7
#define PRI_MOTOR_CTRL  4
#define PRI_MECHANISM   4
#define PRI_CMD_DECODE  3
#define PRI_SENSOR      2
#define PRI_TELEMETRY   2
#define PRI_DEBUG       1

// Queue depths 
#define I2C_RX_QUEUE_DEPTH  32

// Task periods (ms)
#define PERIOD_SAFETY_MS        5
#define PERIOD_MOTOR_CTRL_MS   20
#define PERIOD_MECHANISM_MS     5
#define PERIOD_SENSOR_MS       10
#define PERIOD_TELEMETRY_MS    20
#define PERIOD_DEBUG_MS        50

// Diagnostics — pick at most one.
// RTOS_DEBUG_INSTRUMENTATION : Teleplot-format periodic stream (>key:val).
//                              Use with the Teleplot VS Code extension.
// RTOS_SERIAL_DEBUG          : Human-readable prints — startup banner,
//                              per-command logs, e-stop transitions.
//                              Waits for USB serial on boot (blocks until
//                              monitor opens, so don't use in standalone runs).
#define RTOS_DEBUG_INSTRUMENTATION 1
#define RTOS_SERIAL_DEBUG          0
#define TICK_IN_DEBUG 1

#if RTOS_DEBUG_INSTRUMENTATION && RTOS_SERIAL_DEBUG
#  error "Enable only one of RTOS_DEBUG_INSTRUMENTATION or RTOS_SERIAL_DEBUG, not both."
#endif

// Sensor feature flags
// Each sensor can be toggled independently. When disabled the telemetry slot
// for that sensor is filled with 0xFF (sentinel, matches superloop convention).
#define SENSOR_CURRENT_ENABLED    1   // 8-channel current mux (INA or shunt)
#define SENSOR_ENCODERS_ENABLED   1   // rotary encoders (left + right wheels)
#define SENSOR_LOAD_CELLS_ENABLED 0   // load cells (not used until calibrated)
#define SENSOR_STRING_POT_ENABLED 1   // excavation arm string potentiometer (always on)

#define SIMULATE_CAN 0

// Stall detection flag
// When 0: stall flags are never set; overcurrent bit in egSafetyBits stays
// clear regardless of current readings. Telemetry stall bits send 0.
// When 1: SafetyTask evaluates current thresholds and can set SAFETY_OVERCURRENT.
// Still in testing — keep 0 until thresholds are measured and validated.
#define STALL_DETECTION_ENABLED   0

//  Pin assignments (make sue it matches whatever is currently in pins.h in superloop implem)
#define PIN_RELAY_DRIVER    2
#define PIN_RELAY_READ      3
#define PIN_RELAY_3S_LOW    4
#define PIN_RELAY_6S_LOW    5
#define PIN_STEPPER_DIR     6
#define PIN_STEPPER_STEP    7
#define PIN_STEPPER_ENABLE  8
#define PIN_STEPPER_M0      10
#define PIN_STEPPER_M1      11
#define PIN_STEPPER_M2      12
#define PIN_DEPO_DOOR_ENA   14
#define PIN_I2C_SDA         25
#define PIN_I2C_SCL         24
#define PIN_CURRENT_INPUT   26
#define PIN_CURRENT_SEL0    27
#define PIN_CURRENT_SEL1    28
#define PIN_CURRENT_SEL2    29
#define PIN_VIB_MOTOR       30
#define PIN_STRING_POT      39
#define PIN_DEPO_DOOR_IN1   40
#define PIN_DEPO_DOOR_IN2   41
#define PIN_ENC1_A          21
#define PIN_ENC1_B          20
#define PIN_ENC2_A          19
#define PIN_ENC2_B          18

// Encoder config
#define ENCODER_COUNTS_PER_REV  8192.0f

// Depo door travel limits (ms) — measure and update on hardware
#define DEPO_DOOR_OPEN_TRAVEL_MS   5000UL
#define DEPO_DOOR_CLOSE_TRAVEL_MS  5000UL

// Command protocol
#define CMD_GROUP(c)       (((c) >> 4) & 0xF)
#define CMD_PARAM(c)       ((c) & 0xF)

#define GRP_CONTROL        0x0
#define GRP_LOCO_STOP      0x1
#define GRP_FORWARD        0x2
#define GRP_BACKWARD       0x3
#define GRP_LEFT           0x4
#define GRP_RIGHT          0x5
#define GRP_EXCAVATION     0x6
#define GRP_DEPOSITION     0x7
#define GRP_DATA           0x8

// Subcommand constants for GRP_EXCAVATION (old mapping)
#define EXCAV_VERT_STOP    0
#define EXCAV_VERT_DOWN    1
#define EXCAV_VERT_UP      2
#define EXCAV_BELT_STOP    3
#define EXCAV_BELT_FWD     4
#define EXCAV_BELT_REV     5

// Subcommand constants for GRP_DEPOSITION (old mapping)
#define DEPO_DOOR_OPEN     0
#define DEPO_DOOR_CLOSE    1
#define DEPO_VIB_ON        2
#define DEPO_VIB_OFF       3

// Subcommand for GRP_CONTROL
#define CTRL_SW_ESTOP      0x1

// Speed table: param 0-3 normalized duty fraction
static const float kSpeedTable[4] = { 0.25f, 0.50f, 0.75f, 1.00f };
#define LOCO_DUTY          0.33f
#define LOCO_DUTY_EXCAV    0.10f   // reduced cap while belt spinning
#define EXCAV_BELT_DUTY    0.40f
