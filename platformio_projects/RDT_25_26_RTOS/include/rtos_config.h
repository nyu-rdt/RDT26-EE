#pragma once
#include <stdint.h>

// Stack sizes (words)
#define STACK_SAFETY        512
#define STACK_CMD_DECODE    512
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

// Diagnostics
// Gate behind 0 for competition build.
#define RTOS_DEBUG_INSTRUMENTATION 1

// Sensor feature flags
// Each sensor can be toggled independently. When disabled the telemetry slot
// for that sensor is filled with 0xFF (sentinel, matches superloop convention).
#define SENSOR_CURRENT_ENABLED    1   // 8-channel current mux (INA or shunt)
#define SENSOR_ENCODERS_ENABLED   1   // rotary encoders (left + right wheels)
#define SENSOR_LOAD_CELLS_ENABLED 0   // load cells (not used until calibrated)
#define SENSOR_STRING_POT_ENABLED 1   // excavation arm string potentiometer (always on)

// Stall detection flag 
// When 0: stall flags are never set; overcurrent bit in egSafetyBits stays
// clear regardless of current readings. Telemetry stall bits send 0.
// When 1: SafetyTask evaluates current thresholds and can set SAFETY_OVERCURRENT.
// Still in testing — keep 0 until thresholds are measured and validated.
#define STALL_DETECTION_ENABLED   0
