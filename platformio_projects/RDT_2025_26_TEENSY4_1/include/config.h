#pragma once
#include <stdint.h>
#include "pins.h"

// ── Feature flags ─────────────────────────────────────────────────────────────
#define RAMP_UP        1
#define SERIAL_DEBUG   1
// PLOT_DATA outputs sensor data in Teleplot format (>name:value).
// Mutually exclusive with SERIAL_DEBUG — enabling both corrupts the plotter stream.
#define PLOT_DATA      0
#define PLOT_PERIOD_MS 50
#define USE_TIMEOUT    0
#define ANALOG_VIB_CONTROL 0
#define USE_OLD_HEX_MAPPING 1

// ── Sensor enable flags ───────────────────────────────────────────────────────
// Set ALL_SENSORS_ENABLED=1 to enable everything at once (bring-up only).
#define ALL_SENSORS_ENABLED 0

#if ALL_SENSORS_ENABLED
    #define CURRENT_SENSE_ENABLED   1
    #define ROTARY_ENCODERS_ENABLED 1
    #define LOAD_CELLS_ENABLED      1
    #define STRING_POT_ENABLED      1
    #define GATE_POS_ENABLED        1
#else
    #define CURRENT_SENSE_ENABLED   1
    #define ROTARY_ENCODERS_ENABLED 1
    #define LOAD_CELLS_ENABLED      0
    #define STRING_POT_ENABLED      1
    #define GATE_POS_ENABLED        1
#endif

// ── I2C ───────────────────────────────────────────────────────────────────────
#define I2C_CHILD_ADDRESS  0x08
#define COMMAND_TIMEOUT_MS 1000
// Size of the SW telemetry packet. Must match SW expectation.
// Unimplemented sensors send 0xFF as a sentinel.
#define DATA_PACKET_SIZE 18

// ── CAN bus ───────────────────────────────────────────────────────────────────
#define CAN_BAUD_RATE           500000
#define CAN_ID_LEFT_MOTOR       0x4C
#define CAN_ID_RIGHT_MOTOR      0x78
#define CAN_ID_EXCAVATION_MOTOR 0x48 // VERIFY — may need update

// ── Locomotion ────────────────────────────────────────────────────────────────
#define LOCOMOTION_DUTY_CYCLE       0.33f
#define LOCOMOTION_DUTY_CYCLE_EXCAV 0.1f  // reduced cap while belt is spinning
#define TX_PERIOD_MS                20
#define MAX_SPEED_DELTA_PER_TICK    0.01f

// ── Excavation ────────────────────────────────────────────────────────────────
#define EXCAVATION_DUTY_CYCLE       0.4f
#define MAX_EXCAV_DELTA_PER_TICK    0.01f
// ~900 µs is the no-load speed limit (established by stepper_test).
// 1000 µs gives margin; tune down toward 900 µs only after verifying reliable
// start under full mechanical load.
#define EXCAVATION_STEP_PERIOD_DOWN 1500  // µs full step period descending (slower — active dig)
#define EXCAVATION_STEP_PERIOD_UP   1050  // µs full step period ascending (faster — recovery)

#define STEPS_PER_REVOLUTION        200   // full-step count
#define MICROSTEPPING_FACTOR        1     // 1, 2, 4, 8, 16, or 32

// String pot calibration — board-specific, tune after physical testing
#define STRING_POT_SCALE            37.125f
#define STRING_POT_OFFSET           -3.511f
#define STRING_POT_MAX_RAW          1023.0f
#define STRING_POT_MAX_DISTANCE     ((STRING_POT_SCALE * 3.3f) - STRING_POT_OFFSET)
#define STRING_POT_LOWEST_THRESHOLD  13.0f
#define STRING_POT_HIGHEST_THRESHOLD 31.0f
#define STRING_MOVING 0
#define STRING_LOWEST 1
#define STRING_HIGHEST 2
#define STRING_MIDDLE 3


// ── Deposition ────────────────────────────────────────────────────────────────
// Time-based control: tune OPEN/CLOSE_TRAVEL_MS on hardware before first use
#define DEPOSITION_DOOR_OPEN_TRAVEL_MS    5000UL  // TODO: measure on hardware
#define DEPOSITION_DOOR_CLOSE_TRAVEL_MS   5000UL  // TODO: measure on hardware

// PWM speed control via ENA pin — follows same pattern as ANALOG_VIB_CONTROL
// When 0, ENA is driven HIGH (full speed)
#define ANALOG_DOOR_CONTROL 1
#if ANALOG_DOOR_CONTROL
    #define DEPOSITION_DOOR_ENA_DUTY  128  // TODO: tune (0-255)
#endif

// Current-based end-stop detection — set to 1 once threshold is measured on hardware
#define DEPOSITION_DOOR_ENABLE_CURRENT_STOP    0
#define DEPOSITION_DOOR_CURRENT_SENSOR_INDEX   0
#define DEPOSITION_DOOR_CURRENT_THRESHOLD_A    8.0f  // TODO: measure on hardware
#define DEPOSITION_DOOR_CURRENT_DETECT_MIN_MS  250

#if ANALOG_VIB_CONTROL
    #define VIB_MOTOR_DUTY_CYCLE 0.6f
#endif

// ── Current sensors ───────────────────────────────────────────────────────────
#define NUM_CURRENT_SENSORS  8
// 0-20A maps to 0-2V; ADC 0-3.3V → 0-1023. A = raw * (3.3/1023) * (20/2)
#define CURRENT_SCALING      (33.0f / 1023.0f)
#define CURRENT_PERIOD_MS    200
#define CHANNEL_SETTLE_MS    10

// ── Rotary encoders ───────────────────────────────────────────────────────────
#define ENCODER_COUNTS_PER_REV  8192.0f
#define DEGREES_PER_COUNT       (360.0f / ENCODER_COUNTS_PER_REV)

// ── Command protocol ──────────────────────────────────────────────────────────
#define GRP_CONTROL   0x0
#define GRP_LOCO_STOP 0x1
#define GRP_FORWARD   0x2
#define GRP_BACKWARD  0x3
#define GRP_LEFT      0x4
#define GRP_RIGHT     0x5

#if USE_OLD_HEX_MAPPING
    #define GRP_EXCAVATION  0x6
    #define GRP_DEPOSITION  0x7
    #define GRP_DATA        0x8
#else
    #define GRP_EXCAVATION_BELT 0x6
    #define GRP_EXCAVATION_VERT 0x7
    #define GRP_DEPOSITION_DOOR 0x8
    #define GRP_DEPOSITION_VIB  0x9
    #define GRP_DATA            0xA
#endif

#define CMD_GROUP(c) (((c) >> 4) & 0xF)
#define CMD_PARAM(c) ((c) & 0xF)

#define SPEED_TABLE { 0.25f, 0.50f, 0.75f, 1.00f }

static inline float getSpeed(uint8_t idx, float duty = LOCOMOTION_DUTY_CYCLE) {
    static constexpr float t[] = SPEED_TABLE;
    if (idx > 3U) idx = 3U;
    return t[idx] * duty;
}

#define GET_SPEED(idx)     (getSpeed(static_cast<uint8_t>(idx)))
#define GET_DIRECTION(p)   (((p) == 0x1) ? 1 : ((p) == 0x2) ? -1 : 0)

typedef void (*GroupHandler)(uint8_t param);
