#pragma once

// ── I2C ──────────────────────────────────────────────────────────────────────
#define I2C_CHILD_ADDRESS   0x08    // Teensy listens on Wire2 at this address

// ── E-stop ───────────────────────────────────────────────────────────────────
// Pin 2: drives the relay coil. Keep HIGH to keep relay energised (fail-safe).
// If power is lost or the firmware crashes, the relay de-energises and cuts power.
#define E_STOP_RELAY_DRIVE_PIN  2

// Pin 3: reads relay contact status. LOW = E-stop asserted (active-low, INPUT_PULLUP).
// Change to the real signal pin once confirmed on hardware.
#define E_STOP_READ_PIN         3

// ── CAN ───────────────────────────────────────────────────────────────────────
#define CAN_BAUD_RATE           500000
#define CAN_ID_LEFT_MOTOR       0x4C
#define CAN_ID_RIGHT_MOTOR      0x78
#define CAN_ID_EXCAVATION_MOTOR 0x48    // VERIFY ON HARDWARE before enabling

// ── Timing ───────────────────────────────────────────────────────────────────
#define COMMAND_TIMEOUT_MS      500     // stop motors if no I2C command for this long
#define TX_PERIOD_MS            20      // motor CAN send period (50 Hz)
#define ESTOP_DEBOUNCE_MS       20      // ignore glitches shorter than this

// ── Speed ─────────────────────────────────────────────────────────────────────
#define LOCOMOTION_DUTY_CYCLE   0.33f   // max speed fraction sent on CAN
#define EXCAVATION_DUTY_CYCLE   0.40f
#define MAX_SPEED_DELTA_PER_TICK 0.01f  // ramp rate per TX_PERIOD_MS tick

// ── Command byte decoding ─────────────────────────────────────────────────────
// Format: upper nibble = group, lower nibble = param
#define CMD_GROUP(c)    (((c) >> 4) & 0x0F)
#define CMD_PARAM(c)    ((c) & 0x0F)

// Command groups (matches RDT_2025_26_TEENSY4_1/include/config.h)
#define GRP_CONTROL         0x0
#define GRP_LOCO_STOP       0x1
#define GRP_FORWARD         0x2
#define GRP_BACKWARD        0x3
#define GRP_LEFT            0x4
#define GRP_RIGHT           0x5
#define GRP_EXCAVATION      0x6     // param 0/1/2 = stop/fwd/rev belt

// Speed table: param 0-3 → 25 / 50 / 75 / 100% × LOCOMOTION_DUTY_CYCLE
static inline float getSpeed(uint8_t idx) {
    static constexpr float tbl[] = { 0.25f, 0.50f, 0.75f, 1.00f };
    if (idx > 3U) idx = 3U;
    return tbl[idx] * LOCOMOTION_DUTY_CYCLE;
}

// Direction from param: 0=stop, 1=forward, 2=reverse
static inline float getDirection(uint8_t param) {
    if (param == 1) return  1.0f;
    if (param == 2) return -1.0f;
    return 0.0f;
}
