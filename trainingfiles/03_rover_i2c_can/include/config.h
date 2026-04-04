#pragma once

// ── Simulation flags ─────────────────────────────────────────────────────────
// Set SIMULATE_CAN to 1 when testing without CAN hardware (no motors/controllers
// connected). Without termination and ACK the CAN controller goes bus-off, which
// causes an error-interrupt storm that starves RTOS tasks and breaks E-stop.
#define SIMULATE_CAN        1
#define PRINT_CAN           1       // print CAN messages to Serial even when not simulating
#define CAN_PRINT_PERIOD_MS 500     // how often to print (ms); reduce for faster updates

// ── Experiment knobs — change these, flash, and watch Serial ─────────────────
//
// EXPERIMENT A — overflow the I2C byte queue
//   Reduce I2C_QUEUE_DEPTH to 2, then send a fast burst of I2C commands from the
//   parent Teensy. The ISR will start dropping bytes and TaskI2CDecode will decode
//   garbage or partial commands. Shows why queue depth matters for burst traffic.
//
// EXPERIMENT B — overflow the motor command queue
//   Reduce MOTOR_QUEUE_DEPTH to 1 and send commands faster than every 20ms.
//   You'll see "[I2CDecode] WARNING: motor queue full" — the decode task is
//   producing faster than the 20ms motor loop consumes.
//
// EXPERIMENT C — tune the speed ramp
//   Increase MAX_SPEED_DELTA_PER_TICK to 0.5 for instant speed changes (no ramp).
//   Decrease to 0.002 for a very slow ramp (~10s to reach full speed).
//   With SIMULATE_CAN you can watch the "[CAN SIM]" speed values change each tick.
//
// EXPERIMENT D — command timeout
//   Reduce COMMAND_TIMEOUT_MS to 100 then stop sending I2C commands. Within 100ms
//   you'll see the CAN SIM speed values ramp back to 0 automatically.
//   Raise it to 5000 to give yourself more time between commands.
//
// EXPERIMENT E — E-stop debounce
//   Reduce ESTOP_DEBOUNCE_MS to 1. Brief noise on pin 3 may now trigger false
//   E-stops. Raise to 100 and the pin must be held low for 100ms to register.
//
// EXPERIMENT F — stack size vs. heap (same concept as project 02)
//   Increase MOTOR_TASK_STACK to 4096 and see free heap drop on Serial.

// Raw I2C byte queue depth (bytes from Wire2 ISR before TaskI2CDecode drains).
#define I2C_QUEUE_DEPTH         16

// Decoded MotorCommand_t queue depth (commands waiting for TaskMotorCtrl).
#define MOTOR_QUEUE_DEPTH       4

// Task stack sizes in words (1 word = 4 bytes). See experiment F.
#define MOTOR_TASK_STACK        512
#define DECODE_TASK_STACK       512
#define ESTOP_TASK_STACK        384

// ── I2C ──────────────────────────────────────────────────────────────────────
#define I2C_CHILD_ADDRESS   0x08    // Teensy listens on Wire2 at this address

// ── E-stop ───────────────────────────────────────────────────────────────────
// Pin 2: drives the relay coil. Keep HIGH to keep relay energised (fail-safe).
// If power is lost or the firmware crashes, the relay de-energises and cuts power.
#define E_STOP_RELAY_DRIVE_PIN  2

// Pin 3: reads relay contact status. LOW = E-stop asserted (active-low, INPUT_PULLDOWN).
// With INPUT_PULLDOWN the pin sits LOW when nothing is connected (safe default — asserted).
// The relay NO contact must drive pin 3 HIGH during normal operation to de-assert E-stop.
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
