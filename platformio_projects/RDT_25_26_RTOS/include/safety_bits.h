#pragma once
#include <stdint.h>

// Bits in egSafetyBits (EventGroupHandle_t).
// Single writer: SafetyTask
// Readers: CmdDecodeTask, MotorControlTask, MechanismTask, TelemetryTask.
#define SAFETY_HW_ESTOP        (1UL << 0)   // relay not engaged
#define SAFETY_SW_ESTOP        (1UL << 1)   // software-commanded e-stop
#define SAFETY_COMMS_TIMEOUT   (1UL << 2)   // no command for COMMAND_TIMEOUT_MS
#define SAFETY_OVERCURRENT     (1UL << 3)   // any channel over threshold

// Any bit that forces actuators to zero
#define SAFETY_ANY_STOP  (SAFETY_HW_ESTOP | SAFETY_SW_ESTOP | SAFETY_COMMS_TIMEOUT | SAFETY_OVERCURRENT)
