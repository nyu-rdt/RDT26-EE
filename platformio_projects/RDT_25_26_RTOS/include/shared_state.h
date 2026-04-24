#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "rtos_types.h"
#include "rtos_config.h"

// CAN bus instance owned here so MotorControlTask can write via mCanTx
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> gCan;

// IPC handles
extern QueueHandle_t      qI2cRxBytes;
extern SemaphoreHandle_t  mCanTx;
extern SemaphoreHandle_t  mDesiredState;
extern EventGroupHandle_t egSafetyBits;

// Task handles (for direct notifications)
extern TaskHandle_t hMotorCtrlTask;
extern TaskHandle_t hMechanismTask;

// Shared data
extern DesiredState              gDesiredState;
extern SensorSnapshot            gSensorSnapshot;
extern volatile StepperPulsePlan gStepperPlan;  // written by task, read by timer ISR

// Telemetry double buffer: TelemetryTask writes to the non-active slot,
// then flips gTelemetryReady. I2C request ISR reads the active slot.
// Both the array and the index are volatile so the compiler does not cache
// them across the index swap. TelemetryTask must also issue __DMB() between
// the last write to gTelemetryBuf and the assignment to gTelemetryReady to
// prevent the Cortex-M7 store buffer from reordering the writes.
extern volatile uint8_t gTelemetryReady;
extern volatile uint8_t gTelemetryBuf[2][18];   // 18 = DATA_PACKET_SIZE

// Diagnostic counters.
// Each counter has exactly ONE writer (listed below). The ++ on a volatile
// uint32_t is not atomic on Cortex-M7, so a second writer would be a data race.
// Tasks that read these for display must snapshot inside taskENTER_CRITICAL().
//   diag_i2c_rx_overflow          — ISR_I2C_OnReceive only
//   diag_i2c_short_packets        — ISR_I2C_OnRequest only
//   diag_cmd_rejected_killswitch  — TaskCmdDecode only
//   diag_cmd_invalid              — TaskCmdDecode only
//   diag_timeout_events           — TaskSafety only
//   diag_safety_transitions       — TaskSafety only
//   diag_telemetry_serialize_errors — TaskTelemetry only
extern volatile uint32_t diag_i2c_rx_overflow;
extern volatile uint32_t diag_cmd_rejected_killswitch;
extern volatile uint32_t diag_cmd_invalid;
extern volatile uint32_t diag_timeout_events;
extern volatile uint32_t diag_safety_transitions;
extern volatile uint32_t diag_telemetry_serialize_errors;
extern volatile uint32_t diag_i2c_short_packets;

void SHARED_STATE_Init();
