#pragma once
#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "rtos_types.h"
#include "rtos_config.h"

// IPC handles
extern QueueHandle_t      qI2cRxBytes;
extern SemaphoreHandle_t  mCanTx;
extern SemaphoreHandle_t  mDesiredState;
extern EventGroupHandle_t egSafetyBits;

// Task handles (for direct notifications)
extern TaskHandle_t hMotorCtrlTask;
extern TaskHandle_t hMechanismTask;

// Shared data
extern DesiredState     gDesiredState;
extern SensorSnapshot   gSensorSnapshot;
extern StepperPulsePlan gStepperPlan;

// Telemetry double buffer: TelemetryTask writes to the non-active slot,
// then flips gTelemetryReady. I2C request ISR reads the active slot.
extern volatile uint8_t gTelemetryReady;
extern uint8_t gTelemetryBuf[2][18];   // 18 = DATA_PACKET_SIZE

// Diagnostic counters (volatile — written from ISR and task contexts)
extern volatile uint32_t diag_i2c_rx_overflow;
extern volatile uint32_t diag_cmd_rejected_killswitch;
extern volatile uint32_t diag_cmd_invalid;
extern volatile uint32_t diag_timeout_events;
extern volatile uint32_t diag_safety_transitions;
extern volatile uint32_t diag_telemetry_serialize_errors;
extern volatile uint32_t diag_i2c_short_packets;

void SHARED_STATE_Init();
