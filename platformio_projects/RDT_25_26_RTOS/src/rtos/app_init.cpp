#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include "shared_state.h"
#include "rtos_config.h"

// Task entry points
void TaskSafety(void*);
void TaskCmdDecode(void*);
void TaskMotorControl(void*);
void TaskMechanism(void*);
void TaskSensor(void*);
void TaskTelemetry(void*);
void TaskDebug(void*);

// ISR callbacks
void ISR_I2C_OnReceive(int);
void ISR_I2C_OnRequest();
void ISR_StepperTimer();
void ENCODER_Init();

static IntervalTimer s_stepTimer;

void APP_Init() {
    Serial.begin(115200);

#if RTOS_SERIAL_DEBUG
    while (!Serial && millis() < 3000) {}
    Serial.println("=== RDT26 RTOS BOOT ===");
    Serial.printf("  Tasks:   Safety=%dms  CmdDecode=event  Motor=%dms  Mech=%dms\n",
                  PERIOD_SAFETY_MS, PERIOD_MOTOR_CTRL_MS, PERIOD_MECHANISM_MS);
    Serial.printf("  Sensor=%dms  Telemetry=%dms  Debug=%dms\n",
                  PERIOD_SENSOR_MS, PERIOD_TELEMETRY_MS, PERIOD_DEBUG_MS);
    Serial.printf("  Stall detection: %s\n", STALL_DETECTION_ENABLED ? "ON" : "OFF");
    Serial.println("=======================");
    Serial.flush();
    delay(3000);
#endif

#if RTOS_SERIAL_DEBUG    // No flush here — avoid hanging if flush loops forever post-scheduler.
#define DBG_CHECKPOINT(n) do { delay(2000); Serial.println("[init] step " #n); Serial.flush(); } while(0)
#else
#define DBG_CHECKPOINT(n) do { } while(0)
#endif

    DBG_CHECKPOINT(1);  // relay + gpio

    // Energize relay immediately — fail-safe before scheduler starts.
    // If setup() crashes after this point, the relay stays HIGH rather than floating.
    pinMode(PIN_RELAY_DRIVER, OUTPUT);
    digitalWrite(PIN_RELAY_DRIVER, HIGH);

    // All actuator outputs default to safe (off) state before any task runs.
    pinMode(PIN_STEPPER_DIR,    OUTPUT); digitalWrite(PIN_STEPPER_DIR,    LOW);
    pinMode(PIN_STEPPER_STEP,   OUTPUT); digitalWrite(PIN_STEPPER_STEP,   LOW);
    pinMode(PIN_STEPPER_ENABLE, OUTPUT); digitalWrite(PIN_STEPPER_ENABLE, HIGH);  // HIGH=disable
    pinMode(PIN_STEPPER_M0,     OUTPUT); digitalWrite(PIN_STEPPER_M0,     LOW);
    pinMode(PIN_STEPPER_M1,     OUTPUT); digitalWrite(PIN_STEPPER_M1,     LOW);
    pinMode(PIN_STEPPER_M2,     OUTPUT); digitalWrite(PIN_STEPPER_M2,     LOW);
    pinMode(PIN_DEPO_DOOR_ENA,  OUTPUT); digitalWrite(PIN_DEPO_DOOR_ENA,  LOW);
    pinMode(PIN_DEPO_DOOR_IN1,  OUTPUT); digitalWrite(PIN_DEPO_DOOR_IN1,  LOW);
    pinMode(PIN_DEPO_DOOR_IN2,  OUTPUT); digitalWrite(PIN_DEPO_DOOR_IN2,  LOW);
    pinMode(PIN_VIB_MOTOR,      OUTPUT); digitalWrite(PIN_VIB_MOTOR,      LOW);

    // Relay and battery sense inputs
    pinMode(PIN_RELAY_READ,   INPUT_PULLDOWN);
    pinMode(PIN_RELAY_3S_LOW, INPUT_PULLDOWN);
    pinMode(PIN_RELAY_6S_LOW, INPUT_PULLDOWN);

    DBG_CHECKPOINT(2);  // encoders
    ENCODER_Init();

    DBG_CHECKPOINT(3);  // CAN
    gCan.begin();
    gCan.setBaudRate(500000);

    // Reject all incoming frames at the mailbox layer. We never call gCan.read()
    // and register no listeners, so anything the controller copies into mailbox
    // RAM is stale junk. even with the IRQ disabled below, this keeps
    // mailbox state clean if something ever re-enables the IRQ later.
    gCan.setMBFilter(REJECT_ALL);

    // Disable the FlexCAN NVIC IRQ entirely. The ISR's only job for us would be
    // TX-completion bookkeeping (advancing the SW FIFO into mailboxes), but with
    // 3 writes per 20 ms and 58 free mailboxes we never need the SW FIFO —
    // writes go straight into a mailbox.
    
    // since SysTick (the FreeRTOS tick) sits at NVIC priority
    // 240 and vPortSetupTimerInterrupt() resets every other IRQ to priority 128,
    // CAN1 preempts SysTick. On the robot bus, each TX completion fires the
    // FlexCAN ISR, which loops over mailboxes, reads/clears ESR1, and walks
    // listener arrays — long enough that back-to-back ISR calls starve SysTick.
    // No tick means every vTaskDelayUntil() blocks forever and the whole scheduler hangs.
    // Killing the IRQ is free for us since we use none of its behavior.
    NVIC_DISABLE_IRQ(IRQ_CAN1);

    DBG_CHECKPOINT(4);  // shared state / IPC primitives
    SHARED_STATE_Init();

    DBG_CHECKPOINT(5);  // I2C
    Wire2.begin(0x08);
    Wire2.onReceive(ISR_I2C_OnReceive);
   Wire2.onRequest(ISR_I2C_OnRequest);

    DBG_CHECKPOINT(6);  // task creation
    configASSERT(xTaskCreate(TaskMotorControl, "MotorCtrl",  STACK_MOTOR_CTRL, nullptr, PRI_MOTOR_CTRL, &hMotorCtrlTask) == pdPASS);
    configASSERT(xTaskCreate(TaskMechanism,    "Mechanism",  STACK_MECHANISM,  nullptr, PRI_MECHANISM,  &hMechanismTask) == pdPASS);
    configASSERT(xTaskCreate(TaskSafety,       "Safety",     STACK_SAFETY,     nullptr, PRI_SAFETY,     nullptr)         == pdPASS);
    configASSERT(xTaskCreate(TaskCmdDecode,    "CmdDecode",  STACK_CMD_DECODE, nullptr, PRI_CMD_DECODE, nullptr)         == pdPASS);
    configASSERT(xTaskCreate(TaskSensor,       "Sensor",     STACK_SENSOR,     nullptr, PRI_SENSOR,     nullptr)         == pdPASS);
    configASSERT(xTaskCreate(TaskTelemetry,    "Telemetry",  STACK_TELEMETRY,  nullptr, PRI_TELEMETRY,  nullptr)         == pdPASS);
    configASSERT(xTaskCreate(TaskDebug,        "Debug",      STACK_DEBUG,      nullptr, PRI_DEBUG,      nullptr)         == pdPASS);

    DBG_CHECKPOINT(7);  // stepper timer
    s_stepTimer.begin(ISR_StepperTimer, 525);

    DBG_CHECKPOINT(8);  // handing off to vTaskStartScheduler
}
