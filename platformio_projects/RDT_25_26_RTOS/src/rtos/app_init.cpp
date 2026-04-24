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

static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> s_can;
static IntervalTimer s_stepTimer;

void APP_Init() {
    Serial.begin(115200);

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

    s_can.begin();
    s_can.setBaudRate(500000);

    // Create all IPC primitives before any ISR or task uses them
    SHARED_STATE_Init();

    // Register I2C callbacks after queues exist
    Wire2.begin(0x08);   // I2C_CHILD_ADDRESS
    Wire2.onReceive(ISR_I2C_OnReceive);
    Wire2.onRequest(ISR_I2C_OnRequest);

    // Motor task first so hMotorCtrlTask/hMechanismTask are valid
    // before SafetyTask starts sending notifications
    configASSERT(xTaskCreate(TaskMotorControl, "MotorCtrl",  STACK_MOTOR_CTRL, nullptr, PRI_MOTOR_CTRL, &hMotorCtrlTask) == pdPASS);
    configASSERT(xTaskCreate(TaskMechanism,    "Mechanism",  STACK_MECHANISM,  nullptr, PRI_MECHANISM,  &hMechanismTask) == pdPASS);
    configASSERT(xTaskCreate(TaskSafety,       "Safety",     STACK_SAFETY,     nullptr, PRI_SAFETY,     nullptr)         == pdPASS);
    configASSERT(xTaskCreate(TaskCmdDecode,    "CmdDecode",  STACK_CMD_DECODE, nullptr, PRI_CMD_DECODE, nullptr)         == pdPASS);
    configASSERT(xTaskCreate(TaskSensor,       "Sensor",     STACK_SENSOR,     nullptr, PRI_SENSOR,     nullptr)         == pdPASS);
    configASSERT(xTaskCreate(TaskTelemetry,    "Telemetry",  STACK_TELEMETRY,  nullptr, PRI_TELEMETRY,  nullptr)         == pdPASS);
    configASSERT(xTaskCreate(TaskDebug,        "Debug",      STACK_DEBUG,      nullptr, PRI_DEBUG,      nullptr)         == pdPASS);

    // Start the stepper IntervalTimer before vTaskStartScheduler().
    // Safe because gStepperPlan.enabled initializes to 0 (zero-initialized .bss),
    // so the ISR returns immediately without touching any pin until TaskMechanism
    // explicitly sets enabled=1. Moving this into TaskMechanism's setup block
    // would also be correct if stricter pre-scheduler ISR-free operation is needed.
    // Half-period of EXCAVATION_STEP_PERIOD_UP (1050µs → 525µs half-period).
    s_stepTimer.begin(ISR_StepperTimer, 525);
}
