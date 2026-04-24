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
    // If setup() crashes the relay stays HIGH rather than floating.
    pinMode(2, OUTPUT);              // RELAY_DRIVER_PIN
    digitalWrite(2, HIGH);

    // All actuator outputs default to safe (off) state
    pinMode(6,  OUTPUT); digitalWrite(6,  LOW);    // STEPPER_DIR_PIN
    pinMode(7,  OUTPUT); digitalWrite(7,  LOW);    // STEPPER_STEP_PIN
    pinMode(8,  OUTPUT); digitalWrite(8,  HIGH);   // STEPPER_ENABLE_PIN (HIGH=disable)
    pinMode(10, OUTPUT); digitalWrite(10, LOW);    // STEPPER_M0
    pinMode(11, OUTPUT); digitalWrite(11, LOW);    // STEPPER_M1
    pinMode(12, OUTPUT); digitalWrite(12, LOW);    // STEPPER_M2
    pinMode(14, OUTPUT); digitalWrite(14, LOW);    // DEPO_DOOR_ENA_PIN
    pinMode(40, OUTPUT); digitalWrite(40, LOW);    // DEPO_DOOR_IN1_PIN
    pinMode(41, OUTPUT); digitalWrite(41, LOW);    // DEPO_DOOR_IN2_PIN
    pinMode(30, OUTPUT); digitalWrite(30, LOW);    // VIB_MOTOR_PIN

    // Relay read inputs
    pinMode(3, INPUT_PULLDOWN);    // RELAY_READ_PIN
    pinMode(4, INPUT_PULLDOWN);    // RELAY_3S_LOW_PIN
    pinMode(5, INPUT_PULLDOWN);    // RELAY_6S_LOW_PIN

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

    // Half-period of EXCAVATION_STEP_PERIOD_UP (1050µs → 525µs).
    // MechanismTask will update direction; ISR just toggles the pin.
    s_stepTimer.begin(ISR_StepperTimer, 525);
}
