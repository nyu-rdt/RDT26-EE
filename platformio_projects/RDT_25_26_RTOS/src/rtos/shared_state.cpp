#include "shared_state.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> gCan;

QueueHandle_t      qI2cRxBytes   = nullptr;
SemaphoreHandle_t  mCanTx        = nullptr;
SemaphoreHandle_t  mDesiredState = nullptr;
EventGroupHandle_t egSafetyBits  = nullptr;

TaskHandle_t hMotorCtrlTask = nullptr;
TaskHandle_t hMechanismTask = nullptr;

DesiredState              gDesiredState   = {};
SensorSnapshot            gSensorSnapshot = {};
volatile StepperPulsePlan gStepperPlan    = {};

volatile uint8_t gTelemetryReady    = 0;
volatile uint8_t gTelemetryBuf[2][18] = {};

volatile uint32_t diag_i2c_rx_overflow           = 0;
volatile uint32_t diag_cmd_rejected_killswitch    = 0;
volatile uint32_t diag_cmd_invalid                = 0;
volatile uint32_t diag_timeout_events             = 0;
volatile uint32_t diag_safety_transitions         = 0;
volatile uint32_t diag_telemetry_serialize_errors = 0;
volatile uint32_t diag_i2c_short_packets          = 0;

void SHARED_STATE_Init() {
    qI2cRxBytes   = xQueueCreate(I2C_RX_QUEUE_DEPTH, sizeof(uint8_t));
    mCanTx        = xSemaphoreCreateMutex();
    mDesiredState = xSemaphoreCreateMutex();
    egSafetyBits  = xEventGroupCreate();
    configASSERT(qI2cRxBytes   != nullptr);
    configASSERT(mCanTx        != nullptr);
    configASSERT(mDesiredState != nullptr);
    configASSERT(egSafetyBits  != nullptr);
}
