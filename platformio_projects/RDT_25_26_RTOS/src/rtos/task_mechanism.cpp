#include <Arduino.h>
#include <climits>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

enum class DoorState { IDLE, OPENING, CLOSING };

static void doorStop() {
    digitalWrite(PIN_DEPO_DOOR_ENA, LOW);
    digitalWrite(PIN_DEPO_DOOR_IN1, LOW);
    digitalWrite(PIN_DEPO_DOOR_IN2, LOW);
}
static void doorOpen() {
    analogWrite(PIN_DEPO_DOOR_ENA, 128);
    digitalWrite(PIN_DEPO_DOOR_IN1, HIGH);
    digitalWrite(PIN_DEPO_DOOR_IN2, LOW);
}
static void doorClose() {
    analogWrite(PIN_DEPO_DOOR_ENA, 128);
    digitalWrite(PIN_DEPO_DOOR_IN1, LOW);
    digitalWrite(PIN_DEPO_DOOR_IN2, HIGH);
}

void TaskMechanism(void*) {
    bool      stopped    = true;
    DoorState doorState  = DoorState::IDLE;
    uint32_t  doorStartMs = 0;
    TickType_t lastWake  = xTaskGetTickCount();

    for (;;) {
        uint32_t notif = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &notif, 0) == pdTRUE) {
            if (notif == 1U) { stopped = true; doorStop(); }
            else             stopped = false;
        }
        if (xEventGroupGetBits(egSafetyBits) & SAFETY_ANY_STOP) {
            stopped = true;
            doorStop();
        }

        if (!stopped) {
            int8_t vertDir = 0;
            int8_t doorCmd = 0;
            int8_t vibCmd  = 0;
            if (xSemaphoreTake(mDesiredState, 0) == pdTRUE) {
                vertDir = gDesiredState.excav_vert_dir;
                doorCmd = gDesiredState.depo_door_cmd;
                vibCmd  = gDesiredState.depo_vib_cmd;
                xSemaphoreGive(mDesiredState);
            }

            // Stepper direction and enable
            if (vertDir != 0) {
                digitalWrite(PIN_STEPPER_DIR,    (vertDir == 1) ? HIGH : LOW);
                digitalWrite(PIN_STEPPER_ENABLE, LOW);   // LOW = enabled
            } else {
                digitalWrite(PIN_STEPPER_ENABLE, HIGH);  // disable when stopped
            }
            gStepperPlan.dir     = (vertDir == 1) ? 1 : 0;
            gStepperPlan.enabled = (vertDir != 0) ? 1 : 0;

            // Vib motor
            digitalWrite(PIN_VIB_MOTOR, (vibCmd == 1) ? HIGH : LOW);

            // Depo door state machine (time-based, mirrors superloop DEPO_Update)
            switch (doorState) {
                case DoorState::IDLE:
                    if (doorCmd ==  1) { doorOpen();  doorState = DoorState::OPENING; doorStartMs = millis(); }
                    if (doorCmd == -1) { doorClose(); doorState = DoorState::CLOSING; doorStartMs = millis(); }
                    break;
                case DoorState::OPENING:
                    if ((millis() - doorStartMs) >= 5000UL) { doorStop(); doorState = DoorState::IDLE; }
                    break;
                case DoorState::CLOSING:
                    if ((millis() - doorStartMs) >= 5000UL) { doorStop(); doorState = DoorState::IDLE; }
                    break;
            }
        } else {
            gStepperPlan.enabled = 0;
            digitalWrite(PIN_STEPPER_ENABLE, HIGH);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MECHANISM_MS));
    }
}
