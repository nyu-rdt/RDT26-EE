#include <Arduino.h>
#include <climits>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

// Values match superloop DepoDoorState enum exactly (used in telemetry byte 15).
enum class DoorState { CLOSED = 0, OPENED = 1, OPENING = 2, CLOSING = 4 };

static void doorStop() {
    digitalWrite(PIN_DEPO_DOOR_ENA, LOW);
    digitalWrite(PIN_DEPO_DOOR_IN1, LOW);
    digitalWrite(PIN_DEPO_DOOR_IN2, LOW);
}
static void doorOpen() {
    analogWrite(PIN_DEPO_DOOR_ENA, 128);
    digitalWrite(PIN_DEPO_DOOR_IN1, LOW);
    digitalWrite(PIN_DEPO_DOOR_IN2, HIGH);
}
static void doorClose() {
    analogWrite(PIN_DEPO_DOOR_ENA, 128);
    digitalWrite(PIN_DEPO_DOOR_IN1, HIGH);
    digitalWrite(PIN_DEPO_DOOR_IN2, LOW);
}

void TaskMechanism(void*) {
#if RTOS_SERIAL_DEBUG
    Serial.println("[mechanism] task started");
#endif
    bool      stopped    = true;
    DoorState doorState  = DoorState::CLOSED;
    uint32_t  doorStartMs = 0;
    TickType_t lastWake  = xTaskGetTickCount();
    bool strpotHighLatched = false;
    bool strpotLowLatched  = false;

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

            // String pot over-travel and disconnect guard.
            // Latches are only cleared once the arm moves away by STRPOT_HYST_CM,
            // preventing a noisy dip at the limit from re-enabling movement.
            // A fault (sensor railed high = disconnected) stops vertical entirely.
#if SENSOR_STRING_POT_ENABLED
            {
                float potCm = gSensorSnapshot.string_pot_cm;
                if (strpotHighLatched && potCm < (STRPOT_HIGHEST_CM - STRPOT_HYST_CM))
                    strpotHighLatched = false;
                if (strpotLowLatched  && potCm > (STRPOT_LOWEST_CM  + STRPOT_HYST_CM))
                    strpotLowLatched  = false;
                if (potCm >= STRPOT_HIGHEST_CM) {
                    strpotHighLatched = true;
#if RTOS_SERIAL_DEBUG
                    Serial.printf("[mechanism] string pot high latched at %0.2f cm\n", potCm);
#endif
                }
                if (potCm <= STRPOT_LOWEST_CM) {
                    strpotLowLatched = true;
#if RTOS_SERIAL_DEBUG
                    Serial.printf("[mechanism] string pot low latched at %0.2f cm\n", potCm);
#endif
                }

                bool fault = (potCm > STRPOT_FAULT_HIGH_CM);
#if RTOS_SERIAL_DEBUG
                if (fault) Serial.println("[mechanism] string pot fault - check connection");
#endif
                if (fault ||
                    (vertDir > 0 && strpotHighLatched) ||
                    (vertDir < 0 && strpotLowLatched))
                    vertDir = 0;
            }

#endif

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

            // Depo door state machine (time-based).
            // Any non-zero doorCmd is consumed immediately so it doesn't persist
            // and re-trigger. Opposite commands while moving reverse direction.
            auto consumeDoorCmd = [&]() {
                if (xSemaphoreTake(mDesiredState, 0) == pdTRUE) {
                    gDesiredState.depo_door_cmd = 0;
                    xSemaphoreGive(mDesiredState);
                }
            };
            switch (doorState) {
                case DoorState::CLOSED:
                case DoorState::OPENED:
                    if (doorCmd == 1)  { doorOpen();  doorState = DoorState::OPENING; doorStartMs = millis(); consumeDoorCmd(); }
                    if (doorCmd == -1) { doorClose(); doorState = DoorState::CLOSING; doorStartMs = millis(); consumeDoorCmd(); }
                    break;
                case DoorState::OPENING:
                    if      (doorCmd == -1) { doorClose(); doorState = DoorState::CLOSING; doorStartMs = millis(); consumeDoorCmd(); }
                    else if (doorCmd ==  1) { consumeDoorCmd(); }  // already opening, discard
                    else if ((millis() - doorStartMs) >= DEPO_DOOR_OPEN_TRAVEL_MS) { doorStop(); doorState = DoorState::OPENED; }
                    break;
                case DoorState::CLOSING:
                    if      (doorCmd ==  1) { doorOpen(); doorState = DoorState::OPENING; doorStartMs = millis(); consumeDoorCmd(); }
                    else if (doorCmd == -1) { consumeDoorCmd(); }  // already closing, discard
                    else if ((millis() - doorStartMs) >= DEPO_DOOR_CLOSE_TRAVEL_MS) { doorStop(); doorState = DoorState::CLOSED; }
                    break;
            }
            // Sync to SensorSnapshot each cycle — values match superloop DepoDoorState enum.
            gSensorSnapshot.depo_door_state = static_cast<uint8_t>(doorState);
        } else {
            gStepperPlan.enabled = 0;
            digitalWrite(PIN_STEPPER_ENABLE, HIGH);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MECHANISM_MS));
    }
}
