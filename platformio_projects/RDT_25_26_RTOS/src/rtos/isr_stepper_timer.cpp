#include <Arduino.h>
#include "shared_state.h"
#include "rtos_config.h"

// Fired by IntervalTimer at half the step period.
// Toggles STEP pin each call - one full pulse per two calls.
// ISR rules: no RTOS calls, no prints, no dynamic allocation.
static volatile bool s_stepPinHigh = false;

void ISR_StepperTimer() {
    if (!gStepperPlan.enabled) {
        if (s_stepPinHigh) {
            digitalWriteFast(PIN_STEPPER_STEP, LOW);
            s_stepPinHigh = false;
        }
        return;
    }
    s_stepPinHigh = !s_stepPinHigh;
    digitalWriteFast(PIN_STEPPER_STEP, s_stepPinHigh ? HIGH : LOW);
}
