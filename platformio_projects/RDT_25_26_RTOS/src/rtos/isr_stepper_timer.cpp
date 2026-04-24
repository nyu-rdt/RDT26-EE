#include <Arduino.h>
#include "shared_state.h"

// Fired by IntervalTimer at half the step period.
// Toggles STEP pin each call - one full pulse per two calls.
// ISR rules: no RTOS calls, no prints, no dynamic allocation.
static volatile bool s_stepPinHigh = false;

void ISR_StepperTimer() {
    if (!gStepperPlan.enabled) {
        if (s_stepPinHigh) {
            digitalWriteFast(7, LOW);   // STEPPER_STEP_PIN
            s_stepPinHigh = false;
        }
        return;
    }
    s_stepPinHigh = !s_stepPinHigh;
    digitalWriteFast(7, s_stepPinHigh ? HIGH : LOW);
}
