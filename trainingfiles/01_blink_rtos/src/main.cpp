/*
 * 01_blink_rtos — Introduction to FreeRTOS tasks
 * ================================================
 * Goal: Show that FreeRTOS lets you run multiple independent loops
 * "at the same time" without any extra wiring between them.
 *
 * Two tasks blink the built-in LED (pin 13) at different rates:
 *   TaskBlink500  → toggles every 500 ms
 *   TaskBlink1200 → toggles every 1200 ms
 *
 * In a plain Arduino sketch you could only do one rate cleanly. With RTOS
 * you just write each loop independently and the scheduler handles the rest.
 *
 * Key concepts introduced here:
 *   xTaskCreate()        — register a function as a task
 *   vTaskDelay()         — yield to other tasks for N ticks (1 tick = 1 ms)
 *   pdMS_TO_TICKS()      — convert milliseconds → tick count safely
 *   vTaskStartScheduler() — hand control to the RTOS (never returns)
 *   loop()               — empty; RTOS replaces it
 */

#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"

// ── Task functions ───────────────────────────────────────────────────────────
// Every FreeRTOS task has the signature: void myTask(void *pvParams)
// The pvParams pointer lets you pass data in at creation time; we ignore it here.

void TaskBlink500(void *pvParams) {
    // Each task has its OWN infinite loop — it never returns to main.
    for (;;) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        Serial.println("[500ms] toggle");

        // vTaskDelay suspends THIS task for the given number of ticks and
        // lets other tasks run. It is NOT like delay() — other tasks keep
        // executing while this one sleeps.
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    // A task that exits its loop must call vTaskDelete(NULL) — but since
    // our loop is infinite we never reach here.
}

void TaskBlink1200(void *pvParams) {
    for (;;) {
        // We read back the pin state just to show both tasks sharing the same LED.
        // Watch the serial output — you'll see the 500ms task interleaving here.
        Serial.println("[1200ms] toggle");
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}

// ── setup() — runs once before the scheduler starts ─────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}   // wait for USB serial (up to 3s)

    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("=== 01_blink_rtos ===");
    Serial.println("Creating tasks...");

    // xTaskCreate arguments:
    //   1. task function
    //   2. human-readable name (for debugging)
    //   3. stack size in WORDS (not bytes!) — 256 words = 1024 bytes on 32-bit ARM
    //   4. parameter to pass to the task (NULL = none)
    //   5. priority (higher number = higher priority; max = configMAX_PRIORITIES - 1)
    //   6. task handle pointer (NULL = we don't need to reference this task later)
    BaseType_t ok1 = xTaskCreate(TaskBlink500,  "Blink500",  256, NULL, 1, NULL);
    BaseType_t ok2 = xTaskCreate(TaskBlink1200, "Blink1200", 256, NULL, 1, NULL);

    // Always assert that task creation succeeded — if the heap is too small
    // xTaskCreate returns pdFAIL and the task silently never runs.
    configASSERT(ok1 == pdPASS);
    configASSERT(ok2 == pdPASS);

    Serial.println("Starting scheduler. loop() will never run.");

    // Hand control to the RTOS.  After this line setup() never returns.
    // The scheduler picks the highest-priority ready task and runs it.
    vTaskStartScheduler();

    // If we somehow get here, something went very wrong (heap too small, etc.)
    Serial.println("ERROR: scheduler exited — halt.");
    while (true) {}
}

// ── loop() — intentionally empty ────────────────────────────────────────────
// The RTOS replaces loop(). The idle task runs here if no other task is ready.
void loop() {}

// ── Safety hooks ─────────────────────────────────────────────────────────────
// configCHECK_FOR_STACK_OVERFLOW and configUSE_MALLOC_FAILED_HOOK are enabled
// in FreeRTOSConfig.h. The freertos-teensy library provides default implementations
// that trigger a hard fault — visible as a Teensy reset or halt.
// To customise them (e.g. print to Serial), define the functions below, but note
// that Serial may not be safe to call from the stack-overflow hook context.
//
// void vApplicationStackOverflowHook(TaskHandle_t, char *pcTaskName) { while(1){} }
// void vApplicationMallocFailedHook()                                  { while(1){} }
