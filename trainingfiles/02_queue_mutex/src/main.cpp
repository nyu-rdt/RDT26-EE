/*
 * 02_queue_mutex — Queues and Mutexes
 * =====================================
 * Goal: Show the two most important inter-task communication tools.
 *
 * Three tasks:
 *   TaskProducer  — sends an incrementing counter to a queue every 500 ms
 *   TaskConsumer  — blocks on the queue; prints each value it receives
 *   TaskPrinter   — every 2 s prints a status line, protected by a mutex
 *
 * WHY A QUEUE?
 *   Passing data between tasks via a global variable is unsafe — the compiler
 *   can cache it in a register, and one task can read a half-written value
 *   if the other task is preempted mid-write. A queue copies the data atomically
 *   and wakes the consumer only when data is ready. No polling, no races.
 *
 * WHY A MUTEX?
 *   Serial.print is not thread-safe. If TaskProducer and TaskPrinter both call
 *   Serial.print at the same moment, their output bytes interleave into garbage.
 *   A mutex lets only one task into the Serial section at a time. Take → print
 *   → give, every time.
 *
 * New concepts vs project 01:
 *   xQueueCreate()         — allocate a queue of N items of a given size
 *   xQueueSend()           — post an item (blocks if queue is full)
 *   xQueueReceive()        — pop an item (blocks until one arrives)
 *   xSemaphoreCreateMutex() — allocate a mutual-exclusion lock
 *   xSemaphoreTake()       — acquire the lock (blocks if held by another task)
 *   xSemaphoreGive()       — release the lock
 */

#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "config.h"

// ── Shared IPC handles ───────────────────────────────────────────────────────
// Declare these at file scope so all tasks can reach them.
// They're just integer handles — the actual memory lives in the FreeRTOS heap.
static QueueHandle_t    xQueueCounter = NULL;
static SemaphoreHandle_t xMutexSerial = NULL;

// ── Helper: safe Serial print ────────────────────────────────────────────────
// Any task that wants to print should call this instead of Serial directly.
// The timeout (100 ms) prevents a stuck task from holding the mutex forever.
static void safePrint(const char *msg) {
    if (xSemaphoreTake(xMutexSerial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.print(msg);
        xSemaphoreGive(xMutexSerial);
    }
}

static void safePrintln(const char *msg) {
    if (xSemaphoreTake(xMutexSerial, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println(msg);
        xSemaphoreGive(xMutexSerial);
    }
}

// ── TaskProducer ─────────────────────────────────────────────────────────────
void TaskProducer(void *pvParams) {
    uint32_t counter = 0;
    char buf[64];

    for (;;) {
        counter++;

        // xQueueSend copies `counter` into the queue — not a pointer, the actual
        // value. This is important: the queue owns the data after this call,
        // so it's safe to modify `counter` immediately afterward.
        //
        // The last argument is the block time: how long to wait if the queue
        // is full. pdMS_TO_TICKS(0) = don't wait; drop the item if full.
        // We use 10 ms here so we don't stall production, but we won't silently
        // drop items under normal operation.
        if (xQueueSend(xQueueCounter, &counter, pdMS_TO_TICKS(10)) != pdTRUE) {
            safePrintln("[Producer] WARNING: queue full, item dropped");
        } else {
            snprintf(buf, sizeof(buf), "[Producer] sent: %lu", counter);
            safePrintln(buf);
        }

        vTaskDelay(pdMS_TO_TICKS(PRODUCER_PERIOD_MS));
    }
}

// ── TaskConsumer ─────────────────────────────────────────────────────────────
void TaskConsumer(void *pvParams) {
    uint32_t received;
    char buf[64];

    for (;;) {
        // xQueueReceive blocks indefinitely (portMAX_DELAY) until an item
        // arrives. This task consumes 0 CPU while waiting — much better than
        // a polling loop with a delay inside.
        if (xQueueReceive(xQueueCounter, &received, portMAX_DELAY) == pdTRUE) {
            snprintf(buf, sizeof(buf), "[Consumer] received: %lu", received);
            safePrintln(buf);

            // Blink LED to give a visual indication of activity.
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

            // CONSUMER_SLOW_MS > 0 throttles the consumer so the queue fills up.
            // See config.h experiment A.
            if (CONSUMER_SLOW_MS > 0) {
                vTaskDelay(pdMS_TO_TICKS(CONSUMER_SLOW_MS));
            }
        }
    }
}

// ── TaskPrinter ───────────────────────────────────────────────────────────────
// Lower priority status task — runs every 2 s and prints heap/queue info.
// Demonstrates that the mutex also keeps this task's multi-line output intact.
void TaskPrinter(void *pvParams) {
    char buf[128];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(PRINTER_PERIOD_MS));

        // Take the mutex before the entire multi-line block.
        // Without this, Consumer's single-line prints could interleave here.
        if (xSemaphoreTake(xMutexSerial, pdMS_TO_TICKS(200)) == pdTRUE) {
            Serial.println("---- status ----");
            snprintf(buf, sizeof(buf),
                     "  free heap:    %u bytes", xPortGetFreeHeapSize());
            Serial.println(buf);
            snprintf(buf, sizeof(buf),
                     "  queue waiting: %u items", uxQueueMessagesWaiting(xQueueCounter));
            Serial.println(buf);
            Serial.println("----------------");
            xSemaphoreGive(xMutexSerial);
        }
    }
}

// ── setup() ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    pinMode(LED_BUILTIN, OUTPUT);

    // Create the queue. QUEUE_DEPTH is set in config.h — reduce it to 1 to see
    // drops almost immediately; increase it to buffer large bursts.
    xQueueCounter = xQueueCreate(QUEUE_DEPTH, sizeof(uint32_t));
    configASSERT(xQueueCounter != NULL);

    // Create the mutex. A mutex starts in the "given" (unlocked) state.
    xMutexSerial = xSemaphoreCreateMutex();
    configASSERT(xMutexSerial != NULL);

    Serial.println("=== 02_queue_mutex ===");

    // Priority 2: Producer and Consumer are equal — they time-slice.
    // Priority 1: Printer is lower, runs only when the others are sleeping.
    // Stack size comes from config.h — change TASK_STACK_WORDS to see heap shift.
    configASSERT(xTaskCreate(TaskProducer, "Producer", TASK_STACK_WORDS, NULL, 2, NULL) == pdPASS);
    configASSERT(xTaskCreate(TaskConsumer, "Consumer", TASK_STACK_WORDS, NULL, 2, NULL) == pdPASS);
    configASSERT(xTaskCreate(TaskPrinter,  "Printer",  TASK_STACK_WORDS, NULL, 1, NULL) == pdPASS);

    vTaskStartScheduler();
    Serial.println("ERROR: scheduler exited — halt.");
    while (true) {}
}

void loop() {}

// freertos-teensy provides default vApplicationStackOverflowHook and
// vApplicationMallocFailedHook implementations. Defining them here causes a
// duplicate symbol link error. See 01_blink_rtos/src/main.cpp for notes.
