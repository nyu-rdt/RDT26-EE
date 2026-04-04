#pragma once

// ── Experiment knobs — change these, flash, and watch Serial ─────────────────
//
// EXPERIMENT A — fill the queue
//   Set CONSUMER_SLOW_MS to 2000. Producer fires every 500ms but the consumer
//   takes 2s per item. After ~2s the queue depth hits QUEUE_DEPTH and you'll see
//   "WARNING: queue full" in the status line and dropped-item warnings from Producer.
//   Restore CONSUMER_SLOW_MS to 0 and watch "queue waiting" drain back to 0.
//
// EXPERIMENT B - slowly fill the queue
// Set PRODUCER_PERIOD_MS to 100 and CONSUMER_SLOW_MS to 120. Now the producer 
// fires every 100ms but the consumer takes 120ms per item — just slow enough that 
// the queue fills up gradually. Watch the "queue waiting" in the status output 
// grow and shrink as the producer and consumer battle it out. Also note how the values sent
// and received no longer mathch — the producer is sending faster than the consumer can keep up

// EXPERIMENT B — watch free heap drop
//   Increase TASK_STACK_WORDS from 512 to 4096. Each task's stack is carved from
//   the FreeRTOS heap, so the "free heap" line in the status output will drop by
//   roughly 3 × (4096 − 512) × 4 bytes ≈ 43 KB. Try intermediate values too.
//
// EXPERIMENT C — trigger a stack overflow
//   Reduce TASK_STACK_WORDS to 96 and flash. The overflow hook fires immediately
//   and halts with a Serial message. Restore to 512 afterward.
//   (On Teensy the hook calls configASSERT which loops forever — you must reset.)
//
// EXPERIMENT D — flood the queue
//   Reduce QUEUE_DEPTH to 1 and PRODUCER_PERIOD_MS to 100. Now the producer fires
//   10x/s but the queue only holds 1 item — drops are almost constant.


// How many items the counter queue can hold before sends start dropping.
#define QUEUE_DEPTH             4

// Producer fires once every this many ms.
#define PRODUCER_PERIOD_MS      100

// Extra delay inside the consumer after each received item (ms).
// 0 = normal (consumer keeps up easily). Set to 2000 to throttle the consumer
// and let the queue fill up so you can watch it in the status line.
#define CONSUMER_SLOW_MS        0

// How often TaskPrinter emits the status block (ms).
#define PRINTER_PERIOD_MS       500

// Stack size in words (1 word = 4 bytes on Teensy) for each task.
// Heap cost: 3 tasks × TASK_STACK_WORDS × 4 bytes each.
// Default 512 words = 2 KB per task = 6 KB total from heap.
#define TASK_STACK_WORDS        512
