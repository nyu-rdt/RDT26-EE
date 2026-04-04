# FreeRTOS Training Projects — Teensy 4.1

Three standalone PlatformIO projects in increasing complexity. Work through them in order.

| Project | Concepts |
|---------|----------|
| `01_blink_rtos` | Tasks, `vTaskDelay`, `vTaskStartScheduler`, concurrency basics |
| `02_queue_mutex` | Queues, mutexes, producer/consumer pattern |
| `03_rover_i2c_can` | ISR-safe I2C→queue pipeline, task notifications, CAN bus |

---

## Setup from scratch

### 1. Create the project (PlatformIO UI)

New Project → Board: **Teensy 4.1** → Framework: **Arduino**

You get:
```
my_project/
├── include/
├── lib/
├── src/
│   └── main.cpp
└── platformio.ini
```

### 2. Edit `platformio.ini`

FreeRTOS is **not** bundled with the Teensy framework — you must declare it. Use the GitHub URL; the registry entry is unreliable.

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
monitor_speed = 115200

lib_deps =
    https://github.com/tsandmann/freertos-teensy.git
```

Add other `lib_deps` only for libraries not already bundled with Teensy. You do **not** need to add FlexCAN_T4, Wire, Servo, SPI — those ship with the Teensy framework package and are always available.

### 3. FreeRTOSConfig.h — do you need to copy it?

**No.** The library ships its own `FreeRTOSConfig.h` in its `src/` directory and `FreeRTOS.h` finds it there automatically. You do not need to copy anything for a basic build.

**Only customize it if you need to change settings.** Because `FreeRTOS.h` and `FreeRTOSConfig.h` live in the same directory inside the library, GCC always finds the library's copy first — placing one in `src/`, `include/`, or the project root does **not** override it by itself (tested).

**To override from the project root**, use the `-include` build flag in `platformio.ini`:

```ini
build_flags =
    -include "${PROJECT_DIR}/FreeRTOSConfig.h"
```

This force-injects your file before anything else, setting the `FREERTOS_CONFIG_H` include guard so the library's copy is skipped. Two caveats:
- Your config must start from the **library's** `FreeRTOSConfig.h` as a base — the port has many required settings that a generic template won't have
- If your config uses `configGENERATE_RUN_TIME_STATS == 1`, add `#include <stdint.h>` before the `uint64_t` declaration (it's injected before system headers are available)
- After changing the config, run `pio run -t clean` first — stale cached objects will cause linker errors

### 4. Customizing FreeRTOSConfig.h (optional)

You don't need to touch most of it. The fields that matter:

```c
configTICK_RATE_HZ       1000       // 1ms tick — standard, leave it
configMAX_PRIORITIES     10         // priority levels 0–9; reduce if you don't need many
configMINIMAL_STACK_SIZE 128        // words (not bytes) for the idle task stack
configUSE_MUTEXES        1          // required by port's C++ threading layer
configUSE_TASK_NOTIFICATIONS 1      // set 1 if you use xTaskNotify
configTOTAL_HEAP_SIZE    0          // 0 = use system malloc (Teensy port default)
```

Everything else either has a safe default or gets stripped by the linker if unused.

Two things already set correctly in the template:
- `configCPU_CLOCK_HZ = F_CPU` — uses the Arduino macro, matches your actual Teensy clock speed automatically
- `configSYSTICK_CLOCK_HZ = 100000UL` — Teensy-specific SysTick setting, don't change it

### 5. Write main.cpp

Include order matters for Teensy + FreeRTOS:

```cpp
#include <Arduino.h>      // always first
#include "FreeRTOS.h"
#include "task.h"
// other RTOS headers (queue.h, semphr.h, ...)
// then your own headers
```

Structure:

```cpp
void MyTask(void *pvParams) {
    for (;;) {
        // task body
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    // init hardware here, before the scheduler starts
    xTaskCreate(MyTask, "MyTask", 512, NULL, 1, NULL);
    vTaskStartScheduler(); // never returns
}

void loop() {} // leave empty — the RTOS replaces loop()
```

### 6. Build

`pio run` — first build after adding lib_deps downloads the library (~30s). Subsequent builds are fast.

---

## What needs lib_deps vs what doesn't

| Library | Needs lib_deps? | Notes |
|---------|----------------|-------|
| `FlexCAN_T4` | No | Bundled with Teensy framework |
| `Wire`, `SPI`, `Servo` | No | Bundled |
| `freertos-teensy` | **Yes** | Use GitHub URL |
| `HX711` (bogde) | Yes | |
| `Encoder` (paulstoffregen) | Yes | |

---

## Key RTOS concepts by project

### 01 — Tasks and scheduling

`xTaskCreate(fn, name, stackWords, param, priority, handle)` registers a function as a task. Once `vTaskStartScheduler()` is called, the scheduler runs the highest-priority ready task. `loop()` is never called again.

`vTaskDelay(pdMS_TO_TICKS(N))` suspends the calling task for N milliseconds and lets other tasks run. Unlike Arduino `delay()`, other tasks keep executing while this one sleeps.

### 02 — Queues and mutexes

`xQueueCreate(depth, itemSize)` allocates a queue. `xQueueSend` copies data in; `xQueueReceive` blocks until data arrives. The queue owns the data — no shared pointers, no races.

A mutex (`xSemaphoreCreateMutex`) protects a shared resource. Pattern: `xSemaphoreTake → use resource → xSemaphoreGive`. Always release it, even on error paths. Never hold a mutex across a `vTaskDelay`.

**Interactive experiments** — all knobs are in `02_queue_mutex/include/config.h`. Change one value, flash, and watch Serial.

| Experiment | What to change | What to observe |
|------------|---------------|-----------------|
| A — fill the queue | `CONSUMER_SLOW_MS` → `2000` | `queue waiting` climbs to `QUEUE_DEPTH`, then "[Producer] WARNING: queue full" appears |
| B — drain it back | Restore `CONSUMER_SLOW_MS` → `0` | `queue waiting` drops to 0 within a few seconds |
| C — shrink the buffer | `QUEUE_DEPTH` → `1`, `PRODUCER_PERIOD_MS` → `100` | Drops appear almost every cycle |
| D — watch heap drop | `TASK_STACK_WORDS` → `4096` | `free heap` in status block drops by ~43 KB (3 tasks × 3584 extra words × 4 bytes) |
| E — stack overflow | `TASK_STACK_WORDS` → `96` | Overflow hook fires immediately; device halts — reset to recover |

The status block prints every `PRINTER_PERIOD_MS` ms. Reduce it to `500` if you want faster feedback during experiments.

### 03 — ISR safety and task notifications

Wire callbacks run at interrupt level. Two rules:
1. Never call blocking RTOS functions from an ISR
2. Use `*FromISR` variants only: `xQueueSendFromISR`, `xSemaphoreGiveFromISR`, etc.
3. Always call `portYIELD_FROM_ISR(woken)` at the end of the ISR

Task notifications are lighter than queues for single-flag signals. `xTaskNotify(handle, value, eSetValueWithOverwrite)` sends directly to a task handle. The receiving task calls `xTaskNotifyWait`. Used in project 03 to bypass the command queue for E-stop — guarantees the stop takes effect at the top of the next motor cycle regardless of what's queued.

`vTaskDelayUntil(&lastWake, period)` is the correct way to write a fixed-period task. `vTaskDelay(20)` drifts by execution time each cycle; `vTaskDelayUntil` wakes at absolute tick intervals.

**Interactive experiments** — all knobs are in `03_rover_i2c_can/include/config.h`. Requires a second Teensy sending I2C commands (or use `general_testing/ALL_i2c_can/i2c_parent_CAN`).

| Experiment | What to change | What to observe |
|------------|---------------|-----------------|
| A — CAN without hardware | `SIMULATE_CAN` → `0` (no motors connected) | System freezes; E-stop does nothing — CAN bus-off error-interrupt storm starves all tasks |
| B — I2C queue overflow | `I2C_QUEUE_DEPTH` → `2`, send a burst of commands | ISR drops bytes; commands arrive garbled or not at all |
| C — motor queue overflow | `MOTOR_QUEUE_DEPTH` → `1`, send commands faster than 20ms | "[I2CDecode] WARNING: motor queue full" appears |
| D — instant speed | `MAX_SPEED_DELTA_PER_TICK` → `0.5` | `[CAN SIM]` speed jumps to target in one tick instead of ramping |
| E — slow ramp | `MAX_SPEED_DELTA_PER_TICK` → `0.002` | Speed takes ~10s to reach full; useful for seeing the slew effect |
| F — command timeout | `COMMAND_TIMEOUT_MS` → `100`, stop sending I2C | `[CAN SIM]` speeds ramp to 0 within 100ms of the last command |
| G — E-stop debounce | `ESTOP_DEBOUNCE_MS` → `1` | Brief noise on pin 3 triggers false E-stops |
| H — stack vs heap | `MOTOR_TASK_STACK` → `4096` | Free heap drops by ~14 KB; visible if you add a `Serial.printf` in the motor loop |

> **Note on experiment A:** the freeze happens because the CAN controller has no termination resistor and no ACK from any node. It accumulates TX errors, enters bus-off, and the error-recovery interrupt fires thousands of times per second — faster than FreeRTOS can schedule tasks. `SIMULATE_CAN 1` skips the real write and prints `[CAN SIM]` lines instead.

---

## Reference

- FreeRTOS docs: https://www.freertos.org/Documentation/00-Overview
- tsandmann port: https://github.com/tsandmann/freertos-teensy
