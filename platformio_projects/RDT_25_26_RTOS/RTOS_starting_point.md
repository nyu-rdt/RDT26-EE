# RTOS Starting Point

Reference projects:
- `trainingfiles/01_blink_rtos`
- `trainingfiles/02_queue_mutex`
- `trainingfiles/03_rover_i2c_can`

Common setup pattern:
- `platformio.ini` adds `https://github.com/tsandmann/freertos-teensy.git`
- `setup()` creates tasks and calls `vTaskStartScheduler()`
- `loop()` is intentionally empty
- `Arduino.h` is included before FreeRTOS headers
- ISR paths use `*FromISR` APIs and `portYIELD_FROM_ISR(...)`

Architecture pattern used in `03_rover_i2c_can`:
- ISR only captures bytes quickly and pushes to a queue
- Decode/command logic runs in a task
- Motor control runs in a fixed-period task (`vTaskDelayUntil`)
- E-stop runs in a high-priority task and notifies motor task directly

This pattern is a strong match for our current rover firmware constraints.

## Current superloop snapshot (RDT_2025_26_TEENSY4_1)

`ROVER_update()` currently does:
1. `EE_BOX_Update()`
2. E-stop relay check and possible `ESTOP_Trigger()`
3. Command handling (`newCommand` + `processCommand`)
4. Timeout check (`COMMAND_TIMEOUT_MS`)
5. Periodic subsystem updates:
   - `LOCO_Update()`
   - `EXCAV_Update()`
   - `CURRENT_SENSORS_Update()` (optional)
   - `LOAD_CELL_Update()` (optional)
   - `DEPO_Update()`
   - `COMMS_UpdateFlags()`
   - `DEBUG_Update()` (optional)


## Candidate task split

### Task A: E-stop / safety monitor (highest)
- Responsibility:
  - Read relay state (`EE_BOX_Update` + `EE_BOX_IsRelayEngaged`)
  - Trigger emergency stop immediately on unsafe state
  - Notify motor/control task of hard-stop state
- Why high priority:
  - Safety path; must preempt normal control work
- Target period:
  - 5-10 ms

### Task B: I2C RX + command decode
- Responsibility:
  - ISR captures incoming I2C bytes to queue
  - Task decodes command byte into actions/targets
  - Writes command target state (or pushes decoded command struct)
- Why:
  - Keeps ISR minimal and deterministic
  - Prevents command parsing from blocking time-critical loops
- Trigger model:
  - Event-driven via queue receive (block until data)

### Task C: Motor/control loop (locomotion + excavation command application)
- Responsibility:
  - Apply latest target commands
  - Handle ramp logic at fixed rate (`TX_PERIOD_MS = 20`)
  - Send CAN commands
  - Enforce comms timeout fail-safe
- Why:
  - This is the rover control heartbeat
  - Fixed periodic timing is important for predictable motion
- Target period:
  - 20 ms (`50 Hz`)

### Task D: Mechanism maintenance loop
- Responsibility:
  - `EXCAV_Update()` (stepper + limit logic + optional pot checks)
  - `DEPO_Update()` (door state machine)
- Why:
  - Mechanism internals are periodic but less safety-critical than Task A/C
- Target period:
  - 5-10 ms for stepper smoothness (or split stepper to use a hardware timer interrupt if jitter is an issue)

### Task E: Sensor + flags loop
- Responsibility:
  - `CURRENT_SENSORS_Update()`
  - `LOAD_CELL_Update()`
  - `COMMS_UpdateFlags()`
- Why:
  - These feed telemetry and health flags; useful but not motor-heartbeat critical
- Target period:
  - 10 ms for current mux stepping
  - 50-100 ms acceptable for aggregate flags publishing

### Task F: Debug/telemetry print loop (lowest)
- Responsibility:
  - `DEBUG_Update()` serial plot output
- Why low priority:
  - Human-facing output should never starve safety/control
- Target period:
  - Existing `PLOT_PERIOD_MS` (50 ms) or slower if serial load is high

## Proposed priority order (initial)

1. Task A `EStopSafety` (highest)
2. Task C `MotorControl`
3. Task B `I2CDecode`
4. Task D `MechanismUpdate`
5. Task E `SensorsAndFlags`
6. Task F `DebugTelemetry` (lowest)

Notes:
- If command latency becomes visible, raise Task B above Task C and let Task C consume latest decoded state each cycle.

## Init placement strategy

Use a two-level init pattern:

1. `setup()` does board/global bring-up before scheduler:
- Serial (optional)
- Pin mode defaults for safety outputs
- CAN peripheral init
- I2C bus init and callback registration
- Queue/mutex/task-notification primitive creation
- Task creation and scheduler start

2. Each task does module-local one-time init before its loop:
- Example: initialize task-local timestamps/state variables
- Then enter `for (;;)` loop with `vTaskDelay`/`vTaskDelayUntil`

This mirrors the training projects and keeps task ownership clear.

## Suggested migration phases

1. RTOS scaffold only:
- Add FreeRTOS dependency and empty task skeletons
- Keep behavior equivalent to current firmware

2. Move safety + command pipeline:
- Add ISR queue + decode task
- Add E-stop safety task + notification path

3. Move control heartbeat:
- Create 20 ms motor control task using `vTaskDelayUntil`
- Remove superloop control timing logic

4. Move slower periodic work:
- Sensors, flags, debug prints

5. Tune priorities and stack sizes with runtime measurements.

## Open questions before implementation

- Should stepper pulse timing stay in a shared mechanism task, or become a dedicated high-rate task or hardware timer interrupt from day one?
- Do we want timeout handling only in MotorControl (single owner), or mirrored in Safety task too?
- Should command decoding directly call handlers, or write a normalized command struct consumed by MotorControl/Mechanism tasks?
