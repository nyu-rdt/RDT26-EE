/*
 * 03_rover_i2c_can — Minimal rover firmware with FreeRTOS
 * =========================================================
 * Goal: A real working rover firmware in ~300 lines. Everything the original
 * RDT_2025_26_TEENSY4_1 superloop does, but in three concurrent tasks:
 *
 *   TaskI2CDecode  (priority 3) — decodes commands from the I2C queue,
 *                                 builds MotorCommand_t structs, posts to xQueueMotor
 *   TaskMotorCtrl  (priority 2) — 20ms loop; slews speed toward target; sends CAN
 *   TaskEStop      (priority 5) — 10ms loop; monitors E-stop pin;
 *                                 sends direct task notification to bypass the queue
 *
 * The key difference vs the superloop (child_update):
 *   - E-stop is guaranteed to fire within one 10ms tick regardless of what
 *     the motor or command task is doing. In the superloop a long CAN write
 *     could delay E-stop processing.
 *   - Commands don't block the motor slew loop. A slow I2C burst doesn't
 *     cause a missed 20ms CAN cycle.
 *
 * New concepts vs project 02:
 *   ISR-safe queue send    — I2C callback can't use normal RTOS calls
 *   xQueueSendFromISR()    — the ISR-safe version; sets woken flag
 *   portYIELD_FROM_ISR()   — requests a context switch at ISR exit if needed
 *   Task notifications     — lighter-weight than a queue for a single flag
 *   xTaskNotify()          — send a notification value directly to a task handle
 *   xTaskNotifyWait()      — receive a notification (non-blocking here, poll)
 *   vTaskDelayUntil()      — precise periodic execution (vs vTaskDelay drift)
 */

#include <Arduino.h>
#include <Wire.h>
#include <FlexCAN_T4.h>
#include <climits>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "config.h"

// ── Types ────────────────────────────────────────────────────────────────────

// A single decoded command posted from I2CDecode to MotorCtrl.
typedef struct {
    float leftSpeed;    // +1.0 = full forward, -1.0 = full reverse, 0 = stop
    float rightSpeed;
    float excavSpeed;
    bool  stopAll;      // E-stop or GRP_CONTROL stop — zero everything
    bool  hasLocoCmd;   // true if this command targets locomotion (including stop)
    bool  hasExcavCmd;  // true if this command targets excavation (including stop)
} MotorCommand_t;

// ── IPC handles ──────────────────────────────────────────────────────────────
static QueueHandle_t    xQueueI2C   = NULL;   // raw bytes from Wire2 ISR
static QueueHandle_t    xQueueMotor = NULL;   // decoded MotorCommand_t structs
static SemaphoreHandle_t xMutexCAN  = NULL;   // guards can1.write() calls
static TaskHandle_t     hTaskMotor  = NULL;   // handle for direct E-stop notification

// ── CAN bus ──────────────────────────────────────────────────────────────────
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

static void CAN_SendMotorSpeed(uint32_t id, float speed) {
#if SIMULATE_CAN
    // Skip real CAN write — without termination/ACK the controller goes bus-off
    // and the resulting error-interrupt storm starves all RTOS tasks.
    
#else
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = id;
    msg.len = 4;
    // Speed is encoded as int32 × 100000, big-endian.
    int32_t val = (int32_t)(speed * 100000.0f);
    msg.buf[0] = (val >> 24) & 0xFF;
    msg.buf[1] = (val >> 16) & 0xFF;
    msg.buf[2] = (val >>  8) & 0xFF;
    msg.buf[3] =  val        & 0xFF;
    can1.write(msg);
#endif
}

// ── I2C ISR callbacks ─────────────────────────────────────────────────────────
// IMPORTANT: These run at interrupt level. Rules:
//   1. No blocking calls (no delay, no Serial, no xQueueSend with timeout > 0)
//   2. Only use *FromISR variants of RTOS calls
//   3. Declare woken, pass it in, call portYIELD_FROM_ISR at the end

static void I2C_OnReceive(int numBytes) {
    BaseType_t woken = pdFALSE;

    while (Wire2.available()) {
        uint8_t byte = Wire2.read();
        // xQueueSendFromISR never blocks — if the queue is full, it returns
        // pdFALSE and we silently drop the byte. That's acceptable: the
        // decode task should drain the queue faster than bytes arrive.
        xQueueSendFromISR(xQueueI2C, &byte, &woken);
    }

    // If sending woke a higher-priority task, yield to it immediately at
    // ISR exit instead of returning to whatever was preempted.
    portYIELD_FROM_ISR(woken);
}

// ── TaskI2CDecode ─────────────────────────────────────────────────────────────
// Decodes raw command bytes into MotorCommand_t and posts to the motor queue.
// This keeps all the command logic out of the ISR.
void TaskI2CDecode(void *pvParams) {
    uint8_t        raw;
    MotorCommand_t cmd;

    for (;;) {
        // Block until a byte arrives from the I2C ISR.
        if (xQueueReceive(xQueueI2C, &raw, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint8_t group = CMD_GROUP(raw);
        uint8_t param = CMD_PARAM(raw);

        // Reset command to zero; only set what this group changes.
        memset(&cmd, 0, sizeof(cmd));

        switch (group) {
            case GRP_CONTROL:
                cmd.stopAll = true;
                break;

            case GRP_LOCO_STOP:
                cmd.hasLocoCmd = true;  // speeds stay 0 — stops locomotion only
                break;

            case GRP_FORWARD:
                cmd.leftSpeed  = -getSpeed(param);  // left motor is mechanically reversed
                cmd.rightSpeed =  getSpeed(param);
                cmd.hasLocoCmd = true;
                break;

            case GRP_BACKWARD:
                cmd.leftSpeed  =  getSpeed(param);
                cmd.rightSpeed = -getSpeed(param);
                cmd.hasLocoCmd = true;
                break;

            case GRP_LEFT:
                // Turn left: both motors spin the same direction
                cmd.leftSpeed  = getSpeed(param);
                cmd.rightSpeed = getSpeed(param);
                cmd.hasLocoCmd = true;
                break;

            case GRP_RIGHT:
                cmd.leftSpeed  = -getSpeed(param);
                cmd.rightSpeed = -getSpeed(param);
                cmd.hasLocoCmd = true;
                break;

            case GRP_EXCAVATION:
                cmd.excavSpeed  = getDirection(param) * EXCAVATION_DUTY_CYCLE;
                cmd.hasExcavCmd = true;
                break;

            default:
                // Unknown group — ignore
                continue;
        }

        // Post to motor queue; wait up to 5ms. If it's full something is wrong
        // (motor task not running?) — drop the command rather than stall decode.
        if (xQueueSend(xQueueMotor, &cmd, pdMS_TO_TICKS(5)) != pdTRUE) {
            Serial.println("[I2CDecode] WARNING: motor queue full");
        }
    }
}

// ── TaskMotorCtrl ─────────────────────────────────────────────────────────────
// Runs every TX_PERIOD_MS. Drains the motor queue for the latest command,
// slews speed toward the target, then sends CAN. Also handles command timeout.
void TaskMotorCtrl(void *pvParams) {
    float currentLeft  = 0.0f;
    float currentRight = 0.0f;
    float currentExcav = 0.0f;
    float targetLeft   = 0.0f;
    float targetRight  = 0.0f;
    float targetExcav  = 0.0f;

    uint32_t lastCmdTime = millis();
    MotorCommand_t cmd;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        // ── 1. Check for E-stop notification ──────────────────────────────
        // xTaskNotifyWait with 0 timeout = non-blocking poll.
        // TaskEStop sends notification value 1 on assert, 0 on release.
        // We check this FIRST every cycle so the hard stop takes effect even
        // if a command also arrived this tick.
        uint32_t notifyVal = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &notifyVal, 0) == pdTRUE) {
            if (notifyVal == 1U) {
                // E-stop asserted: zero everything, clear the motor queue
                targetLeft = targetRight = targetExcav = 0.0f;
                while (xQueueReceive(xQueueMotor, &cmd, 0) == pdTRUE) {}
                Serial.println("[MotorCtrl] E-STOP: motors zeroed");
            }
            // notifyVal == 0 means released — SW can send new commands freely
        }

        // ── 2. Drain motor queue for latest command ────────────────────────
        // We drain the whole queue and keep only the last item. This means if
        // multiple commands arrived in one 20ms window we act on the most recent.
        bool gotCmd = false;
        while (xQueueReceive(xQueueMotor, &cmd, 0) == pdTRUE) {
            gotCmd = true;
        }
        if (gotCmd) {
            lastCmdTime = millis();
            if (cmd.stopAll) {
                targetLeft = targetRight = targetExcav = 0.0f;
            } else {
                // Only update targets that the command actually addresses.
                // Locomotion commands leave excavation unchanged, and vice versa.
                if (cmd.hasLocoCmd) {
                    targetLeft  = cmd.leftSpeed;
                    targetRight = cmd.rightSpeed;
                }
                if (cmd.hasExcavCmd) {
                    targetExcav = cmd.excavSpeed;
                }
            }
        }

        // ── 3. Command timeout ─────────────────────────────────────────────
        // If no command has arrived for COMMAND_TIMEOUT_MS, stop motors.
        // Prevents runaway if the Jetson crashes or the I2C cable comes loose.
        if ((millis() - lastCmdTime) > (uint32_t)COMMAND_TIMEOUT_MS) {
            targetLeft = targetRight = targetExcav = 0.0f;
        }

        // ── 4. Slew toward target ──────────────────────────────────────────
        // Instead of jumping to the target speed instantly, ramp at
        // MAX_SPEED_DELTA_PER_TICK per cycle. This prevents current spikes
        // and jerk on the drivetrain.
        auto slew = [](float cur, float tgt) -> float {
            float delta = tgt - cur;
            if (delta >  MAX_SPEED_DELTA_PER_TICK) delta =  MAX_SPEED_DELTA_PER_TICK;
            if (delta < -MAX_SPEED_DELTA_PER_TICK) delta = -MAX_SPEED_DELTA_PER_TICK;
            return cur + delta;
        };
        currentLeft  = slew(currentLeft,  targetLeft);
        currentRight = slew(currentRight, targetRight);
        currentExcav = slew(currentExcav, targetExcav);

        // ── 5. Send CAN ────────────────────────────────────────────────────
        // Mutex guards the CAN peripheral — in the full firmware the anomaly
        // task also writes CAN, so we need this even in the simplified version.
        if (xSemaphoreTake(xMutexCAN, pdMS_TO_TICKS(5)) == pdTRUE) {
            CAN_SendMotorSpeed(CAN_ID_LEFT_MOTOR,       currentLeft);
            CAN_SendMotorSpeed(CAN_ID_RIGHT_MOTOR,      currentRight);
            CAN_SendMotorSpeed(CAN_ID_EXCAVATION_MOTOR, currentExcav);
            xSemaphoreGive(xMutexCAN);
        }

#if PRINT_CAN
        // Print all three speeds together, throttled so Serial isn't flooded.
        // The motor loop runs every TX_PERIOD_MS (20ms) — without throttling
        // that's 150 lines/s. CAN_PRINT_PERIOD_MS in config.h controls the rate.
        static uint32_t lastCanPrint = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastCanPrint >= (uint32_t)CAN_PRINT_PERIOD_MS) {
            lastCanPrint = nowMs;
            Serial.printf("[CAN] L=% .3f  R=% .3f  E=% .3f\n",
                          currentLeft, currentRight, currentExcav);
        }
#endif

        // ── 6. Wait until next 20ms tick ──────────────────────────────────
        // vTaskDelayUntil is preferred over vTaskDelay for periodic tasks.
        // vTaskDelay(20) would drift by execution time each cycle.
        // vTaskDelayUntil wakes at lastWake + 20ms regardless of how long
        // steps 1-5 took (as long as they finished within 20ms).
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TX_PERIOD_MS));
    }
}

// ── TaskEStop ─────────────────────────────────────────────────────────────────
// Highest priority task. Two responsibilities:
//   1. Continuously drives E_STOP_RELAY_DRIVE_PIN HIGH (fail-safe relay).
//      If this task ever stops running the pin goes LOW and cuts motor power.
//   2. Watches E_STOP_READ_PIN for an asserted (LOW) E-stop signal and
//      notifies the motor task to stop immediately — bypassing the command queue.
//
// WHY BYPASS THE QUEUE?
//   The motor queue might have a bunch of speed commands waiting to execute.
//   If we posted a "stop" command to the back of the queue the motors would
//   execute all the earlier commands first. Task notifications go directly
//   to the motor task and are checked at the TOP of every 20ms cycle, before
//   any queued commands are processed.
void TaskEStop(void *pvParams) {
    bool     prevAsserted = false;
    bool     prevPinLow   = false;   // tracks last pin state to detect edges
    uint32_t assertedSince = 0;

    for (;;) {
        // Keep the relay coil energised — this is the fail-safe.
        digitalWrite(E_STOP_RELAY_DRIVE_PIN, HIGH);

        bool pinLow = (digitalRead(E_STOP_READ_PIN) == LOW);

        // Start the debounce timer only on the HIGH→LOW edge, not every tick.
        // Bug if you use (pinLow && !prevAsserted): prevAsserted stays false until
        // debounce fires, so the timer resets every 10ms loop and never accumulates.
        if (pinLow && !prevPinLow) {
            assertedSince = millis();
        }
        prevPinLow = pinLow;

        bool debounced = pinLow && ((millis() - assertedSince) >= ESTOP_DEBOUNCE_MS);

        if (debounced && !prevAsserted) {
            prevAsserted = true;
            Serial.println("[EStop] ASSERTED");
            // Notification value 1 = stop. The motor task checks this each cycle.
            xTaskNotify(hTaskMotor, 1U, eSetValueWithOverwrite);
        } else if (!pinLow && prevAsserted) {
            prevAsserted = false;
            Serial.println("[EStop] released");
            // Notification value 0 = released — motor task can accept commands again.
            xTaskNotify(hTaskMotor, 0U, eSetValueWithOverwrite);
        }

        // Periodic status line so you can confirm pin state without waiting for a
        // transition. Fires at CAN_PRINT_PERIOD_MS so it lines up with the CAN print.
        static uint32_t lastEstopPrint = 0;
        uint32_t nowEstop = millis();
        if (nowEstop - lastEstopPrint >= (uint32_t)CAN_PRINT_PERIOD_MS) {
            lastEstopPrint = nowEstop;
            Serial.printf("[EStop] pin=%s  debounced=%s  asserted=%s\n",
                          pinLow      ? "LOW " : "HIGH",
                          debounced   ? "YES"  : "no",
                          prevAsserted? "YES"  : "no");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ── setup() ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // Energise relay immediately — before the scheduler starts — so that
    // even a crash in setup() doesn't leave the relay in an unknown state.
    pinMode(E_STOP_RELAY_DRIVE_PIN, OUTPUT);
    digitalWrite(E_STOP_RELAY_DRIVE_PIN, HIGH);

    pinMode(E_STOP_READ_PIN, INPUT_PULLDOWN);

    Serial.println("=== 03_rover_i2c_can ===");

    // CAN init must come before tasks that write CAN
    can1.begin();
    can1.setBaudRate(CAN_BAUD_RATE);

    // Create IPC primitives before any task or ISR that uses them.
    // Depths are set in config.h — see experiment A and B.
    xQueueI2C   = xQueueCreate(I2C_QUEUE_DEPTH,   sizeof(uint8_t));
    xQueueMotor = xQueueCreate(MOTOR_QUEUE_DEPTH,  sizeof(MotorCommand_t));
    xMutexCAN   = xSemaphoreCreateMutex();
    configASSERT(xQueueI2C   != NULL);
    configASSERT(xQueueMotor != NULL);
    configASSERT(xMutexCAN   != NULL);

    // Register I2C callbacks AFTER queues exist (callbacks use xQueueI2C)
    Wire2.begin(I2C_CHILD_ADDRESS);
    Wire2.onReceive(I2C_OnReceive);

    // Create tasks.
    // Motor task is created first so hTaskMotor is valid before EStop starts.
    // Stack sizes are set in config.h — see experiment F.
    configASSERT(xTaskCreate(TaskMotorCtrl,  "MotorCtrl",  MOTOR_TASK_STACK,  NULL, 2, &hTaskMotor) == pdPASS);
    configASSERT(xTaskCreate(TaskI2CDecode,  "I2CDecode",  DECODE_TASK_STACK, NULL, 3, NULL)        == pdPASS);
    configASSERT(xTaskCreate(TaskEStop,      "EStop",      ESTOP_TASK_STACK,  NULL, 5, NULL)        == pdPASS);

    Serial.println("Tasks created. Starting scheduler.");
    vTaskStartScheduler();

    Serial.println("ERROR: scheduler exited — halt.");
    while (true) {}
}

void loop() {}

// freertos-teensy provides default vApplicationStackOverflowHook and
// vApplicationMallocFailedHook implementations. See 01_blink_rtos for notes.
