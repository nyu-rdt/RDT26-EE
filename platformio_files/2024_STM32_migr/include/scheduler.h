/**
 * Time-Triggered Scheduler
 * Rover Control System - STM32F446RE
 * 
 * Industry-grade bare-metal scheduling using TIM6 basic timer
 * Provides deterministic task timing without RTOS overhead
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============== Timing Configuration ============== */
/* Base tick rate: 1ms (1kHz) */
#define SCHEDULER_TICK_HZ       1000
#define SCHEDULER_TICK_MS       1

/* Task periods in milliseconds */
#define PERIOD_COMMAND_MS       10      /* 100 Hz - command processing */
#define PERIOD_SENSOR_MS        50      /* 20 Hz - sensor reading */
#define PERIOD_CONTROL_MS       20      /* 50 Hz - excavation control loop */
#define PERIOD_DEBUG_MS         200     /* 5 Hz - debug output */

/* ============== Scheduler State ============== */
typedef struct {
    volatile uint32_t tick_count;       /* Millisecond counter */
    volatile bool run_command;          /* Flag: process commands */
    volatile bool run_sensor;           /* Flag: read sensors */
    volatile bool run_control;          /* Flag: run control loop */
    volatile bool run_debug;            /* Flag: output debug info */
} Scheduler_State_t;

/* Global scheduler state - extern for ISR access */
extern volatile Scheduler_State_t scheduler;

/* ============== API Functions ============== */

/**
 * Initialize TIM6 basic timer for 1kHz tick
 */
void Scheduler_Init(void);

/**
 * Get current tick count (milliseconds since boot)
 */
uint32_t Scheduler_GetTick(void);

/**
 * Blocking delay using scheduler tick
 * WARNING: Only use during initialization, not in main loop
 */
void Scheduler_DelayMs(uint32_t ms);

/**
 * Call from main loop to check and clear flags
 * Returns true if the specified task should run
 */
bool Scheduler_ShouldRunCommand(void);
bool Scheduler_ShouldRunSensor(void);
bool Scheduler_ShouldRunControl(void);
bool Scheduler_ShouldRunDebug(void);

/**
 * ISR handler - call from TIM6_DAC_IRQHandler
 */
void Scheduler_TickHandler(void);

#endif /* SCHEDULER_H */
