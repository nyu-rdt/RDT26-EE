#pragma once
#include <Arduino.h>
#include <stdint.h>

// Public API for load cell driver
void lc_init();
void lc_update();
uint8_t lc_pack_cell(uint8_t cellIndex); // packed per-cell (0.1kg/LSB)
// Force a read/process of a single cell and return the processed kg value

// Sensible defaults if not provided in config/pins
// Minimal Public API for load cell driver
// Only initialize and run periodic update. Use lc_pack_cell() from comms
// to retrieve per-cell packed bytes (0.1kg/LSB).
void lc_init();
void lc_update();
uint8_t lc_pack_cell(uint8_t cellIndex); // packed per-cell (0.1kg/LSB)

// Sensible defaults if not provided in config/pins
#ifndef LC_NUM_CELLS
#define LC_NUM_CELLS 4
#endif

#ifndef LC_MIN_PLAUSIBLE_KG
#define LC_MIN_PLAUSIBLE_KG 0.0f
#endif
#ifndef LC_MAX_PLAUSIBLE_KG
#define LC_MAX_PLAUSIBLE_KG 100.0f
#endif
#ifndef LC_EMA_ALPHA
#define LC_EMA_ALPHA 0.2f
#endif
#ifndef LC_READ_INTERVAL_MS
#define LC_READ_INTERVAL_MS 50
#endif

// Median filter window (odd number). Single-sample spikes will be ignored
// unless they persist in the majority of the window.
#ifndef LC_MEDIAN_WINDOW
#define LC_MEDIAN_WINDOW 5
#endif

// Step-change detection: if median differs from EMA by more than this
// threshold (kg), we treat it as a candidate step and require several
// consecutive confirmations before accepting to avoid reacting to noise.
#ifndef LC_STEP_THRESHOLD
#define LC_STEP_THRESHOLD 0.5f
#endif
#ifndef LC_CONFIRM_SAMPLES
#define LC_CONFIRM_SAMPLES 3
#endif
