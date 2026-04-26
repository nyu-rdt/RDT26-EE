#pragma once
#include <Arduino.h>
#include <stdint.h>

// Minimal Public API for load cell driver
void lc_init();
void lc_update();
float lc_get_weight(uint8_t cellIndex); // get weight in kg for cell 1-4
uint8_t lc_pack_cell(uint8_t cellIndex); // packed byte: 0.1 kg per LSB

// Sensible defaults if not provided in config/pins
#ifndef LC_NUM_CELLS
#define LC_NUM_CELLS 4
#endif

#ifndef LC_READ_INTERVAL_MS
#define LC_READ_INTERVAL_MS 50
#endif
