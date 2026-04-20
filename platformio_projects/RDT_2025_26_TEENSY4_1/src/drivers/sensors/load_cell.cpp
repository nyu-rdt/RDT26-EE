#include <Arduino.h>
#include "config.h"
#include "load_cell.h"

static float cell_mass[NUM_LOAD_CELLS] = {0};

void LOAD_CELL_Init() {
    // TODO: init HX711 instance(s) with pins from pins.h
    // TODO: tare/zero each cell before first read
}

void LOAD_CELL_Update() {
    for (int i = 0; i < NUM_LOAD_CELLS; i++) {
        // TODO: read HX711 channel i and apply scaling:
        // cell_mass[i] = hx711[i].get_units() * LOAD_CELL_SCALE;
        cell_mass[i] = 0.0f;
    }
}

float LOAD_CELL_GetMass(uint8_t index) {
    return (index < NUM_LOAD_CELLS) ? cell_mass[index] : 0.0f;
}

float LOAD_CELL_GetTotalMass() {
    float total = 0.0f;
    for (int i = 0; i < NUM_LOAD_CELLS; i++) total += cell_mass[i];
    return total;
}
