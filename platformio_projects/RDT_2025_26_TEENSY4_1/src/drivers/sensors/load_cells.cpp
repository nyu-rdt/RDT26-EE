#include "load_cells.h"
#include "config.h"
#include "HX711.h"

static HX711 _scales[LC_NUM_CELLS];

// Pin table – index 0 = cell 1, etc.
static const uint8_t _dout[LC_NUM_CELLS] = { LC_DOUT1, LC_DOUT2, LC_DOUT3, LC_DOUT4 };
static const uint8_t _clk [LC_NUM_CELLS] = { LC_CLK1,  LC_CLK2,  LC_CLK3,  LC_CLK4  };
static const float   _cal [LC_NUM_CELLS] = { LC_CAL1,  LC_CAL2,  LC_CAL3,  LC_CAL4  };

// Last readings
static float _weights[LC_NUM_CELLS] = { 0.0f, 0.0f, 0.0f, 0.0f };

// Cyclic reader state
static uint8_t       _currentCell     = 0;          // which cell to poll next
static unsigned long _lastPollTime    = 0;           // millis() of last poll attempt

void lc_init() {
    for (uint8_t i = 0; i < LC_NUM_CELLS; i++) {
        _scales[i].begin(_dout[i], _clk[i]);
        _scales[i].set_scale(_cal[i]);
        _scales[i].tare();
    }
}

void lc_update() {
    unsigned long now = millis();

    if (now - _lastPollTime < LC_READ_INTERVAL_MS) {
        return;
    }
    _lastPollTime = now;

    // Try the current cell; advance regardless
    if (_scales[_currentCell].is_ready()) {
        _weights[_currentCell] = _scales[_currentCell].get_units(1);
    }
    _currentCell = (_currentCell + 1) % LC_NUM_CELLS;
}

float lc_get_weight(uint8_t cellIndex) {
    if (cellIndex < 1 || cellIndex > LC_NUM_CELLS) return 0.0f;
    return _weights[cellIndex - 1];
}

uint8_t lc_pack_cell(uint8_t cellIndex) {
    float weight = lc_get_weight(cellIndex);
    float scaled = weight * 10.0f; // 0.1 kg per LSB
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 255.0f) scaled = 255.0f;
    return (uint8_t)scaled;
}

