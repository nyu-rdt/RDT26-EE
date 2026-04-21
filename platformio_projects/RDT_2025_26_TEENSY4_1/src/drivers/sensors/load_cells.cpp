#include "load_cells.h"
#include "config.h"
#include "HX711.h"

static HX711 _scales[LC_NUM_CELLS];

// Pin table – index 0 = cell 1, etc.
static const uint8_t _dout[LC_NUM_CELLS] = { LC_DOUT1, LC_DOUT2, LC_DOUT3, LC_DOUT4 };
static const uint8_t _clk [LC_NUM_CELLS] = { LC_CLK1,  LC_CLK2,  LC_CLK3,  LC_CLK4  };
static const float   _cal [LC_NUM_CELLS] = { LC_CAL1,  LC_CAL2,  LC_CAL3,  LC_CAL4  };

// EMA buffers – initialised to 0 after tare
static float _ema[LC_NUM_CELLS]         = { 0.0f, 0.0f, 0.0f, 0.0f };

// Median filter buffers
static float _buf[LC_NUM_CELLS][LC_MEDIAN_WINDOW];
static uint8_t _buf_pos[LC_NUM_CELLS]   = { 0, 0, 0, 0 };
static uint8_t _buf_count[LC_NUM_CELLS] = { 0, 0, 0, 0 };

// Last computed median per cell (updated on successful _tryRead)
static float _last_median[LC_NUM_CELLS] = { 0.0f, 0.0f, 0.0f, 0.0f };

// Step-change candidate + confirmation counters
static float _candidate[LC_NUM_CELLS]   = { 0.0f, 0.0f, 0.0f, 0.0f };
static uint8_t _confirmCount[LC_NUM_CELLS] = { 0, 0, 0, 0 };

// Fault counters – incremented when a reading is out of plausible range
static uint16_t _faultCount[LC_NUM_CELLS] = { 0, 0, 0, 0 };

// Cyclic reader state
static uint8_t       _currentCell     = 0;          // which cell to poll next
static unsigned long _lastPollTime    = 0;           // millis() of last poll attempt

static bool _tryRead(uint8_t idx) {
    if (!_scales[idx].is_ready()) return false;
    float raw = _scales[idx].get_units(1);
    return _process_sample(idx, raw);


// Centralized processing of a raw sample for cell idx. Returns true if
// sample accepted and processed, false if discarded (out of range).
static bool _process_sample(uint8_t idx, float raw) {
    // Sanity / fault gate
    if (raw < LC_MIN_PLAUSIBLE_KG || raw > LC_MAX_PLAUSIBLE_KG) {
        _faultCount[idx]++;
        return false; // discard – do not poison buffers or EMA
    }

    // Insert into median buffer (ring)
    _buf[idx][_buf_pos[idx]] = raw;
    _buf_pos[idx] = (_buf_pos[idx] + 1) % LC_MEDIAN_WINDOW;
    if (_buf_count[idx] < LC_MEDIAN_WINDOW) _buf_count[idx]++;

    // Compute median of available samples
    float tmp[LC_MEDIAN_WINDOW];
    uint8_t n = _buf_count[idx];
    for (uint8_t i = 0; i < n; i++) tmp[i] = _buf[idx][i];
    // simple insertion sort for small window
    for (uint8_t i = 1; i < n; i++) {
        float key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    float median;
    if (n == 0) median = raw;
    else if (n % 2 == 1) median = tmp[n / 2];
    else median = 0.5f * (tmp[n/2 - 1] + tmp[n/2]);

    // store last median for use by packer
    _last_median[idx] = median;

    // Step-change detection: if median differs from EMA by more than
    // LC_STEP_THRESHOLD, treat as candidate and require LC_CONFIRM_SAMPLES
    // consecutive confirmations before accepting as a real step.
    float diff = fabs(median - _ema[idx]);
    if (diff > LC_STEP_THRESHOLD) {
        _candidate[idx] = median;
        if (_confirmCount[idx] < 255) _confirmCount[idx]++;
        if (_confirmCount[idx] >= LC_CONFIRM_SAMPLES) {
            // accept step change immediately (replace EMA)
            _ema[idx] = _candidate[idx];
            _confirmCount[idx] = 0;
        }
    } else {
        // no large step — reset confirmation counter and track normally
        _confirmCount[idx] = 0;
        _ema[idx] = LC_EMA_ALPHA * median + (1.0f - LC_EMA_ALPHA) * _ema[idx];
    }
    return true;
}
}

void lc_init() {
    for (uint8_t i = 0; i < LC_NUM_CELLS; i++) {
        _scales[i].begin(_dout[i], _clk[i]);
        _scales[i].set_scale(_cal[i]);
    }

    // Blocking tare with bin loaded – gives us regolith-only readings later.
    // ~100 ms per cell at 10 SPS; acceptable once at startup.
    lc_tare_all();

    (void)0;
}

void lc_update() {
    unsigned long now = millis();

    // Throttle polling to avoid hammering SPI bus
    if (now - _lastPollTime < LC_READ_INTERVAL_MS) {
        return;
    }
    _lastPollTime = now;

    // Try the current cell; advance regardless so we never stall on one
    // noisy or disconnected cell.
    _tryRead(_currentCell);
    _currentCell = (_currentCell + 1) % LC_NUM_CELLS;
}

uint8_t lc_pack_cell(uint8_t cellIndex) {
    if (cellIndex < 1 || cellIndex > LC_NUM_CELLS) return 0xFF;
    uint8_t idx = cellIndex - 1;
    float ema = _ema[idx];
    float median = _last_median[idx];

    // weight depends on how many confirmation samples we've accumulated
    float w = 0.0f;
    if (LC_CONFIRM_SAMPLES > 0) {
        w = (float)_confirmCount[idx] / (float)LC_CONFIRM_SAMPLES;
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
    }

    // blended value: as confirmations grow, pull towards median; otherwise use EMA
    float blended = (w * median) + ((1.0f - w) * ema);
    float val = blended;
    float scaled = val * 10.0f; // 0.1 kg per LSB
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 255.0f) scaled = 255.0f;
    return (uint8_t)scaled;
}

void lc_tare_all() {
    // blocking tare
    for (uint8_t i = 0; i < LC_NUM_CELLS; i++) {
        _scales[i].tare();        // blocks ~100 ms per cell
        _ema[i] = 0.0f;           // reset EMA after tare
        _faultCount[i] = 0;
    }
    (void)0;
}

// debug printing removed; parent prints load-cell values on request

