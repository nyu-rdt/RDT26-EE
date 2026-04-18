// test/test_sensors/test_main.cpp
//
// Hardware sensor monitor — NOT a unit test.
// Streams all sensors enabled in config.h to Teleplot (>name:value format)
// and acts as I2C slave on Wire2 so the Jetson can be wired up simultaneously.
//
// Build + upload:  pio run -e teensy41_sensor_test -t upload
// Monitor:         Teleplot extension or Serial Monitor @ 115200 baud

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif

#if ROTARY_ENCODERS_ENABLED
#include "rotary_encoders.h"
#endif

// Capture bytes arriving over I2C from any master on Wire2
static volatile uint8_t last_i2c_byte = 0;
static volatile bool i2c_received = false;

static void i2c_receive_event(int num_bytes) {
    while (Wire2.available()) {
        last_i2c_byte = Wire2.read();
        i2c_received = true;
    }
}

void setup() {
    Serial.begin(115200);

#if CURRENT_SENSE_ENABLED
    CURRENT_SENSORS_Init();
    Serial.println("# current sensors: ok");
#else
    Serial.println("# current sensors: DISABLED in config.h");
#endif

#if ROTARY_ENCODERS_ENABLED
    ROTARY_ENCODER_Init();
    Serial.println("# rotary encoders: ok");
#else
    Serial.println("# rotary encoders: DISABLED in config.h");
#endif

    // Listen as I2C slave — same address/bus as main firmware, safe to
    // wire the Jetson up while running this monitor.
    Wire2.begin(I2C_CHILD_ADDRESS);
    Wire2.onReceive(i2c_receive_event);
    Serial.print("# i2c slave on Wire2 @ 0x");
    Serial.println(I2C_CHILD_ADDRESS, HEX);

    Serial.println("# streaming to Teleplot...");
}

void loop() {
    static unsigned long last_plot_ms = 0;

#if CURRENT_SENSE_ENABLED
    CURRENT_SENSORS_Update();
#endif

    if (millis() - last_plot_ms >= PLOT_PERIOD_MS) {
        last_plot_ms = millis();

#if CURRENT_SENSE_ENABLED
        const float* currents = CURRENT_SENSORS_GetBuffer();
        for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
            Serial.print(">I"); Serial.print(i);
            Serial.print(":"); Serial.println(currents[i], 2);
        }
#endif

#if ROTARY_ENCODERS_ENABLED
        // Degrees (0-360 wrapping) for position view
        Serial.print(">Enc1_deg:"); Serial.println(ROTARY_ENCODER_getEncoderAngle(1), 1);
        Serial.print(">Enc2_deg:"); Serial.println(ROTARY_ENCODER_getEncoderAngle(2), 1);
        // Raw counts (unbounded) for velocity / direction inspection
        Serial.print(">Enc1_cnt:"); Serial.println(ROTARY_ENCODER_getCount(1));
        Serial.print(">Enc2_cnt:"); Serial.println(ROTARY_ENCODER_getCount(2));
#endif

        // Print any I2C byte received since last plot tick
        if (i2c_received) {
            i2c_received = false;
            Serial.print(">I2C_cmd:"); Serial.println(last_i2c_byte);
        }
    }
}
