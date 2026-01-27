/**
 * @file main.cpp
 * @brief Rover Control System - Teensy 4.1
 * 
 * Main entry point for the excavation rover control system.
 * Receives commands via I2C from Raspberry Pi master.
 * Controls locomotion, excavation, and deposition via CAN bus and PWM.
 * 
 * Modular Architecture:
 *   - config.h:        Pin definitions and constants
 *   - system:          E-stop, emergency stop, system-level functions
 *   - can_driver:      CAN bus communication
 *   - locomotion:      4-wheel drive control
 *   - excavation:      Arm and belt control
 *   - sensors:         HX711 load cells, string pot, encoders
 *   - i2c_commands:    I2C slave command processing
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "system.h"
#include "can_driver.h"
#include "locomotion.h"
#include "excavation.h"
#include "sensors.h"
#include "i2c_commands.h"

// Manual receive function - polls I2C for commands
void manualReceive(void) {
    while (Wire.available()) {
        char c = Wire.read();
        Serial.println(int(c));
        I2C_ProcessCommand(int(c));
    }
}

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait up to 3 seconds for serial connection
    }
    Serial.println("=== Rover Control System ===");
    Serial.println("Teensy 4.1 - Initializing...");
    
    // Initialize all subsystems
    System_Init();   // E-stop relay
    CAN_Init();
    Sensors_Init();  // Also initializes encoders
    Excavation_Init();
    I2C_Init();
    
    Serial.println("Encoders initialized");
    Serial.println("All systems initialized!");
    Serial.println("Waiting for I2C commands...");
}

void loop() {
    // Check E-Stop status first (hardware safety)
    if (System_CheckEStop()) {
        // E-Stop is engaged, skip normal processing
        // Still allow data requests during E-stop
        if (Wire.available()) {
            char c = Wire.read();
            if (int(c) == CMD_REQUEST_DATA) {
                Sensors_Update();
            }
        }
        yield();
        return;
    }
    
    // Poll for I2C commands
    manualReceive();
    
    // Periodic system update (weight reading)
    System_Update();
    
    // Small yield to keep things responsive
    yield();
}