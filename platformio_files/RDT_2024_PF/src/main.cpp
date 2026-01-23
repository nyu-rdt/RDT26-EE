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
 *   - can_driver:      CAN bus communication
 *   - locomotion:      4-wheel drive control
 *   - excavation:      Arm and belt control
 *   - sensors:         HX711 load cells, string pot
 *   - i2c_commands:    I2C slave command processing
 */

#include <Arduino.h>
#include "config.h"
#include "can_driver.h"
#include "locomotion.h"
#include "excavation.h"
#include "sensors.h"
#include "i2c_commands.h"

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait up to 3 seconds for serial connection
    }
    Serial.println("=== Rover Control System ===");
    Serial.println("Teensy 4.1 - Initializing...");
    
    // Initialize all subsystems
    CAN_Init();
    Sensors_Init();
    Excavation_Init();
    I2C_Init();
    
    Serial.println("All systems initialized!");
    Serial.println("Waiting for I2C commands...");
}

void loop() {
    // Main loop is mostly idle - work happens in I2C callbacks
    
    // Debug: print last command if any
    int lastCmd = I2C_GetLastCommand();
    if (lastCmd) {
        // Could add periodic status updates here
    }
    
    // Periodic sensor update (optional, for monitoring)
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 100) {
        Sensors_Update();
        lastSensorUpdate = millis();
    }
}