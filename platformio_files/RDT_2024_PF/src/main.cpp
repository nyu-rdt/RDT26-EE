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
 *   - sensors:         HX711 load cells, string pot, encoders
 *   - i2c_commands:    I2C slave command processing, E-stop handling
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
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
    CAN_Init();
    Sensors_Init();  // Also initializes encoders
    Excavation_Init();
    I2C_Init();      // Also initializes E-stop relay pin
    
    Serial.println("Encoders initialized");
    Serial.println("All systems initialized!");
    Serial.println("Waiting for I2C commands...");
}

void loop() {
    // Check E-Stop status first
    if (I2C_CheckEStop()) {
        // E-Stop is engaged, skip normal processing
        yield();
        return;
    }
    
    // Handle any interrupted position commands
    I2C_HandleInterruptedCommand();
    
    // Poll for I2C commands (non-interrupt based for reliability)
    manualReceive();
    
    // Handle non-blocking weight reading and command refresh
    I2C_RefreshMotorCommands();
    
    // Small yield to keep things responsive
    yield();
}