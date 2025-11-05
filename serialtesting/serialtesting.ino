#include <Arduino.h>
/*
 * Teensy to Raspberry Pi 5 Serial Communication Test
 * 
 * This code communicates with a Raspberry Pi 5 over hardware UART (Serial1)
 * Teensy 4.1 Serial1 pins: TX1 = Pin 1, RX1 = Pin 0
 * 
 * Wiring:
 * - Teensy Pin 1 (TX1) -> Pi5 GPIO 15 (RXD)
 * - Teensy Pin 0 (RX1) -> Pi5 GPIO 14 (TXD)
 * - Teensy GND -> Pi5 GND
 */

// Buffer to store incoming serial data
String inputString = "";
boolean stringComplete = false;

void setup() {
  // Initialize hardware serial (Serial1) for Pi communication at 115200 baud
  Serial1.begin(115200);
  
  // Optional: Use Serial (USB) for debugging
  Serial.begin(9600);
  Serial.println("Teensy-Pi Serial Bridge Ready");
  
  // Send startup message to Pi
  Serial1.println("Teensy Serial Test Ready!");
  Serial1.println("Send any message and I'll respond.");
  Serial1.println("----------------------------------------");
  
  // Reserve 200 bytes for the input string
  inputString.reserve(200);
}

void loop() {
  // Check if data is available from Pi
  while (Serial1.available() > 0) {
    // Read the incoming byte
    char inChar = (char)Serial1.read();
    
    // Add it to the inputString
    inputString += inChar;
    
    // If the incoming character is a newline, set flag
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
  // If a complete string has been received
  if (stringComplete) {
    // Debug: Echo to USB serial
    Serial.print("From Pi: ");
    Serial.println(inputString);
    
    // Echo back to Pi what was received
    Serial1.print("Received: ");
    Serial1.print(inputString);
    
    // Trim whitespace and check for specific commands
    inputString.trim();
    
    if (inputString.equalsIgnoreCase("ping")) {
      Serial1.println("Response: PONG");
    }
    else if (inputString.equalsIgnoreCase("status")) {
      Serial1.println("Response: Teensy is running and healthy!");
    }
    else if (inputString.equalsIgnoreCase("help")) {
      Serial1.println("Response: Available commands:");
      Serial1.println("  - ping: Get a PONG response");
      Serial1.println("  - status: Get system status");
      Serial1.println("  - help: Show this help message");
      Serial1.println("  - Any other text will be echoed back");
    }
    else {
      Serial1.print("Response: Echo - \"");
      Serial1.print(inputString);
      Serial1.println("\"");
    }
    
    Serial1.println("----------------------------------------");
    
    // Clear the string for next input
    inputString = "";
    stringComplete = false;
  }
}
