#include <Arduino.h>
/*
 * Teensy to Raspberry Pi 5 USB Serial Communication Test
 * 
 * This code communicates with a Raspberry Pi 5 over USB serial
 * Connect Teensy to Pi5 via USB cable
 * 
 * On Pi5, the Teensy will appear as /dev/ttyACM0 (or similar)
 * 
 * Setup on Pi:
 *   sudo chmod 666 /dev/ttyACM0  (grant permissions)
 *   
 * Test with:
 *   screen /dev/ttyACM0 115200
 *   or
 *   minicom -D /dev/ttyACM0 -b 115200
 *   
 * Python example:
 *   import serial
 *   ser = serial.Serial('/dev/ttyACM0', 115200)
 *   ser.write(b'ping\n')
 *   print(ser.readline())
 */

// Buffer to store incoming serial data
String inputString = "";
boolean stringComplete = false;

void setup() {
  // Initialize USB serial for Pi communication at 115200 baud
  Serial.begin(115200);
  
  // Wait a moment for serial connection to establish
  delay(1000);
  
  // Send startup message to Pi
  Serial.println("Teensy Serial Test Ready!");
  Serial.println("Send any message and I'll respond.");
  Serial.println("----------------------------------------");
  
  // Reserve 200 bytes for the input string
  inputString.reserve(200);
}

void loop() {
  // Check if data is available from Pi over USB serial
  while (Serial.available() > 0) {
    // Read the incoming byte
    char inChar = (char)Serial.read();
    
    // Add it to the inputString
    inputString += inChar;
    
    // If the incoming character is a newline, set flag
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
  // If a complete string has been received
  if (stringComplete) {
    // Echo back to Pi what was received
    Serial.print("Received: ");
    Serial.print(inputString);
    
    // Trim whitespace and check for specific commands
    inputString.trim();
    
    if (inputString.equalsIgnoreCase("ping")) {
      Serial.println("Response: PONG");
    }
    else if (inputString.equalsIgnoreCase("status")) {
      Serial.println("Response: Teensy is running and healthy!");
    }
    else if (inputString.equalsIgnoreCase("help")) {
      Serial.println("Response: Available commands:");
      Serial.println("  - ping: Get a PONG response");
      Serial.println("  - status: Get system status");
      Serial.println("  - help: Show this help message");
      Serial.println("  - Any other text will be echoed back");
    }
    else {
      Serial.print("Response: Echo - \"");
      Serial.print(inputString);
      Serial.println("\"");
    }
    
    Serial.println("----------------------------------------");
    
    // Clear the string for next input
    inputString = "";
    stringComplete = false;
  }
}
