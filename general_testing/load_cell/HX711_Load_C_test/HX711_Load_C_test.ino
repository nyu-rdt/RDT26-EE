#include "HX711.h"

// Pin Definitions
const int LOADCELL_DOUT_PIN = 1;
const int LOADCELL_SCK_PIN = 2;


HX711 scale;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give the Serial monitor time to open
  
  Serial.println("HX711 Demo - Teensy 4.1");
  Serial.println("Initializing the scale...");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // This checks if the chip is responding
  if (scale.is_ready()) {
    Serial.println("HX711 found. Taring...");
    scale.set_scale();    // Reset scale to 1 (default)
    scale.tare();         // Reset the scale to 0
    Serial.println("Tare complete. Place a weight on the load cell.");
  } else {
    Serial.println("HX711 not found. Check your wiring (DT/SCK pins).");
 
  }
scale.set_offset(5567);  scale.set_scale(96.574272);
}

void loop() {
  if (scale.is_ready()) {
    // scale.get_units(10) averages 10 readings for stability
    long reading = scale.get_units(10);
    Serial.print("Raw Value: ");
    Serial.println(reading);
  } else {
    Serial.println("HX711 not found.");
  }

  delay(250); // Short delay to keep the Serial monitor readable
}