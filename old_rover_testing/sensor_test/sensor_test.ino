#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include "HX711.h"
#include <Servo.h>

// Global Variables and Constants
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// String pot constants
float string_length = 0;
const float VOLTAGE_LENGTH_CONVERT = 27;
const float VOLTAGE_LENGTH_CONVERT_Y_INTERCEPT = 0.719;
#define STRING_POT_PIN A3

// Load cell variables
HX711 scale1, scale2, scale3, scale4;
float calibration_factor1 = -102;
float calibration_factor2 = 105;
float calibration_factor3 = -102;
float calibration_factor4 = 111;
float weight1, weight2, weight3, weight4, weight_avg;

// Load cell pins
#define DOUT1 24
#define CLK1 25
#define DOUT2 26
#define CLK2 27
#define DOUT3 34
#define CLK3 33
#define DOUT4 20
#define CLK4 21

// Encoder pins
const uint8_t ENC1_A = 36, ENC1_B = 35;  // First encoder pins
const uint8_t ENC2_A = 38, ENC2_B = 37;  // Second encoder pins 
const uint8_t ENC3_A = 40, ENC3_B = 39;  // Third encoder pins
const uint8_t ENC4_A = 14, ENC4_B = 15;  // Fourth encoder pins

// Encoder properties
const float COUNTS_PER_REVOLUTION = 8192.0;
const float DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REVOLUTION;

// Encoder counts - volatile because they're modified in interrupts
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;

// Track angles
float angle1 = 0.0;
float angle2 = 0.0;
float angle3 = 0.0; 
float angle4 = 0.0;

// Timing variables
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 200; // Update every 200ms

// ISR routines for encoders
void isr1A() { count1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : +1; } // Swapped +/- signs
void isr1B() { count1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? -1 : +1; } // Swapped +/- signs

void isr2A() { count2 += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : +1; } // Swapped +/- signs
void isr2B() { count2 += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? -1 : +1; } // Swapped +/- signs

void isr3A() { count3 += (digitalRead(ENC3_A) == digitalRead(ENC3_B)) ? +1 : -1; }
void isr3B() { count3 += (digitalRead(ENC3_A) != digitalRead(ENC3_B)) ? +1 : -1; }

void isr4A() { count4 += (digitalRead(ENC4_A) == digitalRead(ENC4_B)) ? -1 : +1; } // Swapped +/- signs
void isr4B() { count4 += (digitalRead(ENC4_A) != digitalRead(ENC4_B)) ? -1 : +1; } // Swapped +/- signs

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial connection
  
  Serial.println("\n\n=== Rover Sensor Diagnostics ===");
  Serial.println("Monitoring all sensors continuously");
  
  // Initialize CAN bus
  can1.begin();
  can1.setBaudRate(500000);
  
  // Initialize load cells
  initLoadCells();
  
  // Configure encoder pins as inputs with pull-ups
  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP); pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP); pinMode(ENC4_B, INPUT_PULLUP);
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), isr1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), isr2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), isr3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_B), isr3B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), isr4A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_B), isr4B, CHANGE);

  Serial.println("All sensors initialized");
  Serial.println("------------------------");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update and print sensor data at regular intervals
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    updateAllData();
    printAllData();
    lastPrintTime = currentTime;
  }
}

void updateAllData() {
  // Update string pot reading
  string_length = getStringPotLength();
  
  // Update load cell readings
  weight1 = scale1.get_units(1);
  weight2 = scale2.get_units(1);
  weight3 = scale3.get_units(1);
  weight4 = scale4.get_units(1);
  weight_avg = (weight1 + weight2 + weight3 + weight4) / 4.0;
  
  // Update encoder angles (done atomically)
  noInterrupts();
  long c1 = count1;
  long c2 = count2;
  long c3 = count3;
  long c4 = count4;
  interrupts();
  
  angle1 = countsToAngle(c1);
  angle2 = countsToAngle(c2);
  angle3 = countsToAngle(c3);
  angle4 = countsToAngle(c4);
}

void printAllData() {
  // Clear previous output
  Serial.print("\033[2J\033[H");  // ANSI escape code to clear screen and move cursor to home
  
  Serial.println("=== ROVER SENSOR DIAGNOSTICS ===");
  Serial.print("Time: "); Serial.print(millis() / 1000.0); Serial.println(" seconds");
  Serial.println();
  
  // String potentiometer
  Serial.println("--- String Potentiometer ---");
  Serial.print("Length: "); Serial.print(string_length); Serial.println(" cm");
  Serial.println();
  
  // Load cells
  Serial.println("--- Load Cells (Weight) ---");
  Serial.print("Cell 1: "); Serial.print(weight1); Serial.println(" kg");
  Serial.print("Cell 2: "); Serial.print(weight2); Serial.println(" kg");
  Serial.print("Cell 3: "); Serial.print(weight3); Serial.println(" kg");
  Serial.print("Cell 4: "); Serial.print(weight4); Serial.println(" kg");
  Serial.print("Average: "); Serial.print(weight_avg); Serial.println(" kg");
  Serial.println();
  
  // Encoders
  Serial.println("--- Encoders (Angles) ---");
  Serial.print("Encoder 1: "); Serial.print(angle1); Serial.println(" degrees");
  Serial.print("Encoder 2: "); Serial.print(angle2); Serial.println(" degrees");
  Serial.print("Encoder 3: "); Serial.print(angle3); Serial.println(" degrees");
  Serial.print("Encoder 4: "); Serial.print(angle4); Serial.println(" degrees");
  Serial.println();
  
  Serial.println("------------------------");
}

void initLoadCells() {
  scale1.begin(DOUT1, CLK1);
  scale2.begin(DOUT2, CLK2);
  scale3.begin(DOUT3, CLK3);
  scale4.begin(DOUT4, CLK4);
  
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  
  Serial.println("Taring load cells... please wait");
  scale1.tare();
  scale2.tare();
  scale3.tare();
  scale4.tare();
  Serial.println("Load cells tared");
}

float getStringPotLength() {
  float sensorValue = analogRead(STRING_POT_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  return VOLTAGE_LENGTH_CONVERT * voltage - VOLTAGE_LENGTH_CONVERT_Y_INTERCEPT;
}

float countsToAngle(long counts) {
  float angle = fmod(counts * DEGREES_PER_COUNT, 360.0);
  if (angle < 0) angle += 360.0;
  return angle;
}