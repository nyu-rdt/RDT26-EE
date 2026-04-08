// Use the pwr/gnd rails on one side for the teensy and the other side for pwr supply with cap
// ENABLE and FAULT not wired
// M0 goes to pin 10, M1 to pin 11, M2 to pin 12
// Reset and Sleep go to power rail --> Teensy 5v
// Step goes to pin 7 and DIR goes to pin 6
// VMOT and GND go to 100E-6 cap on the other side pwr and gnd rails w power supply
// B2 to candy cane green/yellow on stepper, B1 to solid green/green, A1 to solid red/blue, A2 to candy cane red/white
// GND to gnd rail duh
// Power 24V
#include <Arduino.h>
#define dirPin 6
#define stepPin 7

#define M0 10
#define M1 11
#define M2 12

#define stepsPerRevolution 200
#define STEP_DELAY 450  // increase this if motor stalls (try 3000, 4000) [450 is the fastest it will go!!!]

void setMicrostep(int mode) {
  switch (mode) {
    case 1:  digitalWrite(M0, LOW);  digitalWrite(M1, LOW);  digitalWrite(M2, LOW);  break;
    case 2:  digitalWrite(M0, HIGH); digitalWrite(M1, LOW);  digitalWrite(M2, LOW);  break;
    case 4:  digitalWrite(M0, LOW);  digitalWrite(M1, HIGH); digitalWrite(M2, LOW);  break;
    case 8:  digitalWrite(M0, HIGH); digitalWrite(M1, HIGH); digitalWrite(M2, LOW);  break;
    case 16: digitalWrite(M0, LOW);  digitalWrite(M1, LOW);  digitalWrite(M2, HIGH); break;
    case 32: digitalWrite(M0, HIGH); digitalWrite(M1, LOW);  digitalWrite(M2, HIGH); break;
  }
}

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  Serial.begin(9600);

  setMicrostep(1);  // start at full step — easiest for motor to move, change to 16 once working

  digitalWrite(dirPin, HIGH);  // LOW for DOWN, HIGH for UP
  Serial.println(digitalRead(dirPin));
}                                           

void loop() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY);
}
