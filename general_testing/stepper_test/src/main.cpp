// Use the pwr/gnd rails on one side for the teensy and the other side for pwr supply with cap
// ENABLE and FAULT not wired
// M0 goes to pin 10, M1 to pin 11, M2 to pin 12
// Reset and Sleep go to power rail --> Teensy 5v
// Step goes to pin 7 and DIR goes to pin 6
// VMOT and GND go to 100E-6 cap on the other side pwr and gnd rails w power supply
// B2 to candy cane green on stepper, B1 to solid green, A1 to solid red, A2 to candy cane red
// GND to gnd rail duh
// Set pwr supply to ~12V

#include <Arduino.h>

#define dirPin 6
#define stepPin 7

// Microstepping pins on DRV8825:
#define M0 10
#define M1 11
#define M2 12

#define stepsPerRevolution 200  // full-step count

void setMicrostep(int mode) {
  // mode = 1, 2, 4, 8, 16, 32
  switch (mode) {
    case 1:   digitalWrite(M0, LOW);  digitalWrite(M1, LOW);  digitalWrite(M2, LOW);  break;
    case 2:   digitalWrite(M0, HIGH); digitalWrite(M1, LOW);  digitalWrite(M2, LOW);  break;
    case 4:   digitalWrite(M0, LOW);  digitalWrite(M1, HIGH); digitalWrite(M2, LOW);  break;
    case 8:   digitalWrite(M0, HIGH); digitalWrite(M1, HIGH); digitalWrite(M2, LOW);  break;
    case 16:  digitalWrite(M0, LOW);  digitalWrite(M1, LOW);  digitalWrite(M2, HIGH); break;
    case 32:  digitalWrite(M0, HIGH); digitalWrite(M1, LOW);  digitalWrite(M2, HIGH); break;
  }
}

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  setMicrostep(1);  // CHANGE THIS to 1,2,4,8,16,32
}

void loop() {
  int micro = 16;  // microstepping factor used above
  int totalSteps = stepsPerRevolution * micro;

  digitalWrite(dirPin, HIGH);

  // DOWN
  for (int i = 0; i < totalSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);   // adjust speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }

  // delay(1000);

  digitalWrite(dirPin, LOW);

  // UP
  for (int i = 0; i < totalSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }

  delay(1000);
}
