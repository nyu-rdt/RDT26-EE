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
#include "stepper_driver.h"
#include "config.h" 

#define STEPPER_DIR_PIN 6
#define STEPPER_STEP_PIN 7
#define STEPPER_ENABLE_PIN 8

// Microstepping pins on DRV8825:
#define STEPPER_M0_PIN 10
#define STEPPER_M1_PIN 11
#define STEPPER_M2_PIN 12

#define STEPS_PER_REVOLUTION 200  // full-step count
#define MICROSTEPPING_FACTOR 1  // set this to 1,2,4,8,16,32 depending on desired microstepping mode

bool STEPPER_enable = false; // global variable to track if motor should be enabled
unsigned long lastStepTime = 0; // track last step time for timing control, in microseconds
bool isStepPinHigh = false; // track state of step pin for timing control

void STEPPER_Init() {
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_ENABLE_PIN, OUTPUT);
    pinMode(STEPPER_M0_PIN, OUTPUT);
    pinMode(STEPPER_M1_PIN, OUTPUT);
    pinMode(STEPPER_M2_PIN, OUTPUT);
    setMicrostep(MICROSTEPPING_FACTOR); 

    // Start with motor disabled
    digitalWrite(STEPPER_ENABLE_PIN, HIGH); // HIGH to disable
}

void setMicrostep(int mode) {
  // mode = 1, 2, 4, 8, 16, 32
  // set the step mode - full, 1/2, 1/4, 1/8, 1/16, or 1/32 step
  switch (mode) {
    case 1:   digitalWrite(STEPPER_M0_PIN, LOW);  digitalWrite(STEPPER_M1_PIN, LOW);  digitalWrite(STEPPER_M2_PIN, LOW);  break;
    case 2:   digitalWrite(STEPPER_M0_PIN, HIGH); digitalWrite(STEPPER_M1_PIN, LOW);  digitalWrite(STEPPER_M2_PIN, LOW);  break;
    case 4:   digitalWrite(STEPPER_M0_PIN, LOW);  digitalWrite(STEPPER_M1_PIN, HIGH); digitalWrite(STEPPER_M2_PIN, LOW);  break;
    case 8:   digitalWrite(STEPPER_M0_PIN, HIGH); digitalWrite(STEPPER_M1_PIN, HIGH); digitalWrite(STEPPER_M2_PIN, LOW);  break;
    case 16:  digitalWrite(STEPPER_M0_PIN, LOW);  digitalWrite(STEPPER_M1_PIN, LOW);  digitalWrite(STEPPER_M2_PIN, HIGH); break;
    case 32:  digitalWrite(STEPPER_M0_PIN, HIGH); digitalWrite(STEPPER_M1_PIN, LOW);  digitalWrite(STEPPER_M2_PIN, HIGH); break;
  }
}

// dir is 1 for up, -1 for down, 0 for stop
// enable stores if the motor should be on or not
// we set dirPin based on direction input, qhich directly controlls motor
void STEPPER_SetDirection(int direction) {
    // // S = (direction==0) ? false : true; // sets enable true if moving, false if stopped
    STEPPER_enable = (direction!=0);
    (STEPPER_enable) ? (digitalWrite(STEPPER_ENABLE_PIN, LOW)) : (digitalWrite(STEPPER_ENABLE_PIN, HIGH)); // enable must be HIGH when NOT moving
    if (direction == 1) { // UP
        digitalWrite(STEPPER_DIR_PIN, LOW);
    } else if (direction == -1) { // DOWN
        digitalWrite(STEPPER_DIR_PIN, HIGH);
    }
}

// call every loop to step, handles its own timing
// using micros so it doesnt have to stop the rest of the program 
// every period/2 it toggels the output, giving us a rising edge every period, which steps the motor
void STEPPER_Update(unsigned long periodMicroseconds) {
    if (STEPPER_enable) {
        unsigned long currentTime = micros();
        if (currentTime - lastStepTime >= periodMicroseconds/2) {
            isStepPinHigh = !isStepPinHigh; 
            digitalWrite(stepPin, isStepPinHigh ? HIGH : LOW);
            lastStepTime = currentTime;
        }
    }
}
