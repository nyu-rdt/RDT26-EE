#include <Arduino.h>
#include <Servo.h>

// SPARKmini full PWM range:
// stop=1500us, full extend=2500us, full retract=500us.
// NOTE: On this hardware setup, motion only occurs near the endpoints.
// We treat control as binary with thresholds:
//   <= 550us  -> retract
//   >= 2450us -> extend
//   otherwise -> stop
constexpr int kActuatorPin = 6;
constexpr int kPulseStopUs = 1500;
constexpr int kPulseExtendUs = 2400;
constexpr int kPulseRetractUs = 600;
constexpr int kExtendThresholdUs = 2450;
constexpr int kRetractThresholdUs = 550;
constexpr int kArmDelayMs = 2000;

Servo actuator;

void stopActuator() {
  actuator.writeMicroseconds(kPulseStopUs);
  Serial.println("STOP (1500us)");
}

void extendActuator() {
  actuator.writeMicroseconds(kPulseExtendUs);
  Serial.println("EXTEND (2100us)");
}

void retractActuator() {
  actuator.writeMicroseconds(kPulseRetractUs);
  Serial.println("RETRACT (900us)");
}

void oneShotTest() {
  Serial.println("One-shot: extend 2s, stop 1s, retract 2s, stop");
  extendActuator();
  delay(2000);
  stopActuator();
  delay(1000);
  retractActuator();
  delay(2000);
  stopActuator();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  u : extend (2500us)");
  Serial.println("  d : retract (500us)");
  Serial.println("  s : stop (1500us)");
  Serial.println("  t : one-shot test cycle");
  Serial.println("  p <500-2500> : thresholded pulse cmd");
  Serial.println("                 <=550 retract, >=2450 extend, else stop");
  Serial.println("  h : help");
  Serial.println();
}

void thresholdedPulseCommand(int us) {
  us = constrain(us, 0, 3000);
    actuator.writeMicroseconds(us);


  Serial.print("INPUT ");
  Serial.print(us);
  Serial.println("us");
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  char cmd = static_cast<char>(tolower(Serial.read()));
  int value = Serial.parseInt();

  if (cmd == 'u') {
    extendActuator();
  } else if (cmd == 'd') {
    retractActuator();
  } else if (cmd == 's') {
    stopActuator();
  } else if (cmd == 't') {
    oneShotTest();
  } else if (cmd == 'p') {
    thresholdedPulseCommand(value);
  } else if (cmd == 'h') {
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);

  actuator.attach(kActuatorPin); // Set max pulse to accommodate extend range
  actuator.writeMicroseconds(kPulseStopUs);
  delay(kArmDelayMs);

  Serial.println("Linear actuator simple test rig ready.");
  Serial.print("Pin: ");
  Serial.println(kActuatorPin);
  Serial.println("Mode: endpoint-threshold control");
  printHelp();
}

void loop() {
  handleSerial();
}
