#include <Arduino.h>
#include <Servo.h>

// Select control mode here.
// - DIRECT_PWM: PWM + DIR pins (default)
// - SERVO_PWM: RC-style single signal (1500us neutral)
enum class DriveMode { DIRECT_PWM, SERVO_PWM };
constexpr DriveMode kDriveMode = DriveMode::DIRECT_PWM;

// DIRECT_PWM mode wiring/tuning
constexpr int kPwmPin = 6;
constexpr int kDirPin = 7;
constexpr int kMaxPwmDuty = 255;
constexpr int kPwmRampStep = 5;
constexpr int kPwmStepDelayMs = 4;
constexpr int kExtendDirLevel = HIGH;

// SERVO_PWM mode wiring/tuning
constexpr int kSparkSignalPin = 6;
constexpr int kMinUs = 1000;
constexpr int kNeutralUs = 1500;
constexpr int kMaxUs = 2000;
constexpr int kMaxDeltaUs = 450;
constexpr int kServoStepDelayMs = 8;
constexpr int kArmTimeMs = 2000;

// Shared timing
constexpr int kStopHoldMs = 750;
constexpr int kAutoExtendMs = 1200;
constexpr int kAutoRetractMs = 1200;
constexpr int kAutoPauseMs = 600;

Servo spark;
int currentCommandUs = kNeutralUs;
int currentDuty = 0;
int currentDir = 0;  // -1 retract, 0 stopped, +1 extend
bool autoCycle = false;
unsigned long stateStartMs = 0;
int autoState = 0;

void writeCommandUs(int targetUs) {
  int clamped = constrain(targetUs, kMinUs, kMaxUs);
  int step = (clamped >= currentCommandUs) ? 1 : -1;

  for (int us = currentCommandUs; us != clamped; us += step) {
    spark.writeMicroseconds(us);
    delay(kServoStepDelayMs);
  }
  spark.writeMicroseconds(clamped);
  currentCommandUs = clamped;
}

void armSpark() {
  spark.writeMicroseconds(kNeutralUs);
  currentCommandUs = kNeutralUs;
  delay(kArmTimeMs);
}

void writeDuty(int targetDuty) {
  int clamped = constrain(targetDuty, 0, kMaxPwmDuty);
  int step = (clamped >= currentDuty) ? kPwmRampStep : -kPwmRampStep;

  for (int duty = currentDuty; duty != clamped; duty += step) {
    analogWrite(kPwmPin, constrain(duty, 0, kMaxPwmDuty));
    delay(kPwmStepDelayMs);
    if ((step > 0 && duty + step > clamped) || (step < 0 && duty + step < clamped)) {
      break;
    }
  }
  analogWrite(kPwmPin, clamped);
  currentDuty = clamped;
}

void stopMotor() {
  if (kDriveMode == DriveMode::SERVO_PWM) {
    writeCommandUs(kNeutralUs);
  } else {
    writeDuty(0);
    currentDir = 0;
  }
}

void commandDirect(bool extending, int percent) {
  percent = constrain(percent, 0, 100);
  int targetDir = extending ? 1 : -1;
  int targetDuty = (kMaxPwmDuty * percent) / 100;

  if (targetDuty == 0) {
    stopMotor();
    return;
  }

  // Force a neutral pause before reversing actuator direction.
  if (currentDir != 0 && currentDir != targetDir) {
    writeDuty(0);
    delay(kStopHoldMs);
  }

  digitalWrite(kDirPin, extending ? kExtendDirLevel : !kExtendDirLevel);
  currentDir = targetDir;
  writeDuty(targetDuty);
}

void extend(int percent) {
  percent = constrain(percent, 0, 100);
  if (kDriveMode == DriveMode::SERVO_PWM) {
    int targetUs = kNeutralUs + ((kMaxDeltaUs * percent) / 100);
    writeCommandUs(targetUs);
  } else {
    commandDirect(true, percent);
  }
}

void retract(int percent) {
  percent = constrain(percent, 0, 100);
  if (kDriveMode == DriveMode::SERVO_PWM) {
    int targetUs = kNeutralUs - ((kMaxDeltaUs * percent) / 100);
    writeCommandUs(targetUs);
  } else {
    commandDirect(false, percent);
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  e <0-100>  : extend at percent power");
  Serial.println("  r <0-100>  : retract at percent power");
  Serial.println("  u <1000-2000> : raw pulse width in us (SERVO mode)");
  Serial.println("  s          : stop");
  Serial.println("  t          : one-shot full test (SERVO mode)");
  Serial.println("  a          : toggle auto cycle");
  Serial.println("  h          : help");
  Serial.println();
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  char cmd = static_cast<char>(tolower(Serial.read()));
  int value = Serial.parseInt();

  if (cmd == 'e') {
    autoCycle = false;
    extend(value == 0 ? 40 : value);
    Serial.print("Extend ");
    Serial.print(value == 0 ? 40 : value);
    Serial.println("%");
  } else if (cmd == 'r') {
    autoCycle = false;
    retract(value == 0 ? 40 : value);
    Serial.print("Retract ");
    Serial.print(value == 0 ? 40 : value);
    Serial.println("%");
  } else if (cmd == 's') {
    autoCycle = false;
    stopMotor();
    Serial.println("Stop");
  } else if (cmd == 'u') {
    autoCycle = false;
    if (kDriveMode == DriveMode::SERVO_PWM) {
      int us = constrain(value, kMinUs, kMaxUs);
      writeCommandUs(us);
      Serial.print("Pulse ");
      Serial.print(us);
      Serial.println("us");
    }
  } else if (cmd == 't') {
    autoCycle = false;
    if (kDriveMode == DriveMode::SERVO_PWM) {
      Serial.println("Test: 2000us -> stop -> 1000us -> stop");
      writeCommandUs(kMaxUs);
      delay(1500);
      stopMotor();
      delay(800);
      writeCommandUs(kMinUs);
      delay(1500);
      stopMotor();
    }
  } else if (cmd == 'a') {
    autoCycle = !autoCycle;
    stopMotor();
    stateStartMs = millis();
    autoState = 0;
    Serial.println(autoCycle ? "Auto cycle ON" : "Auto cycle OFF");
  } else if (cmd == 'h') {
    printHelp();
  }
}

void runAutoCycle() {
  if (!autoCycle) {
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - stateStartMs;

  switch (autoState) {
    case 0:
      extend(70);
      stateStartMs = now;
      autoState = 1;
      break;
    case 1:
      if (elapsed >= kAutoExtendMs) {
        stopMotor();
        stateStartMs = now;
        autoState = 2;
      }
      break;
    case 2:
      if (elapsed >= kAutoPauseMs) {
        retract(70);
        stateStartMs = now;
        autoState = 3;
      }
      break;
    case 3:
      if (elapsed >= kAutoRetractMs) {
        stopMotor();
        stateStartMs = now;
        autoState = 4;
      }
      break;
    default:
      if (elapsed >= kAutoPauseMs) {
        autoState = 0;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);

  if (kDriveMode == DriveMode::SERVO_PWM) {
    spark.attach(kSparkSignalPin, kMinUs, kMaxUs);
    armSpark();
    delay(kStopHoldMs);
  } else {
    pinMode(kPwmPin, OUTPUT);
    pinMode(kDirPin, OUTPUT);
    analogWrite(kPwmPin, 0);
    digitalWrite(kDirPin, kExtendDirLevel);
    delay(kStopHoldMs);
  }

  Serial.print("Linear actuator test ready. Mode: ");
  Serial.println(kDriveMode == DriveMode::SERVO_PWM ? "SERVO_PWM" : "DIRECT_PWM");
  Serial.print("Signal pin: ");
  Serial.println(kSparkSignalPin);
  printHelp();
}

void loop() {
  handleSerial();
  runAutoCycle();
}
