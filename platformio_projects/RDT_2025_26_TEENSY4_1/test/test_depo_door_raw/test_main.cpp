#include <Arduino.h>

// H-bridge pins
#define IN1 40
#define IN2 41
#define ENA 14

// Board enables — relay must be HIGH for power, stepper enable HIGH disables driver
#define RELAY_PIN   2
#define STEPPER_EN  8

#define TRAVEL_MS 3200
#define PAUSE_MS  2000

void driveUp()   { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
void driveDown() { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
void stop()      { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW); }

void setup() {
    Serial.begin(115200);

    pinMode(RELAY_PIN,  OUTPUT); digitalWrite(RELAY_PIN,  HIGH);
    pinMode(STEPPER_EN, OUTPUT); digitalWrite(STEPPER_EN, HIGH);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT); analogWrite(ENA, 128);

    stop();
    delay(500);
    Serial.println("depo door raw test — UP / STOP / DOWN / STOP repeating");
}

void loop() {
    Serial.println("driving UP");
    driveUp();
    delay(TRAVEL_MS);

    Serial.println("stop");
    stop();
    delay(PAUSE_MS);

    Serial.println("driving DOWN");
    driveDown();
    delay(TRAVEL_MS);

    Serial.println("stop");
    stop();
    delay(PAUSE_MS);
}
