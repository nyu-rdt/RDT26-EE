/*==========================================================================
// Project : Teensy 4.1
// Description : L298N Motor Driver - Single Linear Actuator
// Control one linear actuator using L298N H Bridge Driver
//==========================================================================*/

// WORKING ARDUINO MOTOR TEST CODE FOR L298N MOTOR DRIVER
// Pin definitions - Teensy 4.1 pins connected to L298N
int IN3 = 3;
int IN4 = 4;

void setup() {
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 
}

void loop() {
  // extend for 10s
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(10000);

  // retract for 10s
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(10000);
}