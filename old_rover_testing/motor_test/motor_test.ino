#include <FlexCAN_T4.h>
#include <Servo.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
Servo excavationSystemPWM;
Servo excavationBeltPWM;

// CAN IDs from your system
#define REAR_LEFT_MOTOR_CAN_ID 0x48 
#define FRONT_LEFT_MOTOR_CAN_ID 0x78  
#define FRONT_RIGHT_MOTOR_CAN_ID 0x16 
#define REAR_RIGHT_MOTOR_CAN_ID 0x67 
#define DEPOSITION_MOTOR_CAN_ID 0x34
#define EXCAVATION_BELT_MOTOR_CAN_ID 0x68

// PWM pins
#define EXCAVATION_SYSTEM_PWM_PIN 11

// Motor selection constants
#define MOTOR_FRONT_LEFT 1
#define MOTOR_FRONT_RIGHT 2  
#define MOTOR_REAR_LEFT 3
#define MOTOR_REAR_RIGHT 4
#define MOTOR_DEPOSITION 5
#define MOTOR_EXCAVATION_BELT 6
#define MOTOR_EXCAVATION_SYSTEM 7

// Function prototypes
CAN_message_t craftMessage(int typeofmsg, float val, uint8_t id);
int writeCANMessage(CAN_message_t to_send);
void setMotorSpeed(int motorId, float speed);
void stopAllMotors();
void moveAllWheels(float speed);
void turnLeft(float speed);
void turnRight(float speed);
void displayMenu();
void printMotorInfo(int motorId);
void testIndividualMotor(int motorId);
void continuousMotorControl(int motorId, float speed);
float getSpeedInput();
void runExcavationTest();
void runFullRobotTest();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial
  
  Serial.println("==== Motor Test Program ====");
  Serial.println("Testing CAN and PWM motors");
  
  // Initialize CAN
  can1.begin();
  can1.setBaudRate(500000);
  
  // Initialize PWM motors
  excavationSystemPWM.attach(EXCAVATION_SYSTEM_PWM_PIN);
  excavationSystemPWM.writeMicroseconds(1500); // Neutral
  
  delay(1000);
}

void loop() {
  displayMenu();
  
  while (!Serial.available()) {
    delay(100);
  }
  
  int choice = Serial.parseInt();
  Serial.read(); // Clear the newline character
  
  switch (choice) {
    // Individual motor selections
    case MOTOR_FRONT_LEFT:
    case MOTOR_FRONT_RIGHT:
    case MOTOR_REAR_LEFT:
    case MOTOR_REAR_RIGHT:
    case MOTOR_DEPOSITION:
    case MOTOR_EXCAVATION_BELT:
    case MOTOR_EXCAVATION_SYSTEM:
      printMotorInfo(choice);
      testIndividualMotor(choice);
      break;
      
    // Group operations
    case 8: // All forward
      Serial.println("Moving all wheels forward");
      moveAllWheels(getSpeedInput());
      Serial.println("Press 'q' to stop");
      while (true) {
        if (Serial.available() && Serial.read() == 'q') break;
        delay(100);
      }
      stopAllMotors();
      break;
      
    case 9: // All backward
      Serial.println("Moving all wheels backward");
      moveAllWheels(-getSpeedInput());
      Serial.println("Press 'q' to stop");
      while (true) {
        if (Serial.available() && Serial.read() == 'q') break;
        delay(100);
      }
      stopAllMotors();
      break;
      
    case 10: // Turn left
      Serial.println("Turning left");
      turnLeft(getSpeedInput());
      Serial.println("Press 'q' to stop");
      while (true) {
        if (Serial.available() && Serial.read() == 'q') break;
        delay(100);
      }
      stopAllMotors();
      break;
      
    case 11: // Turn right
      Serial.println("Turning right");
      turnRight(getSpeedInput());
      Serial.println("Press 'q' to stop");
      while (true) {
        if (Serial.available() && Serial.read() == 'q') break;
        delay(100);
      }
      stopAllMotors();
      break;

    // New combined test cases
    case 12: // Excavation System + Belt
      runExcavationTest();
      break;
      
    case 13: // Full Robot Test
      runFullRobotTest();
      break;
      
    case 0: // Stop all motors
      Serial.println("Stopping all motors");
      stopAllMotors();
      break;
      
    default:
      Serial.println("Invalid choice");
      break;
  }
  
  // Clear any remaining input
  while (Serial.available()) Serial.read();
  delay(1000);
}

void displayMenu() {
  Serial.println("\n==== Motor Test Menu ====");
  Serial.println("Individual Motors:");
  Serial.println("1: Front Left Motor (CAN ID: 0x78)");
  Serial.println("2: Front Right Motor (CAN ID: 0x16)");  // Fixed to match actual IDs
  Serial.println("3: Rear Left Motor (CAN ID: 0x48)");
  Serial.println("4: Rear Right Motor (CAN ID: 0x67)");
  Serial.println("5: Deposition Motor (CAN ID: 0x34)");
  Serial.println("6: Excavation Belt Motor (CAN ID: 0x68)");
  Serial.println("7: Excavation System (PWM)");
  Serial.println("\nGroup Movement:");
  Serial.println("8: All Wheels Forward");
  Serial.println("9: All Wheels Backward");
  Serial.println("10: Turn Left");
  Serial.println("11: Turn Right");
  Serial.println("\nCombined Tests:");
  Serial.println("12: Excavation System + Belt");
  Serial.println("13: Full Robot Test");
  Serial.println("\n0: Stop All Motors");
  Serial.print("\nEnter your choice: ");
}

void printMotorInfo(int motorId) {
  Serial.print("Selected: ");
  
  switch (motorId) {
    case MOTOR_FRONT_LEFT:
      Serial.println("Front Left Motor (CAN ID: 0x78)");
      break;
    case MOTOR_FRONT_RIGHT:
      Serial.println("Front Right Motor (CAN ID: 0x16)");
      break;
    case MOTOR_REAR_LEFT:
      Serial.println("Rear Left Motor (CAN ID: 0x48)");
      break;
    case MOTOR_REAR_RIGHT:
      Serial.println("Rear Right Motor (CAN ID: 0x67)");
      break;
    case MOTOR_DEPOSITION:
      Serial.println("Deposition Motor (CAN ID: 0x34)");
      break;
    case MOTOR_EXCAVATION_BELT:
      Serial.println("Excavation Belt Motor (CAN ID: 0x68)");
      break;
    case MOTOR_EXCAVATION_SYSTEM:
      Serial.println("Excavation System (PWM)");
      break;
    default:
      Serial.println("Unknown Motor");
      break;
  }
}

float getSpeedInput() {
  Serial.println("Enter speed (-10 to 10): ");
  while (!Serial.available()) {
    delay(100);
  }
  
  float speed = Serial.parseFloat();
  Serial.read(); // Clear the newline character
  
  // Clamp speed between -10 and 10
  if (speed < -10) speed = -10;
  if (speed > 10) speed = 10;

  speed /= 10;
  
  Serial.print("Using speed: ");
  Serial.println(speed);
  
  return speed;
}

void testIndividualMotor(int motorId) {
  float speed = getSpeedInput();
  
  // Run motor for fixed duration (3 seconds)
  Serial.print("Running motor for 3 seconds at speed: ");
  Serial.println(speed);
  
  // Start the motor
  setMotorSpeed(motorId, speed);
  
  // Run for 3 seconds
  delay(3000);
  
  // Stop the motor
  setMotorSpeed(motorId, 0);
  Serial.println("Motor stopped");
}

void continuousMotorControl(int motorId, float speed) {
  setMotorSpeed(motorId, speed);
  Serial.println("Motor running in continuous mode");
  Serial.println("Press 'q' to stop");
  
  // Clear any pending input
  while (Serial.available()) Serial.read();
  
  // Run until 'q' is pressed
  bool running = true;
  while (running) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'q' || c == 'Q') {
        running = false;
      }
    }
    delay(100);
  }
  
  // Stop the motor
  setMotorSpeed(motorId, 0);
  Serial.println("Motor stopped");
  
  // Clear any remaining input
  while (Serial.available()) Serial.read();
}

void setMotorSpeed(int motorId, float speed) {
  uint32_t canId = 0;
  
  switch (motorId) {
    case MOTOR_FRONT_LEFT:
      canId = FRONT_LEFT_MOTOR_CAN_ID;
      break;
    case MOTOR_FRONT_RIGHT:
      canId = FRONT_RIGHT_MOTOR_CAN_ID;
      break;
    case MOTOR_REAR_LEFT:
      canId = REAR_LEFT_MOTOR_CAN_ID;
      break;
    case MOTOR_REAR_RIGHT:
      canId = REAR_RIGHT_MOTOR_CAN_ID;
      break;
    case MOTOR_DEPOSITION:
      canId = DEPOSITION_MOTOR_CAN_ID;
      break;
    case MOTOR_EXCAVATION_BELT:
      canId = EXCAVATION_BELT_MOTOR_CAN_ID;
      break;
    case MOTOR_EXCAVATION_SYSTEM:
      // PWM motor uses different control method
      if (speed == 0) {
        excavationSystemPWM.writeMicroseconds(1500); // Neutral
        Serial.println("Excavation system stopped");
      } else if (speed > 0) {
        excavationSystemPWM.writeMicroseconds(1450); // Up
        Serial.println("Excavation system moving up");
      } else {
        excavationSystemPWM.writeMicroseconds(1570); // Down
        Serial.println("Excavation system moving down");
      }
      return;
    default:
      return;
  }
  
  // For CAN motors
  if (canId > 0) {
    CAN_message_t msg = craftMessage(0, speed, canId);
    int result = writeCANMessage(msg);
    
    if (result == 1) {
      Serial.print("Message sent to CAN ID 0x");
      Serial.print(canId, HEX);
      Serial.print(" with speed ");
      Serial.println(speed);
    } else {
      Serial.println("Failed to send CAN message");
    }
  }
}

void stopAllMotors() {
  // Stop all CAN motors
  writeCANMessage(craftMessage(0, 0, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, REAR_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, DEPOSITION_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, EXCAVATION_BELT_MOTOR_CAN_ID));
  
  // Stop PWM motor
  excavationSystemPWM.writeMicroseconds(1500); // Neutral
  
  Serial.println("All motors stopped");
}

void moveAllWheels(float speed) {
  // Based on go_forward/backward from singlecompilation.ino
  if (speed > 0) {
    // Forward movement
    writeCANMessage(craftMessage(0, speed, REAR_RIGHT_MOTOR_CAN_ID));
    writeCANMessage(craftMessage(0, speed, FRONT_LEFT_MOTOR_CAN_ID));
    writeCANMessage(craftMessage(0, -speed, REAR_LEFT_MOTOR_CAN_ID));
    writeCANMessage(craftMessage(0, speed, FRONT_RIGHT_MOTOR_CAN_ID));
    Serial.println("All wheels moving forward");
  } else {
    // Backward movement
    writeCANMessage(craftMessage(0, -speed, REAR_RIGHT_MOTOR_CAN_ID));
    writeCANMessage(craftMessage(0, -speed, FRONT_LEFT_MOTOR_CAN_ID));
    writeCANMessage(craftMessage(0, speed, REAR_LEFT_MOTOR_CAN_ID));
    writeCANMessage(craftMessage(0, -speed, FRONT_RIGHT_MOTOR_CAN_ID));
    Serial.println("All wheels moving backward");
  }
}

void turnLeft(float speed) {
  // Based on turn_left from singlecompilation.ino
  float absSpeed = abs(speed);
  writeCANMessage(craftMessage(0, -absSpeed, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, absSpeed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, absSpeed, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, absSpeed, REAR_RIGHT_MOTOR_CAN_ID));
  Serial.print("Turning Left at speed: ");
  Serial.println(absSpeed);
}

void turnRight(float speed) {
  // Based on turn_right from singlecompilation.ino
  float absSpeed = abs(speed);
  writeCANMessage(craftMessage(0, absSpeed, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -absSpeed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -absSpeed, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -absSpeed, REAR_RIGHT_MOTOR_CAN_ID));
  Serial.print("Turning Right at speed: ");
  Serial.println(absSpeed);
}

void runExcavationTest() {
  Serial.println("=== EXCAVATION SYSTEM + BELT TEST ===");
  float beltSpeed = 0.3; // Low speed for belt
  
  Serial.println("Starting test - press 'q' to stop");
  Serial.println("Moving excavation system and belt...");
  
  // Start the excavation belt
  writeCANMessage(craftMessage(0, beltSpeed, EXCAVATION_BELT_MOTOR_CAN_ID));
  
  // Clear any pending input
  while (Serial.available()) Serial.read();
  
  bool running = true;
  bool excavationUp = true;
  unsigned long lastExcavationToggle = millis();
  
  while (running) {
    // Check for stop command
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'q' || c == 'Q') {
        running = false;
      }
    }
    
    // Toggle excavation system direction every 3 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastExcavationToggle > 3000) {
      excavationUp = !excavationUp;
      
      if (excavationUp) {
        excavationSystemPWM.writeMicroseconds(1470); // Small up movement
        Serial.println("Excavation system moving up");
      } else {
        excavationSystemPWM.writeMicroseconds(1540); // Small down movement
        Serial.println("Excavation system moving down");
      }
      
      lastExcavationToggle = currentTime;
    }
    
    delay(100);
  }
  
  // Stop all motors
  stopAllMotors();
  Serial.println("Excavation test stopped");
}

void runFullRobotTest() {
  Serial.println("=== FULL ROBOT TEST ===");
  float wheelSpeed = 0.2;     // Low speed for wheels
  float beltSpeed = 0.1;      // Low speed for belt
  float depositionSpeed = 0.15; // Low speed for deposition
  
  Serial.println("Starting comprehensive test - press 'q' to stop");
  Serial.println("Moving all systems...");
  
  // Clear any pending input
  while (Serial.available()) Serial.read();
  
  bool running = true;
  bool forwardMotion = true;
  bool excavationUp = true;
  
  unsigned long lastWheelToggle = millis();
  unsigned long lastExcavationToggle = millis();
  
  // Start belt and deposition
  writeCANMessage(craftMessage(0, beltSpeed, EXCAVATION_BELT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, depositionSpeed, DEPOSITION_MOTOR_CAN_ID));
  
  // Start with wheels forward
  moveAllWheels(wheelSpeed);
  
  // Start with excavation system up
  excavationSystemPWM.writeMicroseconds(1470); // Small up movement
  
  while (running) {
    // Check for stop command
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'q' || c == 'Q') {
        running = false;
      }
    }
    
    // Toggle wheel direction every 5 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastWheelToggle > 5000) {
      forwardMotion = !forwardMotion;
      
      if (forwardMotion) {
        Serial.println("Wheels moving forward");
        moveAllWheels(wheelSpeed);
      } else {
        Serial.println("Wheels moving backward");
        moveAllWheels(-wheelSpeed);
      }
      
      lastWheelToggle = currentTime;
    }
    
    // Toggle excavation system every 3 seconds
    if (currentTime - lastExcavationToggle > 2000) {
      excavationUp = !excavationUp;
      
      if (excavationUp) {
        excavationSystemPWM.writeMicroseconds(1470); // Small up movement
        Serial.println("Excavation system moving up");
      } else {
        excavationSystemPWM.writeMicroseconds(1530); // Small down movement
        Serial.println("Excavation system moving down");
      }
      
      lastExcavationToggle = currentTime;
    }
    
    delay(100);
  }
  
  // Stop all motors
  stopAllMotors();
  Serial.println("Full robot test stopped");
}

CAN_message_t craftMessage(int typeofmsg, float val, uint8_t id) {
  CAN_message_t new_msg;
  new_msg.flags.extended = 1;
  new_msg.id = (typeofmsg << 8) + id;  // Correctly format the CAN ID
  new_msg.len = 4;
  int32_t value = (int32_t)(val * 100000);
  new_msg.buf[0] = (value >> 24) & 0xFF;
  new_msg.buf[1] = (value >> 16) & 0xFF;
  new_msg.buf[2] = (value >> 8) & 0xFF;
  new_msg.buf[3] = value & 0xFF;
  return new_msg;
}

int writeCANMessage(CAN_message_t to_send) {
  return can1.write(to_send);
}