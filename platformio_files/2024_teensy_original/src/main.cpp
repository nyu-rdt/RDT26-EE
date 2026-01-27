#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include "HX711.h"
#include <Servo.h>

/************************************************************
 * CONSTANTS AND PIN DEFINITIONS
 ************************************************************/

// Pin Definitions
#define RELAY_PIN 2

#define STRING_POT_PIN A3
#define EXCAVATION_SYSTEM_PWM_PIN 11

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

// CAN IDs
#define FRONT_LEFT_MOTOR_CAN_ID 0x78
#define FRONT_RIGHT_MOTOR_CAN_ID 0x16
#define REAR_LEFT_MOTOR_CAN_ID 0x48
#define REAR_RIGHT_MOTOR_CAN_ID 0x67
#define DEPOSITION_MOTOR_CAN_ID 0x34
#define EXCAVATION_BELT_MOTOR_CAN_ID 0x68

// I2C Address
#define SLAVE_I2C_ADDRESS 0x24

// Command Constants
// System commands
#define CMD_EMERGENCY_STOP 1
#define CMD_REQUEST_DATA 128
#define CMD_SWITCH_AUTONOMOUS 129

// Locomotion commands
#define CMD_LOCOMOTION_STOP 16
#define CMD_FORWARD_25 32
#define CMD_FORWARD_50 33
#define CMD_FORWARD_75 34
#define CMD_FORWARD_100 35
#define CMD_BACKWARD_25 48
#define CMD_BACKWARD_50 49
#define CMD_BACKWARD_75 50
#define CMD_BACKWARD_100 51
#define CMD_LEFT_25 64
#define CMD_LEFT_50 65
#define CMD_LEFT_75 66
#define CMD_LEFT_100 67
#define CMD_RIGHT_25 80
#define CMD_RIGHT_50 81
#define CMD_RIGHT_75 82
#define CMD_RIGHT_100 83

// Excavation commands
#define CMD_EXCAVATION_ZERO 96
#define CMD_LOCOMOTION_POSITION 97
#define CMD_EXCAVATION_POSITION 98
#define CMD_BELT_STOP 99
#define CMD_BELT_OUTWARD 100
#define CMD_BELT_INWARD 101
#define CMD_ACME_UP 102
#define CMD_ACME_DOWN 103

// Deposition commands
#define CMD_DEPOSITION_ROTATE_COLLECTION 112
#define CMD_DEPOSITION_ROTATE_DUMPING 113
#define CMD_DEPOSITION_ROTATE_STOP 114

// Position states
#define POSITION_UNKNOWN 0
#define POSITION_LOCOMOTION 1
#define POSITION_EXCAVATION 2

// System calibration constants
const float LOCOMOTION_DUTY_CYCLE = 0.33;
const float DEPOSITION_DUTY_CYCLE = 0.2;
const float EXCAVATION_DUTY_CYCLE = 0.42;

const int EXCAVATION_UP_PWM = 1460;
const int EXCAVATION_DOWN_PWM = 1522;

// String pot constants
const float VOLTAGE_LENGTH_CONVERT = 27;
const float VOLTAGE_LENGTH_CONVERT_Y_INTERCEPT = 0.719;
float LOCOMOTION_LENGTH_THRESHOLD = 29; // approximate
float EXCAVATION_LENGTH_THRESHOLD = 17;

// Load cell calibration
float calibration_factor1 = -102;
float calibration_factor2 = 105;
float calibration_factor3 = -102;
float calibration_factor4 = 111;

// Encoder properties
const float COUNTS_PER_REVOLUTION = 8192.0;
const float DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REVOLUTION;

/************************************************************
 * GLOBAL VARIABLES
 ************************************************************/

// Communication interfaces
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
Servo excavationSystemPWM;

// System state
int currentPosition = POSITION_UNKNOWN;
bool isAutonomousMode = false;
int last_command;
bool currentRelayStatus = false;
bool eStopEngaged = true;

// Sensor data
float string_length = 0;
float weight = 0.0;
volatile float lastWeight = 0;
unsigned long lastWeightUpdate = 0;

// Add these globals for non-blocking weight readings
unsigned long lastScaleIndex = 0;  // Which scale to read next (0-3)
unsigned long nextScaleReadTime = 0;
float scaleReadings[4] = {0, 0, 0, 0};

//motor state for refreshes
float activeBeltSpeed = 0.0;
float activeDepositionSpeed = 0.0;
unsigned long lastCommandRefreshTime = 0;
const unsigned long COMMAND_REFRESH_INTERVAL = 500; // Refresh commands every 500ms

// I2C communication
uint8_t dataPacket[3];
volatile bool newDataReady = false;
byte receivedCommand = 0;
bool commandReceived = false;
unsigned long lastCommandTime = 0;
unsigned int commandCount = 0;
volatile bool positionCommandInterrupted = false;


// Load cell objects
HX711 scale1, scale2, scale3, scale4;

// Encoder counts - volatile because they're modified in interrupts
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;

// Previous counts - for change detection
long prev_count1 = 0;
long prev_count2 = 0;
long prev_count3 = 0;
long prev_count4 = 0;

// Track angles
float angle1 = 0.0;
float angle2 = 0.0;
float angle3 = 0.0; 
float angle4 = 0.0;

// Currently active encoder (1-4) for sending to Pi
uint8_t activeEncoder = 2;

/************************************************************
 * FUNCTION DECLARATIONS
 ************************************************************/

// System initialization functions
void init_pwm_motors();
void start_scales();

// Command processing
void manual_receive();
void manual_interpret(int command_decimal);
void request_data();
void update_data();
void switch_autonomous();

// Locomotion functions
void emergency_stop();
void locomotion_stop();
void go_forward(float speed_fac);
void backward(float speed_fac);
void turn_left(float speed_fac);
void turn_right(float speed_fac);

// Excavation functions
void excavation_stop();
void excavation_up();
void excavation_down();
void zero_excavation();
bool locomotion_position();
bool excavation_position();

// Belt functions
void belt_stop();
void belt_outward();
void belt_inward();

// Deposition functions
void rotate_collection();
void rotate_dumping();
void stop_rotating();

// Sensor functions
float get_length();
float get_weight();
float countsToAngle(long counts);
void setActiveEncoder(uint8_t encoderNum);

// Communication functions
int sendMotorCommand(uint32_t canId, float speedPercent);
CAN_message_t craftMessage(int typeofmsg, float val, uint8_t id);
int writeCANMessage(CAN_message_t to_send);

// ISR routines for encoders
void isr1A();
void isr1B();
void isr2A();
void isr2B();
void isr3A();
void isr3B();
void isr4A();
void isr4B();

/************************************************************
 * ISR IMPLEMENTATIONS
 ************************************************************/

// Modified ISR routines for encoders 1, 2, and 4 (inverting direction)
void isr1A() { count1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : +1; } // Swapped +/- signs
void isr1B() { count1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? -1 : +1; } // Swapped +/- signs

void isr2A() { count2 += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : +1; } // Swapped +/- signs
void isr2B() { count2 += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? -1 : +1; } // Swapped +/- signs

void isr3A() { count3 += (digitalRead(ENC3_A) == digitalRead(ENC3_B)) ? +1 : -1; }
void isr3B() { count3 += (digitalRead(ENC3_A) != digitalRead(ENC3_B)) ? +1 : -1; }

void isr4A() { count4 += (digitalRead(ENC4_A) == digitalRead(ENC4_B)) ? -1 : +1; } // Swapped +/- signs
void isr4B() { count4 += (digitalRead(ENC4_A) != digitalRead(ENC4_B)) ? -1 : +1; } // Swapped +/- signs

/************************************************************
 * MAIN ARDUINO FUNCTIONS
 ************************************************************/

void setup() {
  // Initialize data packet
  dataPacket[0] = POSITION_UNKNOWN;
  dataPacket[1] = 0;
  dataPacket[2] = 0;

  // Initialize CAN bus
  can1.begin();
  can1.setBaudRate(500000);

  // Initialize I2C
  Wire.begin(SLAVE_I2C_ADDRESS);
  Wire.onRequest(request_data);
  Wire.setClock(400000);
  
  // Initialize hardware
  init_pwm_motors();
  start_scales();
  Serial.begin(115200);

  pinMode(RELAY_PIN, INPUT_PULLUP); // pin for E-stop

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

  currentRelayStatus = digitalRead(RELAY_PIN);
  
  Serial.println("Encoders initialized");
  Serial.println("CAN Control Initialized");
}

void loop() {

 /* int raw = digitalRead(RELAY_PIN);
  Serial.print("RELAY PIN = ");
  Serial.println(raw);*/
  // Check relay board status (LOW when relay is OFF/E-stop engaged)
  currentRelayStatus = digitalRead(RELAY_PIN);
  
  // E-stop activated
  if (currentRelayStatus == LOW) {
    // Only perform emergency stop once when state changes
    if (!eStopEngaged) {
      emergency_stop();
      eStopEngaged = true;
      
      // Clear any interrupted position commands for safety
      positionCommandInterrupted = false;
      commandReceived = false;
      
      Serial.println("E-STOP ACTIVATED - Motors stopped");
    }
    
    // Still process commands during E-stop but only certain ones
    if (Wire.available()) {
      char c = Wire.read();
      if (int(c) == CMD_REQUEST_DATA) {
        request_data();  // Always allow data requests
      }
    }
  } 
  // Power restored
  else {
    // Only announce restoration once
    if (eStopEngaged) {
      eStopEngaged = false;
      Serial.println("Power restored - Ready to accept commands");
    }
    
    // Normal operation
    if (positionCommandInterrupted && commandReceived) {
      manual_interpret(int(receivedCommand));
      positionCommandInterrupted = false;
      commandReceived = false;
    }
    manual_receive();
  }

  unsigned long currentMillis = millis();
  
  // Non-blocking weight reading
  if (currentMillis >= nextScaleReadTime && !Wire.available()) {
    lastWeight = get_weight();
    nextScaleReadTime = currentMillis + 500; // Only read every 500ms
  }
  
  // Periodic command refresh
  if (currentMillis - lastCommandRefreshTime >= COMMAND_REFRESH_INTERVAL) {
    // Refresh commands only if not in E-stop
    if (!eStopEngaged) {
      if (activeBeltSpeed != 0.0) {
        writeCANMessage(craftMessage(0, activeBeltSpeed, EXCAVATION_BELT_MOTOR_CAN_ID));
      }
      
      if (activeDepositionSpeed != 0.0) {
        writeCANMessage(craftMessage(0, activeDepositionSpeed, DEPOSITION_MOTOR_CAN_ID));
      }
    }
    
    lastCommandRefreshTime = currentMillis;
  }
  
  // Small yield to keep things responsive
  yield();
}
/************************************************************
 * INITIALIZATION FUNCTIONS
 ************************************************************/

void init_pwm_motors() {
  excavationSystemPWM.attach(EXCAVATION_SYSTEM_PWM_PIN);
  excavationSystemPWM.writeMicroseconds(1500); // Neutral
}

void start_scales() {
  scale1.begin(DOUT1, CLK1);
  scale2.begin(DOUT2, CLK2);
  scale3.begin(DOUT3, CLK3);
  scale4.begin(DOUT4, CLK4);
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  scale1.tare();
  scale2.tare();
  scale3.tare();
  scale4.tare();
}

/************************************************************
 * COMMAND PROCESSING FUNCTIONS
 ************************************************************/

void manual_receive() {
  while (Wire.available()) {
    char c = Wire.read();
    Serial.println(int(c));
    manual_interpret(int(c));
    if (int(c) == 1) {
      last_command = 1000;
    } else {
      last_command = int(c);
    }
  }
}

void manual_interpret(int command_decimal) {
  if (command_decimal != CMD_REQUEST_DATA) {
    eStopEngaged = false;
  }
  switch (command_decimal) {
    // System commands
    case CMD_EMERGENCY_STOP:
      emergency_stop();
      break;
    case CMD_REQUEST_DATA:
      request_data();
      break;
    case CMD_SWITCH_AUTONOMOUS:
      switch_autonomous();
      break;
    
    // Locomotion commands
    case CMD_LOCOMOTION_STOP:
      locomotion_stop();
      break;
    case CMD_FORWARD_25:
      go_forward(0.25);
      break;
    case CMD_FORWARD_50:
      go_forward(0.50);
      break;
    case CMD_FORWARD_75:
      go_forward(0.75);
      break;
    case CMD_FORWARD_100:
      go_forward(1.00);
      break;
    case CMD_BACKWARD_25:
      backward(0.25);
      break;
    case CMD_BACKWARD_50:
      backward(0.50);
      break;
    case CMD_BACKWARD_75:
      backward(0.75);
      break;
    case CMD_BACKWARD_100:
      backward(1.00);
      break;
    case CMD_LEFT_25:
      turn_left(0.25);
      break;
    case CMD_LEFT_50:
      turn_left(0.50);
      break;
    case CMD_LEFT_75:
      turn_left(0.75);
      break;
    case CMD_LEFT_100:
      turn_left(1.00);
      break;
    case CMD_RIGHT_25:
      turn_right(0.25);
      break;
    case CMD_RIGHT_50:
      turn_right(0.50);
      break;
    case CMD_RIGHT_75:
      turn_right(0.75);
      break;
    case CMD_RIGHT_100:
      turn_right(1.00);
      break;
      
    // Excavation system commands
    case CMD_EXCAVATION_ZERO:
      //zero_excavation(); //not used in this implementation i guess
      excavation_stop(); //might actually stay like this cause we have no stop command for excavation otherwise
      break;
    case CMD_LOCOMOTION_POSITION:
      //move_excavation_to_position(POSITION_LOCOMOTION, LOCOMOTION_LENGTH_THRESHOLD, move_up=true);
      
      locomotion_position();

       //- if you want to use this, attach the string pot 
      //to bottom excavation purple thing and test it to find threshold
      //be careful this hasnt been tested much
      //sending zero_excavation command will stop the motor
      //or have your hand ready be the e-stop
      

      break;
    case CMD_EXCAVATION_POSITION:
      //move_excavation_to_position(POSITION_EXCAVATION, EXCAVATION_LENGTH_THRESHOLD, move_up=false);

      excavation_position(); //- if you want to use this, attach the string pot 

      break;
    case CMD_ACME_UP:
      excavation_up();
      break;
    case CMD_ACME_DOWN:
      excavation_down();
      break;
      
    // excavation belt commands
    case CMD_BELT_STOP:
      belt_stop();
      break;
    case CMD_BELT_OUTWARD:
      belt_outward();
      break;
    case CMD_BELT_INWARD:
      belt_inward();
      break;
      
    // Deposition commands
    case CMD_DEPOSITION_ROTATE_COLLECTION:
      rotate_collection();
      break;
    case CMD_DEPOSITION_ROTATE_DUMPING:
      rotate_dumping();
      break;
    case CMD_DEPOSITION_ROTATE_STOP:
      stop_rotating();
      break;
      
    default:
      break;
  }
}

void request_data() {
  update_data(); // Make sure data is fresh
  Wire.write(dataPacket, sizeof(dataPacket)); // Send the data
  Serial.print("Sent to Pi - Weight: ");
  Serial.print(dataPacket[0]);
  Serial.print(", Position: ");
  Serial.print(dataPacket[1]);
  Serial.print(", Encoder (");
  Serial.print(activeEncoder);
  Serial.print("): ");
  Serial.println(dataPacket[2]);
}

void update_data() {
  string_length = get_length();

  //float weight = 10.0; //load cells no work
  //float weight = get_weight(); // Uncomment if you want to read weight directly
  //float weight = lastWeight;
  float weight = lastWeight/20; // Use cached weight instead
  
  // Calculate all encoder angles (done atomically)
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
  
  // Select the active encoder angle
  float activeAngle = 0;
  switch (activeEncoder) {
    case 1: activeAngle = angle1; break;
    case 2: activeAngle = angle2; break;
    case 3: activeAngle = angle3; break;
    case 4: activeAngle = angle4; break;
  }
  
  // Convert to integers for the data packet
  uint8_t weight_int = (uint8_t)constrain(weight, 0, 255);
  //uint8_t weight_int = 10; // Placeholder till we figure out the load cells
  uint8_t string_length_int = (uint8_t)constrain(string_length, 0, 255); // Convert string length to int
  uint8_t angle_int = (uint8_t)(activeAngle * 255.0 / 360.0); // Scale angle to 0-255
  
  // Update the data packet
  // Data order: weight, string_length (instead of position), encoder
  dataPacket[0] = weight_int;
  dataPacket[1] = string_length_int;  // Send string_length as an integer
  dataPacket[2] = angle_int;
  
  // Update the Serial print to show what we're sending
  /*Serial.print("Sent to Pi - Weight: ");
  Serial.print(weight_int);
  Serial.print(", String Length: ");
  Serial.print(string_length);
  Serial.print(" cm (");
  Serial.print(string_length_int);
  Serial.print("), Encoder (");
  Serial.print(activeEncoder);
  Serial.print("): ");
  Serial.println(angle_int);*/
  
  newDataReady = true;
}

void switch_autonomous() {
  isAutonomousMode = true;
  Serial.println("Switched to Autonomous Mode");
}

/************************************************************
 * LOCOMOTION CONTROL FUNCTIONS
 ************************************************************/

void emergency_stop() {
  // Order of operations: 1. Locomotion 2. Excavation 3. Belt
  locomotion_stop();
  excavation_stop();
  belt_stop();
  stop_rotating();
  Serial.println("Emergency Stop Activated");
}

void locomotion_stop() {
  // Consistent order: Front Left, Front Right, Rear Left, Rear Right
  writeCANMessage(craftMessage(0, 0, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, REAR_RIGHT_MOTOR_CAN_ID));
  Serial.println("Locomotion Stopped");
}

void go_forward(float speed_fac) {
  float limited_speed = speed_fac * LOCOMOTION_DUTY_CYCLE;
  // Consistent order: Front Left, Front Right, Rear Left, Rear Right
  writeCANMessage(craftMessage(0, limited_speed, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, REAR_RIGHT_MOTOR_CAN_ID));
  Serial.println("Moving Forward");
}

void backward(float speed_fac) {
  float limited_speed = speed_fac * LOCOMOTION_DUTY_CYCLE;
  // Consistent order: Front Left, Front Right, Rear Left, Rear Right
  writeCANMessage(craftMessage(0, -limited_speed, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, REAR_RIGHT_MOTOR_CAN_ID));
  Serial.println("Moving Backward");
}

void turn_right(float speed_fac) {
  float limited_speed = speed_fac * LOCOMOTION_DUTY_CYCLE;
  // Consistent order: Front Left, Front Right, Rear Left, Rear Right
  writeCANMessage(craftMessage(0, -limited_speed, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, REAR_RIGHT_MOTOR_CAN_ID));
  Serial.println("Turning Left");
}

void turn_left(float speed_fac) {
  float limited_speed = speed_fac * LOCOMOTION_DUTY_CYCLE;
  // Consistent order: Front Left, Front Right, Rear Left, Rear Right
  writeCANMessage(craftMessage(0, limited_speed, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, REAR_RIGHT_MOTOR_CAN_ID));
  Serial.println("Turning Right");
}

/************************************************************
 * EXCAVATION CONTROL FUNCTIONS
 ************************************************************/

void excavation_stop() {
  excavationSystemPWM.writeMicroseconds(1500);  // Neutral
  Serial.println("Excavation Stopped");
}

void excavation_up() {
  excavationSystemPWM.writeMicroseconds(EXCAVATION_UP_PWM);  // Up
  Serial.println("Excavation Moving Up");
}

void excavation_down() {
  excavationSystemPWM.writeMicroseconds(EXCAVATION_DOWN_PWM);  // Down
  Serial.println("Excavation Moving Down");
}

void zero_excavation() {
  currentPosition = POSITION_LOCOMOTION;
  LOCOMOTION_LENGTH_THRESHOLD = get_length();
  Serial.println("Excavation Zeroed");
}

bool locomotion_position() {
  if (currentPosition != POSITION_LOCOMOTION) {
    // Initialize variables for tracking movement
    float prev_length = get_length();

    if (string_length >= LOCOMOTION_LENGTH_THRESHOLD) {
      Serial.println("Already at or beyond locomotion position threshold");
      excavation_stop(); // Ensure motors are stopped
      currentPosition = POSITION_LOCOMOTION;
      return true;
    }

    unsigned int stuck_count = 0;
    unsigned long last_check_time = 0;
    unsigned long last_motor_refresh_time = 0;
    unsigned long movement_start_time = millis();
    
    // Start moving up
    excavation_up();
    
    // Non-blocking loop with millis()
    while (string_length < LOCOMOTION_LENGTH_THRESHOLD) {
      unsigned long current_time = millis();
      

      
      // CHECK FOR E-STOP ACTIVATION
      if (digitalRead(RELAY_PIN) == LOW) {
        eStopEngaged = true;
        emergency_stop();
        Serial.println("E-STOP ACTIVATED during locomotion positioning - Operation aborted");
        currentPosition = POSITION_UNKNOWN;
        return false;
      }
      
      
      // Check for incoming commands (no blocking)
      if (Wire.available()) {
        char c = Wire.read();
        if (int(c) == CMD_EXCAVATION_ZERO) {
          excavation_stop();
          currentPosition = POSITION_UNKNOWN;
          return false;
        }
        else if (int(c) == CMD_EXCAVATION_POSITION) {
          excavation_stop();
          currentPosition = POSITION_UNKNOWN;
          Serial.println("Locomotion positioning interrupted by excavation position command");
          positionCommandInterrupted = true;
          // Store this command to be processed after we exit
          receivedCommand = c;
          commandReceived = true;
          return false;
        }
        manual_interpret(int(c));
      }
      
      // Check position and movement only at intervals (every 150ms)
      if (current_time - last_check_time >= 150) {
        string_length = get_length();
        
        // Check if movement is too small, using a counter for reliability
        /*
        if (abs(string_length - prev_length) < 0.15) {
          stuck_count++;
          if (stuck_count >= 3) {  // Require multiple readings showing minimal movement
            Serial.println("Excavation appears to be stuck - stopping");
            excavation_stop();
            currentPosition = POSITION_UNKNOWN;
            return false;
          }
        } else {
          // Reset stuck counter if we see movement
          stuck_count = 0;
        }*/
        
        prev_length = string_length;
        last_check_time = current_time;
      }

      if (current_time - last_motor_refresh_time >= COMMAND_REFRESH_INTERVAL) {
        // Refresh excavation belt command if active
        if (!eStopEngaged && activeBeltSpeed != 0.0) {
          writeCANMessage(craftMessage(0, activeBeltSpeed, EXCAVATION_BELT_MOTOR_CAN_ID));
        }
        
        // Refresh deposition motor command if active
        if (!eStopEngaged && activeDepositionSpeed != 0.0) {
          writeCANMessage(craftMessage(0, activeDepositionSpeed, DEPOSITION_MOTOR_CAN_ID));
        }
        
        last_motor_refresh_time = current_time;
      }
      
      // Safety timeout
      /*
      if (current_time - movement_start_time > 10000) {
        Serial.println("Excavation timed out - stopping");
        excavation_stop();
        currentPosition = POSITION_UNKNOWN;
        return false;
      }*/
      
      // Small yield to allow other operations to process
      // Much better than delay() which blocks everything
      yield();
    }
    
    excavation_stop();
    currentPosition = POSITION_LOCOMOTION;
    Serial.println("Successfully reached locomotion position");
    return true;
  }
  
  return false;  // Already in locomotion position
}

bool excavation_position() {
  if (currentPosition != POSITION_EXCAVATION) {
    // Initialize variables for tracking movement
    float prev_length = get_length();

    if (prev_length <= EXCAVATION_LENGTH_THRESHOLD) {
      Serial.println("Already at or beyond excavation position threshold");
      excavation_stop(); // Ensure motors are stopped
      currentPosition = POSITION_EXCAVATION;
      return true;
    }

    unsigned int stuck_count = 0;
    unsigned long last_check_time = 0;
    unsigned long last_motor_refresh_time = 0;
    unsigned long movement_start_time = millis();
    
    // Start moving down
    excavation_down();
    
    // Non-blocking loop with millis()
    while (string_length > EXCAVATION_LENGTH_THRESHOLD) {
      unsigned long current_time = millis();

      
      // CHECK FOR E-STOP ACTIVATION
      if (digitalRead(RELAY_PIN) == LOW) {
        eStopEngaged = true;
        // E-stop activated during operation
        emergency_stop();
        Serial.println("E-STOP ACTIVATED during locomotion positioning - Operation aborted");
        currentPosition = POSITION_UNKNOWN;
        return false;
      }
      
      
      // Check for incoming commands (no blocking)
      if (Wire.available()) {
        char c = Wire.read();
        if (int(c) == CMD_EXCAVATION_ZERO) {
          excavation_stop();
          currentPosition = POSITION_UNKNOWN;
          return false;
        }
        else if (int(c) == CMD_LOCOMOTION_POSITION) {
          excavation_stop();
          currentPosition = POSITION_UNKNOWN;
          Serial.println("excavation positioning interrupted by locotion position command");
          positionCommandInterrupted = true;
          // Store this command to be processed after we exit
          receivedCommand = c;
          commandReceived = true;
          return false;
        }
        manual_interpret(int(c));
      }
      
      // Check position and movement only at intervals (every 150ms)
      if (current_time - last_check_time >= 150) {
        string_length = get_length();
        
        // Check if movement is too small, using a counter for reliability
        /*
        if (abs(string_length - prev_length) < 0.15) {
          stuck_count++;
          if (stuck_count >= 3) {  // Require multiple readings showing minimal movement
            Serial.println("Excavation appears to be stuck - stopping");
            excavation_stop();
            currentPosition = POSITION_UNKNOWN;
            return false;
          }
        } else {
          // Reset stuck counter if we see movement
          stuck_count = 0;
        }*/
        
        prev_length = string_length;
        last_check_time = current_time;
      }

      if (current_time - last_motor_refresh_time >= COMMAND_REFRESH_INTERVAL) {
        // Refresh excavation belt command if active
        if (!eStopEngaged && activeBeltSpeed != 0.0) {
          writeCANMessage(craftMessage(0, activeBeltSpeed, EXCAVATION_BELT_MOTOR_CAN_ID));
        }
        
        // Refresh deposition motor command if active
        if (!eStopEngaged && activeDepositionSpeed != 0.0) {
          writeCANMessage(craftMessage(0, activeDepositionSpeed, DEPOSITION_MOTOR_CAN_ID));
        }
        
        last_motor_refresh_time = current_time;
      }
      
      // Safety timeout
      /*
      if (current_time - movement_start_time > 20000) {
        Serial.println("Excavation timed out - stopping");
        excavation_stop();
        currentPosition = POSITION_UNKNOWN;
        return false;
      }*/
      
      // Small yield to allow other operations to process
      yield();
    }
    
    excavation_stop();
    currentPosition = POSITION_EXCAVATION;
    Serial.println("Successfully reached excavation position");
    return true;
  }
  
  return false;  // Already in excavation position
}

bool move_to_position(int targetPosition, float targetThreshold, bool moveUp) {
  if (currentPosition != targetPosition) {
    // Initialize variables for tracking movement
    float prev_length = get_length();
    unsigned int stuck_count = 0;
    unsigned long last_check_time = 0;
    unsigned long movement_start_time = millis();
    unsigned long last_motor_refresh_time = 0;
    
    // Start motion in the appropriate direction
    if (moveUp) {
      excavation_up();
    } else {
      excavation_down();
    }
    
    // Non-blocking loop with millis()
    while (true) {
      // Update current length
      string_length = get_length();
      unsigned long current_time = millis();
      
      // Check if we've reached the target position (only once!)
      bool shouldContinue;
      if (moveUp) {
        shouldContinue = (string_length < targetThreshold);
      } else {
        shouldContinue = (string_length > targetThreshold);
      }
      
      if (!shouldContinue) break; // Exit loop if we've reached the target
      
      // Maintain other active motor commands during positioning
      if (current_time - last_motor_refresh_time >= COMMAND_REFRESH_INTERVAL) {
        // Refresh excavation belt command if active
        if (activeBeltSpeed != 0.0) {
          writeCANMessage(craftMessage(0, activeBeltSpeed, EXCAVATION_BELT_MOTOR_CAN_ID));
        }
        
        // Refresh deposition motor command if active
        if (activeDepositionSpeed != 0.0) {
          writeCANMessage(craftMessage(0, activeDepositionSpeed, DEPOSITION_MOTOR_CAN_ID));
        }
        
        last_motor_refresh_time = current_time;
      }
      
      // CHECK FOR E-STOP ACTIVATION
      if (digitalRead(RELAY_PIN) == LOW) {
        eStopEngaged = true;
        // E-stop activated during operation
        emergency_stop();
        Serial.println("E-STOP ACTIVATED during position movement - Operation aborted");
        
        return false;
      }
      
      
      // Check for incoming commands (no blocking)
      if (Wire.available()) {
        char c = Wire.read();
        if (int(c) == CMD_EXCAVATION_ZERO) {
          excavation_stop();
          return false;
        }
        manual_interpret(int(c));
      }
      
      // Check position and movement only at intervals (every 150ms)
      if (current_time - last_check_time >= 150) {
        // Check if movement is too small, using a counter for reliability
        if (abs(string_length - prev_length) < 0.03) {
          stuck_count++;
          if (stuck_count >= 3) {  // Require multiple readings showing minimal movement
            Serial.println("Excavation appears to be stuck - stopping");
            excavation_stop();
            return false;
          }
        } else {
          // Reset stuck counter if we see movement
          stuck_count = 0;
        }
        
        prev_length = string_length;
        last_check_time = current_time;
      }
      
      // Safety timeout
      
      if (current_time - movement_start_time > 25000) {
        Serial.println("Excavation timed out - stopping");
        excavation_stop();
        return false;
      }
      
      // Small yield to allow other operations to process
      yield();
    }
    
    excavation_stop();
    currentPosition = targetPosition;
    
    // Print appropriate success message
    if (targetPosition == POSITION_LOCOMOTION) {
      Serial.println("Successfully reached locomotion position");
    } else {
      Serial.println("Successfully reached excavation position");
    }
    
    return true;
  }
  
  return false;  // Already in target position
}

/************************************************************
 * EXCAVATON BELT CONTROL FUNCTIONS
 ************************************************************/

 void belt_stop() {
  writeCANMessage(craftMessage(0, 0, EXCAVATION_BELT_MOTOR_CAN_ID));
  activeBeltSpeed = 0.0; 
  Serial.println("Excavation belt stopped");
}

void belt_outward() {
  float speed = -EXCAVATION_DUTY_CYCLE;
  writeCANMessage(craftMessage(0, speed, EXCAVATION_BELT_MOTOR_CAN_ID));
  activeBeltSpeed = speed;
  Serial.println("Excavation belt moving outward");
}

void belt_inward() {
  float speed = EXCAVATION_DUTY_CYCLE;
  writeCANMessage(craftMessage(0, speed, EXCAVATION_BELT_MOTOR_CAN_ID));
  activeBeltSpeed = speed;
  Serial.println("Excavation belt moving inward");
}

/************************************************************
 * DEPOSITION CONTROL FUNCTIONS
 ************************************************************/

 void rotate_collection() {
  float speed = DEPOSITION_DUTY_CYCLE;
  writeCANMessage(craftMessage(0, speed, DEPOSITION_MOTOR_CAN_ID));
  activeDepositionSpeed = speed;  // Update the active speed
  Serial.println("Rotating to Collection Position");
}

void rotate_dumping() {
  float speed = -DEPOSITION_DUTY_CYCLE;
  writeCANMessage(craftMessage(0, speed, DEPOSITION_MOTOR_CAN_ID));
  activeDepositionSpeed = speed;  // Update the active speed
  Serial.println("Rotating to Dumping Position");
}

void stop_rotating() {
  writeCANMessage(craftMessage(0, 0, DEPOSITION_MOTOR_CAN_ID));
  activeDepositionSpeed = 0.0;  // Update the active speed
  Serial.println("Rotation Stopped");
}

/************************************************************
 * SENSOR FUNCTIONS
 ************************************************************/

float get_length() {
  float sensorValue = analogRead(STRING_POT_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  return VOLTAGE_LENGTH_CONVERT * voltage - VOLTAGE_LENGTH_CONVERT_Y_INTERCEPT;
}

float get_weight() {
  return (scale3.get_units(2) + scale4.get_units(2));
}

float countsToAngle(long counts) {
  float angle = fmod(counts * DEGREES_PER_COUNT, 360.0);
  if (angle < 0) angle += 360.0;
  return angle;
}

void setActiveEncoder(uint8_t encoderNum) {
  if (encoderNum >= 1 && encoderNum <= 4) {
    activeEncoder = encoderNum;
  }
}

/************************************************************
 * COMMUNICATION FUNCTIONS
 ************************************************************/

CAN_message_t craftMessage(int typeofmsg, float val, uint8_t id) {
  CAN_message_t new_msg;
  new_msg.flags.extended = 1;
  new_msg.id = (typeofmsg << 8) + id;
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