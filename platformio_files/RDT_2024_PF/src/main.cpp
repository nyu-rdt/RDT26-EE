#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include "HX711.h"
#include <Servo.h>

// Global Variables and Constants
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
Servo excavationBeltPWM;
Servo excavationSystemPWM;

float LOCOMOTION_LENGTH_THRESHOLD = 10;
float EXCAVATION_LENGTH_THRESHOLD = 30;
float string_length = 0;
float weight = 0.0;

HX711 scale1, scale2, scale3, scale4;

float calibration_factor1 = -1000;
float calibration_factor2 = -1000;
float calibration_factor3 = -1000;
float calibration_factor4 = -1000;

byte dataPacket[3];

const float VOLTAGE_LENGTH_CONVERT = 27;
const float VOLTAGE_LENGTH_CONVERT_Y_INTERCEPT = 0.719;

int last_command = 0;

const float duty_cycle = 0.2;
const float deposition_duty_cycle = 0.15;

// Pin Definitions
#define STRING_POT_PIN A1
#define DOUT1 13
#define CLK1 9
#define DOUT2 11
#define CLK2 8
#define DOUT3 7
#define CLK3 6
#define DOUT4 5
#define CLK4 4
#define EXCAVATION_BELT_PWM_PIN 14
#define EXCAVATION_SYSTEM_PWM_PIN 3

// CAN IDs
#define FRONT_LEFT_MOTOR_CAN_ID 0x67
#define FRONT_RIGHT_MOTOR_CAN_ID 0x78
#define REAR_LEFT_MOTOR_CAN_ID 0x16
#define REAR_RIGHT_MOTOR_CAN_ID 0x48
#define DEPOSITION_MOTOR_CAN_ID 0x42

// I2C Address
#define SLAVE_I2C_ADDRESS 0x24

// Command Constants
#define CMD_EMERGENCY_STOP 1
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
#define CMD_EXCAVATION_ZERO 96
#define CMD_EXCAVATION_LOCOMOTION_POS 97
#define CMD_EXCAVATION_POSITION 98
#define CMD_BELT_STOP 99
#define CMD_BELT_OUTWARD 100
#define CMD_BELT_INWARD 101
#define CMD_DEPOSITION_ROTATE_COLLECTION 112
#define CMD_DEPOSITION_ROTATE_DUMPING 113
#define CMD_DEPOSITION_ROTATE_STOP 114
#define CMD_REQUEST_DATA 128
#define CMD_SWITCH_AUTONOMOUS 129

#define POSITION_UNKNOWN 0
#define POSITION_LOCOMOTION 1
#define POSITION_EXCAVATION 2

int currentPosition = POSITION_UNKNOWN;
bool isAutonomousMode = false;

// Function Definitions
void manual_receive(int numBytes);
void manual_interpret(int command_decimal);
void init_pwm_motors();
void emergency_stop();
void locomotion_stop();
void turn_right(float speed_fac);
void turn_left(float speed_fac);
void go_forward(float speed_fac);
void backward(float speed_fac);
void belt_stop();
void belt_outward();
void belt_inward();
void excavation_stop();
void excavation_up();
void excavation_down();
void rotate_collection();
void rotate_dumping();
void stop_rotating();
void zero_excavation();
bool locomotion_position();
bool excavation_position();
void request_data();
void update_data();
void switch_autonomous();
int sendMotorCommand(uint32_t canId, float speedPercent);
float get_length();
void start_scales();
float get_weight();
CAN_message_t craftMessage(int typeofmsg, float val, uint8_t id = 78);
int writeCANMessage(CAN_message_t to_send);

// Function Implementations
void setup() {
  dataPacket[0] = POSITION_UNKNOWN;
  dataPacket[1] = 0;
  dataPacket[2] = 0;

  Wire.onReceive(manual_receive);
  Wire.onRequest(request_data);

  can1.begin();
  can1.setBaudRate(500000);

  Wire.begin(SLAVE_I2C_ADDRESS);
  init_pwm_motors();
  start_scales();
  Serial.begin(115200);

  Serial.println("CAN Control Initialized");
}

void loop() {
  if(last_command){Serial.println(last_command);}
}

void manual_receive(int numBytes) {
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
  switch (command_decimal) {
    case CMD_EMERGENCY_STOP:
      emergency_stop();
      break;
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
    case CMD_EXCAVATION_ZERO:
      zero_excavation();
      break;
    case CMD_EXCAVATION_LOCOMOTION_POS:
      locomotion_position();
      break;
    case CMD_EXCAVATION_POSITION:
      excavation_position();
      break;
    case CMD_BELT_STOP:
      belt_stop();
      break;
    case CMD_BELT_OUTWARD:
      belt_outward();
      break;
    case CMD_BELT_INWARD:
      belt_inward();
      break;
    case CMD_DEPOSITION_ROTATE_COLLECTION:
      rotate_collection();
      break;
    case CMD_DEPOSITION_ROTATE_DUMPING:
      rotate_dumping();
      break;
    case CMD_DEPOSITION_ROTATE_STOP:
      stop_rotating();
      break;
    case CMD_REQUEST_DATA:
      request_data();
      break;
    case CMD_SWITCH_AUTONOMOUS:
      switch_autonomous();
      break;
    default:
      break;
  }
}

void init_pwm_motors() {
  excavationBeltPWM.attach(EXCAVATION_BELT_PWM_PIN);
  excavationSystemPWM.attach(EXCAVATION_SYSTEM_PWM_PIN);

  excavationBeltPWM.writeMicroseconds(1500);   // Neutral
  excavationSystemPWM.writeMicroseconds(1500); // Neutral
}

void emergency_stop() {
  locomotion_stop();
  excavation_stop();
  belt_stop();
  Serial.println("Emergency Stop Activated");
}

void locomotion_stop() {
  writeCANMessage(craftMessage(0, 0, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, 0, REAR_RIGHT_MOTOR_CAN_ID));
  Serial.println("Locomotion Stopped");
}

void turn_right(float speed_fac) {
  float limited_speed = speed_fac * duty_cycle;
  writeCANMessage(craftMessage(0, limited_speed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, REAR_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, FRONT_LEFT_MOTOR_CAN_ID));
  Serial.println("Turning Right");
}

void turn_left(float speed_fac) {
  float limited_speed = speed_fac * duty_cycle;
  writeCANMessage(craftMessage(0, -limited_speed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, REAR_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, REAR_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, FRONT_LEFT_MOTOR_CAN_ID));
  Serial.println("Turning Left");
}

void go_forward(float speed_fac) {
  float limited_speed = speed_fac * duty_cycle;
  writeCANMessage(craftMessage(0, -limited_speed, REAR_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, REAR_LEFT_MOTOR_CAN_ID));
  Serial.println("Moving Forward");
}

void backward(float speed_fac) {
  float limited_speed = speed_fac * duty_cycle;
  writeCANMessage(craftMessage(0, limited_speed, REAR_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, FRONT_RIGHT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, limited_speed, FRONT_LEFT_MOTOR_CAN_ID));
  writeCANMessage(craftMessage(0, -limited_speed, REAR_LEFT_MOTOR_CAN_ID));
  Serial.println("Moving Backward");
}

void belt_stop() {
  excavationBeltPWM.writeMicroseconds(1500);  // Neutral
  Serial.println("Belt Stopped");
}

void belt_outward() {
  excavationBeltPWM.writeMicroseconds(1450);  // Reverse
  Serial.println("Belt Moving Outward");
}

void belt_inward() {
  excavationBeltPWM.writeMicroseconds(1550);  // Forward
  Serial.println("Belt Moving Inward");
}

void excavation_stop() {
  excavationSystemPWM.writeMicroseconds(1500);  // Neutral
  Serial.println("Excavation Stopped");
}

void excavation_up() {
  excavationSystemPWM.writeMicroseconds(1550);  // Up
  Serial.println("Excavation Moving Up");
}

void excavation_down() {
  excavationSystemPWM.writeMicroseconds(1450);  // Down
  Serial.println("Excavation Moving Down");
}

void rotate_collection() {
  writeCANMessage(craftMessage(0, deposition_duty_cycle, DEPOSITION_MOTOR_CAN_ID));
  Serial.println("Rotating to Collection Position");
}

void rotate_dumping() {
  writeCANMessage(craftMessage(0, -deposition_duty_cycle, DEPOSITION_MOTOR_CAN_ID));
  Serial.println("Rotating to Dumping Position");
}

void stop_rotating() {
  writeCANMessage(craftMessage(0, 0, DEPOSITION_MOTOR_CAN_ID));
  Serial.println("Rotation Stopped");
}

void zero_excavation() {
  currentPosition = POSITION_LOCOMOTION;
  LOCOMOTION_LENGTH_THRESHOLD = get_length();
  Serial.println("Excavation Zeroed");
}

bool locomotion_position() {
  if (currentPosition != POSITION_LOCOMOTION) {
    excavation_down();
    unsigned long startTime = millis();
    while (string_length > LOCOMOTION_LENGTH_THRESHOLD) {
      string_length = get_length();
      if (millis() - startTime > 10000) {
        excavation_stop();
        return false;
      }
      delay(50);
    }
    excavation_stop();
    currentPosition = POSITION_LOCOMOTION;
    return true;
  }
  return true;
}

bool excavation_position() {
  if (currentPosition != POSITION_EXCAVATION) {
    excavation_up();
    unsigned long startTime = millis();
    while (string_length < EXCAVATION_LENGTH_THRESHOLD) {
      string_length = get_length();
      if (millis() - startTime > 10000) {
        excavation_stop();
        return false;
      }
      delay(50);
    }
    excavation_stop();
    currentPosition = POSITION_EXCAVATION;
    return true;
  }
  return true;
}

void request_data() {
  update_data();
  Wire.write(dataPacket, 3);
}

void update_data() {
  string_length = get_length();
  weight = get_weight();
  dataPacket[0] = currentPosition;
  dataPacket[1] = (int(string_length)) >> 8;
  dataPacket[2] = (int(weight)) & 0xFF;
}

void switch_autonomous() {
  isAutonomousMode = true;
  Serial.println("Switched to Autonomous Mode");
}

float get_length() {
  int sensorValue = analogRead(STRING_POT_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);
  return VOLTAGE_LENGTH_CONVERT * voltage - VOLTAGE_LENGTH_CONVERT_Y_INTERCEPT;
}

float get_weight() {
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  return (scale1.get_units(10) + scale2.get_units(10) + scale3.get_units(10) + scale4.get_units(10)) / 4.0;
}

void start_scales() {
  scale1.begin(DOUT1, CLK1);
  scale2.begin(DOUT2, CLK2);
  scale3.begin(DOUT3, CLK3);
  scale4.begin(DOUT4, CLK4);
  scale1.set_scale();
  scale2.set_scale();
  scale3.set_scale();
  scale4.set_scale();
  scale1.tare();
  scale2.tare();
  scale3.tare();
  scale4.tare();
}

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