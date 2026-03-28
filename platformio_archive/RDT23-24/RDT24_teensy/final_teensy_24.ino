#include <FlexCAN_T4.h>
#include <Wire.h>
#include <Servo.h>
#include <Encoder.h>

struct i2c_data{
  byte software_command;
  byte value;
};

struct range{
  int low;
  int high;
};

int last_command;

//these values are subject to change, with the low range of values 
range locomotion_pivot{0,12};
range construction_pivot{195,205};
range excavation_pivot{245,255};

bool manual_control = true;
bool autonomous_control = false;
bool control_mode = manual_control;

bool dead = false;

unsigned int angle;
unsigned int old_angle; 
unsigned int current_angle; 

unsigned int old_right_bin_angle;
unsigned int right_bin_angle;
unsigned int current_right_bin_angle;

unsigned int old_left_bin_angle;
unsigned int left_bin_angle;
unsigned int current_left_bin_angle;


int vertical_pivot_angle;
//global variables for depo bin movement
bool UP = true;
bool DOWN = false;
bool direction = UP;

//global position flags for each state
bool locomotion_position_flag = true;
bool excavation_position_flag = false;
bool construction_position_flag = false;

bool dont_move_hinge_flag = false; 

//global variables for absolute left bin position
volatile long new_start_time_left_bin = 0;
volatile long new_end_time_left_bin = 0;
int pulse_width_left_bin = 0;

//global variables for absolute right bin position
volatile long new_start_time_right_bin = 0;
volatile long new_end_time_right_bin = 0;
int pulse_width_right_bin = 0;

//global variables for absolute pivot position
volatile long new_start_time = 0;
volatile long new_end_time = 0;
int pulse_width = 0;

int chassis_pivot_angle;

//this is the value needed for reducing the pulse width out of 1055 so that the initial bin values are 0, and increases 
//afterward. 
int bin_pulse_offset = 894;
int pivot_pulse_offset = 728;

//default starting position angle of pivot
int locomotion_pivot_angle; 
int locomotion_left_bin_angle = 0;
int locomotion_right_bin_angle = 0;

//pre-determined construction angle
int construction_pivot_angle;
int construction_left_bin_angle;
int construction_right_bin_angle;

//change this after tetsing
int excavation_pivot_angle = construction_pivot_angle - 20;//some offset

//maximum angle difference in 360 degree
#define max_angle_difference 3

//pin for the ABS wire of the encoder
#define pivot_ABS 21
#define left_bin_encoder 16 
#define right_bin_encoder 15 

#define hinge_motor_pin 32

//pin number for left and right bin actuator
#define left_bin_pin 6
#define right_bin_pin 9

//note the back left motor is oriented backwards, so write negative to go forward
//note the back right motors is oriented backwards, so write negative to go forward
#define back_left_motor 0x67
#define front_left_motor 0x34
#define front_right_motor 0x78
#define back_right_motor 0x68
#define pivot_motor 0x48
#define conveyor_motor 0x16

//teensy address definition
#define teensy_address 0x24

//hex for autonomous control command 
#define autonomous_left_motor_forward 0x02
#define autonomous_left_motor_backward 0x03
#define autonomous_right_motor_forward 0x04
#define autonomous_right_motor_backward 0x05
#define autonomous_depo_bin_stop 0x06

#define autonomous_depo_bin_action 0x07
#define autonomous_pivot_stop 0x08
#define autonomous_pivot_out 0x09
#define autonomous_pivot_in 0x0A

#define autonomous_belt_stop 0x0B
#define autonomous_belt_collect 0x0C
#define autonomous_belt_dump 0x0D

#define autonomous_positioning 0x0E
#define autonomous_other_actions 0x0F

#define manual_left_25 0x40;
#define manual_left_50 0x41;
#define manual_left_75 0x42;
#define manual_left_100 0x43;

#define manual_right_25 0x50;
#define manual_right_50 0x51;
#define manual_right_75 0x52;
#define manual_right_100 0x53;

#define zero_pivot_command 0x60;

#define belt_pivot_offset 0;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
// FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;

// create servo object to control the left and right linear actuator
Servo Depo_left;
Servo Depo_right;
Servo hinge;

CAN_message_t craftMessage(int typeofmsg,float val, uint8_t id = 67){

  CAN_message_t new_msg;

  new_msg.flags.extended = 1;

  switch(typeofmsg){
    case 0: //set duty cycle
    {
      new_msg.id = (0x00<<8) + id;
      new_msg.len = 4;

      int32_t duty_bytes = (int32_t)(val*1000);
      // Serial.println(duty_bytes,HEX);
      // Serial.println(duty_bytes);
      // uint32_t msg_bytes[4];

      // new_msg.buf = duty_bytes;
      new_msg.buf[0] = (duty_bytes & 0xff000000) >> 24;
      new_msg.buf[1] = (duty_bytes & 0x00ff0000) >> 16;
      new_msg.buf[2] = (duty_bytes & 0x0000ff00) >>  8;
      new_msg.buf[3] = (duty_bytes & 0x000000ff)      ;

      // return new_msg;

      break;
    }
    case 1: //set current
    {
      new_msg.id = (0x01<<8) + id;
      new_msg.len = 4;

      uint32_t curr_bytes = (uint32_t)(val*1000);
      // uint32_t msg_bytes[4];

      // new_msg.buf = duty_bytes;
      new_msg.buf[0] = (curr_bytes & 0xff000000) >> 24;
      new_msg.buf[1] = (curr_bytes & 0x00ff0000) >> 16;
      new_msg.buf[2] = (curr_bytes & 0x0000ff00) >>  8;
      new_msg.buf[3] = (curr_bytes & 0x000000ff)      ;

      // return new_msg;

      break;
    }
    case 2: //set current brake
    {
      new_msg.id = (0x02<<8) + id;
      new_msg.len = 4;

      uint32_t brake_bytes = ((uint32_t)val)*1000;
      // uint32_t msg_bytes[4];

      // new_msg.buf = duty_bytes;
      new_msg.buf[0] = (brake_bytes & 0xff000000) >> 24;
      new_msg.buf[1] = (brake_bytes & 0x00ff0000) >> 16;
      new_msg.buf[2] = (brake_bytes & 0x0000ff00) >>  8;
      new_msg.buf[3] = (brake_bytes & 0x000000ff)      ;

      // return new_msg;

      break;
    }
    case 3: //set rpm
    {

      new_msg.id = (0x03<<8) + id;
      new_msg.len = 4;

      uint32_t rpm_bytes = (uint32_t)(val);
      // uint32_t msg_bytes[4];

      // new_msg.buf = duty_bytes;
      new_msg.buf[0] = (rpm_bytes & 0xff000000) >> 24;
      new_msg.buf[1] = (rpm_bytes & 0x00ff0000) >> 16;
      new_msg.buf[2] = (rpm_bytes & 0x0000ff00) >>  8;
      new_msg.buf[3] = (rpm_bytes & 0x000000ff)      ;

      // return new_msg;

      break;
    }
    case 4: //set position
    {
      new_msg.id = (0x04<<8) + id;
      new_msg.len = 4;

      uint32_t pos_bytes = (uint32_t)(val)*100;
      // uint32_t msg_bytes[4];

      // new_msg.buf = duty_bytes;
      new_msg.buf[0] = (pos_bytes & 0xff000000) >> 24;
      new_msg.buf[1] = (pos_bytes & 0x00ff0000) >> 16;
      new_msg.buf[2] = (pos_bytes & 0x0000ff00) >>  8;
      new_msg.buf[3] = (pos_bytes & 0x000000ff)      ;
      // return new_msg;

      break;
    }
  }
  return new_msg;

}

int writeCANMessage(CAN_message_t to_send){
  can1.write(to_send);
  return 0;
}

//stopping all locomotion motors
void locomotion_stop(){
  writeCANMessage(craftMessage(0,0,front_left_motor));
  writeCANMessage(craftMessage(0,0,back_right_motor));
  writeCANMessage(craftMessage(0,0,back_left_motor));
  writeCANMessage(craftMessage(0,0,front_right_motor));
  Serial.println("Locomotion Stopped");
}

void pivot_stop(){
  Serial.println("Pivot Stopped");
  writeCANMessage(craftMessage(0,0,pivot_motor));
}

void conveyor_stop(){
  Serial.println("Conveyor Stopped");
  writeCANMessage(craftMessage(0,0,conveyor_motor));
}

void bin_stop(){
  Depo_left.writeMicroseconds(1500);
  Depo_right.writeMicroseconds(1500);
  Serial.println("Bin Stopping.");
}

void stop_bot(){
  locomotion_stop();
  pivot_stop();
  conveyor_stop();
  bin_stop();
  Serial.println("The rover has been stopped ");
}

void turn_right(float speed_fac = 1){
  if (speed_fac > 0){
    Serial.println("Turning Right ");
  } else {
    Serial.println("Turning Left ");
  }
  writeCANMessage(craftMessage(0,-5*speed_fac,front_right_motor));
  writeCANMessage(craftMessage(0,5*speed_fac,back_right_motor));
  writeCANMessage(craftMessage(0,5*speed_fac,back_left_motor));
  writeCANMessage(craftMessage(0,-5*speed_fac,front_left_motor)); 
}

void go_forward(float speed_fac = 1){
  Serial.print("Going");
  Serial.print(speed_fac > 0 ? "Forward" : "Backward");
  Serial.print(speed_fac * 25);
  Serial.print("% speed.");
  Serial.println();
  writeCANMessage(craftMessage(0,15*speed_fac,front_right_motor));
  writeCANMessage(craftMessage(0,-15*speed_fac,back_right_motor));
  writeCANMessage(craftMessage(0,15*speed_fac,back_left_motor));
  writeCANMessage(craftMessage(0,-15*speed_fac,front_left_motor)); 
}

void left_motors_forward(float speed_fac = 1){
  Serial.print("Left Motors Going at ");
  Serial.print(speed_fac > 0 ? "Forward" : "Backward");
  Serial.print(speed_fac * 25);
  Serial.print("% speed.");
  Serial.println();
  writeCANMessage(craftMessage(0,15*speed_fac,back_left_motor));
  writeCANMessage(craftMessage(0,-15*speed_fac,front_left_motor)); 
}

void right_motors_forward(float speed_fac = 1){
  Serial.print("Right Motors Going at ");
  Serial.print(speed_fac > 0 ? "Forward" : "Backward");
  Serial.print(speed_fac * 25);
  Serial.print("% speed.");
  Serial.println();
  writeCANMessage(craftMessage(0,15*speed_fac,front_right_motor));
  writeCANMessage(craftMessage(0,15*speed_fac,back_right_motor));
}

void depo_bin_up(){
  // if (get_left_bin_angle() > 80 && get_right_bin_angle() > 80){
  //   Serial.println("Bin at the Top");
  // }
  // else {
    Serial.println("Bin Going Up.");
    Depo_left.writeMicroseconds(1750);
    Depo_right.writeMicroseconds(1750);
    Serial.println("Left Bin at ");
    Serial.println(get_left_bin_angle());
    Serial.println("Right Bin at ");
    Serial.println(get_right_bin_angle());
  // }
}

void depo_bin_down(){
  // if (get_left_bin_angle() < 3 && get_right_bin_angle() < 3){
  //   Serial.println("Bin at the bottom");
  // }
  // else {
    manual_balance_bin();
    Serial.println("Bin Going Down.");
    Depo_left.writeMicroseconds(1250);
    Depo_right.writeMicroseconds(1250);
    Serial.println("Left Bin at ");
    Serial.println(get_left_bin_angle());
    Serial.println("Right Bin at ");
    Serial.println(get_right_bin_angle());
  // }
}

void depo_bin_action(float d_fac = 1){
  if(d_fac == -1){
    bin_stop();
  }
  if(d_fac == 0){
    direction = UP;
    depo_bin_up();
  }
  if(d_fac == 1){
    direction = DOWN;
    depo_bin_down();
  }
}

//negative speed means outward
void pivot(float speed_fac = 1){
  Serial.print("Pivotting ");
  Serial.println(speed_fac < 0 ? "outward" : "inward");
  writeCANMessage(craftMessage(0,12 * speed_fac,pivot_motor)); 
}

void manual_pivot(float d_fac = -1){
  if(d_fac == -1){
    pivot_stop();
  }
  if(d_fac == 0){
    //negative is outward
    pivot(-1);
  }
  if(d_fac == 1){
    //positive is inward
    pivot(1);
  }
}

void move_hinge(int degree){
  int pos = 0;
  if (degree > 0){
    for (pos = 0; pos <= degree; pos += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
      hinge.write(pos); // tell servo to go to position in variable 'pos'
    }
  } else {
    degree *= -1;
    for (pos = degree; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      hinge.write(pos); // tell servo to go to position in variable 'pos'
    }
  }
}

//generic function for manual and autonomy. negative is digging, positive is dumping
void conveyor(float speed_fac = 0){
  if(speed_fac < 0){
    Serial.println("Digging Regolith");
    writeCANMessage(craftMessage(0,12*speed_fac,conveyor_motor)); 
  }
  if(speed_fac == 0){
    conveyor_stop();
  }  
  if(speed_fac > 0){
    Serial.println("Dumping Regolith");
    writeCANMessage(craftMessage(0,12*speed_fac,conveyor_motor)); 
    //move hinge down while this is spinning
    // if (!dont_move_hinge_flag){
    //   move_hinge(270);
    //   dont_move_hinge_flag = true;
    // }
    //writeCANMessage(craftMessage(1,5,conveyor_motor)); 
  }
}

//these value mappings are for manual control hex. This uses another conveyor method which 
//is applicable and generic for manual and autonomous use
void manual_conveyor(float speed_fac = 1){
  if(speed_fac == -1){
    conveyor(-2);
  }
  if(speed_fac == 0){
    conveyor(2);
  }
  if(speed_fac == 1){
    conveyor_stop();
  }
}

void zero_pivot(){
  Serial.println("Current pivot angle has been stored as locomotion position.");
  locomotion_pivot_angle = get_pivot_angle();
}

void send_sensor_data(){
  delay(500);//reduce delay after testing
  Serial.println("Sending Sensor Data");
  //int pivot_angle = get_pivot_angle();
  int pivot_angle = get_pivot_angle();
  if (pivot_angle == 255){
    pivot_angle += 1;
  }

  Wire.write(((int) pivot_angle & 0xff00) >> 8);
  delay(1000);
  Wire.write((int) pivot_angle & 0x00ff);
  delay(1000);
  Wire.write(locomotion_position_flag);
  delay(1000);
  Wire.write(excavation_position_flag);
  delay(1000);
  Wire.write(construction_position_flag);
  Serial.println("Done Sending Sensor Data");
}

//this function will do the math with converting 0-1055 to 0-360
int get_pivot_angle(){
  return current_angle;
}

//manually balancing bin without pid. 
void manual_balance_bin(){
  int left_angle = get_left_bin_angle();
  int right_angle = get_right_bin_angle();
  if (left_angle - right_angle > max_angle_difference || right_angle - left_angle > max_angle_difference) {
    Serial.println("Trying to Balance Bin");
    if (direction == DOWN){
        //make the actuator with higher readings go down more. and make the other actuator stop
        if (left_angle > right_angle){
          Depo_left.writeMicroseconds(1400);
          Depo_right.writeMicroseconds(1500);
          delay(100); //small number to be safe
        } else {
          Depo_right.writeMicroseconds(1400);
          Depo_left.writeMicroseconds(1500);
          delay(100); //small number to be safe
        }
    } else {
      //direction is up
      if (left_angle > right_angle){
          Depo_left.writeMicroseconds(1500);
          Depo_right.writeMicroseconds(1600);
          delay(100); //small number to be safe
        } else {
          Depo_right.writeMicroseconds(1500);
          Depo_left.writeMicroseconds(1600);
          delay(100); //small number to be safe
        }
    }
  }
  bin_stop();
}

//inverted mapping since left angles "decrease" when bin goes up and increase when bin goes down
int get_left_bin_angle(){
  return current_left_bin_angle;
}

int get_right_bin_angle(){
  return current_right_bin_angle;
}

void update_pivot_angle(){
  angle = map(pulseIn(pivot_ABS, HIGH),0 , 1050, 0, 360);
  if(angle != old_angle){
    current_angle = (angle+86)%360;
  } 
  old_angle = angle; 
}

//since the left bin pulse decreases when bin goes down, we can add the pulse width and invert the mapping to achieve the same 
//effect as the right bin encoder behavior. 
void update_left_bin_angle(){
  left_bin_angle = map(pulseIn(left_bin_encoder, HIGH),0 , 1050, 0, 360);
  if(left_bin_angle != old_left_bin_angle){
    current_left_bin_angle = (left_bin_angle+323)%360;
  } 
  old_left_bin_angle = left_bin_angle; 
}

//since the right bin pulse increases when the bin goes up, we can reduce the pulse width with the offset
void update_right_bin_angle(){
  right_bin_angle = map(pulseIn(right_bin_encoder, HIGH),0 , 1050, 0, 360);
  if(right_bin_angle != old_right_bin_angle){
    current_right_bin_angle = (right_bin_angle+52)%360;
    Serial.println(current_right_bin_angle);
  } 
  old_right_bin_angle = right_bin_angle; 
  delay(500);
}

bool check_left_bin(int target){
  Serial.print("Checking if left bin is near ");
  if (target == locomotion_left_bin_angle){
    Serial.println("Locomotion position ");
  } else if (target == construction_left_bin_angle){
    Serial.println("Construction position ");
  }
  return (get_left_bin_angle() >= target - max_angle_difference && get_left_bin_angle() <= target + max_angle_difference);
}

bool check_right_bin(int target){
  Serial.print("Checking if right bin is near ");
  if (target == locomotion_right_bin_angle){
    Serial.println("Locomotion position ");
  } else if (target == construction_right_bin_angle){
    Serial.println("Construction position ");
  }
  return (get_right_bin_angle() >= target - max_angle_difference && get_right_bin_angle() <= target + max_angle_difference);
}


void update_locomotion_position_flag(){
  if(get_pivot_angle() > locomotion_pivot.high){
    Serial.println("Pivot is not at locomotion Position.");
    Serial.println(get_pivot_angle());
    locomotion_position_flag = false;
  } else {
    if(check_left_bin(locomotion_left_bin_angle) && check_right_bin(locomotion_right_bin_angle)){
      locomotion_position_flag = true;
      dont_move_hinge_flag = false; // make it false so it can be moved again
      Serial.println("Rover at locomotion Position.");
    } else {
      Serial.println("Bins are not at locomotion Position.");
      locomotion_position_flag = false;
    }
  }
}

//
void update_construction_position_flag(){
  if(get_pivot_angle() > construction_pivot.low && get_pivot_angle() < construction_pivot.high){
    if(check_left_bin(construction_left_bin_angle) && check_right_bin(construction_right_bin_angle)){
      construction_position_flag = true;
      dont_move_hinge_flag = false; // make it false so it can be moved again
      Serial.println("Pivot and Bin at construction Position.");
      Serial.println("Rover at construction Position.");
    } else {
      Serial.println("Bins are not at construction Position.");
      construction_position_flag = false;
    }
  } else {
    Serial.println("Pivot is not at construction Position.");
    construction_position_flag = false;
  }
}

//for the bins, the excavation position is the same as locomotion position
void update_excavation_position_flag(){
  if(get_pivot_angle() > excavation_pivot.low && get_pivot_angle() < excavation_pivot.high){
    if(check_left_bin(locomotion_left_bin_angle) && check_right_bin(locomotion_right_bin_angle)){
      excavation_position_flag = true;
      dont_move_hinge_flag = false; // make it false so it can be moved again
      Serial.println("Rover at excavation Position.");
    } else {
      Serial.println("Bins are not at excavation Position.");
      excavation_position_flag = false;
    }
  } else {
    Serial.println("Pivot is not at excavation Position.");
    excavation_position_flag = false;
  }
}

void pivot_to_excavation_position(){
  Serial.println("Pivotting to Excavation Position");
  //if greater, the pivot is not touching ground so we need to pivot inward
  if (get_pivot_angle() > excavation_pivot.low && get_pivot_angle() < excavation_pivot.high){
    Serial.println("Pivot at Excavation Position");
  }
  else{ 
    if (get_pivot_angle() > excavation_pivot.high){
      //pivot inward
      Serial.println("Too much outward, between the higher range and chassis angle");
      pivot(-1); 
    }
    else if (get_pivot_angle() < excavation_pivot.low){
      Serial.println("Still needs to go down more, about vertical ");
      pivot(1);
    }
  }
  update_excavation_position_flag();
}

void pivot_to_locomotion_position(){
  Serial.println("Pivotting to Locomotion Position");
  if(get_pivot_angle() > locomotion_pivot.low && get_pivot_angle() < locomotion_pivot.high){
    Serial.println("Pivot at Locomotion Position");
  }
  else {
    //move the hinge down when the pivot is vertical, then
    if (get_pivot_angle() > locomotion_pivot.high){
      //if lower than minimum angle, pivot outward
      Serial.println("Angle too high and needs to pivot inward");
      pivot(-1);
    } 
    //loops back to 0 so it might read 358
    if (get_pivot_angle() > 350){
      Serial.println("Angle too low and needs to pivot outward");
      pivot(1);
    }
  }
  update_locomotion_position_flag();
}

void pivot_to_construction_position(){
  Serial.println("Pivotting to Construction Position");
  if (get_pivot_angle() > construction_pivot.low && get_pivot_angle() < construction_pivot.high){
    Serial.println("Pivot at Construction position");
  }
  else {
    //once the pivot is vertical, move the hinge up so sand is blocked from falling out
    if (get_pivot_angle() < construction_pivot.low){
      //angle is too low and need to pivot outward
      Serial.println("angle is too low and need to pivot outward");
      pivot(1);
    } 
    else{ //angle is too high and need to pivot inward
      Serial.println("Angle is too low and need to pivot inward");
      pivot(-1);
    }
  update_construction_position_flag();
  }
}

//bin to excavation is the same as this
void bin_to_locomotion_position(){
  direction == DOWN;
  manual_balance_bin();
  if (get_left_bin_angle() > locomotion_left_bin_angle + 3){
    depo_bin_down();
  }
  update_locomotion_position_flag();
}

void bin_to_construction_position(){
  direction == UP;
  manual_balance_bin();
  //give it some slack 
  if (get_left_bin_angle() < construction_left_bin_angle + 3){
    depo_bin_up();
  }
  update_construction_position_flag();
}

//SETS up the encoders pins for pivot and depo bin
void encoder_setup(){
  //pinMode(pivot_ABS, INPUT_PULLUP);
  pinMode(pivot_ABS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pivot_ABS), PulseTimer, CHANGE);
  pinMode(left_bin_encoder, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(left_bin_encoder), PulseTimer_left_bin, CHANGE);
  pinMode(right_bin_encoder, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(right_bin_encoder), PulseTimer_right_bin, CHANGE);
  zero_pivot();
}

void PulseTimer() {
  bool encoder = digitalRead(pivot_ABS);
  if (encoder == HIGH) {
    new_start_time = micros();
  }
  else {
    new_end_time = micros();
  }
}

void PulseTimer_left_bin() {
  bool encoder = digitalRead(left_bin_encoder);
  if (encoder == HIGH) {
    new_start_time_left_bin = micros();
  }
  else {
    new_end_time_left_bin = micros();
  }
}

void PulseTimer_right_bin() {
  bool encoder = digitalRead(right_bin_encoder);
  if (encoder == HIGH) {
    new_start_time_right_bin = micros();
  }
  else {
    new_end_time_right_bin = micros();
  }
}

//attaches the pin for each servo object of the bin
void depo_bin_setup(){
  Depo_left.attach(left_bin_pin);
  Depo_right.attach(right_bin_pin);
  hinge.attach(hinge_motor_pin);
}

//
void software_kill_switch(){
  if (!dead){
    dead = true;
    digitalWrite(2,LOW);
    Serial.println("Software has killed the bot");
  } else {
    dead = false;
    digitalWrite(2,HIGH);
    Serial.println("Software has unkilled the bot");
  }
}

void CAN_setup(){
  can1.begin();
  can1.setBaudRate(500000);
}

void setup() {
  CAN_setup();
  Wire.begin(teensy_address);                // join i2c bus with address #24
  Serial.println("Currently in Manual Control Mode");
  depo_bin_setup();
  pinMode(2,OUTPUT); //for software kill switch
  encoder_setup();
  update_pivot_angle();
  update_left_bin_angle();
  update_right_bin_angle();
  Serial.println("Set Up Complete");
}

void manual_receive(){
  while(Wire.available()){
    char c = Wire.read();
    Serial.println(int(c));
    manual_interpret(int(c));
    if (int(c) == 1){
      last_command = 1000;
    } else {
    last_command = int(c);
    }
  }
  manual_interpret(last_command);
}

//manual locomotion stop is 10 in hex, which is 16 in decimal

void manual_interpret(int command_decimal){
  if(command_decimal >= 32 && command_decimal <= 35){
    go_forward(command_decimal-31);
  }
  if(command_decimal >= 103 && command_decimal <= 105){
    //update_left_bin_angle();
    //update_right_bin_angle();
    depo_bin_action(command_decimal-104);
  }
  if(command_decimal >= 100 && command_decimal <= 102){
    //update_pivot_angle();
    manual_pivot(command_decimal-101);
  }
  if(command_decimal >= 112 && command_decimal <= 114){
    manual_conveyor(command_decimal-113);
  }
  if(command_decimal >= 48 && command_decimal <= 51){
    go_forward(-1*(command_decimal-47));
  }
  if(command_decimal >= 64 && command_decimal <= 67){
    turn_right(-1*(command_decimal-63));
  }
  if(command_decimal >= 80 && command_decimal <= 83){
    turn_right(1*(command_decimal-79));
  }
  if(command_decimal == 16){
    Serial.println("Getting a command to stop");
    stop_bot();
  }
  if (command_decimal == 129){
    stop_bot();
    control_mode = autonomous_control;
    Serial.println("Switched to Autonomous Control");
  }
  if (command_decimal == 128){
    send_sensor_data();
  }
  if (command_decimal == 1){
    software_kill_switch();
  }
  if (command_decimal == 96){
    zero_pivot();
  }
  if (command_decimal == 97){
    //for going to locomotion position
    //from construction to locomotion, we need to move bin down first and then pivot for special scenario
      //still be able to do other actions to help the robot is needed
    bin_to_locomotion_position();
    if (get_left_bin_angle() <= 3 && get_right_bin_angle() <= 3){
      pivot_to_locomotion_position();
    }
  }
  if (command_decimal == 99){
    //bin should be at locomotion when we pivot to excavation
      pivot_to_excavation_position();
      if (get_pivot_angle() > excavation_pivot.low && get_pivot_angle() < excavation_pivot.high){
        bin_to_locomotion_position();
      }
    }
  if (command_decimal == 98){
    //for going to construction position
    pivot_to_construction_position();
    //once the pivot is away from bin, we can move the bin so they dont collide
    if (get_pivot_angle() < construction_pivot.high && get_pivot_angle() > construction_pivot.low){
      bin_to_construction_position();
    }
  }
  if (command_decimal == 144){
      Serial.println("Moving Hinge up");
      move_hinge(-135);
      command_decimal = 1000;
      last_command = 1000;
  }
  if (command_decimal == 145){
    Serial.println("Moving Hinge down");
    move_hinge(135);
    command_decimal = 1000;
    last_command = 1000;
  }
}

float value_to_motor_speed(int decimal){
  return (decimal/25);
}

void autonomous_interpret(const i2c_data& data){
  int command = data.software_command;
  if (command == autonomous_left_motor_forward){
    float speed = value_to_motor_speed(data.value);
    left_motors_forward(speed);
  }
  if (command == autonomous_left_motor_backward){
    float speed = value_to_motor_speed(data.value) * -1;
    left_motors_forward(speed);
  }
    if (command == autonomous_right_motor_forward){
    float speed = value_to_motor_speed(data.value);
    right_motors_forward(speed);
  }
  if (command == autonomous_right_motor_backward){
    float speed = value_to_motor_speed(data.value) * -1;
    right_motors_forward(speed);
  }
  if (command == autonomous_depo_bin_stop){
    bin_stop();
  }
  if (command == autonomous_depo_bin_action){
    if (data.value == 1){
      direction = UP;
      depo_bin_up();
    } else {
      direction = DOWN;
      depo_bin_down();
    }
  }
  if (command == autonomous_pivot_stop){
    pivot_stop();
  }
  if (command == autonomous_pivot_out){
    float speed = value_to_motor_speed(data.value);
    pivot(speed);
  }
  if (command == autonomous_pivot_in){
    float speed = value_to_motor_speed(data.value) * -1;
    pivot(speed);
  }
  if (command == autonomous_belt_stop){
    conveyor_stop();
  }
  if (command == autonomous_belt_collect){
    float speed = value_to_motor_speed(data.value) * -1;
    conveyor(speed);
  }
  if (command == autonomous_belt_dump){
    float speed = value_to_motor_speed(data.value);
    conveyor(speed);
  }
  if (command == autonomous_positioning){
    if (data.value == 0){
      //go to locomotion position
      if (get_left_bin_angle() <= 3 && get_right_bin_angle <= 3){
        pivot_to_locomotion_position();
      }
      bin_to_locomotion_position();
    }
    if (data.value == 1){
      //go to construction position
      pivot_to_construction_position();
      //move the pivot away from bin, and then move bin
      if (get_pivot_angle() < construction_pivot.high && get_pivot_angle() > construction_pivot.low){
        bin_to_construction_position();
      }
    }
    if (data.value == 2){
      pivot_to_excavation_position();
      bin_to_locomotion_position();
      //the deposition bin locomotion and excavation position are the same
    }
  }
  if (command == autonomous_other_actions){
    if (data.value == 0){
      send_sensor_data();
    }
    if (data.value == 1){
      stop_bot();
      Serial.println("Switched to Manual Control");
      control_mode = manual_control;
    }
  }
}

void autonomous_receive(){
  i2c_data data;
  while(Wire.available()){
    char c1 = Wire.read();
    data.software_command = int(c1);
    char c2 = Wire.read();
    data.value = int(c2);
    //for autonomy forward of left motors the command holds an int of 2, and a value of int from 1-100
    autonomous_interpret(data);
  }
}


void loop() {
  digitalWrite(2,HIGH);
  if (control_mode == manual_control){
    digitalWrite(2,HIGH);
    manual_receive();
    }
  if (control_mode == autonomous_control){
    digitalWrite(2,HIGH);
    autonomous_receive();
  }
}

  // if ( can1.read(msg) ) {
  //   Serial.print("CAN1 "); 
  //   Serial.print("MB: "); Serial.print(msg.mb);
  //   Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
  //   Serial.print("  EXT: "); Serial.print(msg.flags.extended );
  //   Serial.print("  LEN: "); Serial.print(msg.len);
  //   Serial.print(" DATA: ");
  //   for ( uint8_t i = 0; i < 8; i++ ) {
  //     Serial.print(msg.buf[i]); Serial.print(" ");
  //   }
  //   Serial.print("  TS: "); Serial.println(msg.timestamp);
  // }
  // else if ( can2.read(msg) ) {
  //   Serial.print("CAN2 "); 
  //   Serial.print("MB: "); Serial.print(msg.mb);
  //   Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
  //   Serial.print("  EXT: "); Serial.print(msg.flags.extended );
  //   Serial.print("  LEN: "); Serial.print(msg.len);
  //   Serial.print(" DATA: ");
  //   for ( uint8_t i = 0; i < 8; i++ ) {
  //     Serial.print(msg.buf[i]); Serial.print(" ");
  //   }
  //   Serial.print("  TS: "); Serial.println(msg.timestamp);
  // }