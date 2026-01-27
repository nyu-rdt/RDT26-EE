/**
 * @file config.h
 * @brief Central configuration for Rover Control System
 * 
 * Pin definitions, CAN IDs, I2C addresses, and system constants.
 * All hardware-specific values in one place for easy modification.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============== Pin Definitions ==============
// E-Stop Relay
#define RELAY_PIN           2

// Analog Inputs
#define STRING_POT_PIN      A3

// HX711 Load Cells (4 sensors)
#define HX711_DOUT1         24
#define HX711_CLK1          25
#define HX711_DOUT2         26
#define HX711_CLK2          27
#define HX711_DOUT3         34
#define HX711_CLK3          33
#define HX711_DOUT4         20
#define HX711_CLK4          21

// PWM Outputs (Servo signals)
#define EXCAVATION_SYSTEM_PWM_PIN   11

// Encoder pins
#define ENC1_A              36
#define ENC1_B              35
#define ENC2_A              38
#define ENC2_B              37
#define ENC3_A              40
#define ENC3_B              39
#define ENC4_A              14
#define ENC4_B              15

// ============== CAN Bus Configuration ==============
#define CAN_BAUD_RATE       500000

// Motor CAN IDs (extended frame)
#define CAN_ID_FRONT_LEFT       0x78
#define CAN_ID_FRONT_RIGHT      0x16
#define CAN_ID_REAR_LEFT        0x48
#define CAN_ID_REAR_RIGHT       0x67
#define CAN_ID_DEPOSITION       0x34
#define CAN_ID_EXCAVATION_BELT  0x68

// ============== I2C Configuration ==============
#define I2C_SLAVE_ADDRESS   0x24

// ============== Command Definitions ==============
// Emergency & Control
#define CMD_EMERGENCY_STOP              1
#define CMD_LOCOMOTION_STOP             16

// Forward Movement (speed 25/50/75/100%)
#define CMD_FORWARD_25                  32
#define CMD_FORWARD_50                  33
#define CMD_FORWARD_75                  34
#define CMD_FORWARD_100                 35

// Backward Movement
#define CMD_BACKWARD_25                 48
#define CMD_BACKWARD_50                 49
#define CMD_BACKWARD_75                 50
#define CMD_BACKWARD_100                51

// Turn Left
#define CMD_LEFT_25                     64
#define CMD_LEFT_50                     65
#define CMD_LEFT_75                     66
#define CMD_LEFT_100                    67

// Turn Right
#define CMD_RIGHT_25                    80
#define CMD_RIGHT_50                    81
#define CMD_RIGHT_75                    82
#define CMD_RIGHT_100                   83

// Excavation System
#define CMD_EXCAVATION_ZERO             96
#define CMD_EXCAVATION_LOCOMOTION_POS   97
#define CMD_EXCAVATION_POSITION         98
#define CMD_BELT_STOP                   99
#define CMD_BELT_OUTWARD                100
#define CMD_BELT_INWARD                 101
#define CMD_ACME_UP                     102
#define CMD_ACME_DOWN                   103

// Deposition System
#define CMD_DEPOSITION_ROTATE_COLLECTION 112
#define CMD_DEPOSITION_ROTATE_DUMPING    113
#define CMD_DEPOSITION_ROTATE_STOP       114

// Data & Mode
#define CMD_REQUEST_DATA                128
#define CMD_SWITCH_AUTONOMOUS           129

// ============== Position States ==============
typedef enum {
    POSITION_UNKNOWN = 0,
    POSITION_LOCOMOTION = 1,
    POSITION_EXCAVATION = 2
} ExcavationPosition_t;

// ============== System Parameters ==============
// Speed limiting (safety)
#define LOCOMOTION_DUTY_CYCLE           0.33f   // 33% max speed
#define DEPOSITION_DUTY_CYCLE           0.20f   // 20% for deposition
#define EXCAVATION_DUTY_CYCLE           0.42f   // 42% for excavation belt

// String potentiometer calibration
#define STRING_POT_SCALE                27.0f
#define STRING_POT_OFFSET               0.719f

// Excavation position thresholds (inches)
#define DEFAULT_LOCOMOTION_THRESHOLD    29.0f
#define DEFAULT_EXCAVATION_THRESHOLD    17.0f

// Timeout for position movements (ms)
#define POSITION_MOVE_TIMEOUT_MS        25000

// Command refresh interval (ms) - for CAN motor keepalive
#define COMMAND_REFRESH_INTERVAL        500

// HX711 calibration factors
#define HX711_CAL_FACTOR_1              (-102.0f)
#define HX711_CAL_FACTOR_2              (105.0f)
#define HX711_CAL_FACTOR_3              (-102.0f)
#define HX711_CAL_FACTOR_4              (111.0f)

// PWM microseconds for servo signals
#define PWM_NEUTRAL                     1500
#define PWM_EXCAVATION_UP               1460
#define PWM_EXCAVATION_DOWN             1522

// Encoder properties
#define COUNTS_PER_REVOLUTION           8192.0f
#define DEGREES_PER_COUNT               (360.0f / COUNTS_PER_REVOLUTION)

#endif // CONFIG_H
