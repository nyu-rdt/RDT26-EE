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
// Analog Inputs
#define STRING_POT_PIN      A1

// HX711 Load Cells (4 sensors)
#define HX711_DOUT1         13
#define HX711_CLK1          9
#define HX711_DOUT2         11
#define HX711_CLK2          8
#define HX711_DOUT3         7
#define HX711_CLK3          6
#define HX711_DOUT4         5
#define HX711_CLK4          4

// PWM Outputs (Servo signals)
#define EXCAVATION_BELT_PWM_PIN     14
#define EXCAVATION_SYSTEM_PWM_PIN   3

// ============== CAN Bus Configuration ==============
#define CAN_BAUD_RATE       500000

// Motor CAN IDs (extended frame)
#define CAN_ID_FRONT_LEFT   0x67
#define CAN_ID_FRONT_RIGHT  0x78
#define CAN_ID_REAR_LEFT    0x16
#define CAN_ID_REAR_RIGHT   0x48
#define CAN_ID_DEPOSITION   0x42

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
#define LOCOMOTION_DUTY_CYCLE           0.20f   // 20% max speed
#define DEPOSITION_DUTY_CYCLE           0.15f   // 15% for deposition

// String potentiometer calibration
#define STRING_POT_SCALE                27.0f
#define STRING_POT_OFFSET               0.719f

// Excavation position thresholds (inches)
#define DEFAULT_LOCOMOTION_THRESHOLD    10.0f
#define DEFAULT_EXCAVATION_THRESHOLD    30.0f

// Timeout for position movements (ms)
#define POSITION_MOVE_TIMEOUT_MS        10000

// HX711 calibration factors
#define HX711_CAL_FACTOR_1              (-1000.0f)
#define HX711_CAL_FACTOR_2              (-1000.0f)
#define HX711_CAL_FACTOR_3              (-1000.0f)
#define HX711_CAL_FACTOR_4              (-1000.0f)

// PWM microseconds for servo signals
#define PWM_NEUTRAL                     1500
#define PWM_BELT_FORWARD                1550
#define PWM_BELT_REVERSE                1450
#define PWM_EXCAVATION_UP               1550
#define PWM_EXCAVATION_DOWN             1450

#endif // CONFIG_H
