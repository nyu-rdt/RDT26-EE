/**
 * Command Protocol Definitions
 * Rover Control System
 * 
 * I2C command bytes received from master controller
 * (Same protocol as original Teensy implementation)
 */

#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

/* ============== Command Constants ============== */
#define CMD_EMERGENCY_STOP              1

#define CMD_LOCOMOTION_STOP             16
#define CMD_FORWARD_25                  32
#define CMD_FORWARD_50                  33
#define CMD_FORWARD_75                  34
#define CMD_FORWARD_100                 35
#define CMD_BACKWARD_25                 48
#define CMD_BACKWARD_50                 49
#define CMD_BACKWARD_75                 50
#define CMD_BACKWARD_100                51
#define CMD_LEFT_25                     64
#define CMD_LEFT_50                     65
#define CMD_LEFT_75                     66
#define CMD_LEFT_100                    67
#define CMD_RIGHT_25                    80
#define CMD_RIGHT_50                    81
#define CMD_RIGHT_75                    82
#define CMD_RIGHT_100                   83

#define CMD_EXCAVATION_ZERO             96
#define CMD_EXCAVATION_LOCOMOTION_POS   97
#define CMD_EXCAVATION_POSITION         98
#define CMD_BELT_STOP                   99
#define CMD_BELT_OUTWARD                100
#define CMD_BELT_INWARD                 101

#define CMD_DEPOSITION_ROTATE_COLLECTION 112
#define CMD_DEPOSITION_ROTATE_DUMPING    113
#define CMD_DEPOSITION_ROTATE_STOP       114

#define CMD_REQUEST_DATA                128
#define CMD_SWITCH_AUTONOMOUS           129

/* ============== CAN Motor IDs ============== */
#define FRONT_LEFT_MOTOR_CAN_ID         0x67
#define FRONT_RIGHT_MOTOR_CAN_ID        0x78
#define REAR_LEFT_MOTOR_CAN_ID          0x16
#define REAR_RIGHT_MOTOR_CAN_ID         0x48
#define DEPOSITION_MOTOR_CAN_ID         0x42

/* ============== Position States ============== */
typedef enum {
    POSITION_UNKNOWN    = 0,
    POSITION_LOCOMOTION = 1,
    POSITION_EXCAVATION = 2
} ExcavationPosition_t;

/* ============== Control Constants ============== */
#define DUTY_CYCLE_LIMIT            0.2f
#define DEPOSITION_DUTY_CYCLE       0.15f

#define LOCOMOTION_LENGTH_DEFAULT   10.0f
#define EXCAVATION_LENGTH_DEFAULT   30.0f

/* ADC conversion: STM32 12-bit ADC, 3.3V ref */
/* Original: 27 * voltage - 0.719 with 5V/10-bit */
/* Adjusted for 3.3V/12-bit: same physical sensor */
#define VOLTAGE_LENGTH_CONVERT      27.0f
#define VOLTAGE_LENGTH_Y_INTERCEPT  0.719f
#define ADC_VREF                    3.3f
#define ADC_MAX_VALUE               4095.0f

/* HX711 calibration factors */
#define CALIBRATION_FACTOR_1        (-1000.0f)
#define CALIBRATION_FACTOR_2        (-1000.0f)
#define CALIBRATION_FACTOR_3        (-1000.0f)
#define CALIBRATION_FACTOR_4        (-1000.0f)

/* Servo pulse widths (microseconds) */
#define SERVO_NEUTRAL_US            1500
#define SERVO_BELT_OUTWARD_US       1450
#define SERVO_BELT_INWARD_US        1550
#define SERVO_EXCAVATION_UP_US      1550
#define SERVO_EXCAVATION_DOWN_US    1450

/* Timeouts */
#define EXCAVATION_TIMEOUT_MS       10000

#endif /* COMMANDS_H */
