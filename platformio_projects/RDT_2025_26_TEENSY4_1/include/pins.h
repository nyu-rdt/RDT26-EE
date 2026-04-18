#pragma once

// I2C
#define I2C_SDA_PIN 25
#define I2C_SCL_PIN 24

// EE Box — relay
#define RELAY_DRIVER_PIN    2
#define RELAY_READ_PIN      3
#define RELAY_3S_LOW_PIN    4
#define RELAY_6S_LOW_PIN    5

// Current sensor mux
#define CURRENT_INPUT_PIN      26
#define CURRENT_SELECT_PIN_0   27
#define CURRENT_SELECT_PIN_1   28
#define CURRENT_SELECT_PIN_2   29

// Rotary encoders
#define ENC1_A 21
#define ENC1_B 20
#define ENC2_A 19
#define ENC2_B 18

// Excavation stepper (DRV8825)
#define STEPPER_DIR_PIN    6
#define STEPPER_STEP_PIN   7
#define STEPPER_ENABLE_PIN 8
#define STEPPER_M0_PIN     10
#define STEPPER_M1_PIN     11
#define STEPPER_M2_PIN     12

// Excavation string pot
#define STRING_POT_PIN 39

// Deposition door H-bridge (IN1, IN2, ENA)
#define DEPOSITION_DOOR_IN1_PIN 40
#define DEPOSITION_DOOR_IN2_PIN 41
#define DEPOSITION_DOOR_ENA_PIN 14

// Vibration motor
#define VIB_MOTOR_PIN 30
