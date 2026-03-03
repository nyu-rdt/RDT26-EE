#ifndef CONFIG_H
#define CONFIG_H

// default I2C bus (Wire)
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

// Child Address
#define I2C_CHILD_ADDRESS 0x08

// Command bytes (action)
#define CMD_FORWARD   0x01
#define CMD_BACKWARD  0x02
#define CMD_STOP      0x03
#define CMD_TURN_LEFT 0x04

// Speed bytes
#define SPEED_25      25
#define SPEED_50      50
#define SPEED_75      75
#define SPEED_100     100

#endif